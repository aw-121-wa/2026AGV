#include "agv_run.h"
#include "agv_config.h"
#include "astar.h"
#include "survey.h"
#include "chassis.h"
#include <math.h>

AgvStatus agv;
static unsigned run_mode, dock_step;
static uint8_t moving, anchored;
static uint16_t path_index;
static uint32_t since, move_since, version;
static float origin_x, origin_y, origin_yaw, cs, sn;
static MapPoint home, gateway, cursor, target, segment_start;

/* 把角度归一化到 [-180, 180) 区间。 */
static float Wrap(float a)
{
    a = fmodf(a + 180.0f, 360.0f);
    if (a < 0)
        a += 360;
    return a - 180;
}

/* 进入故障态：立即停车并记录错误码。 */
static void Fail(AgvError error)
{
    Chassis_Stop();
    moving = 0;
    agv.error = error;
    agv.state = AGV_FAULT;
}

/* 外部停车入口：停车并切到停止态。 */
void Agv_Stop(void)
{
    Chassis_Stop();
    moving = 0;
    agv.state = AGV_STOPPED;
}

/* 按运行模式初始化任务：巡检模式建图并规划覆盖路线，其他模式置为空闲。 */
void Agv_Init(unsigned mode, unsigned zone)
{
    run_mode = mode;
    agv = (AgvStatus){0};
    moving = anchored = 0;
    since = HAL_GetTick();
    dock_step = 0;
    path_index = 0;
    if (mode != AGV_MODE_SURVEY)
    {
        agv.state = AGV_IDLE;
        return;
    }
    Map_Init();
    Survey_Init(zone);
    if (!Map_StartPoint(zone, &home))
    {
        Fail(AGV_ERR_POSE);
        return;
    }
    gateway = Survey_Gateway(zone);
    agv.state = AGV_WAIT;
}

/* 用起始位姿把底盘里程计换算成场地坐标。 */
static void Pose(void)
{
    float dx = chassis.pos_x - origin_x, dy = chassis.pos_y - origin_y;
    agv.x = home.x + cs * dx + sn * dy;
    agv.y = home.y - sn * dx + cs * dy;
}

/* 发起一段直线移动：先做碰撞检查，再调用底盘定距并锁定航向。 */
static uint8_t BeginMove(MapPoint p, float margin)
{
    if (!Map_CanSegment(cursor, p, margin))
    {
        Fail(AGV_ERR_PATH);
        return 0;
    }
    float dx = p.x - agv.x, dy = p.y - agv.y;
    if (hypotf(dx, dy) <= AGV_POSITION_TOL_MM)
    {
        cursor = p;
        return 1;
    }
    float angle = Wrap(chassis.yaw - origin_yaw) * 0.01745329252f;
    HAL_StatusTypeDef result = Chassis_MoveDistance(cosf(angle) * dx + sinf(angle) * dy,
                                                    -sinf(angle) * dx + cosf(angle) * dy);
    if (result == HAL_BUSY)
        return 0;
    if (result != HAL_OK)
    {
        Fail(AGV_ERR_CHASSIS);
        return 0;
    }
    chassis.heading_target = origin_yaw;
    target = p;
    segment_start = cursor;
    moving = 1;
    move_since = HAL_GetTick();
    return 1;
}

/* 为当前这一小段做 A* 规划，并把搜索限制在本段走廊内。 */
static void PlanLeg(void)
{
    MapPoint a, b;
    unsigned edge;
    if (!Survey_Leg(agv.leg, &a, &b, &edge))
    {
        agv.state = AGV_DOCK_IN;
        dock_step = 0;
        return;
    }
    MapCell s, g;
    Map_WorldToGrid(a.x, a.y, &s.row, &s.col);
    Map_WorldToGrid(b.x, b.y, &g.row, &g.col);
    /* Restrict to this edge's corridor, preventing coverage shortcuts. */
    int r0 = (s.row < g.row ? s.row : g.row) - 15, c0 = (s.col < g.col ? s.col : g.col) - 15;
    int r1 = (s.row > g.row ? s.row : g.row) + 15, c1 = (s.col > g.col ? s.col : g.col) + 15;
    if (!AStar_Begin(s, g, (uint16_t)r0, (uint16_t)c0, (uint16_t)r1, (uint16_t)c1))
        Fail(AGV_ERR_PATH);
    else
    {
        path_index = 0;
        agv.state = AGV_PLAN;
    }
}

/* 巡检主状态机：等就绪、出库、规划、行驶、入库，每拍推进一步。 */
void Agv_Tick(void)
{
    if (run_mode != AGV_MODE_SURVEY || agv.state == AGV_IDLE || agv.state == AGV_DONE ||
        agv.state == AGV_STOPPED || agv.state == AGV_FAULT)
        return;
    uint32_t now = HAL_GetTick();
    if (chassis.state == CHASSIS_STATE_FAULT)
    {
        Fail(AGV_ERR_CHASSIS);
        return;
    }
    if (agv.state == AGV_WAIT)
    {
        if (now - since > AGV_READY_TIMEOUT_MS)
        {
            Fail(AGV_ERR_READY);
            return;
        }
        if (chassis.state != CHASSIS_STATE_READY || chassis.mode != CHASSIS_MODE_VEL ||
            now - since < AGV_START_DELAY_MS)
            return;
        origin_x = chassis.pos_x;
        origin_y = chassis.pos_y;
        origin_yaw = chassis.yaw;
        cs = cosf(origin_yaw * 0.01745329252f);
        sn = sinf(origin_yaw * 0.01745329252f);
        cursor = home;
        version = Map_Revision();
        anchored = 1;
        Chassis_SetMoveSpeedLimit(AGV_SURVEY_SPEED_MM_S);
        agv.state = AGV_DOCK_OUT;
    }
    if (!anchored)
        return;
    Pose();
    if (version != Map_Revision())
    {
        Fail(AGV_ERR_MAP);
        return;
    }
    float pose_margin =
        (agv.state == AGV_DOCK_OUT || agv.state == AGV_DOCK_IN) ? 0.0f : MAP_MARGIN_MM;
    if (!isfinite(agv.x) || !isfinite(agv.y) || !isfinite(chassis.yaw) ||
        fabsf(Wrap(chassis.yaw - origin_yaw)) > AGV_HEADING_TOL_DEG ||
        !Map_CanPoseHeading(agv.x, agv.y, Wrap(chassis.yaw - origin_yaw), pose_margin))
    {
        Fail(AGV_ERR_POSE);
        return;
    }
    if (moving)
    {
        float cross =
            segment_start.x == target.x ? fabsf(agv.x - target.x) : fabsf(agv.y - target.y);
        if (cross > AGV_CROSS_TRACK_MM)
        {
            Fail(AGV_ERR_POSE);
            return;
        }
        if (now - move_since > AGV_LEG_TIMEOUT_MS)
        {
            Fail(AGV_ERR_TIMEOUT);
            return;
        }
        if (chassis.mode != CHASSIS_MODE_VEL)
            return;
        if (hypotf(agv.x - target.x, agv.y - target.y) > AGV_POSITION_TOL_MM)
        {
            Fail(AGV_ERR_POSE);
            return;
        }
        cursor = target;
        moving = 0;
    }
    if (agv.state == AGV_DOCK_OUT || agv.state == AGV_DOCK_IN)
    {
        uint8_t exiting = agv.state == AGV_DOCK_OUT;
        if (dock_step == 2)
        {
            if (exiting)
            {
                agv.leg = 0;
                PlanLeg();
            }
            else
            {
                Chassis_Stop();
                agv.state = AGV_DONE;
            }
            return;
        }
        MapPoint p = dock_step == 0 ? (MapPoint){home.x, gateway.y} : (exiting ? gateway : home);
        if (BeginMove(p, 0))
            ++dock_step;
        return;
    }
    if (agv.state == AGV_PLAN)
    {
        AStar_Step(AGV_SEARCH_BUDGET);
        if (AStar_Status() == ASTAR_BUSY)
            return;
        if (AStar_Status() != ASTAR_FOUND)
        {
            Fail(AGV_ERR_PATH);
            return;
        }
        agv.state = AGV_DRIVE;
    }
    if (agv.state == AGV_DRIVE)
    {
        if (path_index + 1U >= AStar_PathLength())
        {
            MapPoint a, b;
            unsigned edge;
            if (!Survey_Leg(agv.leg, &a, &b, &edge))
            {
                Fail(AGV_ERR_PATH);
                return;
            }
            agv.covered |= (uint16_t)(1U << edge);
            ++agv.leg;
            PlanLeg();
            return;
        }
        MapCell a, b;
        AStar_PathCell(path_index, &a);
        uint16_t next = path_index + 1U;
        if (!AStar_PathCell(next, &b))
        {
            Fail(AGV_ERR_PATH);
            return;
        }
        int dr = (int)b.row - a.row, dc = (int)b.col - a.col;
        /* Coalesce straight grid steps, bounded work per tick. */
        for (unsigned n = 0; n < 100 && next + 1U < AStar_PathLength(); ++n)
        {
            MapCell c;
            AStar_PathCell(next + 1U, &c);
            if ((int)c.row - b.row != dr || (int)c.col - b.col != dc)
                break;
            b = c;
            ++next;
        }
        MapPoint p;
        Map_GridToWorld(b.row, b.col, &p);
        if (BeginMove(p, MAP_MARGIN_MM))
            path_index = next;
    }
}
