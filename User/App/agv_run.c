#include "agv_run.h"
#include "agv_config.h"
#include "astar.h"
#include "survey.h"
#include "chassis.h"
#include "user_math.h"
#include <math.h>

AgvStatus agv;
static unsigned run_mode, dock_step;
static uint8_t moving, anchored, direct, heading_bad;
static uint16_t path_index;
static uint32_t since, move_since, version, heading_bad_since;
static float origin_x, origin_y, origin_yaw, cs, sn;
static MapPoint home, gateway, cursor, target, segment_start, leg_a, leg_b;

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
    moving = anchored = direct = heading_bad = 0;
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
    float angle = Math_WrapDeg(chassis.yaw - origin_yaw) * MATH_DEG_TO_RAD;
    HAL_StatusTypeDef result = Chassis_MoveDistanceHold(
        cosf(angle) * dx + sinf(angle) * dy, -sinf(angle) * dx + cosf(angle) * dy, origin_yaw);
    if (result == HAL_BUSY)
        return 0;
    if (result != HAL_OK)
    {
        Fail(AGV_ERR_CHASSIS);
        return 0;
    }
    target = p;
    segment_start = cursor;
    moving = 1;
    move_since = HAL_GetTick();
    return 1;
}

/* 当前段的路径长度：直线段只有起终两点，否则取 A* 结果。 */
static uint16_t PathLength(void)
{
    return direct ? 2U : AStar_PathLength();
}

static uint8_t PathPoint(uint16_t index, MapPoint *p)
{
    if (direct)
    {
        if (index > 1U)
            return 0;
        *p = index ? leg_b : leg_a;
        return 1;
    }
    MapCell cell;
    return AStar_PathCell(index, &cell) && Map_GridToWorld(cell.row, cell.col, p);
}

static int Sign(int32_t v)
{
    return (v > 0) - (v < 0);
}

static uint16_t ClampCell(int value, uint16_t limit)
{
    return (uint16_t)(value < 0 ? 0 : (value >= (int)limit ? (int)limit - 1 : value));
}

/* 规划当前段：通道直线无碰撞时直接走直线；否则在本段走廊内做 A*，防止抄近路漏覆盖。 */
static void PlanLeg(void)
{
    unsigned edge;
    if (!Survey_Leg(agv.leg, &leg_a, &leg_b, &edge))
    {
        agv.state = AGV_DOCK_IN;
        dock_step = 0;
        return;
    }
    path_index = 0;
    direct = Map_CanSegment(leg_a, leg_b, MAP_MARGIN_MM);
    if (direct)
    {
        agv.state = AGV_DRIVE;
        return;
    }
    MapCell s, g;
    Map_WorldToGrid(leg_a.x, leg_a.y, &s.row, &s.col);
    Map_WorldToGrid(leg_b.x, leg_b.y, &g.row, &g.col);
    const int w = (int)AGV_CORRIDOR_CELLS;
    uint16_t r0 = ClampCell((s.row < g.row ? s.row : g.row) - w, MAP_ROWS);
    uint16_t c0 = ClampCell((s.col < g.col ? s.col : g.col) - w, MAP_COLS);
    uint16_t r1 = ClampCell((s.row > g.row ? s.row : g.row) + w, MAP_ROWS);
    uint16_t c1 = ClampCell((s.col > g.col ? s.col : g.col) + w, MAP_COLS);
    if (!AStar_Begin(s, g, r0, c0, r1, c1))
        Fail(AGV_ERR_PATH);
    else
        agv.state = AGV_PLAN;
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
        cs = cosf(origin_yaw * MATH_DEG_TO_RAD);
        sn = sinf(origin_yaw * MATH_DEG_TO_RAD);
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
    float heading = Math_WrapDeg(chassis.yaw - origin_yaw);
    if (!isfinite(agv.x) || !isfinite(agv.y) || !isfinite(chassis.yaw) ||
        !Map_CanPoseHeading(agv.x, agv.y, heading, pose_margin))
    {
        Fail(AGV_ERR_POSE);
        return;
    }
    if (fabsf(heading) <= AGV_HEADING_FAULT_DEG)
        heading_bad = 0;
    else if (!heading_bad)
    {
        heading_bad = 1;
        heading_bad_since = now;
    }
    else if (now - heading_bad_since > AGV_HEADING_FAULT_MS)
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
        uint16_t length = PathLength();
        if (path_index + 1U >= length)
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
        MapPoint a, b;
        uint16_t next = path_index + 1U;
        if (!PathPoint(path_index, &a) || !PathPoint(next, &b))
        {
            Fail(AGV_ERR_PATH);
            return;
        }
        int sx = Sign(b.x - a.x), sy = Sign(b.y - a.y);
        /* 合并同方向的连续栅格步，每拍工作量有上限。 */
        for (unsigned n = 0; n < 100U && next + 1U < length; ++n)
        {
            MapPoint c;
            if (!PathPoint(next + 1U, &c) || Sign(c.x - b.x) != sx || Sign(c.y - b.y) != sy)
                break;
            b = c;
            ++next;
        }
        if (BeginMove(b, MAP_MARGIN_MM))
            path_index = next;
    }
}
