#include "agv_run.h"
#include "agv_config.h"
#include "astar.h"
#include "survey.h"
#include "chassis.h"
#include "chassis_telemetry.h"
#include "user_math.h"
#include <math.h>

AgvStatus agv;
static unsigned run_mode, dock_step, planned_lap;
static chassis_waypoint tour[CHASSIS_PATH_CAPACITY];
static uint16_t tour_count, planned_covered;
static uint8_t executing, start_pending;
static uint32_t start_wait_ms;
static uint8_t anchored, direct, heading_bad;
static uint16_t path_index;
static uint32_t since, version, heading_bad_since;
static uint32_t exec_since, exec_timeout_ms;
static float origin_x, origin_y, origin_yaw, cs, sn;
static float tour_length_mm;
static MapPoint home, gateway, cursor, leg_a, leg_b;

/* 进入故障态：立即停车并记录错误码。 */
static void Fail(AgvError error)
{
    Chassis_Stop();
    agv.error = error;
    agv.state = AGV_FAULT;
}

/* 在跑（等底盘、出库、规划、行驶、入库）。 */
static uint8_t Running(void)
{
    return agv.state >= AGV_WAIT && agv.state <= AGV_DOCK_IN;
}

/* 外部停车入口：停车；正在跑的这一趟记成停止，没在跑的状态不变。 */
void Agv_Stop(void)
{
    Chassis_Stop();
    if (Running())
        agv.state = AGV_STOPPED;
}

/* 清掉上一趟的全部进度。 */
static void Clear(void)
{
    agv = (AgvStatus){0};
    anchored = direct = heading_bad = 0;
    since = HAL_GetTick();
    dock_step = 0;
    path_index = 0;
    planned_lap = tour_count = planned_covered = executing = start_pending = 0;
    tour_length_mm = 0.0f;
    exec_since = exec_timeout_ms = 0U;
}

void Agv_Init(unsigned mode)
{
    run_mode = mode;
    Clear();
    agv.state = AGV_IDLE;
}

HAL_StatusTypeDef Agv_Start(unsigned zone)
{
    if (run_mode != AGV_MODE_SURVEY || zone < 1U || zone > 2U)
        return HAL_ERROR;
    if (Running() || chassis.state != CHASSIS_STATE_READY || chassis.mode != CHASSIS_MODE_VEL)
        return HAL_BUSY;
    Clear();
    agv.zone = (uint8_t)zone;
    Map_Init();
    Survey_Init(zone);
    if (!Map_StartPoint(zone, &home))
    {
        Fail(AGV_ERR_POSE);
        return HAL_OK;
    }
    gateway = Survey_Gateway(zone);
    agv.state = AGV_WAIT;
    return HAL_OK;
}

/* 用起始位姿把底盘里程计换算成场地坐标。 */
static void Pose(void)
{
    float dx = chassis.pos_x - origin_x, dy = chassis.pos_y - origin_y;
    agv.x = home.x + cs * dx + sn * dy;
    agv.y = home.y - sn * dx + cs * dy;
}

/* 第 index 个路线点的场地坐标：0 是出发点，后面是 tour 换算回场地系。 */
static MapPoint TourPoint(unsigned index)
{
    if (!index)
        return home;
    float dx = tour[index - 1U].x - origin_x, dy = tour[index - 1U].y - origin_y;
    return (MapPoint){(int32_t)lroundf(home.x + cs * dx + sn * dy),
                      (int32_t)lroundf(home.y - sn * dx + cs * dy)};
}

void Agv_RouteReport(void)
{
    if (executing)
        ChassisTelemetry_SendRoute(tour_count + 1U, TourPoint);
    else
        ChassisTelemetry_Puts("ROUTE none\r\n");
}

/* 校验一段直线移动并追加到连续路径：先做碰撞检查，再换算成底盘系航点。 */
static uint8_t BeginMove(MapPoint p, float margin)
{
    if (!Map_CanSegment(cursor, p, margin))
    {
        Fail(AGV_ERR_PATH);
        return 0;
    }
    if (p.x == cursor.x && p.y == cursor.y)
        return 1;
    if (tour_count >= CHASSIS_PATH_CAPACITY)
    {
        Fail(AGV_ERR_PATH);
        return 0;
    }
    float dx = p.x - home.x, dy = p.y - home.y;
    tour[tour_count++] =
        (chassis_waypoint){origin_x + cs * dx - sn * dy, origin_y + sn * dx + cs * dy, origin_yaw};
    tour_length_mm += hypotf(p.x - cursor.x, p.y - cursor.y);
    cursor = p;
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
        if (++planned_lap < AGV_SURVEY_LAPS)
        {
            agv.leg = 0;
            PlanLeg();
            return;
        }
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
        if (chassis.state != CHASSIS_STATE_READY || chassis.mode != CHASSIS_MODE_VEL)
            return; // 开始的时候是 READY 的，这里只等它刚好掉出 READY 又回来
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
    if (executing)
    {
        if (chassis_path_result.max_cross_mm > AGV_CROSS_TRACK_MM)
        {
            Fail(AGV_ERR_POSE);
            return;
        }
        if (version != Map_Revision())
        {
            Fail(AGV_ERR_MAP);
            return;
        }
        if (!Map_CanPoseHeading(agv.x, agv.y, Math_WrapDeg(chassis.yaw - origin_yaw), 0))
        {
            Fail(AGV_ERR_POSE);
            return;
        }
        if (now - exec_since > exec_timeout_ms)
        {
            Fail(AGV_ERR_TIMEOUT);
            return;
        }
        if (chassis.mode == CHASSIS_MODE_VEL)
        {
            /* 整圈结束只要求底盘报告走完。回库残差是里程计统计量，不拿 5 mm 一刀切判故障，
               否则真实场地里几毫米的回库偏差会让 AGV_DONE 永远走不到。 */
            agv.home_residual_mm = hypotf(agv.x - home.x, agv.y - home.y);
            if (!chassis_path_result.completed)
            {
                Fail(AGV_ERR_POSE);
                return;
            }
            agv.covered = planned_covered;
            agv.state = AGV_DONE;
        }
        return;
    }
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
                if (!start_pending)
                {
                    start_pending = 1;
                    start_wait_ms = now;
                }
                HAL_StatusTypeDef result =
                    Chassis_FollowPath(tour, tour_count, AGV_STOP_EACH_SEGMENT);
                if (result == HAL_OK)
                {
                    /* 整圈按航程给总超时：1.5 倍预计行驶时间 + 一段的兜底时间。 */
                    exec_timeout_ms = AGV_LEG_TIMEOUT_MS +
                                      (uint32_t)(1500.0f * tour_length_mm / AGV_SURVEY_SPEED_MM_S);
                    exec_since = now;
                    executing = 1;
                    agv.state = AGV_DRIVE;
                    Agv_RouteReport();
                }
                else if (result != HAL_BUSY)
                    Fail(AGV_ERR_CHASSIS);
                else if (now - start_wait_ms > AGV_READY_TIMEOUT_MS)
                    Fail(AGV_ERR_READY);
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
            planned_covered |= (uint16_t)(1U << edge);
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
