#include "competition.h"
#if AGV_MODE == AGV_MODE_COMPETITION
#include "astar.h"
#include "chassis.h"
#include "chassis_telemetry.h"
#include "comp_config.h"
#include "map.h"
#include "rplidar.h"
#include "ser_port.h"
#include "user_math.h"
#include <math.h>
#include <string.h>

#define COMP_RUN_STOPS 2U // 二维码板前、场地中央

CompStatus comp;

static MapPoint home;
static float origin_x, origin_y, origin_yaw, cs, sn;
static uint32_t boot_ms, lidar_cmd_ms, lidar_last_sample_ms, hz_ms, hz_scans;
static uint8_t scan_active;
static uint32_t scan_start_ms, scan_base;
static MapPoint scan_from; // 上次扫描的位置
static unsigned obstacles_printed;
static uint32_t pose_ms, mode_ms, report_ms;
static uint8_t obstacles_dirty;
static RPLidarParser parser;
static TaskLine scan_line;
static uint32_t camera_task_ms;
static char last_rejected[VL_MAX_PAYLOAD + 1U];
static const char *task_source;
static MapPoint route[COMP_ROUTE_MAX]; // route[0] 是规划时车的位置，最后一个是目标
static unsigned route_count;
static MapPoint route_goal;
static float route_heading; // 场地航向：走的时候保持这个直角
static const char *route_why;
static uint32_t route_revision;
static unsigned run_stop;  // 正在去第几个停车点
static unsigned run_scans; // 这次流程在几个位置扫过了
static MapPoint run_goal;
static uint32_t run_since;
static const char *run_why;

HAL_StatusTypeDef Comp_Init(unsigned zone)
{
    comp = (CompStatus){0};
    Map_Init();
    if (!Map_StartPoint((uint8_t)zone, &home))
        return HAL_ERROR;
    LidarObs_Init();
    VisionLink_Init();
    RPLidar_Reset(&parser);
    scan_line = (TaskLine){0};
    camera_task_ms = 0;
    last_rejected[0] = '\0';
    route_count = 0U;
    route_why = NULL;
    run_why = NULL;
    scan_active = 0U;
    obstacles_printed = 0U;
    boot_ms = report_ms = HAL_GetTick();
    if (SerPort_Init(SER_LIDAR, COMP_LIDAR_BAUD) != HAL_OK ||
        SerPort_Init(SER_VISION, COMP_VISION_BAUD) != HAL_OK ||
        SerPort_Init(SER_SCANNER, COMP_SCANNER_BAUD) != HAL_OK)
        return HAL_ERROR;
    return HAL_OK;
}

/* 记下当前里程计读数，从现在起 home 对应当前位置，origin_yaw 对应场地航向 0。 */
static void Anchor(float heading_deg)
{
    origin_x = chassis.pos_x;
    origin_y = chassis.pos_y;
    origin_yaw = chassis.yaw - heading_deg;
    cs = cosf(origin_yaw * MATH_DEG_TO_RAD);
    sn = sinf(origin_yaw * MATH_DEG_TO_RAD);
    comp.anchored = 1;
    /* 正在扫描时位姿变了：之前按旧位姿算的点作废，从头扫。 */
    if (scan_active)
    {
        LidarObs_Init();
        scan_base = 0U;
        scan_start_ms = HAL_GetTick();
    }
}

/* 与跑图模式相同的约定：底盘 READY 后记下里程计原点和航向零点，之后换算到场地坐标。 */
static void UpdatePose(uint32_t now)
{
    if (!comp.anchored)
    {
        if (chassis.state != CHASSIS_STATE_READY || now - boot_ms < AGV_START_DELAY_MS)
            return;
        Anchor(0.0f);
        ChassisTelemetry_Printf("COMP pose anchored at (%ld,%ld)\r\n", (long)home.x,
                                (long)home.y);
    }
    float dx = chassis.pos_x - origin_x, dy = chassis.pos_y - origin_y;
    comp.x = home.x + cs * dx + sn * dy;
    comp.y = home.y - sn * dx + cs * dy;
    comp.heading_deg = Math_WrapDeg(chassis.yaw - origin_yaw);
}

/* 启动序列：STOP → 等 COMP_LIDAR_STOP_MS（让雷达停流并准备好接收命令）→ 清缓冲 → SCAN。 */
static void LidarStop(uint32_t now)
{
    SerPort_Write(SER_LIDAR, RPLIDAR_CMD_STOP, 2);
    comp.lidar_state = COMP_LIDAR_STOPPING;
    lidar_cmd_ms = now;
}

static void LidarScan(uint32_t now)
{
    SerPort_Flush(SER_LIDAR);
    RPLidar_Reset(&parser);
    hz_scans = 0;
    SerPort_Write(SER_LIDAR, RPLIDAR_CMD_SCAN, 2);
    comp.lidar_state = COMP_LIDAR_START;
    lidar_cmd_ms = lidar_last_sample_ms = now;
}

/* 不扫描时关掉接收：雷达接着转，但一个字节都不处理，开动时不占 CPU。 */
static void LidarPause(void)
{
    SerPort_EnableRx(SER_LIDAR, 0U);
    comp.lidar_state = COMP_LIDAR_PAUSED;
}

/* 全部扫完：关掉雷达，之后 LidarTick 什么都不做，直到下一次 Comp_Start 或 /scan。 */
static void LidarOff(CompLidarState result)
{
    SerPort_Write(SER_LIDAR, RPLIDAR_CMD_STOP, 2);
    SerPort_EnableRx(SER_LIDAR, 0U);
    comp.lidar_state = result;
    scan_active = 0U;
}

/* 在车现在的位置扫一次，雷达没开就先开。 */
static void ScanBegin(uint32_t now)
{
    scan_active = 1U;
    scan_start_ms = hz_ms = now;
    scan_base = lidar_obs.scans;
    scan_from = (MapPoint){(int32_t)lroundf(comp.x), (int32_t)lroundf(comp.y)};
    hz_scans = parser.scans;
    if (comp.lidar_state == COMP_LIDAR_PAUSED)
    {
        SerPort_EnableRx(SER_LIDAR, 1U);
        comp.lidar_state = COMP_LIDAR_RUN;
        lidar_last_sample_ms = lidar_cmd_ms = now;
    }
    else if (comp.lidar_state >= COMP_LIDAR_DONE)
    {
        SerPort_EnableRx(SER_LIDAR, 1U);
        LidarStop(now);
    }
}

/* 这次扫描结束：够圈数的写进地图，新看到的输出一次。 */
static void ScanFinish(void)
{
    MapPoint p;
    comp.obstacles = (uint8_t)LidarObs_Finish();
    while (LidarObs_PopNew(&p))
        ChassisTelemetry_Printf("COMP obstacle #%u at (%ld,%ld)\r\n", ++obstacles_printed,
                                (long)p.x, (long)p.y);
    obstacles_dirty = 1;
    scan_active = 0U;
    LidarPause();
}

/* 全图扫完，输出结论，关雷达。 */
static void ScanConclude(void)
{
    ChassisTelemetry_Printf("COMP scan done: %u obstacles%s\r\n", (unsigned)comp.obstacles,
                            lidar_obs.map_full ? " MAP FULL" : "");
    LidarOff(COMP_LIDAR_DONE);
}

static void LidarTick(uint32_t now)
{
    if (comp.lidar_state >= COMP_LIDAR_PAUSED)
        return;
    if (scan_active && now - scan_start_ms > COMP_SCAN_TIMEOUT_MS)
    {
        ChassisTelemetry_Printf("COMP scan failed: no lidar data\r\n");
        LidarOff(COMP_LIDAR_FAILED);
        return;
    }
    if (comp.lidar_state == COMP_LIDAR_BOOT)
    {
        if (now - boot_ms >= COMP_LIDAR_BOOT_MS)
            LidarStop(now);
        return;
    }
    if (comp.lidar_state == COMP_LIDAR_STOPPING)
    {
        if (now - lidar_cmd_ms >= COMP_LIDAR_STOP_MS)
            LidarScan(now);
        return;
    }
    uint8_t buf[256];
    unsigned n;
    LidarPose pose = {comp.x, comp.y, comp.heading_deg};
    while ((n = SerPort_Read(SER_LIDAR, buf, sizeof(buf))) > 0U)
        for (unsigned i = 0; i < n; ++i)
        {
            RPLidarSample s;
            if (!RPLidar_Feed(&parser, buf[i], &s))
                continue;
            comp.lidar_state = COMP_LIDAR_RUN;
            lidar_last_sample_ms = now;
            if (scan_active)
                LidarObs_Feed(&s, &pose);
        }
    /* 开始出点了，还不用扫描就先歇着。 */
    if (!scan_active && comp.lidar_state == COMP_LIDAR_RUN)
    {
        LidarPause();
        return;
    }
    /* 第一个圈起点之前的半圈不算，所以要多一圈。 */
    if (scan_active && lidar_obs.scans - scan_base > COMP_SCAN_REVS)
    {
        ScanFinish();
        if (!Comp_Running())
            ScanConclude();
        return;
    }
    /* 没应答或断流：重新走 STOP → SCAN。 */
    if (now - lidar_last_sample_ms > COMP_LIDAR_SILENT_MS &&
        now - lidar_cmd_ms > COMP_LIDAR_SILENT_MS)
    {
        ++comp.lidar_restarts;
        LidarStop(now);
        return;
    }
    if (now - hz_ms >= 1000U)
    {
        comp.lidar_hz = (float)(parser.scans - hz_scans) * 1000.0f / (float)(now - hz_ms);
        hz_scans = parser.scans;
        hz_ms = now;
    }
}

static void ScanClear(void)
{
    if (comp.obstacles)
        Map_Init();
    LidarObs_Init();
    comp.obstacles = 0;
    obstacles_printed = 0U;
    obstacles_dirty = 1;
}

HAL_StatusTypeDef Comp_Rescan(void)
{
    if (!comp.anchored)
        return HAL_BUSY;
    ScanClear();
    ScanBegin(HAL_GetTick());
    return HAL_OK;
}

static void Send(const uint8_t *frame, unsigned len)
{
    if (len)
        SerPort_Write(SER_VISION, frame, len);
}

void Comp_SetVisionMode(VisionMode mode, uint8_t color)
{
    uint8_t f[VL_FRAME_MAX];
    comp.vision_mode = mode;
    comp.vision_color = color;
    Send(f, VisionLink_EncodeMode(mode, color, f));
    mode_ms = HAL_GetTick();
}

void Comp_TaskReport(void)
{
    if (!comp.task_valid)
    {
        ChassisTelemetry_Printf("COMP task none\r\n");
        return;
    }
    const TaskCode *code = &comp.task;
    ChassisTelemetry_Printf("COMP task %s from %s, batch 2 storage rings %u %u %u\r\n", code->text,
                            task_source, (unsigned)code->storage_ring[1][0],
                            (unsigned)code->storage_ring[1][1], (unsigned)code->storage_ring[1][2]);
}

/* 扫码模块和相机读到的文本都走这里。合法的码输出一次就定下来，之后不再读码；
   不合法的打印原文，现场能看到读到了什么。 */
static void AcceptTask(const char *text, unsigned len, const char *source)
{
    static const char *const reason[] = {"", "format", "color", "ring", "batch"};
    TaskCode code;
    TaskCodeResult result = TaskCode_Parse(text, len, &code);
    if (result != TASK_CODE_OK)
    {
        ++comp.task_rejected;
        if (strcmp(text, last_rejected) != 0) // 连续读码时同样的内容只打印一次
        {
            memcpy(last_rejected, text, len + 1U);
            ChassisTelemetry_Printf("COMP task rejected (%s) from %s: %s\r\n", reason[result],
                                    source, text);
        }
        return;
    }
    if (comp.task_valid && strcmp(code.text, comp.task.text) == 0)
        return;
    comp.task = code;
    comp.task_valid = 1;
    task_source = source;
    Comp_TaskReport();
    SerPort_EnableRx(SER_SCANNER, 0U);
    if (comp.vision_mode == VISION_QR)
        Comp_SetVisionMode(VISION_OFF, COLOR_ANY);
}

void Comp_InjectTask(const char *text, unsigned len)
{
    AcceptTask(text, len, "console");
}

void Comp_ClearTask(void)
{
    comp.task_valid = 0;
    last_rejected[0] = '\0';
    scan_line = (TaskLine){0};
    SerPort_EnableRx(SER_SCANNER, 1U);
}

HAL_StatusTypeDef Comp_SetPose(float x_mm, float y_mm, float heading_deg)
{
    if (chassis.state != CHASSIS_STATE_READY)
        return HAL_BUSY;
    home = (MapPoint){(int32_t)lroundf(x_mm), (int32_t)lroundf(y_mm)};
    Anchor(heading_deg);
    return HAL_OK;
}

static void ScannerTick(uint32_t now)
{
    uint8_t buf[64];
    unsigned n;
    while (!comp.task_valid && (n = SerPort_Read(SER_SCANNER, buf, sizeof(buf))) > 0U)
        for (unsigned i = 0; i < n && !comp.task_valid; ++i)
            if (TaskLine_Feed(&scan_line, buf[i], now))
            {
                AcceptTask(scan_line.text, scan_line.len, "scanner");
                scan_line.len = 0;
            }
    if (!comp.task_valid && TaskLine_Idle(&scan_line, now, COMP_SCANNER_IDLE_MS))
    {
        AcceptTask(scan_line.text, scan_line.len, "scanner");
        scan_line.len = 0;
    }
}

static void VisionTick(uint32_t now)
{
    uint8_t buf[128], f[VL_FRAME_MAX];
    unsigned n;
    while ((n = SerPort_Read(SER_VISION, buf, sizeof(buf))) > 0U)
        for (unsigned i = 0; i < n; ++i)
            (void)VisionLink_Feed(buf[i], now);
    if (vision.task_ms != camera_task_ms)
    {
        camera_task_ms = vision.task_ms;
        if (!comp.task_valid)
            AcceptTask(vision.task_text, vision.task_len, "camera");
    }
    uint8_t online = VisionLink_Online(now, COMP_VISION_TIMEOUT_MS);
    if (online != comp.vision_online)
    {
        comp.vision_online = online;
        ChassisTelemetry_Printf("COMP vision %s\r\n", online ? "ONLINE" : "OFFLINE");
        obstacles_dirty = 1; // 上位机（重新）上线时补发一次障碍物
    }
    if (now - pose_ms >= COMP_POSE_PERIOD_MS)
    {
        pose_ms = now;
        Send(f, VisionLink_EncodePose(comp.x, comp.y, comp.heading_deg, comp.anchored, f));
    }
    if (now - mode_ms >= COMP_MODE_PERIOD_MS)
    {
        mode_ms = now;
        Send(f, VisionLink_EncodeMode(comp.vision_mode, comp.vision_color, f));
    }
    if (obstacles_dirty)
    {
        MapPoint list[16];
        int32_t xy[16][2];
        unsigned k = LidarObs_List(list, 16);
        for (unsigned i = 0; i < k; ++i)
        {
            xy[i][0] = list[i].x;
            xy[i][1] = list[i].y;
        }
        Send(f, VisionLink_EncodeObstacles((const int32_t(*)[2])xy, k, f));
        obstacles_dirty = 0;
    }
}

/* ---------- 路线 ---------- */
static MapPoint RoutePoint(unsigned index)
{
    return route[index];
}

static void RoutePrintState(void)
{
    static const char *const text[] = {"none", "planning", "ready", "following",
                                       "done", "stopped",  "failed"};
    if (route_why)
        ChassisTelemetry_Printf("ROUTE %s %s\r\n", text[comp.route_state], route_why);
    else
        ChassisTelemetry_Printf("ROUTE %s\r\n", text[comp.route_state]);
}

static void RouteSet(CompRouteState state, const char *why)
{
    comp.route_state = state;
    route_why = why;
    RoutePrintState();
}

/* 失败的路线不留着：调试台上看到的线都是还能走的或刚走过的。 */
static void RouteFail(const char *why)
{
    route_count = 0U;
    RouteSet(COMP_ROUTE_FAILED, why);
}

void Comp_RouteReport(void)
{
    RoutePrintState();
    if (route_count)
        ChassisTelemetry_SendRoute(route_count, RoutePoint);
}

static void RouteSearch(void)
{
    MapCell s, g;
    if (!Map_WorldToGrid(route[0].x, route[0].y, &s.row, &s.col) || !Map_CanStand(s.row, s.col))
        RouteFail("start");
    else if (!Map_WorldToGrid(route_goal.x, route_goal.y, &g.row, &g.col) ||
             !Map_CanStand(g.row, g.col))
        RouteFail("goal");
    else if (!AStar_Begin(s, g, 0, 0, MAP_ROWS - 1U, MAP_COLS - 1U))
        RouteFail("nopath");
    else
        RouteSet(COMP_ROUTE_PLANNING, NULL);
}

HAL_StatusTypeDef Comp_RoutePlan(int32_t x_mm, int32_t y_mm)
{
    if (chassis.state != CHASSIS_STATE_READY || !comp.anchored)
        return HAL_BUSY;
    route_count = 0U;
    route[0] = (MapPoint){(int32_t)lroundf(comp.x), (int32_t)lroundf(comp.y)};
    route_goal = (MapPoint){x_mm, y_mm};
    /* 地图按车身和场地轴平行算，270 见方的车转 90° 占地不变，其他角度不适用。 */
    route_heading = 90.0f * roundf(comp.heading_deg / 90.0f);
    if (fabsf(Math_WrapDeg(comp.heading_deg - route_heading)) > AGV_HEADING_FAULT_DEG)
        RouteFail("heading");
    else
        RouteSearch();
    return HAL_OK;
}

/* A* 结果只留拐点：同方向的连续栅格步合并。终点用目标的准确坐标，不用栅格中心。
   路线最长两千来格，只走一遍数组，百来微秒，不用分拍。 */
static void RouteBuild(void)
{
    uint16_t cells = AStar_PathLength();
    MapCell a, b;
    int last_dr = 0, last_dc = 0;
    route_count = 1U;
    (void)AStar_PathCell(0U, &a);
    for (uint16_t i = 1U; i < cells; ++i)
    {
        (void)AStar_PathCell(i, &b);
        int dr = (int)b.row - a.row, dc = (int)b.col - a.col;
        if (i > 1U && (dr != last_dr || dc != last_dc))
        {
            if (route_count + 1U >= COMP_ROUTE_MAX)
            {
                RouteFail("long");
                return;
            }
            (void)Map_GridToWorld(a.row, a.col, &route[route_count++]);
        }
        last_dr = dr;
        last_dc = dc;
        a = b;
    }
    route[route_count++] = route_goal;
    route_revision = Map_Revision();
    RouteSet(COMP_ROUTE_READY, NULL);
    ChassisTelemetry_SendRoute(route_count, RoutePoint);
}

/* 从 a 直走到 b 经过的格子都能站车吗。 */
static uint8_t RouteSegmentClear(MapPoint a, MapPoint b)
{
    int32_t dx = b.x - a.x, dy = b.y - a.y;
    int32_t adx = dx < 0 ? -dx : dx, ady = dy < 0 ? -dy : dy;
    int32_t steps = (adx > ady ? adx : ady) / MAP_CELL_MM + 1;
    for (int32_t i = 0; i <= steps; ++i)
    {
        uint16_t r, c;
        if (!Map_WorldToGrid(a.x + dx * i / steps, a.y + dy * i / steps, &r, &c) ||
            !Map_CanStand(r, c))
            return 0;
    }
    return 1;
}

/* 从车现在的位置到终点这一段还能走吗。身后走过的段不管。 */
static uint8_t RouteAheadClear(unsigned next)
{
    MapPoint here = {(int32_t)lroundf(comp.x), (int32_t)lroundf(comp.y)};
    for (unsigned i = next; i < route_count; ++i)
    {
        if (!RouteSegmentClear(here, route[i]))
            return 0;
        here = route[i];
    }
    return 1;
}

/* 走规划好路线的前 legs 段。 */
static HAL_StatusTypeDef RouteFollow(unsigned legs)
{
    if (comp.route_state != COMP_ROUTE_READY)
        return HAL_ERROR;
    if (chassis.state != CHASSIS_STATE_READY)
        return HAL_BUSY;
    if (hypotf(comp.x - (float)route[0].x, comp.y - (float)route[0].y) > COMP_ROUTE_START_TOL_MM ||
        fabsf(Math_WrapDeg(comp.heading_deg - route_heading)) > AGV_HEADING_FAULT_DEG)
    {
        RouteFail("moved");
        return HAL_OK;
    }
    if (!RouteAheadClear(1U))
    {
        RouteFail("blocked");
        return HAL_OK;
    }
    static chassis_waypoint waypoints[COMP_ROUTE_MAX];
    for (unsigned i = 0U; i < legs; ++i)
    {
        float dx = (float)(route[i + 1U].x - home.x), dy = (float)(route[i + 1U].y - home.y);
        waypoints[i] = (chassis_waypoint){origin_x + cs * dx - sn * dy, origin_y + sn * dx + cs * dy,
                                          origin_yaw + route_heading};
    }
    Chassis_SetMoveSpeedLimit(chassis_tune.cmd_speed);
    if (Chassis_FollowPath(waypoints, legs, 0U) != HAL_OK)
    {
        RouteFail("chassis");
        return HAL_OK;
    }
    route_revision = Map_Revision();
    RouteSet(COMP_ROUTE_FOLLOWING, NULL);
    return HAL_OK;
}

HAL_StatusTypeDef Comp_RouteGo(void)
{
    return RouteFollow(route_count - 1U);
}

static void RouteTick(void)
{
    if (comp.route_state == COMP_ROUTE_PLANNING)
    {
        AStar_Step(AGV_SEARCH_BUDGET);
        switch (AStar_Status())
        {
        case ASTAR_BUSY:
            break;
        case ASTAR_FOUND:
            RouteBuild();
            break;
        case ASTAR_INVALIDATED:
            RouteSearch(); // 搜索途中地图变了，按新地图重来
            break;
        default:
            RouteFail("nopath");
            break;
        }
    }
    else if (comp.route_state == COMP_ROUTE_FOLLOWING)
    {
        if (chassis.mode != CHASSIS_MODE_PATH)
            RouteSet(chassis_path_result.completed ? COMP_ROUTE_DONE : COMP_ROUTE_STOPPED, NULL);
        else if (chassis_path_result.max_cross_mm > AGV_CROSS_TRACK_MM)
        {
            Chassis_Stop();
            RouteSet(COMP_ROUTE_STOPPED, "cross");
        }
        else if (route_revision != Map_Revision())
        {
            route_revision = Map_Revision();
            if (!RouteAheadClear(Chassis_PathTarget()))
            {
                Chassis_Stop();
                RouteSet(COMP_ROUTE_STOPPED, "blocked");
            }
        }
    }
}

/* ---------- 开局流程 ---------- */
uint8_t Comp_Running(void)
{
    return comp.run_state >= COMP_RUN_SCAN && comp.run_state <= COMP_RUN_WAIT;
}

void Comp_RunReport(void)
{
    static const char *const text[] = {"idle", "scan", "plan", "move",
                                       "wait", "done", "stopped", "failed"};
    if (run_why)
        ChassisTelemetry_Printf("RUN %s %s\r\n", text[comp.run_state], run_why);
    else
        ChassisTelemetry_Printf("RUN %s\r\n", text[comp.run_state]);
}

static void RunSet(CompRunState state, const char *why)
{
    comp.run_state = state;
    run_why = why;
    Comp_RunReport();
}

/* 第 stop 个停车点。被障碍物占了就在附近找一个能站的位置，找不到返回 0。 */
static uint8_t RunStopPoint(unsigned stop, MapPoint *p)
{
    static const MapPoint stops[COMP_RUN_STOPS] = {{COMP_QR_STOP_X_MM, COMP_QR_STOP_Y_MM},
                                                   {COMP_CENTER_MM, COMP_CENTER_MM}};
    static const int16_t shift[] = {0, 60, -60, 120, -120, 180, -180};
    for (int32_t dx = 0; dx <= 50; dx += 50)
        for (unsigned i = 0; i < sizeof(shift) / sizeof(shift[0]); ++i)
        {
            uint16_t r, c;
            *p = stops[stop];
            p->x += dx;
            p->y += shift[i];
            if (Map_WorldToGrid(p->x, p->y, &r, &c) && Map_CanStand(r, c))
                return 1;
        }
    return 0;
}

static void RunScan(uint32_t now)
{
    /* 雷达已经判定没数据就不再试：每次都要等到超时，比赛时间耗不起。 */
    if (comp.lidar_state == COMP_LIDAR_FAILED)
        return;
    ScanBegin(now);
    ++run_scans;
    RunSet(COMP_RUN_SCAN, NULL);
}

/* 一段路走完或一次扫描结束后决定下一步。 */
static void RunNext(uint32_t now)
{
    /* 到停车点了：二维码板前要等到任务码才能走。 */
    if (run_stop < COMP_RUN_STOPS &&
        hypotf(comp.x - (float)run_goal.x, comp.y - (float)run_goal.y) <= COMP_ROUTE_START_TOL_MM)
    {
        if (run_stop == 0U && !comp.task_valid)
        {
            if (comp.run_state != COMP_RUN_WAIT)
            {
                run_since = now;
                Comp_SetVisionMode(VISION_QR, COLOR_ANY); // 扫码模块没读到，相机也帮着找
                RunSet(COMP_RUN_WAIT, NULL);
            }
            return;
        }
        ++run_stop;
    }
    /* 场地中央只是为了换个视角：去不了，或者半路上已经够三个位置了，就不用再去。
       二维码板前去不了才算失败。 */
    while (run_stop < COMP_RUN_STOPS && !RunStopPoint(run_stop, &run_goal))
    {
        if (run_stop == 0U)
        {
            RunSet(COMP_RUN_FAILED, "route");
            return;
        }
        ++run_stop;
    }
    if (run_stop >= COMP_RUN_STOPS ||
        (run_stop > 0U && (run_scans >= COMP_SCAN_PLACES || comp.lidar_state == COMP_LIDAR_FAILED)))
    {
        if (comp.lidar_state != COMP_LIDAR_FAILED)
            ScanConclude();
        /* 去场地中央的路线可能没走完，不留着，免得看着像还要走。 */
        route_count = 0U;
        RouteSet(COMP_ROUTE_NONE, NULL);
        RunSet(COMP_RUN_DONE, NULL);
        return;
    }
    RunSet(COMP_RUN_PLAN, NULL);
    (void)Comp_RoutePlan(run_goal.x, run_goal.y);
}

HAL_StatusTypeDef Comp_Start(void)
{
    if (chassis.state != CHASSIS_STATE_READY || !comp.anchored)
        return HAL_BUSY;
    uint32_t now = HAL_GetTick();
    ScanClear();
    route_count = 0U;
    comp.route_state = COMP_ROUTE_NONE;
    route_why = NULL;
    run_stop = 0U;
    run_scans = 1U;
    /* 还没有停车点时，让“到了吗”的判断不成立。 */
    run_goal = (MapPoint){-1000, -1000};
    ScanBegin(now);
    RunSet(COMP_RUN_SCAN, NULL);
    return HAL_OK;
}

void Comp_Stop(void)
{
    if (Comp_Running())
        RunSet(COMP_RUN_STOPPED, NULL);
    if (scan_active)
    {
        scan_active = 0U;
        if (comp.lidar_state == COMP_LIDAR_RUN)
            LidarPause();
    }
}

static void RunTick(uint32_t now)
{
    /* 底盘故障：别等了，流程到此为止。清完故障要重新开始。 */
    if (Comp_Running() && chassis.state != CHASSIS_STATE_READY)
    {
        Comp_Stop();
        RunSet(COMP_RUN_FAILED, "chassis");
        return;
    }
    switch (comp.run_state)
    {
    case COMP_RUN_SCAN:
        if (!scan_active)
            RunNext(now);
        break;
    case COMP_RUN_WAIT:
        if (comp.task_valid)
            RunNext(now);
        else if (now - run_since > COMP_QR_TIMEOUT_MS)
            RunSet(COMP_RUN_FAILED, "task");
        break;
    case COMP_RUN_PLAN:
        if (comp.route_state == COMP_ROUTE_READY)
        {
            (void)RouteFollow(1U); // 只走第一段：这一段从刚扫描的位置出发，不会撞上看不见的
            if (comp.route_state == COMP_ROUTE_FOLLOWING)
                RunSet(COMP_RUN_MOVE, NULL);
        }
        if (comp.route_state == COMP_ROUTE_FAILED)
        {
            if (run_stop == 0U)
                RunSet(COMP_RUN_FAILED, "route");
            else
            {
                ++run_stop;
                RunNext(now);
            }
        }
        break;
    case COMP_RUN_MOVE:
        if (comp.route_state == COMP_ROUTE_DONE)
        {
            if (hypotf(comp.x - (float)scan_from.x, comp.y - (float)scan_from.y) >=
                COMP_SCAN_MIN_MOVE_MM)
                RunScan(now);
            if (comp.run_state == COMP_RUN_MOVE)
                RunNext(now);
        }
        else if (comp.route_state == COMP_ROUTE_STOPPED)
            RunSet(COMP_RUN_STOPPED, NULL);
        break;
    default:
        break;
    }
}

/* 周期文本只报还在进行的事：雷达扫完、任务码定下来之后，这两项就不再报了。D0 全部关掉。 */
static void Report(uint32_t now)
{
    if (now - report_ms < COMP_REPORT_PERIOD_MS)
        return;
    report_ms = now;
    if (!ChassisTelemetry_Periodic())
        return;
    if (comp.lidar_state < COMP_LIDAR_PAUSED)
    {
        static const char *const lidar_text[] = {"BOOT", "STOP", "START", "RUN"};
        ChassisTelemetry_Printf("COMP lidar=%s %sHz revs=%lu/%u pts=%lu used=%lu clu=%lu rs=%lu/%lu\r\n",
                                lidar_text[comp.lidar_state], ChassisTelemetry_Num(comp.lidar_hz, 1),
                                (unsigned long)(lidar_obs.scans - scan_base), COMP_SCAN_REVS,
                                (unsigned long)lidar_obs.points, (unsigned long)lidar_obs.used,
                                (unsigned long)lidar_obs.clusters,
                                (unsigned long)comp.lidar_restarts, (unsigned long)parser.resync);
    }
    if (!comp.task_valid)
    {
        const SerPortStats *scan = SerPort_Stats(SER_SCANNER);
        ChassisTelemetry_Printf("COMP task=none rej=%lu scan=%lu/%lu\r\n",
                                (unsigned long)comp.task_rejected, (unsigned long)scan->rx_bytes,
                                (unsigned long)scan->hw_errors);
    }
    ChassisTelemetry_Printf("COMP vision=%s mode=%u mat=%u ring=%u%s crc=%lu\r\n",
                            comp.vision_online ? "ON" : "OFF", (unsigned)comp.vision_mode,
                            (unsigned)vision.material_count, (unsigned)vision.ring_found,
                            vision.ring_aligned ? "(aligned)" : "",
                            (unsigned long)vision.crc_errors);
}

void Comp_Tick(void)
{
    uint32_t now = HAL_GetTick();
    UpdatePose(now);
    LidarTick(now);
    VisionTick(now);
    ScannerTick(now);
    RouteTick();
    RunTick(now);
    Report(now);
}
#endif
