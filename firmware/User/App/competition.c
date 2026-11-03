#include "competition.h"
#if AGV_MODE == AGV_MODE_COMPETITION
#include "astar.h"
#include "chassis.h"
#include "chassis_telemetry.h"
#include "comp_config.h"
#include "map.h"
#include "rplidar.h"
#include "ser_port.h"
#include "servo.h"
#include "user_math.h"
#include <math.h>
#include <string.h>

/* 开局要去的地方，按顺序。 */
enum
{
    RUN_STOP_BOARD,  // 二维码板前：沿板找码
    RUN_STOP_CENTRE, // 场地中央：换个视角再扫一次
    COMP_RUN_STOPS
};

CompStatus comp;

static MapPoint home;
static float home_heading; // 开局时车头的场地航向
static float origin_x, origin_y, origin_yaw, cs, sn;
static uint32_t boot_ms, lidar_cmd_ms, lidar_last_sample_ms, hz_ms, hz_scans;
static uint8_t scan_active;
static uint32_t scan_start_ms, scan_base;
static unsigned start_zone; // 当前启停区，用于选择扫码起点
static MapPoint scan_from; // 上次扫描的位置
static float servo_deg;    // 雷达舵机最近一次转到的角度
static uint32_t servo_ms;  // 从什么时候开始转的
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
static MapPoint scan_places[COMP_SCAN_PLACES];
static unsigned scan_place_count;
static uint32_t scan_total_start, scan_stable_since;
static unsigned run_scans; // 这次流程在几个位置扫过了
static MapPoint run_goal;
static const MapPoint no_stop = {-1000, -1000}; // 还没有停车点时，让“到了吗”的判断不成立
static uint8_t run_to_row; // 刚离开二维码板：先沿着板子走回十字路口那一行
static float run_heading;  // 原地转向的目标航向
static uint8_t run_turning; // 0：等雷达收回来，1：正在转
static uint8_t scan_resume; // 为原地转停了扫描，转完接着扫
static uint32_t run_since;
static const char *run_why;
static const char route_blocked[] = "blocked"; // 走着时新看到的障碍物挡了路：重新规划

/* 雷达舵机跟着扫描走：扫描时伸出去，平时收着；/set 改了角度也马上转过去，台架上能直接看位置对不对。
   每拍在 LidarTick 之后、RunTick 之前调：扫完的那一拍就开始收，流程等收好了才走。 */
static void ServoTick(uint32_t now)
{
    float deg = scan_active ? lidar_tune.servo_scan : lidar_tune.servo_stow;
    if (deg == servo_deg)
        return;
    servo_deg = deg;
    servo_ms = now;
    Servo_SetDeg(deg);
}

/* 舵机停在 deg 了吗。没有位置反馈：转一次给它 COMP_LIDAR_SERVO_MS。 */
static uint8_t ServoAt(float deg, uint32_t now)
{
    return servo_deg == deg && now - servo_ms >= COMP_LIDAR_SERVO_MS;
}

/* 雷达没确定收好（伸着或者正在转），它就在车身左后角外面：按最近的直角航向（路线上车头保持直角）
   换算成场地偏移，交给地图一起算能不能站、怎么走。原地转之前已经收回来了（RunTurn）。 */
static void FootprintTick(uint32_t now)
{
    if (ServoAt(lidar_tune.servo_stow, now))
    {
        Map_SetAttachment(0.0f, 0.0f, 0.0f);
        return;
    }
    float h = 90.0f * roundf(comp.heading_deg / 90.0f) * MATH_DEG_TO_RAD, c = cosf(h), s = sinf(h);
    Map_SetAttachment(c * lidar_tune.lidar_x - s * lidar_tune.lidar_y,
                      s * lidar_tune.lidar_x + c * lidar_tune.lidar_y,
                      COMP_LIDAR_RADIUS_MM + MAP_MARGIN_MM);
}

HAL_StatusTypeDef Comp_Init(unsigned zone)
{
    comp = (CompStatus){0};
    Map_Init();
    if (!Map_StartPoint((uint8_t)zone, &home))
        return HAL_ERROR;
    start_zone = zone;
    home_heading = COMP_START_HEADING_DEG(zone);
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
    Servo_Init();
    servo_deg = -1.0f; // 上电时舵机在哪不知道：第一拍就收起来
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

/* 底盘 READY 后记下里程计原点：这时车在启停区中心、车头朝 COMP_START_HEADING_DEG(zone)（左后角的
   雷达朝场地里面），之后换算到场地坐标。 */
static void UpdatePose(uint32_t now)
{
    if (!comp.anchored)
    {
        if (chassis.state != CHASSIS_STATE_READY || now - boot_ms < AGV_START_DELAY_MS)
            return;
        Anchor(home_heading);
        ChassisTelemetry_Printf("COMP pose anchored at (%ld,%ld) heading %d\r\n", (long)home.x,
                                (long)home.y, (int)home_heading);
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

/* 在车现在的位置扫一次：舵机把雷达伸出去，到位了才开始用点（LidarTick）。雷达没开就先开。 */
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

/* 这次扫描结束：够圈数的写进地图，新看到的输出一次，雷达收回来。 */
static void ScanFinish(void)
{
    MapPoint p;
    unsigned previous = comp.obstacles;
    comp.obstacles = (uint8_t)LidarObs_Finish();
    if (comp.obstacles != previous || LidarObs_HasPending())
        scan_stable_since = lidar_obs.scans;
    while (LidarObs_PopNew(&p))
        ChassisTelemetry_Printf("COMP obstacle #%u at (%ld,%ld)\r\n", ++obstacles_printed,
                                (long)p.x, (long)p.y);
    obstacles_dirty = 1;
    if (Comp_Running())
    {
        /* 开局流程里边走边扫：这一轮记进地图，接着扫下一轮。 */
        scan_base = lidar_obs.scans;
        scan_start_ms = HAL_GetTick();
    }
    else
    {
        scan_active = 0U;
        LidarPause();
    }
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
    if (scan_active && Comp_Running() && now - scan_total_start >= COMP_SCAN_TOTAL_MS)
    {
        ScanFinish();
        ChassisTelemetry_Printf("COMP scan end: time limit\r\n");
        ScanConclude();
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
            /* 舵机还在转：雷达不在 lidar_x、lidar_y 那儿，换算出来的点全是错的。 */
            if (scan_active && ServoAt(lidar_tune.servo_scan, now))
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
        if (Comp_Running())
        {
            uint8_t distinct = 1U;
            for (unsigned i = 0; i < scan_place_count; ++i)
                if (hypotf(comp.x - scan_places[i].x, comp.y - scan_places[i].y) < COMP_SCAN_MIN_MOVE_MM)
                    distinct = 0U;
            if (distinct && scan_place_count < COMP_SCAN_PLACES)
                scan_places[scan_place_count++] = (MapPoint){(int32_t)lroundf(comp.x), (int32_t)lroundf(comp.y)};
            run_scans = scan_place_count;
            uint8_t stable = scan_place_count >= COMP_SCAN_STABLE_PLACES &&
                !LidarObs_HasPending() && !lidar_obs.map_full &&
                lidar_obs.scans - scan_stable_since >= COMP_SCAN_STABLE_REVS;
            if (stable || scan_place_count >= COMP_SCAN_PLACES)
            {
                ChassisTelemetry_Printf("COMP scan end: %s, places=%u\r\n",
                    stable ? "stable observations" : "place limit", scan_place_count);
                ScanConclude();
            }
        }
        else
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

/* 走规划好路线的前 legs 段，限速 speed。 */
static HAL_StatusTypeDef RouteFollow(unsigned legs, float speed)
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
        RouteFail(route_blocked);
        return HAL_OK;
    }
    static chassis_waypoint waypoints[COMP_ROUTE_MAX];
    for (unsigned i = 0U; i < legs; ++i)
    {
        float dx = (float)(route[i + 1U].x - home.x), dy = (float)(route[i + 1U].y - home.y);
        waypoints[i] = (chassis_waypoint){origin_x + cs * dx - sn * dy, origin_y + sn * dx + cs * dy,
                                          origin_yaw + route_heading};
    }
    Chassis_SetMoveSpeedLimit(speed);
    if (Chassis_FollowPath(waypoints, legs, 0U) != HAL_OK)
    {
        RouteFail("chassis");
        return HAL_OK;
    }
    route_revision = Map_Revision();
    RouteSet(COMP_ROUTE_FOLLOWING, NULL);
    return HAL_OK;
}

/* 从车现在的位置直走到 to，不经过 A*：两点路线，照样显示在调试台上。 */
static void RouteStraight(MapPoint to, float speed)
{
    route[0] = (MapPoint){(int32_t)lroundf(comp.x), (int32_t)lroundf(comp.y)};
    route[1] = route_goal = to;
    route_count = 2U;
    route_heading = 90.0f * roundf(comp.heading_deg / 90.0f);
    RouteSet(COMP_ROUTE_READY, NULL);
    ChassisTelemetry_SendRoute(route_count, RoutePoint);
    (void)RouteFollow(1U, speed);
}

HAL_StatusTypeDef Comp_RouteGo(void)
{
    return RouteFollow(route_count - 1U, chassis_tune.cmd_speed);
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
                RouteSet(COMP_ROUTE_STOPPED, route_blocked);
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
    static const char *const text[] = {"idle", "scan", "plan", "move", "turn",
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

/* (x, y) 能停车吗；turn 时还要能原地转：车转到 45° 时的外接方框也放得下，转的时候车角扫过的圆
   就在这个方框里。 */
static uint8_t RunStandAt(int32_t x, int32_t y, uint8_t turn, MapPoint *p)
{
    uint16_t r, c;
    *p = (MapPoint){x, y};
    if (turn)
        return Map_CanPoseHeading((float)x, (float)y, 45.0f, MAP_MARGIN_MM);
    return Map_WorldToGrid(x, y, &r, &c) && Map_CanStand(r, c);
}

/* 扫码模块还没对着二维码板那面墙（启停区 2 出发），到了板前要原地转。 */
static uint8_t RunNeedsTurn(void)
{
    return fabsf(Math_WrapDeg(comp.heading_deg - COMP_QR_HEADING_DEG)) > AGV_HEADING_FAULT_DEG;
}

/* 第 stop 个停车点，找不到返回 0。
   - 二维码板前：沿板找码的起点，启停区 1 从上面来是 1300、启停区 2 从下面来是 1100，到了还要原地转的话
     得留够转的地方。被障碍物占了，先在找码范围里往另一头找，再往范围外两头找，都不行就离板子远 50 mm
     再找；
   - 场地中央：被障碍物占了就在附近找一个能站的位置。 */
static uint8_t RunStopPoint(unsigned stop, MapPoint *p)
{
    static const int16_t shift[] = {0, 60, -60, 120, -120, 180, -180};
    uint8_t turn = stop == RUN_STOP_BOARD && RunNeedsTurn();
    for (int32_t dx = 0; dx <= 50; dx += 50)
    {
        if (stop == RUN_STOP_BOARD)
        {
            const int32_t x = COMP_QR_STOP_X_MM - dx;
            /* 区 1 从 1300 向下找；区 2 保持从 1100 向上找。 */
            for (int32_t offset = 0; offset <= COMP_QR_Y_MAX_MM - COMP_QR_Y_MIN_MM; offset += MAP_CELL_MM)
                if (RunStandAt(x, start_zone == 1U ? COMP_QR_Y_MAX_MM - offset : COMP_QR_Y_MIN_MM + offset, turn, p))
                    return 1;
            for (int32_t d = MAP_CELL_MM; d <= 180; d += MAP_CELL_MM)
                if (RunStandAt(x, COMP_QR_Y_MIN_MM - d, turn, p) ||
                    RunStandAt(x, COMP_QR_Y_MAX_MM + d, turn, p))
                    return 1;
        }
        else
            for (unsigned i = 0; i < sizeof(shift) / sizeof(shift[0]); ++i)
                if (RunStandAt(COMP_CENTER_MM + dx, COMP_CENTER_MM + shift[i], 0U, p))
                    return 1;
    }
    return 0;
}

static void RunScan(uint32_t now)
{
    /* 雷达已经判定没数据就不再试：每次都要等到超时，比赛时间耗不起。 */
    if (comp.lidar_state == COMP_LIDAR_FAILED || comp.lidar_state == COMP_LIDAR_DONE)
        return;
    ScanBegin(now);
    ++run_scans;
}

/* 正在走的这一段不走了，路线也不留着，免得调试台上看着像还要走。 */
static void RunHalt(void)
{
    if (comp.route_state == COMP_ROUTE_FOLLOWING)
        Chassis_Stop();
    route_count = 0U;
    if (comp.route_state != COMP_ROUTE_NONE) // 已经没有路线了不再说一遍
        RouteSet(COMP_ROUTE_NONE, NULL);
}

/* 沿二维码板慢慢走向 y，被障碍物挡住就走到能站车的最远处。一格都走不了返回 0。 */
static uint8_t RunSweepTo(int32_t y)
{
    MapPoint here = {(int32_t)lroundf(comp.x), (int32_t)lroundf(comp.y)}, end = here, next;
    while (end.y != y)
    {
        int32_t step = y - end.y; // 一次最多一格
        if (step > MAP_CELL_MM)
            step = MAP_CELL_MM;
        else if (step < -MAP_CELL_MM)
            step = -MAP_CELL_MM;
        if (!RunStandAt(end.x, end.y + step, 0U, &next))
            break;
        end = next;
    }
    int32_t moved = end.y - here.y;
    if (moved < MAP_CELL_MM && moved > -MAP_CELL_MM)
        return 0U;
    RouteStraight(end, COMP_QR_SWEEP_MM_S);
    return 1U;
}

/* 沿二维码板来回找码：在下半段就往上走，否则往下走；这个方向走不了就换个方向。 */
static void RunSweep(void)
{
    int32_t to = comp.y < 0.5f * (COMP_QR_Y_MIN_MM + COMP_QR_Y_MAX_MM) ? COMP_QR_Y_MAX_MM
                                                                       : COMP_QR_Y_MIN_MM;
    if (!RunSweepTo(to))
        (void)RunSweepTo(COMP_QR_Y_MIN_MM + COMP_QR_Y_MAX_MM - to);
}

/* 原地转到场地航向 heading。停车点按收着雷达能原地转挑过（RunStopPoint）：雷达伸着转，扫过的圆从
   车角的 191 mm 变成 247 mm，板前就会碰到板子。所以先停下扫描，雷达收好了才转（RunTick），转完接着扫。 */
static void RunTurn(float heading)
{
    run_heading = heading;
    run_turning = 0U;
    if (scan_active)
    {
        scan_active = 0U; // LidarTick 接着把雷达接收停了
        scan_resume = 1U;
    }
    RunSet(COMP_RUN_TURN, NULL);
}

/* 沿车体一个轴走了 along，雷达在这个轴上的位置是 mount：雷达伸出车身 half 以外时，要往雷达那边
   （或偏得不多，tol 是负的余量）走，车身扫过的地方才不被车身挡着。 */
static uint8_t RunTowardLidar(float along, float mount, float half, float tol)
{
    return fabsf(mount) <= half || (mount < 0.0f ? -along : along) >= tol;
}

/* 从 a 直走到 b 这段雷达看清了吗。雷达伸在车角外，车身挡住的是朝车身那 90°：往雷达那两边走
   （左后角就是往左、往后），车身扫过的地方都看得到；往车头或右边走，扫过的地方被车身挡着。
   偏得不多（COMP_SEEN_TOL_DEG）不算。雷达装在车身中间、一圈都看得到时都算看清了；雷达坏了都看不清。 */
static uint8_t RunLegSeen(MapPoint a, MapPoint b)
{
    if (comp.lidar_state == COMP_LIDAR_FAILED)
        return 0U;
    float h = comp.heading_deg * MATH_DEG_TO_RAD, dx = (float)(b.x - a.x), dy = (float)(b.y - a.y);
    float fx = cosf(h) * dx + sinf(h) * dy, fy = cosf(h) * dy - sinf(h) * dx; // 车体系
    float tol = -hypotf(dx, dy) * sinf(COMP_SEEN_TOL_DEG * MATH_DEG_TO_RAD);
    return RunTowardLidar(fx, lidar_tune.lidar_x, ROBOT_LENGTH_MM * 0.5f, tol) &&
           RunTowardLidar(fy, lidar_tune.lidar_y, ROBOT_WIDTH_MM * 0.5f, tol);
}

/* 这个停车点的事做完了，去下一个。离开二维码板时先沿着板子走回十字路口那一行。 */
static void RunLeaveStop(void)
{
    run_to_row = run_stop == RUN_STOP_BOARD;
    ++run_stop;
}

/* 一段路走完或一次扫描结束后决定下一步。 */
static void RunNext(uint32_t now)
{
    /* 到停车点了：二维码板前没读到任务码，就沿着板子慢慢走着找，读到才能走。扫码模块还没对着板子
       （启停区 2 车头朝 +X 出发）就先原地转过来，转之前收雷达、转完接着扫（RunTurn）。走过来时看到了
       挡着转的东西，就在下面换一个能转的停车点。按停车点查，不按车现在的位置：车停得差一两毫米会落到
       隔壁一格，查出来不一样，就会在原地来回规划。 */
    if (run_stop < COMP_RUN_STOPS &&
        hypotf(comp.x - (float)run_goal.x, comp.y - (float)run_goal.y) <= COMP_ROUTE_START_TOL_MM)
    {
        MapPoint p;
        if (run_stop != RUN_STOP_BOARD || comp.task_valid)
            RunLeaveStop();
        else if (!RunNeedsTurn())
        {
            run_since = now;
            Comp_SetVisionMode(VISION_QR, COLOR_ANY); // 扫码模块没读到，相机也帮着找
            RunSet(COMP_RUN_WAIT, NULL);
            RunSweep();
            return;
        }
        else if (RunStandAt(run_goal.x, run_goal.y, 1U, &p))
        {
            RunTurn(COMP_QR_HEADING_DEG);
            return;
        }
    }
    /* 场地中央只是为了换个视角接着扫：去不了，或者扫描已经结束（够三个位置又没有新目标），就不用再去。
       二维码板前去不了才算失败。 */
    while (run_stop < COMP_RUN_STOPS && !RunStopPoint(run_stop, &run_goal))
    {
        if (run_stop == RUN_STOP_BOARD)
        {
            RunSet(COMP_RUN_FAILED, "route");
            return;
        }
        ++run_stop;
    }
    if (run_stop >= COMP_RUN_STOPS ||
        (run_stop > RUN_STOP_BOARD &&
         (comp.lidar_state == COMP_LIDAR_DONE || comp.lidar_state == COMP_LIDAR_FAILED)))
    {
        /* 还在扫：这一轮已经确认的也记下再关雷达。已经扫完的不再输出一次结论。 */
        if (scan_active)
            ScanFinish();
        if (comp.lidar_state < COMP_LIDAR_DONE)
            ScanConclude();
        RunHalt(); // 去场地中央的路线可能没走完
        RunSet(COMP_RUN_DONE, NULL);
        return;
    }
    /* 刚离开二维码板去场地中央：读到码的地方离十字路口那一行远（中间车道只有这一行宽），先沿着板子
       走回这一行，再往左平移过去。这一段在板前找码的范围里，和找码一样不查看没看清。 */
    if (run_to_row)
    {
        MapPoint here = {(int32_t)lroundf(comp.x), (int32_t)lroundf(comp.y)};
        MapPoint row = {here.x, COMP_CENTER_MM};
        run_to_row = 0U;
        if (fabsf(comp.y - (float)COMP_CENTER_MM) >= (float)MAP_CELL_MM && RouteSegmentClear(here, row))
        {
            RouteStraight(row, chassis_tune.cmd_speed);
            if (comp.route_state == COMP_ROUTE_FOLLOWING)
            {
                RunSet(COMP_RUN_MOVE, NULL);
                return;
            }
        }
    }
    RunSet(COMP_RUN_PLAN, NULL);
    (void)Comp_RoutePlan(run_goal.x, run_goal.y);
}

/* 离上次扫描的位置够远，视角变了，先扫一次再决定下一步。 */
static void RunScanOrNext(uint32_t now)
{
    if (!scan_active && hypotf(comp.x - (float)scan_from.x, comp.y - (float)scan_from.y) >= COMP_SCAN_MIN_MOVE_MM)
        RunScan(now);
    if (comp.run_state != COMP_RUN_SCAN)
        RunNext(now);
}

/* 沿板找码时读到了：不用走完，停下去下一个停车点。 */
static void RunLeaveBoard(uint32_t now)
{
    RunHalt();
    RunLeaveStop();
    run_goal = no_stop;
    RunScanOrNext(now);
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
    run_stop = RUN_STOP_BOARD;
    run_scans = scan_place_count = 0U;
    scan_total_start = now;
    scan_stable_since = lidar_obs.scans;
    run_to_row = 0U;
    run_goal = no_stop;
    run_turning = scan_resume = 0U;
    /* 先在启停区停着扫完第一轮再走：第一段路要看清了才能走，之后边走边扫（RunTick）。 */
    ScanBegin(now);
    RunSet(COMP_RUN_SCAN, NULL);
    return HAL_OK;
}

void Comp_Stop(void)
{
    scan_resume = 0U;
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
        /* 第一轮扫完（障碍物已经记进地图）就走，雷达伸着接着扫；雷达没数据、扫描停了也走，路段都报 blind。 */
        if (scan_place_count > 0U || !scan_active)
            RunNext(now);
        break;
    case COMP_RUN_WAIT:
        if (comp.task_valid)
            RunLeaveBoard(now);
        else if (now - run_since > COMP_QR_TIMEOUT_MS)
        {
            RunHalt();
            RunSet(COMP_RUN_FAILED, "task");
        }
        else if (comp.route_state == COMP_ROUTE_STOPPED)
        {
            if (route_why == route_blocked)
                RunSweep(); // 新看到的障碍物挡住了：走到还能走的地方
            else
                RunSet(COMP_RUN_STOPPED, NULL);
        }
        else if (comp.route_state == COMP_ROUTE_DONE)
            RunSweep(); // 走到头还没读到：掉头
        break;
    case COMP_RUN_PLAN:
        if (comp.route_state == COMP_ROUTE_READY)
        {
            /* 只走第一段，到拐点按新地图重新规划。第一段从启停区停着扫过的位置出发，雷达看清了的就不会撞上
               看不见的（它和雷达的连线落在车身扫过的地方，挡住它的东西也在那里，会被看到）；后面的路段是
               边走边看过的。往车身挡住的方向走报 blind。 */
            uint8_t seen = RunLegSeen(route[0], route[1]);
            (void)RouteFollow(1U, chassis_tune.cmd_speed);
            if (comp.route_state == COMP_ROUTE_FOLLOWING)
                RunSet(COMP_RUN_MOVE, seen ? NULL : "blind");
        }
        if (comp.route_state == COMP_ROUTE_FAILED)
        {
            if (run_stop == RUN_STOP_BOARD)
                RunSet(COMP_RUN_FAILED, "route");
            else
            {
                ++run_stop;
                RunNext(now);
            }
        }
        break;
    case COMP_RUN_TURN:
        if (!run_turning)
        {
            if (!ServoAt(lidar_tune.servo_stow, now))
                break; // 雷达还没收好
            if (Chassis_TurnTo(Math_WrapDeg(origin_yaw + run_heading)) != HAL_OK)
            {
                RunSet(COMP_RUN_FAILED, "chassis");
                break;
            }
            run_turning = 1U;
            break;
        }
        if (chassis.mode == CHASSIS_MODE_TURN)
            break;
        if (fabsf(Math_WrapDeg(comp.heading_deg - run_heading)) > AGV_HEADING_FAULT_DEG)
        {
            RunSet(COMP_RUN_FAILED, "turn");
            break;
        }
        run_turning = 0U;
        /* 原地转也会挪几毫米：转完就算到了板前。 */
        run_goal = (MapPoint){(int32_t)lroundf(comp.x), (int32_t)lroundf(comp.y)};
        if (scan_resume)
        {
            scan_resume = 0U;
            ScanBegin(now);
        }
        RunNext(now);
        break;
    case COMP_RUN_MOVE:
        if (comp.route_state == COMP_ROUTE_DONE)
            RunScanOrNext(now);
        else if (comp.route_state == COMP_ROUTE_STOPPED)
        {
            if (route_why == route_blocked)
                RunNext(now); // 边走边扫看到了挡路的新障碍物：已经停下，按新地图重新规划
            else
                RunSet(COMP_RUN_STOPPED, NULL);
        }
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
    ServoTick(now);
    FootprintTick(now);
    VisionTick(now);
    ScannerTick(now);
    RouteTick();
    RunTick(now);
    Report(now);
}
#endif
