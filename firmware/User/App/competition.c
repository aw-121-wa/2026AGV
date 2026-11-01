#include "competition.h"
#if AGV_MODE == AGV_MODE_COMPETITION
#include "chassis.h"
#include "chassis_telemetry.h"
#include "comp_config.h"
#include "map.h"
#include "rplidar.h"
#include "ser_port.h"
#include "user_math.h"
#include <math.h>

CompStatus comp;

static MapPoint home;
static float origin_x, origin_y, origin_yaw, cs, sn;
static uint32_t boot_ms, lidar_cmd_ms, lidar_last_sample_ms, hz_ms, hz_scans;
static uint32_t pose_ms, mode_ms, report_ms;
static uint8_t obstacles_dirty;
static RPLidarParser parser;

HAL_StatusTypeDef Comp_Init(unsigned zone)
{
    comp = (CompStatus){0};
    Map_Init();
    if (!Map_StartPoint((uint8_t)zone, &home))
        return HAL_ERROR;
    LidarObs_Init();
    VisionLink_Init();
    RPLidar_Reset(&parser);
    boot_ms = HAL_GetTick();
    if (SerPort_Init(SER_LIDAR, COMP_LIDAR_BAUD) != HAL_OK ||
        SerPort_Init(SER_VISION, COMP_VISION_BAUD) != HAL_OK)
        return HAL_ERROR;
    return HAL_OK;
}

/* 与跑图模式相同的约定：底盘 READY 后记下里程计原点和航向零点，之后换算到场地坐标。 */
static void UpdatePose(uint32_t now)
{
    if (!comp.anchored)
    {
        if (chassis.state != CHASSIS_STATE_READY || now - boot_ms < AGV_START_DELAY_MS)
            return;
        origin_x = chassis.pos_x;
        origin_y = chassis.pos_y;
        origin_yaw = chassis.yaw;
        cs = cosf(origin_yaw * MATH_DEG_TO_RAD);
        sn = sinf(origin_yaw * MATH_DEG_TO_RAD);
        comp.anchored = 1;
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

static void LidarTick(uint32_t now)
{
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
            if (comp.anchored)
                LidarObs_Feed(&s, &pose);
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
    MapPoint p;
    while (LidarObs_PopNew(&p))
    {
        comp.obstacles = lidar_obs.confirmed;
        obstacles_dirty = 1;
        ChassisTelemetry_Printf("COMP obstacle #%u at (%ld,%ld)%s\r\n", (unsigned)comp.obstacles,
                                (long)p.x, (long)p.y, lidar_obs.map_full ? " MAP FULL" : "");
    }
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

static void VisionTick(uint32_t now)
{
    uint8_t buf[128], f[VL_FRAME_MAX];
    unsigned n;
    while ((n = SerPort_Read(SER_VISION, buf, sizeof(buf))) > 0U)
        for (unsigned i = 0; i < n; ++i)
            (void)VisionLink_Feed(buf[i], now);
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

/* 定点打印一位小数（ChassisTelemetry 的 printf 不保证支持 %f）。 */
static const char *Fix1(char *buf, float v)
{
    long t = lroundf(v * 10.0f);
    char sign = t < 0 ? '-' : 0;
    if (t < 0)
        t = -t;
    char *p = buf;
    if (sign)
        *p++ = sign;
    long ip = t / 10;
    char tmp[12];
    int n = 0;
    do
        tmp[n++] = (char)('0' + ip % 10);
    while ((ip /= 10) > 0 && n < 11);
    while (n)
        *p++ = tmp[--n];
    *p++ = '.';
    *p++ = (char)('0' + t % 10);
    *p = 0;
    return buf;
}

static void Report(uint32_t now)
{
    char yaw[16], hz[16];
    if (now - report_ms < COMP_REPORT_PERIOD_MS)
        return;
    report_ms = now;
    static const char *const lidar_text[] = {"BOOT", "STOP", "START", "RUN"};
    ChassisTelemetry_Printf(
        "COMP pose=(%ld,%ld,%s) lidar=%s %sHz pts=%lu used=%lu clu=%lu "
        "obs=%u rs=%lu/%lu | vision=%s mode=%u task=%u mat=%u ring=%u%s crc=%lu\r\n",
        (long)lroundf(comp.x), (long)lroundf(comp.y), Fix1(yaw, comp.heading_deg),
        lidar_text[comp.lidar_state], Fix1(hz, comp.lidar_hz),
        (unsigned long)lidar_obs.points, (unsigned long)lidar_obs.used,
        (unsigned long)lidar_obs.clusters, (unsigned)comp.obstacles,
        (unsigned long)comp.lidar_restarts, (unsigned long)parser.resync,
        comp.vision_online ? "ON" : "OFF", (unsigned)comp.vision_mode, (unsigned)vision.task_len,
        (unsigned)vision.material_count, (unsigned)vision.ring_found,
        vision.ring_aligned ? "(aligned)" : "", (unsigned long)vision.crc_errors);
}

void Comp_Tick(void)
{
    uint32_t now = HAL_GetTick();
    UpdatePose(now);
    LidarTick(now);
    VisionTick(now);
    Report(now);
}
#endif
