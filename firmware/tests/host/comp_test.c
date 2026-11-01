/* 比赛模式新增纯软件模块的主机测试：RPLIDAR 协议解析、视觉串口协议、雷达障碍物识别。 */
#include "comp_config.h"
#include "lidar_obstacle.h"
#include "map.h"
#include "rplidar.h"
#include "user_math.h"
#include "vision_link.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void EncodeSample(uint8_t *b, float angle, float dist, uint8_t start)
{
    uint16_t a = (uint16_t)lroundf(angle * 64.0f), d = (uint16_t)lroundf(dist * 4.0f);
    b[0] = (uint8_t)((15U << 2) | (start ? 1U : 2U));
    b[1] = (uint8_t)(((a & 0x7FU) << 1) | 1U);
    b[2] = (uint8_t)(a >> 7);
    b[3] = (uint8_t)d;
    b[4] = (uint8_t)(d >> 8);
}

static void TestParser(void)
{
    RPLidarParser p;
    RPLidarSample s;
    RPLidar_Reset(&p);
    const uint8_t junk[] = {0x00, 0xA5, 0x5A, 0x13, 0xA5};
    for (unsigned i = 0; i < sizeof(junk); ++i)
        assert(!RPLidar_Feed(&p, junk[i], &s));
    const uint8_t desc[] = {0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81};
    for (unsigned i = 0; i < sizeof(desc); ++i)
        assert(!RPLidar_Feed(&p, desc[i], &s));
    assert(p.streaming);
    uint8_t b[5];
    EncodeSample(b, 123.5f, 812.25f, 1);
    unsigned got = 0;
    for (unsigned i = 0; i < 5; ++i)
        got += RPLidar_Feed(&p, b[i], &s);
    assert(got == 1 && fabsf(s.angle_deg - 123.5f) < 0.02f && fabsf(s.dist_mm - 812.25f) < 0.01f &&
           s.start && s.quality == 15);
    /* 插一个坏字节，解析器应当重新对齐并继续出点。 */
    assert(!RPLidar_Feed(&p, 0x00, &s) || 1);
    EncodeSample(b, 10.0f, 500.0f, 0);
    got = 0;
    for (int k = 0; k < 3; ++k)
        for (unsigned i = 0; i < 5; ++i)
            got += RPLidar_Feed(&p, b[i], &s);
    assert(got >= 2 && p.resync >= 1 && fabsf(s.angle_deg - 10.0f) < 0.02f);
    printf("rplidar parser OK (resync=%u)\n", (unsigned)p.resync);
}

static void TestVisionLink(void)
{
    VisionLink_Init();
    uint8_t f[VL_FRAME_MAX];
    /* 上位机物料帧：红色前 12.3 mm 左 -4.5 mm；蓝色 0/0。 */
    uint8_t mp[11] = {2, 1, 123, 0, (uint8_t)-45, 0xFF, 5, 0, 0, 0, 0};
    unsigned n = VisionLink_Encode(VL_HOST_MATERIALS, mp, sizeof(mp), f);
    uint8_t ok = 0;
    VisionLink_Feed(0x13, 1); // 前导垃圾
    for (unsigned i = 0; i < n; ++i)
        ok |= VisionLink_Feed(f[i], 100);
    assert(ok && vision.material_count == 2 && vision.material[0].color == 1);
    assert(fabsf(vision.material[0].fwd_mm - 12.3f) < 1e-3f &&
           fabsf(vision.material[0].left_mm + 4.5f) < 1e-3f && vision.material_ms == 100);
    /* CRC 错误帧不应被接受。 */
    uint8_t rp[6] = {1, 1, 10, 0, 20, 0};
    n = VisionLink_Encode(VL_HOST_RING, rp, sizeof(rp), f);
    f[n - 1] ^= 0x5A;
    for (unsigned i = 0; i < n; ++i)
        assert(!VisionLink_Feed(f[i], 200));
    assert(vision.crc_errors == 1 && !vision.ring_found);
    n = VisionLink_Encode(VL_HOST_RING, rp, sizeof(rp), f);
    for (unsigned i = 0; i < n; ++i)
        VisionLink_Feed(f[i], 300);
    assert(vision.ring_found && vision.ring_aligned && fabsf(vision.ring_left_mm - 2.0f) < 1e-3f);
    uint8_t tp[4] = {3, 1, 2, 3};
    n = VisionLink_Encode(VL_HOST_TASK, tp, sizeof(tp), f);
    for (unsigned i = 0; i < n; ++i)
        VisionLink_Feed(f[i], 400);
    assert(vision.task_len == 3 && vision.task[2] == 3);
    assert(VisionLink_Online(450, 500) && !VisionLink_Online(1000, 500));
    /* 长度不符的心跳帧计入 bad_frames。 */
    n = VisionLink_Encode(VL_HOST_HEARTBEAT, tp, 3, f);
    for (unsigned i = 0; i < n; ++i)
        VisionLink_Feed(f[i], 500);
    assert(vision.bad_frames == 1);
    /* 下行位姿帧格式。 */
    n = VisionLink_EncodePose(1234.4f, -20.0f, -90.5f, 1, f);
    assert(n == 13 && f[0] == 0xAA && f[1] == 0x55 && f[2] == 7 && f[3] == VL_MCU_POSE);
    assert((int16_t)(f[5] | f[6] << 8) == 1234 && (int16_t)(f[7] | f[8] << 8) == -20 &&
           (int16_t)(f[9] | f[10] << 8) == -9050 && f[11] == 1);
    assert(VisionLink_Crc8(f + 2, 10) == f[12]);
    /* CRC-8/0x07 标准校验值："123456789" → 0xF4 */
    assert(VisionLink_Crc8((const uint8_t *)"123456789", 9) == 0xF4);
    printf("vision link OK\n");
}

/* ---- 雷达仿真：圆柱障碍 + 场地四个岛 + 围挡 ---- */
typedef struct
{
    float x, y, r;
} Cyl;

static float RayRect(float ox, float oy, float dx, float dy, float x0, float y0, float x1,
                     float y1)
{
    float tmin = 0, tmax = 1e9f;
    float o[2] = {ox, oy}, d[2] = {dx, dy}, lo[2] = {x0, y0}, hi[2] = {x1, y1};
    for (int k = 0; k < 2; ++k)
    {
        if (fabsf(d[k]) < 1e-9f)
        {
            if (o[k] < lo[k] || o[k] > hi[k])
                return 1e9f;
            continue;
        }
        float t0 = (lo[k] - o[k]) / d[k], t1 = (hi[k] - o[k]) / d[k];
        if (t0 > t1)
        {
            float t = t0;
            t0 = t1;
            t1 = t;
        }
        tmin = fmaxf(tmin, t0);
        tmax = fminf(tmax, t1);
    }
    return tmin <= tmax && tmin > 0 ? tmin : 1e9f;
}

static float Raycast(float ox, float oy, float dx, float dy, const Cyl *c, unsigned nc)
{
    float best = 1e9f;
    for (unsigned i = 0; i < nc; ++i)
    {
        float px = c[i].x - ox, py = c[i].y - oy, b = px * dx + py * dy;
        float disc = b * b - (px * px + py * py - c[i].r * c[i].r);
        if (disc > 0 && b > 0)
            best = fminf(best, b - sqrtf(disc));
    }
    const float L = MAP_ISLAND_LOW, H = MAP_ISLAND_HIGH, S = MAP_ISLAND_SIZE;
    float isl[4][2] = {{L, L}, {H, L}, {L, H}, {H, H}};
    for (int i = 0; i < 4; ++i)
        best = fminf(best, RayRect(ox, oy, dx, dy, isl[i][0], isl[i][1], isl[i][0] + S,
                                   isl[i][1] + S));
    /* 围挡：场地外 20 mm 厚的墙 */
    if (dx > 1e-6f)
        best = fminf(best, (MAP_WIDTH_MM - ox) / dx);
    if (dx < -1e-6f)
        best = fminf(best, -ox / dx);
    if (dy > 1e-6f)
        best = fminf(best, (MAP_HEIGHT_MM - oy) / dy);
    if (dy < -1e-6f)
        best = fminf(best, -oy / dy);
    return best;
}

static unsigned rng = 12345;
static float Noise(float amp)
{
    rng = rng * 1103515245U + 12345U;
    return ((float)((rng >> 8) & 0xFFFF) / 65535.0f * 2.0f - 1.0f) * amp;
}

static void RunScans(const LidarPose *pose, const Cyl *c, unsigned nc, unsigned scans)
{
    for (unsigned k = 0; k < scans; ++k)
        for (unsigned i = 0; i < 500; ++i) // 0.72° 分辨率
        {
            float a = i * 0.72f + 0.3f * (k & 1);
            float world = (pose->heading_deg + COMP_LIDAR_YAW_DEG - a) * MATH_DEG_TO_RAD;
            float hr = pose->heading_deg * MATH_DEG_TO_RAD;
            float ox = pose->x + cosf(hr) * COMP_LIDAR_X_MM - sinf(hr) * COMP_LIDAR_Y_MM;
            float oy = pose->y + sinf(hr) * COMP_LIDAR_X_MM + cosf(hr) * COMP_LIDAR_Y_MM;
            float d = Raycast(ox, oy, cosf(world), sinf(world), c, nc);
            RPLidarSample s = {a, d > 12000 ? 0 : d + Noise(15.0f), 47, i == 0};
            LidarObs_Feed(&s, pose);
        }
}

static void TestObstacles(void)
{
    Map_Init();
    LidarObs_Init();
    const Cyl cyl[] = {{1200, 1700, 25}, {1720, 1180, 25}, {300, 1200, 25}};
    LidarPose pose = {1200, 1200, 30.0f};
    uint32_t rev = Map_Revision();
    RunScans(&pose, cyl, 3, 2);
    assert(lidar_obs.confirmed == 0); // 两圈不足以确认
    RunScans(&pose, cyl, 3, 6);
    MapPoint list[16];
    unsigned n = LidarObs_List(list, 16);
    printf("obstacles: %u (clusters=%u used=%u/%u)\n", n, (unsigned)lidar_obs.clusters,
           (unsigned)lidar_obs.used, (unsigned)lidar_obs.points);
    for (unsigned i = 0; i < n; ++i)
        printf("  (%ld,%ld)\n", (long)list[i].x, (long)list[i].y);
    assert(n == 3);
    for (unsigned j = 0; j < 3; ++j)
    {
        float best = 1e9f;
        for (unsigned i = 0; i < n; ++i)
            best = fminf(best, hypotf(list[i].x - cyl[j].x, list[i].y - cyl[j].y));
        assert(best < 20.0f);
    }
    assert(Map_Revision() == rev + 3);
    /* 继续扫描不会重复登记。 */
    RunScans(&pose, cyl, 3, 10);
    assert(LidarObs_List(list, 16) == 3 && Map_Revision() == rev + 3);
    MapPoint p;
    unsigned popped = 0;
    while (LidarObs_PopNew(&p))
        ++popped;
    assert(popped == 3 && !LidarObs_PopNew(&p));
    /* 登记后障碍物附近的栅格不可走。 */
    uint16_t r, c;
    assert(Map_WorldToGrid(1200, 1700, &r, &c) && !Map_IsFree(r, c));

    /* 空场地（只有岛和围挡）：不应产生任何障碍。 */
    Map_Init();
    LidarObs_Init();
    LidarPose p2 = {1200, 250, -15.0f};
    RunScans(&p2, cyl, 0, 20);
    assert(LidarObs_List(list, 16) == 0);
    printf("lidar obstacle OK\n");
}

int main(void)
{
    TestParser();
    TestVisionLink();
    TestObstacles();
    printf("comp_test OK\n");
    return 0;
}
