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
    uint8_t mp[11] = {2, 1, 123, 0, (uint8_t)-45, 0xFF, 3, 0, 0, 0, 0};
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
    /* 最长的文本也放得下并以 NUL 结尾。 */
    uint8_t long_text[VL_MAX_PAYLOAD];
    memset(long_text, 'x', sizeof(long_text));
    n = VisionLink_Encode(VL_HOST_TASK, long_text, sizeof(long_text), f);
    for (unsigned i = 0; i < n; ++i)
        VisionLink_Feed(f[i], 390);
    assert(vision.task_len == VL_MAX_PAYLOAD && vision.task_text[VL_MAX_PAYLOAD - 1U] == 'x' &&
           vision.task_text[VL_MAX_PAYLOAD] == '\0' && vision.task_ms == 390);
    /* TASK 帧带二维码原文，字节与 tests/vision_protocol_test.py 里上位机发出的相同；
       比上一帧短也要正确结束，不残留上一帧的内容。 */
    static const uint8_t task_frame[] = {0xAA, 0x55, 15,  0x02, 0x00, '1', '5', '6', '+', '1', '2',
                                         '3',  '+',  '5', '1',  '6',  '+', '2', '3', '1', 0xD6};
    for (unsigned i = 0; i < sizeof(task_frame); ++i)
        VisionLink_Feed(task_frame[i], 400);
    assert(vision.task_len == 15 && strcmp(vision.task_text, "156+123+516+231") == 0 &&
           vision.task_ms == 400);
    /* 空文本是坏帧，不覆盖已有内容。 */
    n = VisionLink_Encode(VL_HOST_TASK, long_text, 0, f);
    for (unsigned i = 0; i < n; ++i)
        VisionLink_Feed(f[i], 420);
    assert(vision.bad_frames == 1 && vision.task_len == 15 && vision.task_ms == 400);
    assert(VisionLink_Online(450, 500) && !VisionLink_Online(1000, 500));
    /* 长度不符的心跳帧计入 bad_frames。 */
    const uint8_t hb[3] = {1, 2, 3};
    n = VisionLink_Encode(VL_HOST_HEARTBEAT, hb, sizeof(hb), f);
    for (unsigned i = 0; i < n; ++i)
        VisionLink_Feed(f[i], 500);
    assert(vision.bad_frames == 2);
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

/* 雷达在仿真车上的真实安装位置，默认与配置一致。 */
static float mount_x = COMP_LIDAR_X_MM, mount_y = COMP_LIDAR_Y_MM, mount_yaw = COMP_LIDAR_YAW_DEG;

static void RunScans(const LidarPose *pose, const Cyl *c, unsigned nc, unsigned scans)
{
    for (unsigned k = 0; k < scans; ++k)
        for (unsigned i = 0; i < 500; ++i) // 0.72° 分辨率
        {
            float a = i * 0.72f + 0.3f * (k & 1);
            float world = (pose->heading_deg + mount_yaw - a) * MATH_DEG_TO_RAD;
            float hr = pose->heading_deg * MATH_DEG_TO_RAD;
            float ox = pose->x + cosf(hr) * mount_x - sinf(hr) * mount_y;
            float oy = pose->y + sinf(hr) * mount_x + cosf(hr) * mount_y;
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
    RunScans(&pose, cyl, 3, 200);
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
    RunScans(&p2, cyl, 0, 200);
    assert(LidarObs_List(list, 16) == 0);
    printf("lidar obstacle OK\n");
}

/* 改运行时参数后，真实的识别结果要跟着变。三个圆柱离车分别约 500、520、900 mm。 */
static unsigned CountObstacles(unsigned scans, MapPoint *list)
{
    static const Cyl cyl[] = {{1200, 1700, 25}, {1720, 1180, 25}, {300, 1200, 25}};
    LidarPose pose = {1200, 1200, 30.0f};
    RunScans(&pose, cyl, 3, scans);
    return LidarObs_List(list, 16);
}

static void BeginTuning(void)
{
    Map_Init();
    LidarObs_Init();
    lidar_tune = lidar_tune_default;
    mount_x = COMP_LIDAR_X_MM;
    mount_y = COMP_LIDAR_Y_MM;
    mount_yaw = COMP_LIDAR_YAW_DEG;
}

static void TestLidarTuning(void)
{
    MapPoint list[16];
    BeginTuning();
    assert(CountObstacles(3, list) == 0); // 默认要连续 4 圈
    BeginTuning();
    lidar_tune.obs_confirm = 2.0f;
    assert(CountObstacles(3, list) == 3);

    BeginTuning();
    lidar_tune.lidar_min = 600.0f; // 只剩 900 mm 那个
    assert(CountObstacles(8, list) == 1 && fabsf((float)list[0].x - 300.0f) < 20.0f);
    BeginTuning();
    lidar_tune.lidar_max = 600.0f; // 去掉 900 mm 那个
    assert(CountObstacles(8, list) == 2);

    BeginTuning();
    lidar_tune.obs_width = 10.0f; // Ø50 的簇都比这宽
    assert(CountObstacles(8, list) == 0);
    BeginTuning();
    lidar_tune.obs_gap = 1.0f; // 相邻点都被拆开，凑不成簇
    assert(CountObstacles(8, list) == 0);
    BeginTuning();
    lidar_tune.obs_match = 0.1f; // 测距噪声下每圈的圆心都对不上
    assert(CountObstacles(8, list) == 0);

    /* 雷达实际装在车体前 100、左 40 mm，转了 90°：参数没跟上时位置全错，改对后回到真值。 */
    BeginTuning();
    mount_x = 100.0f;
    mount_y = 40.0f;
    mount_yaw = 90.0f;
    unsigned wrong = CountObstacles(8, list);
    float far_off = 0.0f;
    for (unsigned i = 0; i < wrong; ++i)
        far_off = fmaxf(far_off, hypotf((float)list[i].x - 1200.0f, (float)list[i].y - 1700.0f));
    assert(wrong != 3 || far_off > 100.0f);

    BeginTuning();
    mount_x = lidar_tune.lidar_x = 100.0f;
    mount_y = lidar_tune.lidar_y = 40.0f;
    mount_yaw = lidar_tune.lidar_yaw = 90.0f;
    static const float truth[3][2] = {{1200, 1700}, {1720, 1180}, {300, 1200}};
    assert(CountObstacles(8, list) == 3);
    for (unsigned j = 0; j < 3; ++j)
    {
        float best = 1e9f;
        for (unsigned i = 0; i < 3; ++i)
            best = fminf(best, hypotf((float)list[i].x - truth[j][0], (float)list[i].y - truth[j][1]));
        assert(best < 20.0f);
    }
    BeginTuning();
    printf("lidar tuning OK\n");
}

/* 无噪声的一小段弧：雷达 0° 方向 ±2°，距离 300 mm。 */
static void FeedArc(const LidarPose *pose, unsigned scans)
{
    for (unsigned k = 0; k < scans; ++k)
        for (int a = -2; a <= 2; ++a)
        {
            RPLidarSample s = {(float)(a < 0 ? a + 360 : a), 300.0f, 47, a == -2};
            LidarObs_Feed(&s, pose);
        }
}

/* 圆心沿视线外推，视线要从雷达算起，不是从车体中心。外推量 25 × 2/π = 15.9 mm。 */
static void TestSightLineStartsAtLidar(void)
{
    MapPoint list[16];

    /* 雷达在车左 150 mm、朝车头看：视线沿 +X，y 不偏。从车体中心算会偏 7 mm。
       弧放在十字通道中间，离四个岛够远，不被静态结构过滤。 */
    LidarPose left_mount = {1200, 1050, 0.0f};
    BeginTuning();
    lidar_tune.lidar_y = 150.0f;
    FeedArc(&left_mount, 6);
    assert(LidarObs_List(list, 16) == 1);
    assert(fabsf((float)list[0].x - 1516.0f) <= 1.0f && fabsf((float)list[0].y - 1200.0f) <= 1.0f);

    /* 雷达在车前 150 mm、转 90° 朝车左看：视线沿 +Y，x 不偏。 */
    LidarPose front_mount = {1050, 1200, 0.0f};
    BeginTuning();
    lidar_tune.lidar_x = 150.0f;
    lidar_tune.lidar_yaw = 90.0f;
    FeedArc(&front_mount, 6);
    assert(LidarObs_List(list, 16) == 1);
    assert(fabsf((float)list[0].x - 1200.0f) <= 1.0f && fabsf((float)list[0].y - 1516.0f) <= 1.0f);
    BeginTuning();
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    TestParser();
    TestVisionLink();
    TestObstacles();
    TestLidarTuning();
    TestSightLineStartsAtLidar();
    printf("comp_test OK\n");
    return 0;
}
