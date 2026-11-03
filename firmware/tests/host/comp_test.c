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
    MapPoint list[16];
    /* 扫完才写地图，圈数不够的不算。 */
    assert(Map_Revision() == rev && LidarObs_List(list, 16) == 0);
    assert(LidarObs_Finish() == 0 && Map_Revision() == rev);
    RunScans(&pose, cyl, 3, 28);
    assert(Map_Revision() == rev && LidarObs_List(list, 16) == 0);
    assert(LidarObs_Finish() == 3);
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
    /* 再调一次不会重复登记。 */
    assert(LidarObs_Finish() == 3 && LidarObs_List(list, 16) == 3 && Map_Revision() == rev + 3);
    /* 每个障碍物只报告一次。 */
    MapPoint p;
    unsigned popped = 0;
    while (LidarObs_PopNew(&p))
        ++popped;
    assert(popped == 3 && !LidarObs_PopNew(&p));
    /* 换个位置接着扫：写进地图的位置不再变，和地图一致。 */
    MapPoint before[16], after[16];
    assert(LidarObs_List(before, 16) == 3);
    LidarPose elsewhere = {1200, 700, 0.0f};
    RunScans(&elsewhere, cyl, 3, 20);
    assert(LidarObs_Finish() == 3 && LidarObs_List(after, 16) == 3 && Map_Revision() == rev + 3);
    assert(memcmp(before, after, sizeof(MapPoint) * 3U) == 0);
    /* 登记后障碍物附近的栅格不可走。 */
    uint16_t r, c;
    assert(Map_WorldToGrid(1200, 1700, &r, &c) && !Map_IsFree(r, c));

    /* 空场地（只有围挡）：不应产生任何障碍。 */
    Map_Init();
    LidarObs_Init();
    LidarPose p2 = {1200, 250, -15.0f};
    RunScans(&p2, cyl, 0, 200);
    assert(LidarObs_Finish() == 0);
    printf("lidar obstacle OK\n");
}

/* C1 的模型比 RunScans 严：0.72° 步进、每圈相位随机、测距 ±30 mm，圆柱边缘打不回来（只有中间
   70% 的宽度有回波）。对角 2.9 m 处 Ø50 每圈只打到一个点左右。从雷达的安装位置打。 */
static unsigned ScanOne(float sx, float sy, float heading, float cx, float cy, MapPoint *list)
{
    Map_Init();
    LidarObs_Init();
    LidarPose pose = {sx, sy, heading};
    float hr = pose.heading_deg * MATH_DEG_TO_RAD;
    float ox = sx + cosf(hr) * mount_x - sinf(hr) * mount_y, oy = sy + sinf(hr) * mount_x + cosf(hr) * mount_y;
    for (unsigned k = 0; k <= COMP_SCAN_REVS; ++k)
    {
        float phase = Noise(0.36f) + 0.36f;
        for (unsigned i = 0; i < 500; ++i)
        {
            float a = i * 0.72f + phase, world = (pose.heading_deg + mount_yaw - a) * MATH_DEG_TO_RAD;
            float dx = cosf(world), dy = sinf(world);
            float px = cx - ox, py = cy - oy, b = px * dx + py * dy;
            float miss = hypotf(px - b * dx, py - b * dy);
            float d = b > 0.0f && miss < 25.0f * 0.7f ? b - sqrtf(625.0f - miss * miss) : 0.0f;
            RPLidarSample s = {a, d > 0.0f ? d + Noise(30.0f) : 0.0f, 47, i == 0};
            LidarObs_Feed(&s, &pose);
        }
    }
    (void)LidarObs_Finish();
    return LidarObs_List(list, 16);
}

/* 识别精度：在启停区扫一次，车道上任意位置都认得出，紧贴黄色区域的也算（它们是平的，不挡雷达）。
   定位误差不能超过地图里留的余量，否则车会蹭到。这里不算车身挡住的 90°：哪些地方看得到见
   comp_route_test 和 README 的漏检率。 */
static void TestGlobalScanSeesTheWholeField(void)
{
    static const float box[][4] = {{550, 550, 1000, 1000},  {1400, 550, 1850, 1000},
                                   {550, 1400, 1000, 1850}, {1400, 1400, 1850, 1850},
                                   {0, 910, 150, 1490},     {910, 0, 1490, 150}};
    MapPoint list[16];
    float worst = 0.0f;
    unsigned tried = 0U;
    for (unsigned zone = 1; zone <= 2; ++zone)
    {
        MapPoint home;
        assert(Map_StartPoint((uint8_t)zone, &home));
        for (float x = 110.0f; x <= 2290.0f; x += 180.0f)
            for (float y = 110.0f; y <= 2230.0f; y += 180.0f)
            {
                uint8_t lane = hypotf(x - home.x, y - home.y) > 400.0f;
                for (unsigned i = 0; i < 6U; ++i)
                    if (x > box[i][0] - 25.0f && x < box[i][2] + 25.0f && y > box[i][1] - 25.0f &&
                        y < box[i][3] + 25.0f)
                        lane = 0U;
                if (!lane)
                    continue;
                ++tried;
                assert(ScanOne((float)home.x, (float)home.y, COMP_START_HEADING_DEG(zone), x, y, list) == 1U);
                worst = fmaxf(worst, hypotf(list[0].x - x, list[0].y - y));
            }
    }
    /* 紧贴黄色区域边上，朝车的那一面离它不到 40 mm。 */
    MapPoint home;
    assert(Map_StartPoint(1, &home));
    assert(ScanOne((float)home.x, (float)home.y, COMP_START_HEADING_DEG(1), 1370.0f, 1200.0f, list) == 1U);
    worst = fmaxf(worst, hypotf(list[0].x - 1370.0f, list[0].y - 1200.0f));
    printf("global scan: %u positions, worst error %.1f mm\n", tried, (double)worst);
    assert(worst <= (float)COMP_OBS_MAP_RADIUS - COMP_OBS_RADIUS_MM + (float)MAP_MARGIN_MM);
    Map_Init();
    LidarObs_Init();
}

/* 从第一个位置被挡住的，换个位置看到后追加进地图，只报告这一个新的。启停区 1 的雷达在
   (2095,2095)，(265,1287) 正好在 (1200,1700) 后面。 */
static void TestLaterScanAddsHiddenObstacle(void)
{
    static const Cyl lined_up[2] = {{1200, 1700, 25}, {265, 1287, 25}};
    LidarPose start = {2250, 2250, COMP_START_HEADING_DEG(1)};
    LidarPose board = {COMP_QR_STOP_X_MM, COMP_QR_Y_MIN_MM, COMP_QR_HEADING_DEG};
    MapPoint list[16], p;
    Map_Init();
    LidarObs_Init();
    uint32_t rev = Map_Revision();
    RunScans(&start, lined_up, 2, 20);
    assert(LidarObs_Finish() == 1 && Map_Revision() == rev + 1 && LidarObs_PopNew(&p));
    assert(hypotf(p.x - 1200.0f, p.y - 1700.0f) < 30.0f && !LidarObs_PopNew(&p));
    RunScans(&board, lined_up, 2, 20);
    assert(LidarObs_Finish() == 2 && Map_Revision() == rev + 2 && LidarObs_PopNew(&p));
    assert(hypotf(p.x - 265.0f, p.y - 1287.0f) < 30.0f && !LidarObs_PopNew(&p));
    assert(LidarObs_List(list, 16) == 2);
    Map_Init();
    LidarObs_Init();
}

/* 转盘是场内唯一高出地面的固定物，它占的那段上边不算障碍物；同样靠上边、但不在转盘范围内的算。 */
static void TestOnlyTurntableIsIgnored(void)
{
    MapPoint list[16];
    const float h = COMP_START_HEADING_DEG(1);
    assert(ScanOne(2250.0f, 2250.0f, h, 1200.0f, 2320.0f, list) == 0U);
    assert(ScanOne(2250.0f, 2250.0f, h, 600.0f, 2290.0f, list) == 1U);
    Map_Init();
    LidarObs_Init();
}

/* 噪声把一个物体拆成两个目标时只写一个进地图。 */
static void TestSplitTargetCountsOnce(void)
{
    LidarPose pose = {1200, 500, 90.0f};
    Map_Init();
    LidarObs_Init();
    /* 雷达 0° 朝 +Y，距离 700、764 mm：相隔 64 mm，成为两个目标；之后越测越靠近。 */
    const float gap_angle = 90.0f; // 中间夹一个不算簇的点，两次测量才不会并成一簇
    static const float first[2] = {700.0f, 764.0f};
    for (unsigned k = 0; k < 2U; ++k)
    {
        RPLidarSample s = {0.0f, first[k], 47, 1};
        LidarObs_Feed(&s, &pose);
    }
    for (unsigned k = 0; k < 40U; ++k)
    {
        RPLidarSample s = {0.0f, k % 2U ? 752.0f : 712.0f, 47, 1};
        RPLidarSample nothing = {gap_angle, 0.0f, 47, 0};
        LidarObs_Feed(&s, &pose);
        LidarObs_Feed(&nothing, &pose);
    }
    uint32_t revision = Map_Revision();
    assert(LidarObs_Finish() == 1U && Map_Revision() == revision + 1U);
    Map_Init();
    LidarObs_Init();
}

/* 改运行时参数后，真实的识别结果要跟着变。三个圆柱离车分别约 500、520、900 mm。 */
static unsigned CountObstacles(unsigned scans, MapPoint *list)
{
    static const Cyl cyl[] = {{1200, 1700, 25}, {1720, 1180, 25}, {300, 1200, 25}};
    LidarPose pose = {1200, 1200, 30.0f};
    RunScans(&pose, cyl, 3, scans);
    (void)LidarObs_Finish();
    return LidarObs_List(list, 16);
}

/* 参数恢复默认，雷达放在车体中心：这几个测试和安装位置无关，要改安装位置的自己改。 */
static void BeginTuning(void)
{
    Map_Init();
    LidarObs_Init();
    lidar_tune = lidar_tune_default;
    mount_x = lidar_tune.lidar_x = 0.0f;
    mount_y = lidar_tune.lidar_y = 0.0f;
    mount_yaw = lidar_tune.lidar_yaw = 0.0f;
}

static void TestLidarTuning(void)
{
    MapPoint list[16];
    BeginTuning();
    assert(CountObstacles(3, list) == 0); // 默认要看到 8 圈
    BeginTuning();
    lidar_tune.obs_confirm = 2.0f;
    assert(CountObstacles(3, list) == 3);

    BeginTuning();
    lidar_tune.lidar_min = 600.0f; // 只剩 900 mm 那个
    assert(CountObstacles(12, list) == 1 && fabsf((float)list[0].x - 300.0f) < 20.0f);
    BeginTuning();
    lidar_tune.lidar_max = 600.0f; // 去掉 900 mm 那个
    assert(CountObstacles(12, list) == 2);

    BeginTuning();
    lidar_tune.obs_width = 10.0f; // Ø50 的簇都比这宽
    assert(CountObstacles(12, list) == 0);
    BeginTuning();
    lidar_tune.obs_gap = 1.0f; // 相邻点都被拆开：单点也算，还是能认出来
    assert(CountObstacles(12, list) == 3 && lidar_obs.clusters > 3U * 12U);
    BeginTuning();
    lidar_tune.obs_match = 0.1f; // 测距噪声下每圈的圆心都对不上
    assert(CountObstacles(12, list) == 0);

    /* 雷达实际装在车体前 100、左 40 mm，转了 90°：参数没跟上时位置全错，改对后回到真值。 */
    BeginTuning();
    mount_x = 100.0f;
    mount_y = 40.0f;
    mount_yaw = 90.0f;
    unsigned wrong = CountObstacles(12, list);
    float far_off = 0.0f;
    for (unsigned i = 0; i < wrong; ++i)
        far_off = fmaxf(far_off, hypotf((float)list[i].x - 1200.0f, (float)list[i].y - 1700.0f));
    assert(wrong != 3 || far_off > 100.0f);

    BeginTuning();
    mount_x = lidar_tune.lidar_x = 100.0f;
    mount_y = lidar_tune.lidar_y = 40.0f;
    mount_yaw = lidar_tune.lidar_yaw = 90.0f;
    static const float truth[3][2] = {{1200, 1700}, {1720, 1180}, {300, 1200}};
    assert(CountObstacles(12, list) == 3);
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
    FeedArc(&left_mount, 12);
    assert(LidarObs_Finish() == 1 && LidarObs_List(list, 16) == 1);
    assert(fabsf((float)list[0].x - 1516.0f) <= 1.0f && fabsf((float)list[0].y - 1200.0f) <= 1.0f);

    /* 雷达在车前 150 mm、转 90° 朝车左看：视线沿 +Y，x 不偏。 */
    LidarPose front_mount = {1050, 1200, 0.0f};
    BeginTuning();
    lidar_tune.lidar_x = 150.0f;
    lidar_tune.lidar_yaw = 90.0f;
    FeedArc(&front_mount, 12);
    assert(LidarObs_Finish() == 1 && LidarObs_List(list, 16) == 1);
    assert(fabsf((float)list[0].x - 1200.0f) <= 1.0f && fabsf((float)list[0].y - 1516.0f) <= 1.0f);
    BeginTuning();
}

/* 无噪声的一小段弧：雷达角度 centre ±2°，距离 dist。 */
static void FeedArcAt(const LidarPose *pose, int centre, float dist, unsigned scans)
{
    for (unsigned k = 0; k < scans; ++k)
        for (int a = centre - 2; a <= centre + 2; ++a)
        {
            RPLidarSample s = {(float)((a + 360) % 360), dist, 47, a == centre - 2};
            LidarObs_Feed(&s, pose);
        }
}

/* 雷达在左后角外 (-155,155)：朝车身打到的是车身，离雷达 115 mm 也是车体自己，不算障碍物（按离雷达
   多远来分，这些点就成了压在车底下的障碍物）；往车外打的才算。 */
static void TestBodyPointsAreIgnored(void)
{
    MapPoint list[16];
    LidarPose pose = {1200, 1200, 0.0f};
    BeginTuning();
    lidar_tune = lidar_tune_default;
    assert(lidar_tune.lidar_x == -155.0f && lidar_tune.lidar_y == 155.0f);
    FeedArcAt(&pose, 10, 115.0f, 12); // 雷达角度顺时针：10° 是车头偏右，打在车身左边上
    assert(LidarObs_Finish() == 0);
    FeedArcAt(&pose, 270, 200.0f, 12); // 车左边，离车身 220 mm
    assert(LidarObs_Finish() == 1 && LidarObs_List(list, 16) == 1);
    assert(fabsf((float)list[0].x - 1045.0f) <= 1.0f && fabsf((float)list[0].y - 1571.0f) <= 1.0f);
    BeginTuning();
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    TestParser();
    TestVisionLink();
    TestObstacles();
    TestGlobalScanSeesTheWholeField();
    TestLaterScanAddsHiddenObstacle();
    TestOnlyTurntableIsIgnored();
    TestSplitTargetCountsOnce();
    TestLidarTuning();
    TestSightLineStartsAtLidar();
    TestBodyPointsAreIgnored();
    printf("comp_test OK\n");
    return 0;
}
