/* 比赛模式闭环：真实的底盘、控制台、文本输出、雷达识别、地图和 A*，电机、陀螺仪、舵机和串口是假的。
   假雷达在车身左后角外，朝车身那 90° 打到车身；舵机把它伸出去到位才测得到东西。
   验证开局流程边走边扫找全障碍物、看得清的路段不撞看不见的障碍物（伸着的雷达也算），路线能规划能走到，
   以及文本输出不改变底盘控制。 */
#define main original_chassis_tests
#include "chassis_test.c"
#undef main
#include "agv_run.h"
#include "comp_config.h"
#include "competition.h"
#include "ser_port.h"
#include "servo.h"

static UART_HandleTypeDef uart;
static uint8_t boot_zone = 1U;

/* ---------- 假舵机：记下最近一次给的角度和时间 ---------- */
static float servo_at = -1.0f;
static uint32_t servo_since;
static unsigned servo_calls;

void Servo_Init(void)
{
}

void Servo_SetDeg(float deg)
{
    servo_at = deg;
    servo_since = ms;
    ++servo_calls;
}

/* 雷达伸出去到位了吗。没到位时雷达还在车身里（或者正在动），测不到东西。 */
static uint8_t LidarDeployed(void)
{
    return servo_at == lidar_tune.servo_scan && ms - servo_since >= COMP_LIDAR_SERVO_MS;
}

/* ---------- 假串口：雷达按 C1 的 5 kHz、0.72° 出点；扫码模块在二维码板前连续读码 ---------- */
/* 扫码模块对着 +X：车头朝开局方向、板子中心正对它 ±15 mm、车走得慢时才读得到。比真模块苛刻，
   逼着车沿板找。 */
#define SCANNER_WINDOW_MM    15.0f
#define SCANNER_MAX_SPEED_MM 80.0f
static float board_y = 1200.0f, robot_speed, last_x, last_y;
static SerPortStats ser_stats[SER_COUNT];
static uint8_t ser_rx_on[SER_COUNT];
static unsigned ser_reads[SER_COUNT], lidar_commands, lidar_stops;
static uint8_t lidar_connected = 1, lidar_on, lidar_descriptor;
static unsigned lidar_budget, lidar_index;
static float world[8][2];
static unsigned world_count;
static const char *scanner_code = "156+123+516+231\r\n";

HAL_StatusTypeDef SerPort_Init(SerId id, uint32_t baud)
{
    (void)baud;
    ser_rx_on[id] = 1U;
    return HAL_OK;
}

void SerPort_EnableRx(SerId id, uint8_t on)
{
    ser_rx_on[id] = on;
}

void SerPort_Flush(SerId id)
{
    (void)id;
}

const SerPortStats *SerPort_Stats(SerId id)
{
    return &ser_stats[id];
}

uint8_t SerPort_Write(SerId id, const uint8_t *src, unsigned len)
{
    if (id == SER_LIDAR && len == 2U)
    {
        ++lidar_commands;
        if (src[1] == RPLIDAR_CMD_SCAN[1] && lidar_connected)
        {
            lidar_on = 1U;
            lidar_descriptor = 1U;
        }
        else if (src[1] == RPLIDAR_CMD_STOP[1])
        {
            ++lidar_stops;
            lidar_on = 0U;
        }
    }
    return 1U;
}

static float Ray(float ox, float oy, float dx, float dy)
{
    float best = 0.0f;
    for (unsigned i = 0; i < world_count; ++i)
    {
        float px = world[i][0] - ox, py = world[i][1] - oy, b = px * dx + py * dy;
        float disc = b * b - (px * px + py * py - 25.0f * 25.0f);
        if (disc > 0.0f && b > 0.0f)
        {
            float d = b - sqrtf(disc);
            if (best == 0.0f || d < best)
                best = d;
        }
    }
    return best;
}

/* 雷达这一线打出去多远（雷达角度顺时针）。雷达在车身外：朝车身打的那些打到 270 见方的车身边上，
   其余的在场地里找障碍物，没打到返回 0。 */
static float LidarRange(float angle)
{
    float body = (lidar_tune.lidar_yaw - angle) * MATH_DEG_TO_RAD; // 车体系方向，逆时针
    const float o[2] = {lidar_tune.lidar_x, lidar_tune.lidar_y}, d[2] = {cosf(body), sinf(body)};
    float t0 = 0.0f, t1 = 1e9f;
    for (unsigned k = 0; k < 2U; ++k)
    {
        if (fabsf(d[k]) < 1e-6f)
        {
            if (fabsf(o[k]) > 135.0f)
                t1 = -1.0f;
            continue;
        }
        float a = (-135.0f - o[k]) / d[k], b = (135.0f - o[k]) / d[k];
        t0 = fmaxf(t0, fminf(a, b));
        t1 = fminf(t1, fmaxf(a, b));
    }
    if (t0 > 0.0f && t0 <= t1)
        return t0 + 1.0f;
    float h = comp.heading_deg * MATH_DEG_TO_RAD, ch = cosf(h), sh = sinf(h);
    float ox = comp.x + ch * o[0] - sh * o[1], oy = comp.y + sh * o[0] + ch * o[1];
    return Ray(ox, oy, ch * d[0] - sh * d[1], sh * d[0] + ch * d[1]);
}

unsigned SerPort_Read(SerId id, uint8_t *dst, unsigned max)
{
    ++ser_reads[id];
    if (id == SER_SCANNER && scanner_code && ser_rx_on[id] && comp.anchored && comp.x > 2100.0f &&
        fabsf(Math_WrapDeg(comp.heading_deg - COMP_QR_HEADING_DEG)) < 10.0f &&
        fabsf(comp.y - board_y) < SCANNER_WINDOW_MM && robot_speed < SCANNER_MAX_SPEED_MM)
    {
        size_t length = strlen(scanner_code);
        memcpy(dst, scanner_code, length);
        return (unsigned)length;
    }
    if (id != SER_LIDAR || !lidar_on || !ser_rx_on[id])
        return 0U;
    if (lidar_descriptor)
    {
        static const uint8_t descriptor[] = {0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81};
        lidar_descriptor = 0U;
        memcpy(dst, descriptor, sizeof(descriptor));
        return sizeof(descriptor);
    }
    unsigned n = 0U;
    while (n + 5U <= max && lidar_budget)
    {
        --lidar_budget;
        float angle = (float)lidar_index * 0.72f;
        float d = comp.anchored && LidarDeployed() ? LidarRange(angle) : 0.0f;
        uint16_t aq = (uint16_t)lroundf(angle * 64.0f), dq = (uint16_t)lroundf(d * 4.0f);
        dst[n++] = (uint8_t)((15U << 2) | (lidar_index == 0U ? 1U : 2U));
        dst[n++] = (uint8_t)(((aq & 0x7FU) << 1) | 1U);
        dst[n++] = (uint8_t)(aq >> 7);
        dst[n++] = (uint8_t)dq;
        dst[n++] = (uint8_t)(dq >> 8);
        lidar_index = (lidar_index + 1U) % 500U;
    }
    return n;
}

/* ---------- 一拍：顺序和 StartChassisTask 一样，串口按 115200 的真实速度发 ---------- */
static uint32_t uart_done_ms, extra_busy_cycles;

static void Step(void)
{
    plant();
    if (uart.gState && ms >= uart_done_ms)
        uart.gState = 0;
    uint8_t was_idle = !uart.gState;
    ChassisCmd_Poll();
    Agv_Tick();
    if (ser_rx_on[SER_LIDAR]) // 接收关着的时候，雷达发的数据就丢了
        lidar_budget += 25U;
    Comp_Tick();
    if (was_idle && uart.gState)
        uart_done_ms = ms + tx_length * 87U / 1000U + 1U;
    host_clock.CYCCNT += extra_busy_cycles;
    Chassis_CycleEnd();
    robot_speed = hypotf(comp.x - last_x, comp.y - last_y) * 200.0f; // 一拍 5 ms
    last_x = comp.x;
    last_y = comp.y;
}

static void Steps(unsigned n)
{
    while (n--)
        Step();
}

static void Burst(const char *text)
{
    size_t n = strlen(text);
    while (n)
    {
        size_t chunk = n > CMD_BUF_SIZE ? CMD_BUF_SIZE : n;
        memcpy(rx_buf, text, chunk);
        event_type = chunk == n ? HAL_UART_RXEVENT_IDLE : 0U;
        HAL_UARTEx_RxEventCallback(&uart, (uint16_t)chunk);
        ChassisCmd_Poll();
        text += chunk;
        n -= chunk;
    }
    event_type = HAL_UART_RXEVENT_IDLE;
}

static void Line(const char *text)
{
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "%s\r\n", text);
    Burst(buffer);
}

/* 把缓冲里的文本全部发完。 */
static const char *Output(void)
{
    drain(&uart);
    return tx_log;
}

static void ClearOutput(void)
{
    drain(&uart);
    tx_log[0] = 0;
}

static unsigned CountOutput(const char *needle)
{
    unsigned count = 0U;
    for (const char *p = strstr(Output(), needle); p; p = strstr(p + 1, needle))
        ++count;
    return count;
}

/* 重新上电：底盘回到上电状态，比赛框架重新初始化。 */
static void Boot(unsigned obstacle_count, const float (*obstacles)[2])
{
    FDCAN_HandleTypeDef can = {0};
    Chassis_Stop();
    memset(motors, 0, sizeof(motors));
    memset(output, 0, sizeof(output));
    memset(cached, 0, sizeof(cached));
    memset(s_prev_deg, 0, sizeof(s_prev_deg));
    memset(s_prev_seq, 0, sizeof(s_prev_seq));
    memset(s_odom_sample_ms, 0, sizeof(s_odom_sample_ms));
    s_odom_valid = 0U;
    s_heading_integral = 0.0f;
    s_heading_hold = 0U;
    memset(&imu, 0, sizeof(imu));
    wheel[CHASSIS_WHEEL_LF] = NULL;
    assert(Chassis_Init(&can) == HAL_OK);
    tick(1);
    world_count = obstacle_count;
    if (obstacle_count)
        memcpy(world, obstacles, obstacle_count * sizeof(world[0]));
    lidar_on = lidar_descriptor = 0U;
    lidar_index = lidar_budget = lidar_commands = lidar_stops = 0U;
    memset(ser_reads, 0, sizeof(ser_reads));
    Agv_Init(AGV_MODE, boot_zone);
    assert(Comp_Init(boot_zone) == HAL_OK);
    Line("/reset");
    Line("/set cmd_speed 400");
    Line("D0");
    ClearOutput();
}

static void WaitAnchored(void)
{
    for (unsigned i = 0; i < 1000U && !comp.anchored; ++i)
        Step();
    assert(comp.anchored);
}

static void WaitScanEnd(void)
{
    for (unsigned i = 0; i < 2500U && comp.lidar_state < COMP_LIDAR_DONE; ++i)
        Step();
    assert(comp.lidar_state >= COMP_LIDAR_DONE);
}

/* 上电，等位姿定下来，在启停区扫一次，等雷达收好（没收好时规划要把伸着的雷达算进去）。 */
static void BootAndScan(unsigned obstacle_count, const float (*obstacles)[2])
{
    Boot(obstacle_count, obstacles);
    WaitAnchored();
    Line("/scan");
    WaitScanEnd();
    assert(comp.lidar_state == COMP_LIDAR_DONE);
    while (ms - servo_since < COMP_LIDAR_SERVO_MS)
        Step();
    ClearOutput();
}

/* 按行读 ROUTE 列表，中间夹着别的行也没关系。返回点数。 */
static unsigned ReadRoute(int points[][2], unsigned max)
{
    const char *p = strstr(Output(), "ROUTE points ");
    assert(p);
    unsigned total = 0U, got = 0U;
    assert(sscanf(p, "ROUTE points %u", &total) == 1 && total <= max);
    for (p = strchr(p, '\n') + 1; *p; p = strchr(p, '\n') + 1)
    {
        unsigned index;
        int x, y;
        if (strncmp(p, "ROUTE end\r\n", 11U) == 0)
            break;
        if (sscanf(p, "ROUTE %u %d %d", &index, &x, &y) == 3)
        {
            assert(index == got);
            points[got][0] = x;
            points[got][1] = y;
            ++got;
        }
    }
    assert(*p && got == total);
    return total;
}

static void PlanAndWait(const char *line)
{
    Line(line);
    for (unsigned i = 0; i < 4000U && comp.route_state == COMP_ROUTE_PLANNING; ++i)
        Step();
    Steps(120U); // 路线一拍一行，等列表发完
}

static void FollowToEnd(void)
{
    for (unsigned i = 0; i < 40000U && comp.route_state == COMP_ROUTE_FOLLOWING; ++i)
    {
        Step();
        assert(Map_CanPoseHeading(comp.x, comp.y, comp.heading_deg, 0.0f));
    }
}

/* ---------- 开局扫描 ---------- */
static const float kThree[3][2] = {{1200, 1700}, {1720, 1180}, {300, 1000}};

static void TestScanRunsOnceThenLeavesLidarAlone(void)
{
    Boot(3U, kThree);
    Line("D1");
    ClearOutput();
    /* 没启动就不扫：雷达转起来以后接收关着，什么都不处理。 */
    WaitAnchored();
    Steps(600U);
    assert(comp.lidar_state == COMP_LIDAR_PAUSED && !ser_rx_on[SER_LIDAR]);
    assert(lidar_obs.scans == 0U && comp.obstacles == 0U);
    unsigned idle_reads = ser_reads[SER_LIDAR];
    Steps(100U);
    assert(ser_reads[SER_LIDAR] == idle_reads);

    Line("/scan");
    WaitScanEnd();
    assert(comp.lidar_state == COMP_LIDAR_DONE && comp.anchored);
    assert(lidar_obs.scans > COMP_SCAN_REVS && lidar_obs.scans <= COMP_SCAN_REVS + 2U);
    assert(strstr(Output(), "COMP lidar=RUN ")); // 扫描期间能看到进度

    /* 结果输出一次，三个都认出来，位置准。 */
    assert(CountOutput("COMP obstacle #") == 3U);
    assert(CountOutput("COMP scan done: 3 obstacles\r\n") == 1U);
    assert(comp.obstacles == 3U);
    MapPoint list[MAP_MAX_OBSTACLES];
    assert(LidarObs_List(list, MAP_MAX_OBSTACLES) == 3U);
    for (unsigned j = 0; j < 3U; ++j)
    {
        float best = 1e9f;
        for (unsigned i = 0; i < 3U; ++i)
            best = fminf(best, hypotf((float)list[i].x - kThree[j][0], (float)list[i].y - kThree[j][1]));
        assert(best < 30.0f);
        uint16_t r, c;
        assert(Map_WorldToGrid((int32_t)kThree[j][0], (int32_t)kThree[j][1], &r, &c));
        assert(!Map_IsFree(r, c));
    }

    /* 之后就不管了：雷达关掉，不再读它，不再报它，地图不再变。 */
    assert(!lidar_on && lidar_stops >= 1U && !ser_rx_on[SER_LIDAR]);
    unsigned reads = ser_reads[SER_LIDAR], commands = lidar_commands;
    uint32_t revision = Map_Revision();
    world[world_count][0] = 1200.0f;
    world[world_count][1] = 500.0f;
    ++world_count;
    ClearOutput();
    Steps(2000U);
    assert(ser_reads[SER_LIDAR] == reads && lidar_commands == commands);
    assert(Map_Revision() == revision && !strstr(Output(), "COMP lidar="));
    assert(!strstr(tx_log, "COMP obstacle") && !strstr(tx_log, "COMP scan"));

    /* 要看结果就问一次。 */
    ClearOutput();
    Line("/obs");
    assert(CountOutput("  #") == 3U && CountOutput("OK   3 obstacles, scan done\r\n") == 1U);
}

static void TestScanGivesUpWithoutLidar(void)
{
    lidar_connected = 0U;
    Boot(0U, NULL);
    WaitAnchored();
    Line("/scan");
    for (unsigned i = 0; i < 6000U && comp.lidar_state < COMP_LIDAR_DONE; ++i)
        Step();
    assert(comp.lidar_state == COMP_LIDAR_FAILED);
    assert(CountOutput("COMP scan failed: no lidar data\r\n") == 1U && !ser_rx_on[SER_LIDAR]);
    unsigned commands = lidar_commands;
    ClearOutput();
    Steps(1000U);
    assert(lidar_commands == commands && !strstr(Output(), "COMP"));
    ClearOutput();
    Line("/obs");
    assert(CountOutput("OK   0 obstacles, scan failed\r\n") == 1U);
    lidar_connected = 1U;
}

/* 台架上摆完障碍物重新扫：旧结果清掉，只有明确发 /scan 才会再动雷达。 */
static void TestRescanIsExplicit(void)
{
    BootAndScan(3U, kThree);
    world_count = 1U;
    Line("/scan");
    assert(CountOutput("OK   scan restarted\r\n") == 1U);
    assert(comp.obstacles == 0U && ser_rx_on[SER_LIDAR]);
    WaitScanEnd();
    assert(comp.lidar_state == COMP_LIDAR_DONE && comp.obstacles == 1U);
    assert(CountOutput("COMP scan done: 1 obstacles\r\n") == 1U && !ser_rx_on[SER_LIDAR]);

    ClearOutput();
    Line("F500");
    Line("/scan");
    assert(CountOutput("BUSY") == 1U && comp.lidar_state == COMP_LIDAR_DONE);
    Line("S");
}

/* 扫描中途改位姿：之前的点都是按旧位姿算的，要从头扫。 */
static void TestPoseDuringScanRestartsIt(void)
{
    Boot(1U, kThree);
    WaitAnchored();
    Line("/scan");
    for (unsigned i = 0; i < 2000U && lidar_obs.scans < 5U; ++i)
        Step();
    assert(comp.lidar_state < COMP_LIDAR_DONE && lidar_obs.scans >= 5U);
    Line("/pose 2250 150 0");
    assert(lidar_obs.scans == 0U);
    WaitScanEnd();
    assert(comp.lidar_state == COMP_LIDAR_DONE && lidar_obs.scans > COMP_SCAN_REVS);
}

/* 舵机：平时收着，不会每拍都发；开局先伸出去，到位以前一个雷达点都不用，第一轮扫完车才走；走着雷达
   一直伸着、接着用点，扫完才收回来。/set 改角度马上转过去，台架上能直接看伸出去的位置对不对。 */
static void TestLidarServoMovesAroundScans(void)
{
    Boot(1U, kThree);
    WaitAnchored();
    servo_calls = 0U;
    Steps(100U);
    assert(servo_at == COMP_LIDAR_SERVO_STOW_DEG && servo_calls == 0U);
    Line("/start");
    unsigned points = lidar_obs.points;
    Steps(1U);
    assert(servo_at == COMP_LIDAR_SERVO_SCAN_DEG && lidar_obs.points == points);
    while (ms - servo_since < COMP_LIDAR_SERVO_MS)
    {
        assert(lidar_obs.points == points);
        Step();
    }
    while (comp.run_state == COMP_RUN_SCAN)
    {
        assert(chassis.mode == CHASSIS_MODE_VEL);
        Step();
    }
    assert(lidar_obs.points > points && comp.obstacles == 1U);
    for (unsigned i = 0; i < 2000U && chassis.mode != CHASSIS_MODE_PATH; ++i)
        Step();
    assert(comp.run_state == COMP_RUN_MOVE && chassis.mode == CHASSIS_MODE_PATH);
    assert(servo_at == COMP_LIDAR_SERVO_SCAN_DEG);
    points = lidar_obs.points;
    Steps(100U);
    assert(chassis.mode == CHASSIS_MODE_PATH && lidar_obs.points > points);
    for (unsigned i = 0; i < 40000U && Comp_Running(); ++i)
        Step();
    Steps(1U);
    assert(comp.run_state == COMP_RUN_DONE && comp.lidar_state == COMP_LIDAR_DONE);
    assert(servo_at == COMP_LIDAR_SERVO_STOW_DEG && servo_calls == 2U);

    Line("/set servo_stow 20");
    Steps(1U);
    assert(servo_at == 20.0f);
    Line("/set servo_scan 120");
    Line("/scan");
    Steps(1U);
    assert(servo_at == 120.0f);
    WaitScanEnd();
    assert(comp.lidar_state == COMP_LIDAR_DONE && comp.obstacles == 1U && servo_at == 20.0f);
    Line("/reset");
    Steps(1U);
    assert(servo_at == COMP_LIDAR_SERVO_STOW_DEG);
}

/* ---------- 开局流程 ---------- */
/* 雷达没确定收好（伸着或者正在转），它就在车身左后角外面。 */
static uint8_t LidarOut(void)
{
    return servo_at != lidar_tune.servo_stow || ms - servo_since < COMP_LIDAR_SERVO_MS;
}

/* 车身（270 见方）压到了真实的障碍物（包括雷达看不到的）吗；雷达伸着时，它（C1 外径 55.6）也不能碰
   障碍物、出场地。 */
static uint8_t HitsSomething(void)
{
    for (unsigned i = 0; i < world_count; ++i)
    {
        float dx = fmaxf(fabsf(world[i][0] - comp.x) - 135.0f, 0.0f);
        float dy = fmaxf(fabsf(world[i][1] - comp.y) - 135.0f, 0.0f);
        if (hypotf(dx, dy) < 25.0f)
            return 1U;
    }
    if (!LidarOut())
        return 0U;
    const float r = 27.8f, h = comp.heading_deg * MATH_DEG_TO_RAD;
    float lx = comp.x + cosf(h) * lidar_tune.lidar_x - sinf(h) * lidar_tune.lidar_y;
    float ly = comp.y + sinf(h) * lidar_tune.lidar_x + cosf(h) * lidar_tune.lidar_y;
    if (lx < r || ly < r || lx > MAP_WIDTH_MM - r || ly > MAP_HEIGHT_MM - r)
        return 1U;
    for (unsigned i = 0; i < world_count; ++i)
        if (hypotf(world[i][0] - lx, world[i][1] - ly) < 25.0f + r)
            return 1U;
    return 0U;
}

static void RunToEnd(void)
{
    for (unsigned i = 0; i < 40000U && Comp_Running(); ++i)
    {
        Step();
        assert(!HitsSomething());
    }
    assert(!Comp_Running());
}

/* 这三个都不挡路。开局先在启停区停着扫完一轮，然后伸着雷达边走边扫：走着的时候雷达接收开着，每拍都在读。
   换过三个位置、一直没有新目标就收雷达，结果和任务码都只输出一次。启停区 1 车头一直朝 +Y、不用转：往下去
   板子是后退，朝着左后角的雷达那边，没有看不清的路段。扫完时还在板前，读到码就结束，不去场地中央。 */
static void TestOpeningScansWhileDriving(void)
{
    static const float aside[3][2] = {{1200, 1800}, {300, 1000}, {600, 2000}};
    Boot(3U, aside);
    WaitAnchored();
    Steps(10U);
    ClearOutput();
    Line("/start");
    assert(comp.run_state == COMP_RUN_SCAN && chassis.mode == CHASSIS_MODE_VEL);
    assert(strstr(Output(), "RUN scan\r\n"));

    unsigned moving_ticks = 0U, moving_reads = 0U;
    for (unsigned i = 0; i < 40000U && Comp_Running(); ++i)
    {
        if (comp.run_state == COMP_RUN_SCAN)
            assert(chassis.mode == CHASSIS_MODE_VEL); // 第一轮扫完才走
        unsigned reads = ser_reads[SER_LIDAR];
        uint8_t moving = chassis.mode == CHASSIS_MODE_PATH;
        Step();
        if (moving && chassis.mode == CHASSIS_MODE_PATH && comp.lidar_state == COMP_LIDAR_RUN)
        {
            ++moving_ticks;
            moving_reads += ser_reads[SER_LIDAR] != reads;
        }
        assert(!HitsSomething());
    }
    assert(moving_ticks > 200U && moving_reads == moving_ticks);
    assert(comp.run_state == COMP_RUN_DONE && CountOutput("RUN done\r\n") == 1U);
    assert(CountOutput("RUN scan\r\n") == 1U);
    assert(CountOutput("COMP scan end: stable observations, places=3\r\n") == 1U);
    assert(CountOutput("COMP scan done: 3 obstacles\r\n") == 1U && CountOutput("COMP obstacle #") == 3U);
    assert(CountOutput("RUN move blind") == 0U && CountOutput("RUN move\r\n") >= 2U);
    assert(CountOutput("RUN turn") == 0U);
    assert(fabsf(Math_WrapDeg(comp.heading_deg - COMP_START_HEADING_DEG(1))) < 1.0f);
    assert(CountOutput("COMP task 156+123+516+231 from scanner") == 1U);
    assert(comp.vision_mode == VISION_OFF);
    assert(fabsf(comp.x - (float)COMP_QR_STOP_X_MM) < 4.0f && fabsf(comp.y - board_y) < SCANNER_WINDOW_MM);

    /* 雷达和扫码口都关了。 */
    assert(comp.lidar_state == COMP_LIDAR_DONE && !lidar_on && !ser_rx_on[SER_LIDAR]);
    assert(!ser_rx_on[SER_SCANNER]);
    unsigned lidar_reads = ser_reads[SER_LIDAR], scanner_reads = ser_reads[SER_SCANNER];
    uint32_t revision = Map_Revision();
    Steps(500U);
    assert(ser_reads[SER_LIDAR] == lidar_reads && ser_reads[SER_SCANNER] == scanner_reads);
    assert(Map_Revision() == revision);

    /* 再跑一遍是全新的一次：上次的障碍物作废，还要去二维码板前。 */
    world_count = 1U;
    ClearOutput();
    Line("/start");
    assert(comp.obstacles == 0U && comp.run_state == COMP_RUN_SCAN);
    uint16_t r, c;
    assert(Map_WorldToGrid(600, 2000, &r, &c) && Map_IsFree(r, c));
    float rightmost = 0.0f;
    for (unsigned i = 0; i < 40000U && Comp_Running(); ++i)
    {
        Step();
        rightmost = fmaxf(rightmost, comp.x);
    }
    assert(comp.run_state == COMP_RUN_DONE && comp.obstacles == 1U && rightmost > 2100.0f);
}

/* 中间横着的车道被 (1720,1180) 挡住了：不用为了到场地中央绕一大圈，边走边扫够了就在板前结束。 */
static void TestOpeningDoesNotDetourForTheCentre(void)
{
    Boot(3U, kThree);
    WaitAnchored();
    Line("/start");
    RunToEnd();
    assert(comp.run_state == COMP_RUN_DONE && comp.task_valid);
    assert(CountOutput("RUN scan\r\n") == 1U && CountOutput("COMP scan done: 3 obstacles\r\n") == 1U);
    assert(hypotf(comp.x - (float)COMP_CENTER_MM, comp.y - (float)COMP_CENTER_MM) > 300.0f);
    /* 没走完的路线不留着。 */
    assert(comp.route_state == COMP_ROUTE_NONE);
    ClearOutput();
    Line("/route");
    assert(strcmp(Output(), "ROUTE none\r\n") == 0);
}

/* 从启停区 1 的雷达 (2095,2095) 看，(265,1287) 完全在 (1200,1700) 后面；走着的时候换了视角就看到了。 */
static void TestOpeningFindsHiddenObstacle(void)
{
    static const float lined_up[2][2] = {{1200, 1700}, {265, 1287}};
    Boot(2U, lined_up);
    WaitAnchored();
    Line("/start");
    for (unsigned i = 0; i < 2000U && comp.run_state == COMP_RUN_SCAN; ++i)
        Step();
    assert(comp.obstacles == 1U); // 第一个位置确实只看到一个
    RunToEnd();
    assert(comp.run_state == COMP_RUN_DONE && comp.obstacles == 2U);
    assert(CountOutput("COMP scan done: 2 obstacles\r\n") == 1U);
}

/* 右边车道被 (2220,1850) 挡住，路线要往左绕：每走完一段在拐点重新规划。绕过去以后往右平移回车道去板子，
   雷达被车身挡着，这几段报 blind（其实之前看到过，固件只按这一段从哪儿出发、往哪边走来判断，宁可多报）。 */
static void TestOpeningDetoursWhenBlocked(void)
{
    static const float blocker[1][2] = {{2220, 1850}};
    Boot(1U, blocker);
    WaitAnchored();
    Line("/start");
    RunToEnd();
    assert(comp.run_state == COMP_RUN_DONE && comp.obstacles == 1U && comp.task_valid);
    assert(CountOutput("RUN scan\r\n") == 1U && CountOutput("RUN move\r\n") > 3U);
    assert(CountOutput("RUN move blind\r\n") >= 1U);
}

/* 走着的时候才看到挡路的障碍物（开局扫的时候它还被挡着，走近了才露出来）：停下来按新地图绕过去，
   接着跑完，不撞。车速放慢：一轮扫描（约 2 秒）里车走不远。 */
static void TestOpeningReplansAroundNewObstacle(void)
{
    Boot(0U, NULL);
    Line("/set cmd_speed 100");
    WaitAnchored();
    Line("/start");
    for (unsigned i = 0; i < 4000U && chassis.mode != CHASSIS_MODE_PATH; ++i)
        Step();
    assert(comp.run_state == COMP_RUN_MOVE && comp.y > 2200.0f);
    world[0][0] = 2250.0f;
    world[0][1] = comp.y - 600.0f; // 第一段路正前方
    world_count = 1U;
    RunToEnd();
    assert(comp.run_state == COMP_RUN_DONE && comp.task_valid && comp.obstacles == 1U);
    assert(CountOutput("ROUTE stopped blocked\r\n") == 1U);
}

/* 读不到任务码报失败时：从车停的位置沿着板子走，被障碍物挡着到不了正对板子中心的地方，才说得过去。 */
static uint8_t BoardWithinReach(void)
{
    int32_t x = (int32_t)lroundf(comp.x), y = (int32_t)lroundf(comp.y);
    int32_t step = board_y > comp.y ? MAP_CELL_MM : -MAP_CELL_MM;
    for (; fabsf((float)y - board_y) >= SCANNER_WINDOW_MM; y += step)
    {
        uint16_t r, c;
        if (!Map_WorldToGrid(x, y + step, &r, &c) || !Map_CanStand(r, c))
            return 0U;
    }
    return 1U;
}

typedef struct
{
    unsigned runs, placed, missed, done, blocked, unreadable, blind, hits;
} RandomStats;

/* 随机摆放的障碍物（板子也随机放），从启停区 zone 出发，真实固件从头跑到尾。 */
static RandomStats RunRandomLayouts(unsigned zone)
{
    RandomStats st = {0};
    uint32_t seed = 12345U;
    boot_zone = (uint8_t)zone;
    for (unsigned t = 0; t < 24U; ++t)
    {
        float layout[6][2];
        unsigned count = 3U + t % 4U;
        for (unsigned i = 0; i < count; ++i)
        {
            /* 只放在车道上，别贴着两个启停区，贴着也走不出来。 */
            uint8_t ok = 0U;
            while (!ok)
            {
                seed = seed * 1103515245U + 12345U;
                layout[i][0] = 110.0f + (float)((seed >> 8) % 2180U);
                seed = seed * 1103515245U + 12345U;
                layout[i][1] = 110.0f + (float)((seed >> 8) % 2120U);
                static const float box[6][4] = {{550, 550, 1000, 1000},  {1400, 550, 1850, 1000},
                                                {550, 1400, 1000, 1850}, {1400, 1400, 1850, 1850},
                                                {0, 910, 150, 1490},     {910, 0, 1490, 150}};
                ok = hypotf(layout[i][0] - 2250.0f, layout[i][1] - 2250.0f) > 400.0f &&
                     hypotf(layout[i][0] - 2250.0f, layout[i][1] - 150.0f) > 400.0f;
                for (unsigned b = 0; b < 6U; ++b)
                    if (layout[i][0] > box[b][0] - 25.0f && layout[i][0] < box[b][2] + 25.0f &&
                        layout[i][1] > box[b][1] - 25.0f && layout[i][1] < box[b][3] + 25.0f)
                        ok = 0U;
            }
        }
        board_y = (float)(COMP_QR_Y_MIN_MM + (int)((t * 53U) % 201U));
        Boot(count, layout);
        WaitAnchored();
        Line("/start");
        uint8_t hit = 0U;
        for (unsigned i = 0; i < 40000U && Comp_Running(); ++i)
        {
            Step();
            hit = hit || HitsSomething();
        }
        assert(!Comp_Running());
        ++st.runs;
        /* 雷达看清了的路段一定不撞；看不清的（报 blind）只能碰运气。 */
        uint8_t went_blind = CountOutput("RUN move blind") > 0U;
        assert(!hit || went_blind);
        st.blind += went_blind;
        st.hits += hit;
        if (comp.run_state != COMP_RUN_DONE)
        {
            /* 没跑完只能是这两种：去二维码板的路堵死了；或者障碍物挡在板子前面，车站不到正对板子的位置。 */
            if (strstr(Output(), "RUN failed task"))
            {
                assert(!BoardWithinReach());
                ++st.unreadable;
            }
            else
            {
                assert(strstr(Output(), "RUN failed route"));
                ++st.blocked;
            }
            continue;
        }
        ++st.done;
        MapPoint list[MAP_MAX_OBSTACLES];
        unsigned found = LidarObs_List(list, MAP_MAX_OBSTACLES);
        for (unsigned i = 0; i < count; ++i)
        {
            uint8_t seen = 0U;
            for (unsigned j = 0; j < found; ++j)
                seen = seen || hypotf((float)list[j].x - layout[i][0], (float)list[j].y - layout[i][1]) < 60.0f;
            ++st.placed;
            st.missed += !seen;
        }
    }
    board_y = 1200.0f;
    boot_zone = 1U;
    printf("random openings from zone %u: %u of %u finished, %u of %u obstacles missed; %u blocked, "
           "%u board out of reach; %u went blind, %u hit something\n",
           zone, st.done, st.runs, st.missed, st.placed, st.blocked, st.unreadable, st.blind, st.hits);
    assert(st.done + st.blocked + st.unreadable == st.runs);
    return st;
}

static void TestOpeningOnRandomLayouts(void)
{
    /* 启停区 1：去板子是后退，朝着雷达那边。没跑完的几组是障碍物挡在板子前面，假扫码模块只认正对的
       ±15 mm，车站不过去就读不到，报失败、停下是对的。绕障碍物时有的路段往右或往车头走，报 blind，
       这几组车身和伸着的雷达都没碰到东西。 */
    RandomStats one = RunRandomLayouts(1U);
    assert(one.done >= one.runs - 4U && one.missed == 0U && one.hits == 0U);
    /* 启停区 2：车头朝 +X 出发，去板子是往左平移，到了板前收起雷达原地转过来再读码。板前障碍物多、
       找不到转得开的地方时报 RUN failed route。 */
    RandomStats two = RunRandomLayouts(2U);
    assert(two.done >= two.runs - 4U && two.missed == 0U && two.hits == 0U);
}

/* 启停区 2 在板子下面，车头朝 +X 摆：左后角的雷达朝场地里面，往上去板子是往左平移，看得清。
   扫码模块这时朝下：到了板前原地转到 +Y 才读码，之后一直朝 +Y。转之前停下扫描把雷达收回来（伸着转会
   碰到板子），转完又伸出去接着扫。 */
static void TestZoneTwoTurnsAtTheBoard(void)
{
    boot_zone = 2U;
    Boot(0U, NULL);
    WaitAnchored();
    assert(fabsf(Math_WrapDeg(comp.heading_deg - COMP_START_HEADING_DEG(2))) < 0.01f);
    assert(strstr(Output(), "COMP pose anchored at (2250,150) heading 0\r\n"));
    Line("/start");
    float turn_x = 0.0f, turn_y = 0.0f;
    unsigned turning = 0U, spinning = 0U;
    uint8_t redeployed = 0U;
    for (unsigned i = 0; i < 40000U && Comp_Running(); ++i)
    {
        Step();
        assert(!HitsSomething());
        if (comp.run_state == COMP_RUN_TURN)
        {
            if (!turning++)
            {
                turn_x = comp.x;
                turn_y = comp.y;
            }
            assert(!comp.task_valid && hypotf(comp.x - turn_x, comp.y - turn_y) < 5.0f);
        }
        if (chassis.mode == CHASSIS_MODE_TURN)
        {
            ++spinning;
            assert(!LidarOut()); // 收好了才转
        }
        else if (spinning && servo_at == COMP_LIDAR_SERVO_SCAN_DEG)
            redeployed = 1U;
    }
    assert(comp.run_state == COMP_RUN_DONE && comp.task_valid);
    assert(spinning > 50U && redeployed && CountOutput("RUN turn\r\n") == 1U);
    assert(fabsf(turn_x - (float)COMP_QR_STOP_X_MM) < 4.0f && fabsf(turn_y - (float)COMP_QR_Y_MIN_MM) < 4.0f);
    assert(CountOutput("RUN move blind") == 0U && CountOutput("RUN scan\r\n") == 1U);
    assert(fabsf(Math_WrapDeg(comp.heading_deg - COMP_QR_HEADING_DEG)) < 1.0f);
    assert(hypotf(comp.x - (float)COMP_CENTER_MM, comp.y - (float)COMP_CENTER_MM) < 4.0f);
    boot_zone = 1U;
}

/* 启停区 2：(1965,1100) 离 1100 那个停车点 225 mm，站得下但转不开（转的时候车角扫过 191 mm）：
   往上找到第一个转得开的地方再转。 */
static void TestZoneTwoTurnsWhereThereIsRoom(void)
{
    static const float near_board[1][2] = {{1965, 1100}};
    boot_zone = 2U;
    Boot(1U, near_board);
    WaitAnchored();
    Line("/start");
    float turn_y = 0.0f;
    for (unsigned i = 0; i < 40000U && Comp_Running(); ++i)
    {
        Step();
        assert(!HitsSomething());
        if (comp.run_state == COMP_RUN_TURN && turn_y == 0.0f)
            turn_y = comp.y;
    }
    assert(comp.run_state == COMP_RUN_DONE && comp.task_valid && comp.obstacles == 1U);
    assert(turn_y > 1100.0f + 240.0f && CountOutput("RUN turn\r\n") == 1U);
    boot_zone = 1U;
}

static void TestRunLocksEverythingButStop(void)
{
    Boot(1U, kThree);
    ClearOutput();
    Line("/start"); // 底盘还没就绪，位姿不知道
    assert(CountOutput("BUSY") == 1U && comp.run_state == COMP_RUN_IDLE);
    WaitAnchored();
    Line("/start");
    ClearOutput();
    /* 规则禁止运行中遥控：能改车和参数的命令一律不接受。 */
    static const char *const refused[] = {
        "F100", "L100", "T90", "A0", "W", "/set heading_kp 3", "/reset", "/vision 2 1",
        "/task 156+123+516+231", "/task clear", "/scan", "/pose 100 100 0", "/route 300 300",
        "/route go", "/start"};
    float kp = chassis_tune.heading_kp;
    for (unsigned i = 0; i < sizeof(refused) / sizeof(refused[0]); ++i)
    {
        ClearOutput();
        Line(refused[i]);
        assert(CountOutput("BUSY") == 1U);
        assert(Comp_Running() && chassis.mode == CHASSIS_MODE_VEL);
    }
    assert(chassis_tune.heading_kp == kp && !comp.task_valid);

    /* 只看不改的可以。 */
    static const char *const allowed[] = {"/list heading_kp", "/obs", "/task", "/route", "/run", "/mode", "I", "D1"};
    for (unsigned i = 0; i < sizeof(allowed) / sizeof(allowed[0]); ++i)
    {
        ClearOutput();
        Line(allowed[i]);
        assert(CountOutput("BUSY") == 0U && CountOutput("ERR") == 0U && strlen(Output()) > 0U);
    }
    Line("D0");

    /* S 随时能停：扫描时。 */
    ClearOutput();
    Line("S");
    assert(comp.run_state == COMP_RUN_STOPPED && CountOutput("RUN stopped\r\n") == 1U);
    Steps(1U);
    assert(comp.lidar_state == COMP_LIDAR_PAUSED && !ser_rx_on[SER_LIDAR]);
    unsigned lidar_reads = ser_reads[SER_LIDAR];
    Steps(1000U);
    assert(comp.run_state == COMP_RUN_STOPPED && chassis.mode == CHASSIS_MODE_VEL);
    assert(ser_reads[SER_LIDAR] == lidar_reads && !strstr(Output(), "COMP scan"));

    /* 开动时。 */
    Line("/start");
    for (unsigned i = 0; i < 4000U && comp.run_state != COMP_RUN_MOVE; ++i)
        Step();
    assert(comp.run_state == COMP_RUN_MOVE);
    Steps(100U);
    assert(chassis.mode == CHASSIS_MODE_PATH);
    Line("S");
    assert(comp.run_state == COMP_RUN_STOPPED && chassis.mode == CHASSIS_MODE_VEL);
    Steps(1000U);
    assert(chassis.mode == CHASSIS_MODE_VEL);

    /* 停下来之后手动命令又能用了。 */
    ClearOutput();
    Line("F20");
    assert(strstr(Output(), "OK   path queued"));
    Line("S");
}

static void WaitSweep(void)
{
    for (unsigned i = 0; i < 40000U && comp.run_state != COMP_RUN_WAIT; ++i)
        Step();
    assert(comp.run_state == COMP_RUN_WAIT);
}

/* 启停区 1 从上面下来，找码从 1300 开始。板子在 1100，离得最远：到了停车点读不到，沿板慢慢往下找，
   读到就停下，不用走到头。这时边走边扫已经结束，读到码就收工，不去场地中央。 */
static void TestBoardSweepFindsTheFarEnd(void)
{
    board_y = (float)COMP_QR_Y_MIN_MM;
    Boot(0U, NULL);
    WaitAnchored();
    Line("/start");
    WaitSweep();
    assert(hypotf(comp.x - (float)COMP_QR_STOP_X_MM, comp.y - (float)COMP_QR_Y_MAX_MM) < 4.0f);
    assert(comp.vision_mode == VISION_QR); // 扫码模块没读到，相机也帮着找
    float fastest = 0.0f, read_y = 0.0f;
    for (unsigned i = 0; i < 40000U && Comp_Running(); ++i)
    {
        uint8_t sweeping = comp.run_state == COMP_RUN_WAIT;
        Step();
        assert(!HitsSomething());
        if (sweeping)
        {
            fastest = fmaxf(fastest, robot_speed);
            assert(fabsf(comp.x - (float)COMP_QR_STOP_X_MM) < 4.0f);
        }
        if (comp.task_valid && read_y == 0.0f)
            read_y = comp.y;
    }
    assert(comp.run_state == COMP_RUN_DONE && comp.task_valid);
    assert(fabsf(read_y - board_y) < SCANNER_WINDOW_MM);
    assert(fastest > 0.8f * COMP_QR_SWEEP_MM_S && fastest < 1.05f * COMP_QR_SWEEP_MM_S);
    assert(CountOutput("RUN wait\r\n") == 1U && CountOutput("RUN scan\r\n") == 1U);
    assert(CountOutput("RUN turn") == 0U); // 启停区 1 一直没转
    assert(comp.lidar_state == COMP_LIDAR_DONE && fabsf(comp.x - (float)COMP_QR_STOP_X_MM) < 4.0f &&
           fabsf(comp.y - read_y) < 20.0f);
    board_y = 1200.0f;
}

/* 板子正好在启停区 1 找码的起点 1300：一到停车点就读到了，不用沿板找。 */
static void TestBoardReadAtTheStop(void)
{
    board_y = (float)COMP_QR_Y_MAX_MM;
    Boot(0U, NULL);
    WaitAnchored();
    Line("/start");
    RunToEnd();
    assert(comp.run_state == COMP_RUN_DONE && comp.task_valid);
    assert(CountOutput("RUN wait\r\n") == 0U && CountOutput("COMP task 156+123+516+231") == 1U);
    board_y = 1200.0f;
}

/* 一直读不到：沿板来回找，不出 1100~1300、不超速，到时间报失败并停下；找的时候 S 也能停。 */
static void TestRunWithoutTaskCodeSweepsThenStops(void)
{
    const char *code = scanner_code;
    scanner_code = NULL;
    Boot(0U, NULL);
    WaitAnchored();
    Line("/start");
    WaitSweep();
    float top = comp.y, back = 1e9f;
    unsigned ticks = 0U;
    for (; ticks < 40000U && Comp_Running(); ++ticks)
    {
        Step();
        assert(fabsf(comp.x - (float)COMP_QR_STOP_X_MM) < 4.0f);
        assert(comp.y > COMP_QR_Y_MIN_MM - 4.0f && comp.y < COMP_QR_Y_MAX_MM + 4.0f);
        assert(robot_speed < 1.05f * COMP_QR_SWEEP_MM_S);
        top = fmaxf(top, comp.y);
        if (top > COMP_QR_Y_MAX_MM - 4.0f)
            back = fminf(back, comp.y);
    }
    assert(comp.run_state == COMP_RUN_FAILED && CountOutput("RUN failed task\r\n") == 1U);
    assert(ticks * 5U >= COMP_QR_TIMEOUT_MS && ticks * 5U <= COMP_QR_TIMEOUT_MS + 20U);
    assert(back < COMP_QR_Y_MIN_MM + 4.0f); // 从 1300 往下走到 1100 掉头，来回找
    assert(chassis.mode == CHASSIS_MODE_VEL && comp.route_state == COMP_ROUTE_NONE);
    float y = comp.y;
    Steps(400U);
    assert(fabsf(comp.y - y) < 0.5f);

    Line("/start");
    WaitSweep();
    Steps(200U);
    assert(chassis.mode == CHASSIS_MODE_PATH);
    Line("S");
    assert(comp.run_state == COMP_RUN_STOPPED && chassis.mode == CHASSIS_MODE_VEL);
    y = comp.y;
    Steps(400U);
    assert(fabsf(comp.y - y) < 0.5f && comp.run_state == COMP_RUN_STOPPED);

    /* 扫码模块好了，重新开始就能跑完。 */
    Line("/start");
    for (unsigned i = 0; i < 40000U && comp.run_state != COMP_RUN_SCAN; ++i)
        Step();
    scanner_code = code;
    RunToEnd();
    assert(comp.run_state == COMP_RUN_DONE);
}

/* 底盘故障：扫描和规划时也要发现，不能一直等下去。 */
static void TestRunEndsOnChassisFault(void)
{
    Boot(0U, NULL);
    WaitAnchored();
    Line("/start");
    Steps(5U);
    assert(comp.run_state == COMP_RUN_SCAN);
    Chassis_Fault(CHASSIS_FAULT_SENSOR, 0U);
    ClearOutput();
    Steps(2U);
    assert(comp.run_state == COMP_RUN_FAILED && CountOutput("RUN failed chassis\r\n") == 1U);
    unsigned lidar_reads = ser_reads[SER_LIDAR];
    Steps(500U);
    assert(ser_reads[SER_LIDAR] == lidar_reads && comp.run_state == COMP_RUN_FAILED);
}

/* 雷达坏了也要接着跑：只等一次超时，后面不再试。 */
static void TestRunWithoutLidar(void)
{
    lidar_connected = 0U;
    Boot(0U, NULL);
    WaitAnchored();
    Line("/start");
    RunToEnd();
    assert(comp.run_state == COMP_RUN_DONE && comp.lidar_state == COMP_LIDAR_FAILED);
    /* 雷达坏了，换位置也没用，沿板读到码就结束。一路都没看清，都报 blind。 */
    assert(fabsf(comp.x - (float)COMP_QR_STOP_X_MM) < 4.0f && fabsf(comp.y - board_y) < SCANNER_WINDOW_MM);
    assert(CountOutput("COMP scan failed: no lidar data\r\n") == 1U && CountOutput("RUN scan\r\n") == 1U);
    assert(!strstr(Output(), "COMP scan done") && CountOutput("RUN move\r\n") == 0U);
    lidar_connected = 1U;
}

/* 找码的起点 1100 被障碍物挡了：从找码范围里能站的地方开始找；场地中央被占了：不去了。 */
static void TestRunAdaptsToBlockedStops(void)
{
    static const float on_stops[2][2] = {{2205, 1000}, {1205, 1205}};
    Boot(2U, on_stops);
    WaitAnchored();
    Line("/start");
    RunToEnd();
    assert(comp.run_state == COMP_RUN_DONE && comp.task_valid);
    assert(comp.obstacles == 2U && hypotf(comp.x - 1205.0f, comp.y - 1205.0f) > 100.0f);
}

/* ---------- 路线 ---------- */
static void TestPlanPrintsRouteAndWalksIt(void)
{
    BootAndScan(0U, NULL);
    PlanAndWait("/route 300 300");
    assert(comp.route_state == COMP_ROUTE_READY);
    assert(CountOutput("ROUTE planning\r\n") == 1U && CountOutput("ROUTE ready\r\n") == 1U);
    int points[COMP_ROUTE_MAX][2];
    unsigned n = ReadRoute(points, COMP_ROUTE_MAX);
    assert(n >= 2U && n <= 6U); // 只留拐点，不是几百个栅格
    assert(points[0][0] == 2250 && points[0][1] == 2250);
    assert(points[n - 1U][0] == 300 && points[n - 1U][1] == 300); // 终点是目标本身，不是栅格中心
    for (unsigned i = 1; i < n; ++i)
        assert(abs(points[i][0] - points[i - 1U][0]) < 10 ||
               abs(points[i][1] - points[i - 1U][1]) < 10);

    /* 规划只是看，车不动。 */
    assert(chassis.mode == CHASSIS_MODE_VEL);
    Line("/set cmd_speed 350");
    ClearOutput();
    Line("/route go");
    assert(CountOutput("ROUTE following\r\n") == 1U);
    assert(comp.route_state == COMP_ROUTE_FOLLOWING && chassis.mode == CHASSIS_MODE_PATH);
    assert(fabsf(Chassis_SpeedLimit() - 350.0f) < 0.01f); // 用 /set cmd_speed 的速度
    FollowToEnd();
    assert(comp.route_state == COMP_ROUTE_DONE && CountOutput("ROUTE done\r\n") == 1U);
    assert(chassis_path_result.completed && chassis.mode == CHASSIS_MODE_VEL);
    assert(hypotf(comp.x - 300.0f, comp.y - 300.0f) <= chassis_tune.distance_tol + 0.5f);
    assert(chassis_path_result.max_cross_mm <= CHASSIS_PATH_DEVIATION_MM);
    /* 走完不能再走一遍：车已经不在起点。 */
    ClearOutput();
    Line("/route go");
    assert(CountOutput("ERR  no route ready") == 1U);
}

static void TestRouteGoesAroundScannedObstacles(void)
{
    BootAndScan(3U, kThree);
    PlanAndWait("/route 1200 1200");
    assert(comp.route_state == COMP_ROUTE_READY);
    Line("/route go");
    FollowToEnd();
    assert(comp.route_state == COMP_ROUTE_DONE);
    assert(hypotf(comp.x - 1200.0f, comp.y - 1200.0f) <= chassis_tune.distance_tol + 0.5f);
}

/* 从任意直角航向出发都按这个航向走，到点时航向不变。 */
static void TestRouteHoldsRightAngleHeading(void)
{
    BootAndScan(0U, NULL);
    Line("/pose 1200 1200 90");
    Steps(2U);
    PlanAndWait("/route 1200 2000");
    assert(comp.route_state == COMP_ROUTE_READY);
    Line("/route go");
    FollowToEnd();
    assert(comp.route_state == COMP_ROUTE_DONE && hypotf(comp.x - 1200.0f, comp.y - 2000.0f) < 4.0f);
    assert(fabsf(Math_WrapDeg(comp.heading_deg - 90.0f)) <= chassis_tune.heading_tol + 0.05f);
}

static void TestPlanRefusals(void)
{
    BootAndScan(0U, NULL);
    static const struct
    {
        const char *pose, *route, *reply;
    } cases[] = {
        {"/pose 2250 2250 0", "/route 700 700", "ROUTE failed goal\r\n"},    // 黄色区域里
        {"/pose 700 700 0", "/route 300 300", "ROUTE failed start\r\n"},     // 车在禁区里
        {"/pose 1200 1200 45", "/route 300 300", "ROUTE failed heading\r\n"}, // 车身不正，构型空间不适用
    };
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i)
    {
        Line(cases[i].pose);
        Steps(2U);
        ClearOutput();
        Line(cases[i].route);
        assert(CountOutput(cases[i].reply) == 1U && comp.route_state == COMP_ROUTE_FAILED);
        ClearOutput();
        Line("/route");
        Steps(5U);
        assert(CountOutput(cases[i].reply) == 1U); // 问一次也能看到原因
        assert(!strstr(tx_log, "ROUTE points"));   // 失败的路线不留着
    }

    Line("/pose 2250 2250 0");
    Steps(2U);
    static const char *const bad[] = {"/route 1200", "/route 2400 100", "/route 100 -1",
                                      "/route x 100", "/route 1.5 100", "/route go now"};
    for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i)
    {
        ClearOutput();
        Line(bad[i]);
        assert(CountOutput("ERR  usage: /route") == 1U);
    }
    ClearOutput();
    Line("/route go");
    assert(CountOutput("ERR  no route ready") == 1U);

    /* 车在动：不能规划，也不能开始。 */
    ClearOutput();
    Line("F500");
    Line("/route 300 300");
    Line("/route go");
    assert(CountOutput("BUSY") == 2U && comp.route_state != COMP_ROUTE_PLANNING);
    float x = comp.x;
    ClearOutput();
    Line("/pose 100 100 0");
    Line("/start");
    Steps(1U);
    assert(CountOutput("BUSY") == 2U && comp.run_state == COMP_RUN_IDLE && fabsf(comp.x - x) < 20.0f);
    Line("S");
}

/* A* 分很多拍做完：一拍只做一点，不能占控制周期。全场清表就要一百多拍。 */
static void TestPlanningIsSpreadOverTicks(void)
{
    BootAndScan(0U, NULL);
    Line("/route 300 300");
    unsigned ticks = 0U;
    while (comp.route_state == COMP_ROUTE_PLANNING && ticks < 4000U)
    {
        Step();
        ++ticks;
    }
    assert(comp.route_state == COMP_ROUTE_READY && ticks > 100U);
}

/* 里程计突然跳了（打滑）：偏离路线太多要停车，不能接着开。 */
static void TestLargeCrossTrackStops(void)
{
    BootAndScan(0U, NULL);
    PlanAndWait("/route 300 300");
    Line("/route go");
    Steps(100U);
    assert(comp.route_state == COMP_ROUTE_FOLLOWING);
    chassis.pos_x += 50.0f; // 两个方向都跳：不管这段沿哪个轴走，都有 50 mm 横偏
    chassis.pos_y += 50.0f;
    ClearOutput();
    Steps(5U);
    assert(comp.route_state == COMP_ROUTE_STOPPED && CountOutput("ROUTE stopped cross\r\n") == 1U);
    assert(chassis.mode == CHASSIS_MODE_VEL);
}

static void TestNoPathIsReported(void)
{
    BootAndScan(0U, NULL);
    /* 左下角只有两条路出去，各放一个障碍物。 */
    assert(Map_AddObstacle(700, 275, 35) && Map_AddObstacle(275, 700, 35));
    PlanAndWait("/route 300 300");
    assert(comp.route_state == COMP_ROUTE_FAILED && CountOutput("ROUTE failed nopath\r\n") == 1U);
    assert(!strstr(tx_log, "ROUTE points"));
}

/* 规划途中地图变了：旧搜索作废，自动重新搜。 */
static void TestMapChangeDuringPlanningReplans(void)
{
    BootAndScan(0U, NULL);
    Line("/route 300 300");
    Steps(3U);
    assert(comp.route_state == COMP_ROUTE_PLANNING);
    assert(Map_AddObstacle(1200, 1200, 35));
    for (unsigned i = 0; i < 4000U && comp.route_state == COMP_ROUTE_PLANNING; ++i)
        Step();
    assert(comp.route_state == COMP_ROUTE_READY && CountOutput("ROUTE planning\r\n") == 2U);
}

static void TestMovedAfterPlanRefusesGo(void)
{
    BootAndScan(0U, NULL);
    PlanAndWait("/route 300 300");
    Line("F50");
    while (chassis.mode != CHASSIS_MODE_VEL)
        Step();
    ClearOutput();
    Line("/route go");
    assert(CountOutput("ROUTE failed moved\r\n") == 1U && chassis.mode == CHASSIS_MODE_VEL);
    ClearOutput();
    Line("/route");
    Steps(5U);
    assert(CountOutput("ROUTE failed moved\r\n") == 1U && !strstr(tx_log, "ROUTE points"));

    BootAndScan(0U, NULL);
    PlanAndWait("/route 300 300");
    Line("/pose 2100 2100 0");
    Steps(2U);
    ClearOutput();
    Line("/route go");
    assert(CountOutput("ROUTE failed moved\r\n") == 1U);

    BootAndScan(0U, NULL);
    PlanAndWait("/route 300 300");
    Line("/pose 2250 2250 45");
    Steps(2U);
    ClearOutput();
    Line("/route go");
    assert(CountOutput("ROUTE failed moved\r\n") == 1U);

    /* 规划后多了障碍物把路挡了，开走前要发现。 */
    BootAndScan(0U, NULL);
    PlanAndWait("/route 300 300");
    int points[COMP_ROUTE_MAX][2];
    unsigned n = ReadRoute(points, COMP_ROUTE_MAX);
    assert(Map_AddObstacle(points[n - 1U][0] + 200, points[n - 1U][1], 35));
    ClearOutput();
    Line("/route go");
    assert(CountOutput("ROUTE failed blocked\r\n") == 1U && chassis.mode == CHASSIS_MODE_VEL);
}

static void TestStopWhileFollowing(void)
{
    BootAndScan(0U, NULL);
    PlanAndWait("/route 300 300");
    Line("/route go");
    Steps(100U);
    ClearOutput();
    Line("S");
    Steps(2U);
    assert(comp.route_state == COMP_ROUTE_STOPPED && CountOutput("ROUTE stopped\r\n") == 1U);
    assert(chassis.mode == CHASSIS_MODE_VEL);
    ClearOutput();
    Line("/route go");
    assert(CountOutput("ERR  no route ready") == 1U);
    /* 停下后重新规划就能接着走。 */
    PlanAndWait("/route 300 300");
    Line("/route go");
    FollowToEnd();
    assert(comp.route_state == COMP_ROUTE_DONE);
}

/* 跟着走的时候地图变了（/scan 不能在动的时候发，这里模拟比赛流程以后可能的来源）：
   挡在前面才停，在身后或别处不停。 */
static void TestMapChangeWhileFollowing(void)
{
    BootAndScan(0U, NULL);
    PlanAndWait("/route 300 300");
    int points[COMP_ROUTE_MAX][2];
    unsigned n = ReadRoute(points, COMP_ROUTE_MAX);
    assert(n >= 3U);
    Line("/route go");
    for (unsigned i = 0; i < 20000U && Chassis_PathTarget() < 2U; ++i)
        Step();
    assert(Chassis_PathTarget() == 2U);
    /* 离开拐点足够远以后，拐点就在身后了。 */
    for (unsigned i = 0; i < 2000U && hypotf(comp.x - (float)points[1][0], comp.y - (float)points[1][1]) < 400.0f; ++i)
        Step();
    assert(Map_AddObstacle(points[1][0], points[1][1], 35));
    Steps(5U);
    assert(comp.route_state == COMP_ROUTE_FOLLOWING && chassis.mode == CHASSIS_MODE_PATH);
    /* 前面最后一段的中点。 */
    ClearOutput();
    assert(Map_AddObstacle((points[n - 2U][0] + points[n - 1U][0]) / 2,
                           (points[n - 2U][1] + points[n - 1U][1]) / 2, 35));
    Steps(2U);
    assert(comp.route_state == COMP_ROUTE_STOPPED && CountOutput("ROUTE stopped blocked\r\n") == 1U);
    assert(chassis.mode == CHASSIS_MODE_VEL);
}

static void TestRouteReportOnRequest(void)
{
    BootAndScan(0U, NULL);
    Line("/route");
    assert(CountOutput("ROUTE none\r\n") == 1U && !strstr(tx_log, "ROUTE points"));
    PlanAndWait("/route 300 300");
    int first[COMP_ROUTE_MAX][2], second[COMP_ROUTE_MAX][2];
    unsigned n = ReadRoute(first, COMP_ROUTE_MAX);
    ClearOutput();
    Line("/route"); // 中途连上的调试台靠这个拿到路线
    Steps(60U);
    assert(CountOutput("ROUTE ready\r\n") == 1U);
    assert(ReadRoute(second, COMP_ROUTE_MAX) == n && memcmp(first, second, n * sizeof(first[0])) == 0);
}

/* ---------- 文本输出不能影响控制 ---------- */
static MapPoint FakePoint(unsigned index)
{
    return (MapPoint){(int32_t)index, (int32_t)index * 2};
}

/* 路线再长，一拍也只多一行；发送缓冲过半就等，别的文本有地方放。 */
static void TestRouteLinesArePaced(void)
{
    BootAndScan(0U, NULL);
    ChassisTelemetry_SendRoute(200U, FakePoint);
    unsigned most = 0U;
    for (unsigned i = 0; i < 400U; ++i)
    {
        unsigned before = tx_head;
        Step();
        unsigned added = (tx_head + TX_RING_SIZE - before) % TX_RING_SIZE;
        if (added > most)
            most = added;
    }
    assert(most > 0U && most <= 24U);
    assert(CountOutput("ROUTE end\r\n") == 1U);
    int points[200][2];
    assert(ReadRoute(points, 200U) == 200U && points[199][0] == 199 && points[199][1] == 398);

    /* 缓冲已经过半：一行都不加，等它发出去。 */
    ClearOutput();
    uart.gState = 1;
    uart_done_ms = ms + 100U;
    static char filler[1200];
    memset(filler, 'x', sizeof(filler) - 1U);
    ChassisTelemetry_Puts(filler);
    ChassisTelemetry_SendRoute(10U, FakePoint);
    unsigned before = tx_head;
    Steps(5U);
    assert(tx_head == before);
    ChassisTelemetry_Puts("\r\n");
    uart.gState = 0;
    Steps(800U);
    assert(strstr(Output(), "ROUTE end\r\n"));
}

/* 状态行、曲线和路线一起发也不超过串口带宽，不丢任何一行。 */
static void TestAllOutputFitsTheLink(void)
{
    BootAndScan(0U, NULL);
    Line("D10");
    Line("/plot wheels 100");
    PlanAndWait("/route 300 300");
    unsigned long dropped = tx_dropped;
    Line("/route go");
    ChassisTelemetry_SendRoute(200U, FakePoint);
    for (unsigned i = 0; i < 2000U && comp.route_state == COMP_ROUTE_FOLLOWING; ++i)
        Step();
    assert(tx_dropped == dropped);
    Line("/plot off");
    Line("S");
}

static uint32_t MixDigest(uint32_t hash, const void *data, size_t size)
{
    const uint8_t *bytes = data;
    for (size_t i = 0; i < size; ++i)
        hash = (hash ^ bytes[i]) * 16777619U;
    return hash;
}

/* 同一条路线跑两遍，一遍完全不输出，一遍把所有输出开到最大、再不断查询：
   每一拍下发的四轮转速和位姿必须逐位相同。 */
static uint32_t RunRoute(uint8_t noisy, chassis_path_stats *stats, float *end)
{
    BootAndScan(0U, NULL);
    PlanAndWait("/route 300 300");
    if (noisy)
    {
        Line("D10");
        Line("/plot wheels 100");
    }
    Line("/route go");
    uint32_t hash = 2166136261U;
    for (unsigned i = 0; i < 40000U && comp.route_state == COMP_ROUTE_FOLLOWING; ++i)
    {
        Step();
        hash = MixDigest(hash, output, sizeof(output));
        hash = MixDigest(hash, &chassis.pos_x, sizeof(chassis.pos_x));
        hash = MixDigest(hash, &chassis.pos_y, sizeof(chassis.pos_y));
        hash = MixDigest(hash, &chassis.yaw, sizeof(chassis.yaw));
        if (noisy && i % 200U == 0U)
        {
            static const char *const queries[] = {"/route", "/list", "I", "H", "/obs", "/task"};
            Line(queries[i / 200U % (sizeof(queries) / sizeof(queries[0]))]);
        }
    }
    assert(comp.route_state == COMP_ROUTE_DONE);
    *stats = chassis_path_result;
    end[0] = chassis.pos_x;
    end[1] = chassis.pos_y;
    end[2] = chassis.yaw;
    Line("/plot off");
    ClearOutput();
    return hash;
}

static void TestOutputDoesNotChangeMotion(void)
{
    chassis_path_stats quiet, noisy;
    float end_quiet[3], end_noisy[3];
    uint32_t a = RunRoute(0U, &quiet, end_quiet);
    uint32_t b = RunRoute(1U, &noisy, end_noisy);
    assert(a == b);
    assert(memcmp(&quiet, &noisy, sizeof(quiet)) == 0);
    assert(memcmp(end_quiet, end_noisy, sizeof(end_quiet)) == 0);
    printf("route with all output off vs on: identical, %.2f s, max_cross %.2f mm\n",
           (double)quiet.elapsed_ms * 0.001, (double)quiet.max_cross_mm);
}

/* 每条路径报告这段时间的节拍偏差和单拍占用，实车上用这两个数确认输出没有打乱控制周期。 */
static void TestPathReportsCycleStats(void)
{
    BootAndScan(0U, NULL);
    Line("F20");
    Steps(5U);
    extra_busy_cycles = 480000U; // 1 ms，这一拍占用 1 ms，下一拍晚 1 ms
    Step();
    extra_busy_cycles = 0U;
    for (unsigned i = 0; i < 2000U && chassis.mode != CHASSIS_MODE_VEL; ++i)
        Step();
    assert(chassis_path_result.completed);
    assert(chassis_path_result.max_loop_us == 1000U && chassis_path_result.max_jitter_us == 1000U);
    Steps(2U);
    assert(strstr(Output(), "max_heading=") && strstr(tx_log, "deg jitter=1000us loop=1000us\r\n"));
    ClearOutput();
    Line("I");
    assert(strstr(Output(), " loop=1000us "));
    /* 下一条路径重新统计。 */
    Line("B20");
    for (unsigned i = 0; i < 2000U && chassis.mode != CHASSIS_MODE_VEL; ++i)
        Step();
    assert(chassis_path_result.completed && chassis_path_result.max_loop_us == 0U);
    assert(chassis_path_result.max_jitter_us == 0U);
}

/* 调试台靠这些文本工作，格式不能变。 */
static void TestTextFormatsForUi(void)
{
    BootAndScan(0U, NULL);
    Line("/mode");
    assert(CountOutput("OK   mode COMPETITION\r\n") == 1U);
    Line("/pose 1234 567 90");
    Steps(1U);
    Line("D1");
    ClearOutput();
    Steps(205U);
    assert(strstr(Output(), "\r\n          POSE (1234.0,567.0,90.00)\r\n"));
    assert(!strstr(tx_log, "AGV state=")); // 巡检才有
    assert(strstr(tx_log, "COMP vision=OFF mode=0 mat=0 ring=0 crc=0\r\n"));
    assert(strstr(tx_log, "COMP task=none rej=0 scan=0/0\r\n"));
    assert(!strstr(tx_log, "COMP lidar=")); // 扫完了
    ClearOutput();
    Line("/run");
    assert(strcmp(Output(), "RUN idle\r\n") == 0);

    /* D0 关掉所有周期输出。 */
    Line("D0");
    ClearOutput();
    Steps(1000U);
    assert(strlen(Output()) == 0U);

    Boot(0U, NULL);
    WaitAnchored();
    Line("D1");
    Line("/scan");
    ClearOutput();
    Steps(205U);
    unsigned long revs, revs_total, pts, used, clu, restarts, resync;
    char state[8];
    float hz;
    const char *line = strstr(Output(), "COMP lidar=RUN ");
    assert(line);
    assert(sscanf(line, "COMP lidar=%7s %fHz revs=%lu/%lu pts=%lu used=%lu clu=%lu rs=%lu/%lu", state, &hz,
                  &revs, &revs_total, &pts, &used, &clu, &restarts, &resync) == 9);
    assert(revs_total == COMP_SCAN_REVS);
}

int main(void)
{
    FDCAN_HandleTypeDef can = {0};
    assert(Chassis_Init(&can) == HAL_OK);
    tick(1);
    assert(ChassisCmd_Init(&uart) == HAL_OK);
    TestScanRunsOnceThenLeavesLidarAlone();
    TestScanGivesUpWithoutLidar();
    TestRescanIsExplicit();
    TestPoseDuringScanRestartsIt();
    TestLidarServoMovesAroundScans();
    TestOpeningScansWhileDriving();
    TestOpeningDoesNotDetourForTheCentre();
    TestOpeningFindsHiddenObstacle();
    TestOpeningDetoursWhenBlocked();
    TestOpeningReplansAroundNewObstacle();
    TestOpeningOnRandomLayouts();
    TestZoneTwoTurnsAtTheBoard();
    TestZoneTwoTurnsWhereThereIsRoom();
    TestRunLocksEverythingButStop();
    TestBoardSweepFindsTheFarEnd();
    TestBoardReadAtTheStop();
    TestRunWithoutTaskCodeSweepsThenStops();
    TestRunEndsOnChassisFault();
    TestRunWithoutLidar();
    TestRunAdaptsToBlockedStops();
    TestPlanPrintsRouteAndWalksIt();
    TestRouteGoesAroundScannedObstacles();
    TestRouteHoldsRightAngleHeading();
    TestPlanRefusals();
    TestPlanningIsSpreadOverTicks();
    TestLargeCrossTrackStops();
    TestNoPathIsReported();
    TestMapChangeDuringPlanningReplans();
    TestMovedAfterPlanRefusesGo();
    TestStopWhileFollowing();
    TestMapChangeWhileFollowing();
    TestRouteReportOnRequest();
    TestRouteLinesArePaced();
    TestAllOutputFitsTheLink();
    TestOutputDoesNotChangeMotion();
    TestPathReportsCycleStats();
    TestTextFormatsForUi();
    puts("PASS: scan on demand, opening sequence on hidden and random layouts, run lock, route "
         "planning and following, output never changes motion");
    return 0;
}
