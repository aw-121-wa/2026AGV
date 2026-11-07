/* 调参日志的主机测试：真实的底盘控制、命令解析和日志代码，车换成有延迟、惯性、打滑的模型，
   串口按 115200 的速度出字节。最后照调试台“自动调参”的那组动作跑几遍，串口输出写到 argv[1]，
   tools/pid_tune_test.js 用它检查辨识出来的模型和这里的假车对不对得上。 */
#define main original_chassis_tests
#include "chassis_test.c"
#undef main

static UART_HandleTypeDef uart;

/* ---------- 假车 ---------- */
/* 下发转速先晚 delay 拍，再按时间常数 tau 一阶惯性到轮子（驱动器、电机、车身惯量）；
   车身转速 = slip × 轮子算出来的转速 + 平移带出来的扰动（cx、cy：deg/s 每 mm/s）；
   陀螺仪按 HWT101 的分辨率量化，再加噪声。 */
typedef struct
{
    float slip, tau, cx, cy, noise;
    unsigned delay;
} fake_car;

#define DELAY_MAX 16U
static fake_car car = {1.0f, 0.005f, 0.0f, 0.0f, 0.0f, 0U};
static float sent[DELAY_MAX][4], wheel_rpm[4];
static unsigned sent_head;
static double yaw_true;
static uint32_t rng = 12345U;

static float Gauss(void)
{
    float sum = 0.0f; // 12 个均匀数相加近似正态
    for (int i = 0; i < 12; ++i)
    {
        rng = rng * 1664525U + 1013904223U;
        sum += (float)(rng >> 8) / 16777216.0f;
    }
    return sum - 6.0f;
}

static void UseCar(fake_car c)
{
    car = c;
    yaw_true = imu.yaw;
}

static void CarStep(void)
{
    const float *u = sent[(sent_head + DELAY_MAX - car.delay) % DELAY_MAX];
    float a = 1.0f - expf(-0.005f / car.tau), v[4];
    for (int i = 0; i < 4; ++i)
    {
        wheel_rpm[i] += (u[i] - wheel_rpm[i]) * a;
        motors[i].speed_rpm = wheel_rpm[i];
        motors[i].position_deg += wheel_rpm[i] * 6.0f * 0.005f;
        v[i] = wheel_rpm[i] * 3.14159265f * CHASSIS_WHEEL_DIA_MM / 60.0f;
    }
    float vx = 0.25f * (v[0] + v[1] + v[2] + v[3]);
    float vy = 0.25f * (-v[0] + v[1] + v[2] - v[3]);
    float wz = (-v[0] + v[1] - v[2] + v[3]) / (4 * CHASSIS_K_MM) * 57.2957795f;
    yaw_true += (car.slip * wz + car.cx * vx + car.cy * vy) * 0.005;
    const float q = 180.0f / 32768.0f;
    imu.yaw = Math_WrapDeg(q * roundf(((float)yaw_true + car.noise * Gauss()) / q));
}

/* ---------- 串口：所有输出都收下来；一块按 115200 波特发完才算发完 ---------- */
static char out[1 << 22];
static size_t out_len;
static uint32_t uart_free_ms;

static void Collect(void)
{
    size_t n = strlen(tx_log);
    if (!n)
        return;
    assert(out_len + n < sizeof(out));
    memcpy(out + out_len, tx_log, n + 1U);
    out_len += n;
    tx_log[0] = '\0';
    uart_free_ms = ms + (uint32_t)((n * 10U * 1000U + 115199U) / 115200U);
}

static void Send(const char *line)
{
    command(&uart, line);
    Collect();
}

/* 和固件的底盘任务同一个顺序；fresh = 0 时这一拍收不到反馈。 */
static void Step(int fresh)
{
    CarStep();
    ms += 5;
    host_clock.CYCCNT += 2400000;
    if (fresh)
        feedback();
    Chassis_Update();
    TuneLog_Tick();
    sent_head = (sent_head + 1U) % DELAY_MAX;
    memcpy(sent[sent_head], output, sizeof(output));
    if (uart.gState && ms >= uart_free_ms)
        uart.gState = 0U;
    ChassisCmd_Poll();
    Collect();
}

/* 一直走到输出里出现 text，返回它的位置；超过 max_steps 拍算失败。 */
static const char *RunUntil(size_t from, const char *text, unsigned max_steps)
{
    for (unsigned i = 0; i <= max_steps; ++i)
    {
        const char *at = strstr(out + from, text);
        if (at)
            return at;
        Step(1);
    }
    fprintf(stderr, "no '%s' after %u steps\n", text, max_steps);
    exit(1);
}

static unsigned Count(size_t from, const char *text)
{
    unsigned n = 0U;
    for (const char *p = strstr(out + from, text); p; p = strstr(p + 1, text))
        ++n;
    return n;
}

/* ---------- 读日志：只认 LOG 和 L 行，中间夹着的其他输出跳过 ---------- */
typedef struct
{
    unsigned n, trunc, imu, lines, tail;
    char mode[8], end[8];
    float kp, ki;
    long first_target, last_yaw;
    unsigned first_mode, modes; // modes：模式变了几次
    int in_order;
} parsed_log;

/* 从 from 开始找第一段完整的日志，返回这段后面的位置；没有完整的一段返回 NULL。 */
static const char *ParseLog(const char *from, parsed_log *log)
{
    const char *begin = strstr(from, "LOG begin ");
    if (!begin)
        return NULL;
    memset(log, 0, sizeof(*log));
    log->in_order = 1;
    assert(sscanf(begin, "LOG begin n=%u mode=%7s end=%7s trunc=%u imu=%u", &log->n, log->mode,
                  log->end, &log->trunc, &log->imu) == 5);
    unsigned last_mode = 99U;
    for (const char *p = strchr(begin, '\n'); p && *++p; p = strchr(p, '\n'))
    {
        unsigned i, mode, moving;
        long target, yaw;
        int pid, cmd, vx, vy;
        if (strncmp(p, "LOG end n=", 10) == 0)
        {
            assert((unsigned)atoi(p + 10) == log->n);
            return p;
        }
        if (strncmp(p, "LOG abort", 9) == 0)
            return NULL;
        const char *kp = strstr(p, "heading_kp=");
        if (strncmp(p, "LOG param ", 10) == 0 && kp && kp < strchr(p, '\n'))
            assert(sscanf(kp, "heading_kp=%f heading_ki=%f", &log->kp, &log->ki) == 2);
        if (sscanf(p, "L %u %u %u %ld %ld %d %d %d %d", &i, &mode, &moving, &target, &yaw, &pid,
                   &cmd, &vx, &vy) != 9 ||
            p[0] != 'L' || p[1] != ' ')
            continue;
        if (i != log->lines)
            log->in_order = 0;
        if (log->lines == 0U)
        {
            log->first_mode = mode;
            log->first_target = target;
        }
        if (mode != last_mode && log->lines)
            ++log->modes;
        log->tail = mode == 0U ? log->tail + 1U : 0U;
        last_mode = mode;
        log->last_yaw = yaw;
        ++log->lines;
    }
    return NULL;
}

/* 等一段日志的最后一行整行到齐（一行可能被拆在两块里发）。 */
static void WaitLogEnd(size_t from)
{
    const char *end = RunUntil(from, "LOG end n=", 6000U);
    size_t at = (size_t)(end - out);
    RunUntil(at, "\n", 100U);
}

/* 发一条运动命令，等它的日志发完。 */
static void MotionLog(const char *line, parsed_log *log)
{
    size_t mark = out_len;
    Send(line);
    WaitLogEnd(mark);
    assert(ParseLog(out + mark, log));
}

static void Idle(unsigned steps)
{
    for (unsigned i = 0; i < steps; ++i)
        Step(1);
}

/* ---------- 用例 ---------- */
/* 停着时 T 命令从现在的航向算起，第一拍的目标航向就是它。 */
static long TargetFromHere(float degrees)
{
    return lroundf((chassis.yaw + degrees) * 1000.0f);
}

static void TestNothingRecordedYet(void)
{
    size_t mark = out_len;
    Send("/log\n");
    RunUntil(mark, "LOG none\r\n", 100U);
}

/* 每段：开头是动作的第一拍，结尾多记 0.4 s；参数是动作开始时的；每行序号连着。 */
static void TestAutoSendAfterTurn(void)
{
    size_t mark = out_len;
    Send("/log on\n");
    RunUntil(mark, "OK   log auto on\r\n", 100U);
    parsed_log log;
    long target = TargetFromHere(10.0f);
    MotionLog("T10\n", &log);
    assert(strcmp(log.mode, "path") == 0 && strcmp(log.end, "done") == 0);
    assert(log.trunc == 0U && log.imu == 1U && log.lines == log.n && log.in_order);
    assert(log.first_mode == CHASSIS_MODE_PATH && labs(log.first_target - target) <= 2);
    assert(log.tail == TUNE_LOG_TAIL + 1U); // 结束那一拍加上尾巴
    assert(labs(log.last_yaw - target) <= 150);
    assert(fabsf(log.kp - chassis_tune.heading_kp) < 1e-3f &&
           fabsf(log.ki - chassis_tune.heading_ki) < 1e-3f);
    assert(Count(mark, "LOG param acc_angular=720 heading_tol=0.100 settle_ms=200") == 1U);
}

/* 状态行开着也不丢字：日志等发送缓冲有一半空着才排，状态行照常出来。 */
static void TestLogLeavesRoomForOtherOutput(void)
{
    Send("D10\n");
    unsigned long dropped = tx_dropped;
    size_t mark = out_len;
    parsed_log log;
    MotionLog("T20\n", &log);
    assert(log.lines == log.n && log.in_order);
    assert(tx_dropped == dropped);
    const char *begin = strstr(out + mark, "LOG begin");
    const char *end = strstr(begin, "LOG end");
    const char *status = strstr(begin, "\n[");
    assert(status && status < end); // 发日志的时候状态行还在出
    Send("D0\n");
}

/* 动作中发 /log：做完再发；停着再发 /log 把同一段重发一遍。 */
static void TestRequestWhileMoving(void)
{
    Send("/log off\n");
    size_t mark = out_len;
    long target = TargetFromHere(5.0f);
    Send("T5\n");
    Idle(10);
    Send("/log\n");
    RunUntil(mark, "OK   log after this motion\r\n", 100U);
    WaitLogEnd(mark);
    parsed_log first, again;
    assert(ParseLog(out + mark, &first) && labs(first.first_target - target) <= 2);
    Idle(5);
    mark = out_len;
    Send("/log\n");
    WaitLogEnd(mark);
    assert(ParseLog(out + mark, &again) && again.n == first.n && again.lines == first.lines);
    mark = out_len;
    Send("T-5\n");
    Idle(600);
    assert(Count(mark, "LOG begin") == 0U); // 自动发关了
}

/* 发到一半又开始动：这段作废，记新的。 */
static void TestNewMotionAbortsSend(void)
{
    Send("/log on\n");
    size_t mark = out_len;
    Send("T30\n");
    RunUntil(mark, "LOG begin", 6000U);
    Idle(20);
    long target = TargetFromHere(-30.0f);
    Send("T-30\n");
    RunUntil(mark, "LOG abort\r\n", 40U); // 排在已经进缓冲的半缓冲日志后面
    size_t abort_at = out_len;
    WaitLogEnd(abort_at);
    parsed_log log;
    assert(ParseLog(out + abort_at, &log));
    assert(labs(log.first_target - target) <= 2 && strcmp(log.end, "done") == 0);
}

/* 前一个动作的尾巴里又开始动：接着记成一段。 */
static void TestQuickMotionsMerge(void)
{
    size_t mark = out_len;
    long first = TargetFromHere(5.0f);
    Send("T5\n");
    while (chassis.mode != CHASSIS_MODE_VEL || Count(mark, "DONE path") == 0U)
        Step(1);
    Idle(10);
    long second = TargetFromHere(-5.0f);
    Send("T-5\n");
    WaitLogEnd(mark);
    parsed_log log;
    assert(ParseLog(out + mark, &log));
    assert(Count(mark, "LOG begin") == 1U && log.modes == 3U); // 路径、停、路径、停
    assert(labs(log.first_target - first) <= 2 && labs(log.last_yaw - second) <= 150);
}

/* 超过 10 s 的动作只留前 10 s。 */
static void TestLongMotionTruncated(void)
{
    Send("/set cmd_speed 50\n");
    parsed_log log;
    MotionLog("F600\n", &log);
    assert(log.trunc == 1U && log.n == TUNE_LOG_CAPACITY && log.lines == log.n);
    assert(strcmp(log.mode, "path") == 0 && strcmp(log.end, "done") == 0);
    Send("/set cmd_speed 400\n");
    MotionLog("B600\n", &log);
    assert(log.trunc == 0U);
}

/* 动作中反馈断了进故障：这段照样发出来，标 fault。 */
static void TestFaultEndsLog(void)
{
    size_t mark = out_len;
    Send("T90\n");
    Idle(20);
    while (chassis.state != CHASSIS_STATE_FAULT)
        Step(0);
    WaitLogEnd(mark);
    parsed_log log;
    assert(ParseLog(out + mark, &log) && strcmp(log.end, "fault") == 0);
    Send("S\n");
    for (int i = 0; i < 100 && chassis.state != CHASSIS_STATE_READY; ++i)
        Step(1);
    assert(chassis.state == CHASSIS_STATE_READY);
    Send("A0\n"); // 回到 0°，后面的样本从 0° 开始
    WaitLogEnd(out_len);
}

/* ---------- 给 node 测试的日志：调试台自动调参发的那组动作 ---------- */
static const char *const k_sequence[] = {"T5\n",   "T-5\n",  "T20\n",  "T-20\n", "T90\n",
                                         "T-90\n", "F300\n", "B300\n", "L300\n", "R300\n"};

static void Fixture(FILE *file, fake_car c, float kp, float ki)
{
    char line[64];
    snprintf(line, sizeof(line), "/set heading_kp %.3f\n", (double)kp);
    Send(line);
    snprintf(line, sizeof(line), "/set heading_ki %.3f\n", (double)ki);
    Send(line);
    UseCar(c);
    Idle(100);
    fprintf(file, "# car slip=%.3f tau=%.4f delay=%.4f cx=%.4f cy=%.4f noise=%.4f\n",
            (double)c.slip, (double)c.tau, (double)(c.delay * 0.005f), (double)c.cx, (double)c.cy,
            (double)c.noise);
    size_t mark = out_len;
    for (unsigned i = 0; i < sizeof(k_sequence) / sizeof(k_sequence[0]); ++i)
    {
        parsed_log log;
        MotionLog(k_sequence[i], &log);
        assert(strcmp(log.end, "done") == 0 && log.trunc == 0U && log.lines == log.n);
        Idle(60);
    }
    fwrite(out + mark, 1U, out_len - mark, file);
}

int main(int argc, char **argv)
{
    FDCAN_HandleTypeDef can = {0};
    assert(Chassis_Init(&can) == HAL_OK);
    assert(ChassisCmd_Init(&uart) == HAL_OK);
    ChassisTelemetry_SetRate(0);
    for (int i = 0; i < 4; ++i)
        Step(1);
    assert(chassis.state == CHASSIS_STATE_READY);
    UseCar((fake_car){0.9f, 0.04f, 0.002f, 0.008f, 0.003f, 4U});

    TestNothingRecordedYet();
    TestAutoSendAfterTurn();
    TestLogLeavesRoomForOtherOutput();
    TestRequestWhileMoving();
    TestNewMotionAbortsSend();
    TestQuickMotionsMerge();
    TestLongMotionTruncated();
    TestFaultEndsLog();

    FILE *file = fopen(argc > 1 ? argv[1] : "build/host/tune_fixture.log", "wb");
    assert(file);
    Send("/log on\n");
    Fixture(file, (fake_car){0.9f, 0.04f, 0.002f, 0.008f, 0.003f, 4U}, 2.0f, 1.0f);
    Fixture(file, (fake_car){0.9f, 0.04f, 0.002f, 0.008f, 0.003f, 4U}, 5.0f, 2.0f);
    Fixture(file, (fake_car){0.95f, 0.025f, 0.0f, 0.005f, 0.0f, 2U}, 2.0f, 1.0f);
    fclose(file);
    printf("tune_log_test passed\n");
    return 0;
}
