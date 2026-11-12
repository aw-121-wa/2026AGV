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
   陀螺仪按 HWT101 的分辨率量化，再加噪声。到这里为止就是调参工具假设的那个模型。
   后面几项是模型里没有的（不写就是 0），用来看工具在对不上模型的车上还管不管用：
   tau2 车身再晚一个惯性；slip_fast 转到 180 deg/s 时多打滑这么多；ax、ay 加减速时车身扭一下
   （deg/s 每 mm/s²）；wander 地面带来的随机转动（deg/s，0.3 s 相关的有色噪声）；
   imu_div 陀螺仪每几拍才出一个新数。 */
typedef struct
{
    float slip, tau, cx, cy, noise;
    unsigned delay;
    float tau2, slip_fast, ax, ay, wander;
    unsigned imu_div;
} fake_car;

#define DELAY_MAX 16U
static fake_car car = {1.0f, 0.005f, 0.0f, 0.0f, 0.0f, 0U, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0U};
static float sent[DELAY_MAX][4], wheel_rpm[4];
static unsigned sent_head;
static double yaw_true;
static float body_wz, body_vx, body_vy, wander_now;
static unsigned imu_wait;
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
    wander_now = 0.0f;
}

static void CarStep(void)
{
    const float dt = 0.005f;
    const float *u = sent[(sent_head + DELAY_MAX - car.delay) % DELAY_MAX];
    float a = 1.0f - expf(-dt / car.tau), v[4];
    for (int i = 0; i < 4; ++i)
    {
        wheel_rpm[i] += (u[i] - wheel_rpm[i]) * a;
        WheelSpeed(i, wheel_rpm[i]);
        WheelTurn(i, wheel_rpm[i] * 6.0f * dt);
        v[i] = wheel_rpm[i] * 3.14159265f * CHASSIS_WHEEL_DIA_MM / 60.0f;
    }
    // 侧移打滑、斜着走：横着实际走的是轮子算出来的 CHASSIS_LATERAL_SCALE 倍，每横移 1 mm 前后偏
    // CHASSIS_LATERAL_SKEW mm，固件按这两个系数标定过
    float vy = 0.25f * (-v[0] + v[1] + v[2] - v[3]) * CHASSIS_LATERAL_SCALE;
    float vx = 0.25f * (v[0] + v[1] + v[2] + v[3]) + CHASSIS_LATERAL_SKEW * vy;
    float wz = (-v[0] + v[1] - v[2] + v[3]) / (4 * CHASSIS_K_MM) * 57.2957795f;
    body_wz = car.tau2 > 0.0f ? body_wz + (wz - body_wz) * (1.0f - expf(-dt / car.tau2)) : wz;
    float slip = car.slip - car.slip_fast * fminf(1.0f, fabsf(body_wz) / 180.0f);
    float twist = (car.ax * (vx - body_vx) + car.ay * (vy - body_vy)) / dt;
    body_vx = vx;
    body_vy = vy;
    if (car.wander > 0.0f) // 一阶有色噪声，稳态标准差 = wander
        wander_now += -wander_now * dt / 0.3f + car.wander * sqrtf(2.0f * dt / 0.3f) * Gauss();
    yaw_true += (slip * body_wz + car.cx * vx + car.cy * vy + twist + wander_now) * dt;
    if (++imu_wait < car.imu_div)
        return; // 陀螺仪这一拍没有新数，固件读到的还是上一个
    imu_wait = 0U;
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
    char third[128]; // 新加的三个航向环参数、编译进去的轮速上限、横移系数
    snprintf(third, sizeof(third),
             "LOG param heading_brake=%.0f heading_ff_x=%.2f heading_ff_y=%.2f max_wheel_rpm=%u "
             "lateral_scale=%.3f lateral_skew=%.4f\r\n",
             (double)CHASSIS_HEADING_BRAKE, (double)CHASSIS_HEADING_FF_X,
             (double)CHASSIS_HEADING_FF_Y, (unsigned)CHASSIS_MAX_WHEEL_RPM,
             (double)CHASSIS_LATERAL_SCALE, (double)CHASSIS_LATERAL_SKEW);
    assert(Count(mark, third) == 1U);
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
    char line[40]; // 回到开机默认的指令速度：后面的日志和调试台实际测的一样
    snprintf(line, sizeof(line), "/set cmd_speed %.0f\n", (double)chassis_tune_default.cmd_speed);
    Send(line);
    assert(chassis_tune.cmd_speed == chassis_tune_default.cmd_speed);
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
/* 平移距离和调试台按指令速度算的默认值一样：1200 mm/s、2000 mm/s² 时 1100 mm，中间有一小段匀速。 */
static const char *const k_sequence[] = {"T5\n",   "T-5\n",   "T20\n",   "T-20\n",  "T90\n",
                                         "T-90\n", "F1100\n", "B1100\n", "L1100\n", "R1100\n"};

static void Fixture(FILE *file, fake_car c, float kp, float ki)
{
    char line[64];
    snprintf(line, sizeof(line), "/set heading_kp %.3f\n", (double)kp);
    Send(line);
    snprintf(line, sizeof(line), "/set heading_ki %.3f\n", (double)ki);
    Send(line);
    UseCar(c);
    Idle(100);
    fprintf(file, "# car slip=%.3f tau=%.4f delay=%.4f cx=%.4f cy=%.4f noise=%.4f exact=%d\n",
            (double)c.slip, (double)c.tau, (double)(c.delay * 0.005f), (double)c.cx, (double)c.cy,
            (double)c.noise,
            c.tau2 == 0.0f && c.slip_fast == 0.0f && c.ax == 0.0f && c.ay == 0.0f &&
                c.wander == 0.0f && c.imu_div <= 1U);
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

/* ---------- 裁判：一组参数在某辆假车上到底怎么样 ---------- */
/* 用法：tune_log_test --bench <车> <命令或动作> ...
   “/”开头的是控制台命令（先发），其余是要量的动作。
   每个动作量真实航向（不是陀螺仪读数），打一行：
     BENCH <动作> done= settle= over= final= peak= rms= limit=
   done 固件判走完，s；settle 进 heading_tol + 0.05° 再没出去，s，没稳住是 -1；
   over 转过头，°；final 走完 0.8 s 后还差，°；peak、rms 平移时航向差的最大、均方根，°；
   limit 平移时超过 0.2° 的拍数。
   tools/pid_tune_test.js 用它检查：调参工具给的推荐值放回这辆车上，是不是真的比原来准。 */
static float TrueError(void)
{
    return Math_WrapDeg(chassis.heading_target - (float)yaw_true);
}

typedef struct
{
    float done, settle, over, final, peak, rms; // 含义同 BENCH 行
    float stop;                                 // 平移到什么时候为止，s
    int limit;
} bench_result;

static bench_result Measure(const char *name)
{
    char line[32];
    snprintf(line, sizeof(line), "%s\n", name);
    Send(line);
    for (int i = 0; i < 20 && chassis.mode == CHASSIS_MODE_VEL; ++i)
        Step(1);
    assert(chassis.mode == CHASSIS_MODE_PATH);
    const float band = chassis_tune.heading_tol + 0.05f;
    float dir = 0.0f, over = 0.0f, peak = 0.0f;
    double sum = 0.0;
    int ticks = 0, done = -1, last_out = -1, moved = 0, limit = 0, stop = 0;
    for (; ticks < 6000 && (done < 0 || ticks < done + 160); ++ticks)
    {
        Step(1);
        float err = TrueError();
        if (ticks == 0) // 路径的第一拍才把目标航向换成这个动作的
            dir = err < 0.0f ? -1.0f : 1.0f;
        if (fabsf(err) > band)
            last_out = ticks;
        over = fmaxf(over, -err * dir);
        if (chassis.mode == CHASSIS_MODE_PATH &&
            (chassis.target.vx != 0.0f || chassis.target.vy != 0.0f))
        {
            peak = fmaxf(peak, fabsf(err));
            sum += (double)err * err;
            limit += fabsf(err) > CHASSIS_DYNAMIC_TOL_DEG;
            ++moved;
            stop = ticks + 1;
        }
        if (done < 0 && chassis.mode == CHASSIS_MODE_VEL)
            done = ticks;
    }
    assert(done >= 0 && chassis.state == CHASSIS_STATE_READY);
    return (bench_result){
        .done = (float)(done + 1) * 0.005f,
        .settle = last_out < ticks - 1 ? (float)(last_out + 1) * 0.005f : -1.0f,
        .over = moved ? 0.0f : over,
        .final = fabsf(TrueError()),
        .peak = peak,
        .rms = moved ? (float)sqrt(sum / moved) : 0.0f,
        .stop = (float)stop * 0.005f,
        .limit = limit,
    };
}

static void BenchMotion(const char *name)
{
    bench_result r = Measure(name);
    printf("BENCH %s done=%.3f settle=%.3f over=%.3f final=%.3f peak=%.3f rms=%.3f limit=%d\n",
           name, (double)r.done, (double)r.settle, (double)r.over, (double)r.final, (double)r.peak,
           (double)r.rms, r.limit);
}

/* 0：调参模型假设的那种车。1：反应快、没噪声。2：模型里没有的都加上（二阶惯性、转得快多打滑、
   加减速扭车身、地面随机扰动、陀螺仪 100 Hz）。 */
static const fake_car k_cars[] = {
    {0.9f, 0.04f, 0.002f, 0.008f, 0.003f, 4U, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0U},
    {0.95f, 0.025f, 0.0f, 0.005f, 0.0f, 2U, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0U},
    {0.93f, 0.02f, 0.0015f, 0.0045f, 0.004f, 3U, 0.015f, 0.06f, 0.0002f, 0.0004f, 0.25f, 2U},
};
#define CAR_COUNT (sizeof(k_cars) / sizeof(k_cars[0]))

static int Bench(int argc, char **argv)
{
    unsigned which = argc > 0 ? (unsigned)atoi(argv[0]) : 0U;
    assert(which < CAR_COUNT);
    UseCar(k_cars[which]);
    Idle(100);
    for (int i = 1; i < argc; ++i)
        if (argv[i][0] == '/')
        {
            char line[64];
            size_t mark = out_len;
            snprintf(line, sizeof(line), "%s\n", argv[i]);
            Send(line);
            Idle(4);
            if (!strstr(out + mark, "OK  "))
            {
                fprintf(stderr, "bench: '%s' rejected: %s", argv[i], out + mark);
                return 1;
            }
        }
    for (int i = 1; i < argc; ++i)
        if (argv[i][0] != '/')
        {
            BenchMotion(argv[i]);
            Idle(60);
        }
    return 0;
}

/* ---------- 航向环为自动调参加的三样：平移停下清积分、刹车曲线、平移前馈 ---------- */
static void Set(const char *name, float value)
{
    char line[64];
    size_t mark = out_len;
    snprintf(line, sizeof(line), "/set %s %.2f\n", name, (double)value);
    Send(line);
    RunUntil(mark, "OK   ", 20U);
}

/* 长距离横移：平移时积分一直顶着带偏，车一停扰动就没了。积分不清，它和比例项对着干，车头要 4 s
   才回到 容差里，这 4 s 路径都判不了“走完”；清掉以后半秒内走完。 */
static void TestIntegralClearedWhenMoveStops(void)
{
    bench_result r = Measure("L1500");
    assert(r.peak > 0.5f); // 没开前馈，带偏是真的，积分确实攒起来了
    assert(r.done - r.stop < 0.5f);
    Measure("R1500");
}

/* 刹车曲线：kp 调到 8、关掉刹车曲线时 90° 冲过头好几度；开了 250 就不过头，而且更早稳住。
   小角度不受影响。默认值本身就是调好的曲线，所以“没有它”这组要显式置 0；kp/ki 也钉在固定值，
   只比较刹车曲线本身（默认整定值改了不该影响这条用例的结论）。 */
static void TestBrakeCurve(void)
{
    float kp = chassis_tune.heading_kp;
    float ki = chassis_tune.heading_ki;
    float brake = chassis_tune.heading_brake;
    Set("heading_kp", 8.0f);
    Set("heading_ki", 2.14f); // 固定积分增益，别让默认整定值的改动混进这条对比
    Set("heading_brake", 0.0f); // 先关掉编译进去的刹车曲线，才量得到“没有它”冲过头多少
    bench_result bare = Measure("T90"), bare_small = Measure("T5");
    Measure("T-95");
    Set("heading_brake", 250.0f);
    bench_result curved = Measure("T90"), curved_small = Measure("T5");
    Measure("T-95");
    assert(bare.over > 2.0f && curved.over < 0.5f);
    assert(curved.settle > 0.0f && curved.settle < bare.settle);
    assert(fabsf(curved_small.done - bare_small.done) < 0.02f);
    Set("heading_brake", brake);
    Set("heading_ki", ki);
    Set("heading_kp", kp);
}

/* 平移前馈：按这辆假车的带偏量反着补，横移时车头基本不动，也就没有超过 0.2° 的拍。 */
static void TestFeedForward(void)
{
    bench_result before = Measure("L800");
    Measure("R800");
    Set("heading_ff_y", -car.cy / car.slip * 1000.0f);
    bench_result after = Measure("L800");
    Measure("R800");
    assert(before.peak > 0.5f && before.limit > 100);
    assert(after.peak < 0.05f && after.limit == 0);
    Set("heading_ff_y", 0.0f);
}

/* 跑图的逐段停车，在这辆轮子晚 60 ms 的车上，1200 mm/s：直线中间的点不停；拐角刹车按 path_lag
   提前，冲过头不超过拐角容差，也不倒回去追航点（倒回去就是车还在滑、轮子已经反转）。
   path_lag 设 0（原来的刹车曲线）每个点都冲过去 50 mm 以上再倒回来。 */
typedef struct
{
    float time, over, pass_speed, back;
    uint8_t completed;
} corner_run;

static corner_run CornerRun(void)
{
    float x0 = chassis.pos_x, y0 = chassis.pos_y, h = chassis.yaw;
    const chassis_waypoint pts[] = {{x0 + 500, y0, h},        {x0 + 1000, y0, h},
                                    {x0 + 1000, y0 + 800, h}, {x0 + 200, y0 + 800, h},
                                    {x0 + 200, y0, h},        {x0, y0, h}};
    const unsigned n = sizeof(pts) / sizeof(pts[0]);
    Chassis_SetMoveSpeedLimit(1200.0f);
    assert(Chassis_FollowPath(pts, n, 1U) == HAL_OK);
    corner_run r = {0};
    int ticks = 0;
    for (; chassis.mode == CHASSIS_MODE_PATH && ticks < 6000; ++ticks)
    {
        Step(1);
        unsigned k = Chassis_PathTarget() - 1U; // 正在去的航点
        if (k >= n)
            continue;
        chassis_waypoint a = k ? pts[k - 1U] : (chassis_waypoint){x0, y0, h}, b = pts[k];
        float ux = b.x - a.x, uy = b.y - a.y, l = hypotf(ux, uy);
        ux /= l;
        uy /= l;
        r.over = fmaxf(r.over, (chassis.pos_x - b.x) * ux + (chassis.pos_y - b.y) * uy);
        // 沿这段往前的目标车速，负的就是倒回去追航点（航向是 0，车体系就是世界系）
        r.back = fminf(r.back, chassis.target.vx * ux + chassis.target.vy * uy);
        if (k == 1U && fabsf(chassis.pos_x - pts[0].x) < 10.0f)
            r.pass_speed = fmaxf(r.pass_speed, hypotf(body_vx, body_vy));
    }
    r.time = (float)ticks * 0.005f;
    r.completed = chassis_path_result.completed;
    Idle(60);
    return r;
}

static void TestStopEachCornersOnLaggingCar(void)
{
    UseCar(k_cars[0]); // 延迟 20 ms + 惯性 40 ms
    Idle(100);
    corner_run now = CornerRun();
    assert(now.completed && now.pass_speed > 1000.0f);
    assert(now.over <= chassis_tune.path_deviation + 0.5f && now.back > -5.0f);
    Set("path_lag", 0.0f);
    corner_run old = CornerRun();
    assert(old.completed && old.over > 50.0f && old.back < -20.0f);
    assert(now.time < old.time - 0.5f);
    Set("path_lag", chassis_tune_default.path_lag);
    printf("stop at corners, 1200 mm/s, wheels 60 ms late: %.2f s, overshoot %.1f mm "
           "(braking without the lag: %.2f s, overshoot %.1f mm, backs up at %.0f mm/s)\n",
           (double)now.time, (double)now.over, (double)old.time, (double)old.over,
           (double)-old.back);
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
    if (argc > 1 && strcmp(argv[1], "--bench") == 0)
        return Bench(argc - 2, argv + 2);
    UseCar(k_cars[0]);

    TestNothingRecordedYet();
    TestAutoSendAfterTurn();
    TestLogLeavesRoomForOtherOutput();
    TestRequestWhileMoving();
    TestNewMotionAbortsSend();
    TestQuickMotionsMerge();
    TestLongMotionTruncated();
    TestFaultEndsLog();
    Send("/log off\n");
    TestIntegralClearedWhenMoveStops();
    TestBrakeCurve();
    TestFeedForward();
    TestStopEachCornersOnLaggingCar();

    FILE *file = fopen(argc > 1 ? argv[1] : "build/host/tune_fixture.log", "wb");
    assert(file);
    Send("/log on\n");
    Fixture(file, k_cars[0], 2.0f, 1.0f);
    Fixture(file, k_cars[0], 5.0f, 2.0f);
    Fixture(file, k_cars[1], 2.0f, 1.0f);
    Fixture(file, k_cars[2], 4.0f, 1.5f);
    fclose(file);
    printf("tune_log_test passed\n");
    return 0;
}
