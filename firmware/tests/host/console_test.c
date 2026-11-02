/* 串口控制台的主机测试：真实的底盘控制、命令解析和文本输出，只替换硬件。
   改参数的用例都看控制输出或运动结果，不只看变量值。 */
#define main original_chassis_tests
#include "chassis_test.c"
#undef main

static UART_HandleTypeDef uart;

/* 一包串口数据：超过 HAL 接收缓冲就分几次回调，idle = 1 时包尾有串口空闲事件。 */
static void Burst(const char *text, int idle)
{
    size_t n = strlen(text);
    while (n)
    {
        size_t chunk = n > CMD_BUF_SIZE ? CMD_BUF_SIZE : n;
        memcpy(rx_buf, text, chunk);
        event_type = (idle && chunk == n) ? HAL_UART_RXEVENT_IDLE : 0U;
        HAL_UARTEx_RxEventCallback(&uart, (uint16_t)chunk);
        ChassisCmd_Poll();
        text += chunk;
        n -= chunk;
    }
    event_type = HAL_UART_RXEVENT_IDLE;
}

/* 带回车换行发送一行，调试台界面就是这样发的。 */
static void Line(const char *text)
{
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "%s\r\n", text);
    Burst(buffer, 1);
}

static void Wait(uint32_t duration_ms)
{
    ms += duration_ms;
    ChassisCmd_Poll();
}

static void ClearOutput(void)
{
    drain(&uart);
    tx_log[0] = 0;
}

static const char *Output(void)
{
    drain(&uart);
    return tx_log;
}

static unsigned CountOutput(const char *needle)
{
    unsigned count = 0;
    drain(&uart);
    for (const char *p = strstr(tx_log, needle); p; p = strstr(p + 1, needle))
        ++count;
    return count;
}

/* 每个用例从默认值、停车状态开始。 */
static void Begin(void)
{
    Line("S");
    Line("/reset");
    for (int i = 0; i < 4; ++i)
        motors[i].speed_rpm = 0.0f;
    imu.yaw = 0.0f;
    tick(1);
    s_heading_integral = 0.0f;
    ClearOutput();
}

static float PathSpeed(void)
{
    return hypotf(chassis.target.vx, chassis.target.vy);
}

static float Control(float error, uint8_t stationary)
{
    chassis.heading_error = error;
    return Chassis_HeadingControl(0.005f, stationary);
}

static void AssertNear(float got, float want, float tolerance)
{
    if (fabsf(got - want) > tolerance)
    {
        fprintf(stderr, "got %f want %f\n", (double)got, (double)want);
        assert(fabsf(got - want) <= tolerance);
    }
}

static void TestHeadingLoopFollowsParameters(void)
{
    Begin();
    AssertNear(Control(1.0f, 0), 2.005f, 1e-4f); // 默认 2 × 1° + 1 × 1° × 5 ms

    Line("/set heading_kp 4");
    s_heading_integral = 0.0f;
    AssertNear(Control(1.0f, 0), 4.005f, 1e-4f);

    Line("/set heading_ki 3");
    s_heading_integral = 0.0f;
    AssertNear(Control(1.0f, 0), 4.015f, 1e-4f);

    Line("/set heading_i_max 0.01"); // 积分 3 × 1° × 5 ms = 0.015，被限到 0.01
    s_heading_integral = 0.0f;
    AssertNear(Control(1.0f, 0), 4.01f, 1e-4f);

    Line("/set heading_i_zone 0.5"); // 1° 在积分区外
    s_heading_integral = 0.0f;
    AssertNear(Control(1.0f, 0), 4.0f, 1e-4f);

    Line("/set turn_max_speed 3");
    AssertNear(Control(1.0f, 0), 3.0f, 1e-4f);

    Line("/set heading_tol 2"); // 停稳判定：1° 算到位，撤去修正
    AssertNear(Control(1.0f, 1), 0.0f, 1e-6f);
}

/* KI 调成 0 后，旧积分不能留在输出里。 */
static void TestZeroKiClearsIntegral(void)
{
    Begin();
    Line("/set heading_ki 0");
    s_heading_integral = 3.0f;
    AssertNear(Control(1.0f, 0), 2.0f, 1e-4f);
    AssertNear(s_heading_integral, 0.0f, 1e-6f);
}

static void TestDistanceLoopFollowsParameters(void)
{
    Begin();
    Line("/set distance_kp 0.1");
    assert(Chassis_MoveDistance(500, 0) == HAL_OK);
    tick(1);
    AssertNear(chassis.target.vx, 50.0f, 0.01f); // 0.1 × 500 mm
    Line("S");

    Begin();
    Line("/set distance_tol 40");
    assert(Chassis_MoveDistance(30, 0) == HAL_OK);
    tick(1);
    AssertNear(chassis.target.vx, 0.0f, 1e-3f); // 已在容差内，不再前进
}

static void TestSettleTimeFollowsParameter(void)
{
    Begin();
    Line("/set distance_tol 40");
    Line("/set settle_ms 100"); // 每拍 5 ms，约 20 拍
    assert(Chassis_MoveDistance(30, 0) == HAL_OK);
    for (int i = 0; i < 15; ++i)
        tick(1);
    assert(chassis.mode == CHASSIS_MODE_MOVE);
    for (int i = 0; i < 10; ++i)
        tick(1);
    assert(chassis.mode == CHASSIS_MODE_VEL);
}

/* 距离在容差内算停稳：航向误差也在容差内时撤去修正。 */
static void TestDistanceTolDefinesStationary(void)
{
    Begin();
    Line("/set distance_tol 40");
    assert(Chassis_MoveDistance(30, 0) == HAL_OK);
    imu.yaw = 0.05f;
    tick(1);
    AssertNear(chassis.target.wz, 0.0f, 1e-5f);
}

static void TestHeadingTolDefinesSettled(void)
{
    Begin();
    Line("/set distance_tol 40");
    Line("/set heading_tol 2");
    Line("/set settle_ms 50");
    assert(Chassis_MoveDistance(30, 0) == HAL_OK);
    imu.yaw = 1.0f; // 默认容差 0.1° 下永远停不稳
    for (int i = 0; i < 15; ++i)
        tick(1);
    assert(chassis.mode == CHASSIS_MODE_VEL);
}

/* 路径刹车曲线：逐段停车时到航点要减到 0，v = sqrt(2 × a × d)；
   后面还有 300 mm 总路程时单看总路程是 245 mm/s，要看到当前航点的 141 mm/s 才对。 */
static void TestPathBrakesToWaypointWithTunedAcceleration(void)
{
    Begin();
    Line("/set acc_linear 100");
    Chassis_SetMoveSpeedLimit(400.0f);
    float x = chassis.pos_x, y = chassis.pos_y, h = chassis.yaw;
    chassis_waypoint points[2] = {{x + 100, y, h}, {x + 300, y, h}};
    assert(Chassis_FollowPath(points, 2, 1) == HAL_OK);
    tick(1);
    AssertNear(PathSpeed(), 141.4f, 1.0f);
}

/* 向前看的刹车：中间航点共线（航向不同所以不合并）不减速，但 200 mm 外要停，
   v = sqrt(2 × 100 × 200)。 */
static void TestPathLooksAheadWithTunedAcceleration(void)
{
    Begin();
    Line("/set acc_linear 100");
    Chassis_SetMoveSpeedLimit(400.0f);
    float x = chassis.pos_x, y = chassis.pos_y, h = chassis.yaw;
    chassis_waypoint points[2] = {{x + 100, y, h}, {x + 200, y, h + 10.0f}};
    assert(Chassis_FollowPath(points, 2, 0) == HAL_OK);
    tick(1);
    AssertNear(PathSpeed(), 200.0f, 2.0f);
}

/* 接近掉头的尖角不做贝塞尔过渡，出弯速度 = sqrt(a × 偏差 × c / (1 - c))，c = sqrt((1 + cosθ) / 2)。
   航点 (100,0) 再到 (0,10)：cosθ = -0.99504，c / (1 - c) = 0.05243。离航点 1 mm 时
   v = sqrt(出弯速度² + 2 × a × 1)。 */
static float SpeedOneMillimetreBeforeSharpCorner(void)
{
    float x = chassis.pos_x, y = chassis.pos_y, h = chassis.yaw;
    chassis_waypoint points[2] = {{x + 100, y, h}, {x, y + 10, h}};
    Chassis_SetMoveSpeedLimit(400.0f);
    assert(Chassis_FollowPath(points, 2, 0) == HAL_OK);
    chassis.pos_x = x + 99.0f;
    tick(0);
    return PathSpeed();
}

static void TestCornerExitSpeedFollowsParameters(void)
{
    Begin();
    Line("/set acc_linear 100");
    AssertNear(SpeedOneMillimetreBeforeSharpCorner(), 15.04f, 0.1f); // sqrt(26.2 + 200)
    Begin();
    Line("/set acc_linear 100");
    Line("/set path_deviation 20");
    AssertNear(SpeedOneMillimetreBeforeSharpCorner(), 17.46f, 0.1f); // sqrt(104.9 + 200)
}

/* 路径的到位判定也用运行时容差。 */
static void TestDistanceTolDefinesPathFinish(void)
{
    Begin();
    Line("/set distance_tol 40");
    Line("F30");
    assert(chassis.mode == CHASSIS_MODE_PATH);
    chassis.current.vx = 50.0f; // 还没减速完，路径不算结束
    imu.yaw = 0.05f;
    tick(1);
    assert(chassis.mode == CHASSIS_MODE_PATH);
    AssertNear(PathSpeed(), 0.0f, 1e-3f);       // 已在容差内，不再前进
    AssertNear(chassis.target.wz, 0.0f, 1e-5f); // 航向也算到位，撤去修正
    chassis.current.vx = 0.0f;
    tick(1);
    assert(chassis.mode == CHASSIS_MODE_VEL);
}

static void TestHeadingTolDefinesPathFinish(void)
{
    Begin();
    Line("/set distance_tol 40");
    Line("/set heading_tol 2");
    Line("F30");
    assert(chassis.mode == CHASSIS_MODE_PATH);
    imu.yaw = 1.0f;
    tick(1);
    assert(chassis.mode == CHASSIS_MODE_VEL);
}

static void TestRampsFollowParameters(void)
{
    Begin();
    Line("/set acc_linear 400"); // 每拍 2 mm/s
    Chassis_SetVelocity(100, 50, 0);
    tick(1);
    AssertNear(chassis.current.vx, 2.0f, 0.05f);
    AssertNear(chassis.current.vy, 2.0f, 0.05f);

    Begin();
    Line("/set acc_angular 200"); // 每拍 1 deg/s
    Chassis_SetVelocity(0, 0, 50);
    tick(1);
    AssertNear(chassis.current.wz, 1.0f, 0.05f);
}

/* 拐角限速 = sqrt(加速度 / 4 × 允许偏差)，见 chassis_path.inc。 */
static void TestCornerSpeedFollowsParameters(void)
{
    Begin();
    Line("/set path_deviation 1");
    Line("F300 L300");
    assert(chassis.mode == CHASSIS_MODE_PATH);
    float low = 1e9f;
    for (unsigned i = 0; i < s_path_count; ++i)
        low = fminf(low, s_path_limit[i]);
    AssertNear(low, 22.36f, 0.05f);

    Begin();
    Line("/set acc_linear 500");
    Line("F300 L300");
    low = 1e9f;
    for (unsigned i = 0; i < s_path_count; ++i)
        low = fminf(low, s_path_limit[i]);
    AssertNear(low, 25.0f, 0.05f);
}

/* 拐角内部全程按拐角限速。path_deviation 调大后贝塞尔的小段也变长，判断“小段”要跟着它变。 */
static void TestCornerLimitCoversWholeCorner(void)
{
    Begin();
    Line("/set path_deviation 20");
    Line("/set cmd_speed 400");
    float corner_x = chassis.pos_x + 300, corner_y = chassis.pos_y;
    Line("F300 L300");
    assert(chassis.mode == CHASSIS_MODE_PATH);
    float peak = 0.0f;
    for (unsigned i = 0; i < 5900U && chassis.mode == CHASSIS_MODE_PATH; ++i)
    {
        plant();
        if (hypotf(chassis.pos_x - corner_x, chassis.pos_y - corner_y) < 25.0f)
            peak = fmaxf(peak, hypotf(chassis.current.vx, chassis.current.vy));
    }
    assert(chassis_path_result.completed);
    assert(peak > 90.0f && peak <= 105.0f); // sqrt(2000 / 4 × 20) = 100
}

/* cmd_speed 从下一条路径开始生效，正在走的路径保持原限速。 */
static void TestCommandSpeedAppliesToNextPath(void)
{
    Begin();
    Line("/set cmd_speed 200");
    Line("F100");
    assert(chassis.mode == CHASSIS_MODE_PATH);
    AssertNear(Chassis_SpeedLimit(), 200.0f, 1e-3f);
    assert(strstr(Output(), "path queued (200 mm/s)"));

    Line("/set cmd_speed 300");
    assert(chassis.mode == CHASSIS_MODE_PATH);
    AssertNear(Chassis_SpeedLimit(), 200.0f, 1e-3f);

    Line("S");
    Line("F100");
    AssertNear(Chassis_SpeedLimit(), 300.0f, 1e-3f);
    Line("H");
    assert(strstr(Output(), "Move speed 300 mm/s"));
}

static void TestWheelDiameterScalesKinematics(void)
{
    Begin();
    chassis_vel forward = {100, 0, 0};
    int16_t rpm_x10[CHASSIS_WHEEL_NUM];
    Chassis_Solve(&forward, rpm_x10);
    assert(abs(rpm_x10[0]) == 248); // 100 mm/s ÷ (π × 77 mm) × 600

    Line("/set wheel_dia 140");
    Chassis_Solve(&forward, rpm_x10);
    assert(abs(rpm_x10[0]) == 136);

    /* 里程计：四轮各转一圈 = π × 140 mm。 */
    float x0 = chassis.pos_x;
    for (int i = 0; i < 4; ++i)
        motors[i].position_deg += 360.0f;
    tick(1);
    AssertNear(chassis.pos_x - x0, 439.8f, 0.5f);
}

/* 路径控制用轮速预测当前位置，预测也要跟着轮径变：60 RPM × 40 ms 在 140 mm 轮径下走 17.6 mm，
   离 25 mm 外的终点还剩 7.4 mm，近终点限速 8 × 距离 = 59 mm/s。还用 77 mm 的话是 100 mm/s。 */
static void TestWheelDiameterScalesPathPrediction(void)
{
    Begin();
    Line("/set wheel_dia 140");
    Line("F25");
    assert(chassis.mode == CHASSIS_MODE_PATH);
    for (int i = 0; i < 4; ++i)
        motors[i].speed_rpm = 60.0f;
    for (int i = 0; i < 9; ++i)
        tick(0);
    AssertNear(chassis.target.vx, 59.3f, 1.0f);
}

static void TestChassisKScalesRotation(void)
{
    Begin();
    chassis_vel turn = {0, 0, 90};
    int16_t rpm_x10[CHASSIS_WHEEL_NUM];
    Chassis_Solve(&turn, rpm_x10);
    assert(abs(abs(rpm_x10[0]) - 858) <= 1); // 220.2 mm × 90°/s ÷ (π × 77 mm) × 600

    Line("/set chassis_k 110.1");
    Chassis_Solve(&turn, rpm_x10);
    assert(abs(abs(rpm_x10[0]) - 429) <= 1);

#if !CHASSIS_USE_IMU
    /* 无陀螺仪时航向靠轮子推算：K 减半，同样的轮差算出两倍转角。 */
    float yaw0 = chassis.yaw;
    motors[0].position_deg -= 36.0f;
    motors[1].position_deg += 36.0f;
    motors[2].position_deg -= 36.0f;
    motors[3].position_deg += 36.0f;
    tick(1);
    /* 每轮走 π × 77 × 36 / 360 = 24.19 mm，dθ = 24.19 ÷ 110.1 rad = 12.59°。 */
    AssertNear(chassis.yaw - yaw0, 12.59f, 0.05f);
#endif
}

/* 这几个值一改，运动中的速度或路径就会跳，只能停车改。 */
static void TestStoppedOnlyParametersRefusedWhileMoving(void)
{
    static const char *const refused[] = {"/set wheel_dia 80", "/set chassis_k 200",
                                          "/set path_deviation 8", "/reset"};
    Begin();
    Line("/set wheel_dia 78");
    Line("F500");
    assert(chassis.mode == CHASSIS_MODE_PATH);
    for (unsigned i = 0; i < sizeof(refused) / sizeof(refused[0]); ++i)
    {
        ClearOutput();
        Line(refused[i]);
        assert(strstr(Output(), "BUSY"));
        assert(chassis.mode == CHASSIS_MODE_PATH); // 拒绝不等于停车
    }
    AssertNear(chassis_tune.wheel_dia, 78.0f, 1e-4f);
    AssertNear(chassis_tune.chassis_k, chassis_tune_default.chassis_k, 1e-4f);
    AssertNear(chassis_tune.path_deviation, chassis_tune_default.path_deviation, 1e-4f);

    /* 控制增益可以边跑边调。 */
    ClearOutput();
    Line("/set heading_kp 3");
    assert(strstr(Output(), "OK"));
    AssertNear(chassis_tune.heading_kp, 3.0f, 1e-4f);

    Line("S");
    Line("/set wheel_dia 80");
    AssertNear(chassis_tune.wheel_dia, 80.0f, 1e-4f);
}

static void TestBadInputChangesNothing(void)
{
    static const char *const bad[] = {
        "/set heading_kp 21",
        "/set heading_kp -1",
        "/set heading_kp nan",
        "/set heading_kp inf",
        "/set heading_kp 1e3",
        "/set heading_kp 2.5x",
        "/set heading_kp .",
        "/set heading_kp 1.2.3",
        "/set heading_kp --1",
        "/set heading_kp",
        "/set heading_kp 1 2",
        "/set nope 1",
        "/set",
        "/reset now",
        "/bogus",
        "/plot",
        "/plot nope",
        "/plot heading 0",
        "/plot heading 101",
        "/plot heading 1.5",
        "/list a b",
        "/list zzz",
        "/set heading_kp 999999999",    // 取整时会溢出的数也要被拒绝
        "/set heading_kp 1234567890",   // 超过 9 位
        "/set heading_kp 1.2345678901", // 位数太多会溢出成范围内的错误值
        "/plot off 5",
    };
    Begin();
    for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i)
    {
        ClearOutput();
        Line(bad[i]);
        if (!strstr(Output(), "ERR"))
        {
            fprintf(stderr, "no ERR for: %s\n", bad[i]);
            assert(0);
        }
        AssertNear(chassis_tune.heading_kp, 2.0f, 1e-6f);
    }
    assert(plot_group == TELEMETRY_PLOT_OFF);
}

/* 范围两端都能设。 */
static void TestRangeLimitsInclusive(void)
{
    Begin();
    Line("/set heading_kp 20");
    AssertNear(chassis_tune.heading_kp, 20.0f, 1e-6f);
    Line("/set heading_kp 0");
    AssertNear(chassis_tune.heading_kp, 0.0f, 1e-6f);
    Line("/set heading_kp +3"); // 正号也是合法写法
    AssertNear(chassis_tune.heading_kp, 3.0f, 1e-6f);
    Line("/set cmd_speed 400");
    AssertNear(chassis_tune.cmd_speed, 400.0f, 1e-6f);
    ClearOutput();
    Line("/set cmd_speed 401"); // 硬限速不能被控制台绕过
    assert(strstr(Output(), "ERR"));
    AssertNear(chassis_tune.cmd_speed, 400.0f, 1e-6f);
}

/* 存进去的值按小数位数取整，显示的就是实际用的。 */
static void TestValuesRoundToDisplayedDecimals(void)
{
    Begin();
    Line("/set heading_kp 2.3456");
    assert(chassis_tune.heading_kp == 2.346f);
    ClearOutput();
    Line("/list heading_kp");
    assert(strstr(Output(), "= 2.346 "));
    Line("/set settle_ms 123.6");
    AssertNear(chassis_tune.settle_ms, 124.0f, 1e-4f);
}

static void TestResetRestoresDefaults(void)
{
    Begin();
    Line("/set heading_ki 2");
    Line("/set wheel_dia 80");
    Line("/reset");
    AssertNear(chassis_tune.heading_ki, 1.0f, 1e-6f);
    AssertNear(chassis_tune.wheel_dia, 77.0f, 1e-6f);
}

/* 调试台界面按这些文本格式解析，格式变了界面就读不到，所以逐字固定。 */
static void TestReplyFormatsForUi(void)
{
    Begin();
    Line("/list heading_kp");
    assert(strcmp(Output(), "heading_kp      = 2.000  [0.000..20.000] default 2.000\r\n") == 0);

    ClearOutput();
    Line("/list wheel_dia");
    assert(strcmp(Output(),
                  "wheel_dia       = 77.00  [40.00..150.00] default 77.00 (stopped only)\r\n") ==
           0);

    ClearOutput();
    Line("/set heading_kp 2.5");
    assert(strcmp(Output(), "OK   heading_kp 2.000 -> 2.500\r\n") == 0);

    ClearOutput();
    Line("/set heading_kp 21");
    assert(strcmp(Output(), "ERR  heading_kp must be 0.000..20.000\r\n") == 0);

    Line("F500");
    ClearOutput();
    Line("/set wheel_dia 80");
    assert(strcmp(Output(), "BUSY wheel_dia can only change while stopped\r\n") == 0);
}

static void TestStatusLineFormatForUi(void)
{
    Begin();
    ChassisTelemetry_SetRate(1);
    chassis.pos_x = 12.5f;
    chassis.pos_y = -3.0f;
    chassis.yaw = 1.5f;
    chassis.heading_target = 2.0f;
    chassis.heading_error = 0.5f;
    chassis.current.vx = 10.0f;
    chassis.current.vy = -5.0f;
    chassis.current.wz = 1.5f;
    static const int16_t target[4] = {120, -130, 140, -150};
    static const float meas[4] = {11.0f, -12.0f, 13.0f, -14.0f};
    memcpy(chassis.wheel_rpm_x10, target, sizeof(target));
    memcpy(chassis.wheel_rpm_meas, meas, sizeof(meas));
    Wait(1000);
    assert(strstr(Output(), "] READY STOP x=12.5 y=-3.0 yaw=1.50 tgt=2.00 err=0.50 v=10/-5/1.5 "
                            "rpm(t/m) LF 12/11 RF -13/-12 LR 14/13 RR -15/-14\r\n"));
    ChassisTelemetry_SetRate(0);
    memset(&chassis.current, 0, sizeof(chassis.current));
    memset(chassis.wheel_rpm_x10, 0, sizeof(chassis.wheel_rpm_x10));
    memset(chassis.wheel_rpm_meas, 0, sizeof(chassis.wheel_rpm_meas));
}

static void TestList(void)
{
    static const char *const names[] = {
        "heading_kp",  "heading_ki",   "heading_i_max", "heading_i_zone", "turn_max_speed",
        "distance_kp", "distance_tol", "heading_tol",   "settle_ms",      "acc_linear",
        "acc_angular", "cmd_speed",    "wheel_dia",     "chassis_k",      "path_deviation",
    };
    Begin();
    Line("/list");
    for (unsigned i = 0; i < sizeof(names) / sizeof(names[0]); ++i)
        assert(strstr(Output(), names[i]));

    /* 前缀过滤：heading 开头的 5 个，其他都不显示。 */
    ClearOutput();
    Line("/list heading");
    assert(CountOutput("heading_") == 5U);
    assert(!strstr(Output(), "cmd_speed"));

    /* 改过后当前值和默认值都看得到。 */
    Line("/set heading_kp 3");
    ClearOutput();
    Line("/list heading_kp");
    assert(strstr(Output(), "= 3.000 ") && strstr(Output(), "default 2.000"));
}

/* 这个文件在指令模式下编译，比赛模式专用的命令不存在。 */
static void TestCompetitionCommandsAbsentInCommandMode(void)
{
    static const char *const absent[] = {
        "/task 156+123+516+231", "/vision 2 3",    "/obs",
        "/pose 0 0 0",           "/set lidar_x 1", "/list obs_gap"};
    Begin();
    for (unsigned i = 0; i < sizeof(absent) / sizeof(absent[0]); ++i)
    {
        ClearOutput();
        Line(absent[i]);
        assert(strstr(Output(), "ERR"));
    }
}

/* 名字里的 s 不能触发停车；控制台行之外的 S 仍然立即停车。 */
static void TestConsoleLineDoesNotStopRobot(void)
{
    Begin();
    Line("F500");
    assert(chassis.mode == CHASSIS_MODE_PATH);
    Line("/set heading_tol 0.2");
    assert(chassis.mode == CHASSIS_MODE_PATH);
    Line("/list heading_tol");
    assert(chassis.mode == CHASSIS_MODE_PATH);
    AssertNear(chassis_tune.heading_tol, 0.2f, 1e-6f);
    Line("S");
    assert(chassis.mode == CHASSIS_MODE_VEL);
}

static void TestNormalCommandsStillWork(void)
{
    Begin();
    Line("/set heading_kp 3");
    Line("F100");
    assert(chassis.mode == CHASSIS_MODE_PATH);
    Line("S");
    assert(chassis.mode == CHASSIS_MODE_VEL);
}

/* '/' 只有在行首才进控制台。 */
static void TestSlashOnlyCountsAtLineStart(void)
{
    Begin();
    Line("   /set heading_kp 3"); // 行首空格不算
    AssertNear(chassis_tune.heading_kp, 3.0f, 1e-6f);

    ClearOutput();
    Line("F100 /plot heading");
    assert(strstr(Output(), "ERR"));
    assert(chassis.mode == CHASSIS_MODE_VEL); // 整行作废
    assert(plot_group == TELEMETRY_PLOT_OFF);

    /* 紧跟命令字母、紧跟乱码、前面已有错误命令，都不是行首。 */
    const char *const not_at_start[] = {"F/plot heading", "Q/plot heading", "Q1 /plot heading"};
    for (unsigned i = 0; i < 3U; ++i)
    {
        ClearOutput();
        Line(not_at_start[i]);
        assert(strstr(Output(), "ERR"));
        assert(plot_group == TELEMETRY_PLOT_OFF);
    }

    /* S 之后这一行剩下的内容都丢弃。 */
    ClearOutput();
    Line("S /set heading_kp 9");
    AssertNear(chassis_tune.heading_kp, 3.0f, 1e-6f);
}

static void TestCommandsCaseInsensitive(void)
{
    Begin();
    Line("/SET Heading_KP 3");
    AssertNear(chassis_tune.heading_kp, 3.0f, 1e-6f);
}

static void TestLineEndingVariants(void)
{
    Begin();
    Burst("/set heading_kp 3\r", 1);
    AssertNear(chassis_tune.heading_kp, 3.0f, 1e-6f);
    Burst("/set heading_kp 4\n", 1);
    AssertNear(chassis_tune.heading_kp, 4.0f, 1e-6f);
    /* 串口助手不加换行：串口空闲就算行尾，和单字母命令一样。 */
    Burst("/set heading_kp 6", 1);
    AssertNear(chassis_tune.heading_kp, 6.0f, 1e-6f);
    /* 一包多行。 */
    Burst("/set heading_kp 5\r\n/set heading_ki 2\r\n/set cmd_speed 150\r\n", 1);
    AssertNear(chassis_tune.heading_kp, 5.0f, 1e-6f);
    AssertNear(chassis_tune.heading_ki, 2.0f, 1e-6f);
    AssertNear(chassis_tune.cmd_speed, 150.0f, 1e-6f);
    /* HAL 缓冲写满的回调没有空闲事件，后面的数据还属于这一行。 */
    Burst("/set heading_kp 2", 0);
    AssertNear(chassis_tune.heading_kp, 5.0f, 1e-6f);
    Burst(".5\r\n", 1);
    AssertNear(chassis_tune.heading_kp, 2.5f, 1e-6f);
}

/* 太长的行整行拒绝：这一行截断后恰好是合法命令，不能只执行前半段。 */
static void TestTooLongLineRejected(void)
{
    char longline[120];
    memset(longline, 0, sizeof(longline));
    memcpy(longline, "/set heading_kp 3", 17);
    memset(longline + 17, ' ', 80);
    Begin();
    Line(longline);
    assert(strstr(Output(), "ERR"));
    AssertNear(chassis_tune.heading_kp, 2.0f, 1e-6f);
    assert(chassis.mode == CHASSIS_MODE_VEL);
    Line("/set heading_kp 3");
    AssertNear(chassis_tune.heading_kp, 3.0f, 1e-6f);
}

/* 接收溢出：照旧停车、丢弃这一行，之后控制台还能用。 */
static void TestReceiveOverflowDuringConsoleLine(void)
{
    Begin();
    Line("F500");
    assert(chassis.mode == CHASSIS_MODE_PATH);
    Burst("/set heading_kp 3", 0);
    for (unsigned i = 0; i < 300; ++i)
        Push('x');
    ChassisCmd_Poll();
    assert(chassis.mode == CHASSIS_MODE_VEL);
    Burst("\r\n", 1); // 溢出前收到的半行要作废，不能在这里被当成完整命令执行
    AssertNear(chassis_tune.heading_kp, 2.0f, 1e-6f);
    /* 溢出那一行没结束时，后面的 '/' 也不算行首。 */
    Burst("/set heading_kp 3", 0);
    for (unsigned i = 0; i < 300; ++i)
        Push('x');
    ChassisCmd_Poll();
    Burst("/set heading_kp 9\r\n", 1);
    AssertNear(chassis_tune.heading_kp, 2.0f, 1e-6f);
    Line("/set heading_kp 3");
    AssertNear(chassis_tune.heading_kp, 3.0f, 1e-6f);
}

static void TestHelpTexts(void)
{
    Begin();
    Line("H");
    /* 帮助的最后几行也要完整出来，不能被单次 Printf 的长度上限截断。 */
    assert(strstr(Output(), "H or ?"));
    assert(strstr(Output(), "Move speed 100 mm/s"));
    assert(strstr(Output(), "/help"));

    static const char *const console_help[] = {"/help", "/"};
    for (unsigned i = 0; i < 2; ++i)
    {
        ClearOutput();
        Line(console_help[i]);
        assert(strstr(Output(), "/set <name> <value>"));
        assert(strstr(Output(), "/plot"));
        assert(strstr(Output(), "/reset"));
        assert(!strstr(Output(), "ERR"));
    }
}

static void TestPlotChannels(void)
{
    Begin();
    Line("/plot heading 100");
    ClearOutput();
    chassis.heading_target = 12.5f;
    chassis.yaw = 10.0f;
    chassis.heading_error = 2.5f;
    chassis.target.wz = 1.0f;
    chassis.current.wz = 0.5f;
    Wait(10);
    assert(strstr(Output(), "plot:12.500,10.000,2.500,1.000,0.500\r\n"));

    Line("/plot speed 100");
    ClearOutput();
    chassis.target.vx = 100.0f;
    chassis.target.vy = -50.0f;
    chassis.current.vx = 90.0f;
    chassis.current.vy = -45.0f;
    for (int i = 0; i < 4; ++i)
        chassis.wheel_rpm_meas[i] = 60.0f; // 四轮同速：前进 60 RPM × π × 77 mm ÷ 60 s
    Wait(10);
    assert(strstr(Output(), "plot:100.0,-50.0,90.0,-45.0,241.9,0.0\r\n"));
    /* 横移：实测速度落在 vy。 */
    ClearOutput();
    chassis.wheel_rpm_meas[0] = -60.0f;
    chassis.wheel_rpm_meas[1] = 60.0f;
    chassis.wheel_rpm_meas[2] = 60.0f;
    chassis.wheel_rpm_meas[3] = -60.0f;
    Wait(10);
    assert(strstr(Output(), "plot:100.0,-50.0,90.0,-45.0,0.0,241.9\r\n"));

    Line("/plot wheels 100");
    ClearOutput();
    chassis.wheel_rpm_x10[0] = 100;
    chassis.wheel_rpm_x10[1] = -200;
    chassis.wheel_rpm_x10[2] = 300;
    chassis.wheel_rpm_x10[3] = -400;
    chassis.wheel_rpm_meas[0] = 9.5f;
    chassis.wheel_rpm_meas[1] = -19.5f;
    chassis.wheel_rpm_meas[2] = 29.5f;
    chassis.wheel_rpm_meas[3] = -39.5f;
    Wait(10);
    assert(strstr(Output(), "plot:10.0,-20.0,30.0,-40.0,9.5,-19.5,29.5,-39.5\r\n"));

    Line("/plot off");
    memset(&chassis.target, 0, sizeof(chassis.target));
    memset(&chassis.current, 0, sizeof(chassis.current));
    memset(chassis.wheel_rpm_x10, 0, sizeof(chassis.wheel_rpm_x10));
    memset(chassis.wheel_rpm_meas, 0, sizeof(chassis.wheel_rpm_meas));
}

/* 实测速度由轮速换算，要用标定后的轮径：60 RPM × π × 140 mm ÷ 60 s。 */
static void TestPlotMeasuredSpeedUsesWheelDiameter(void)
{
    Begin();
    Line("/set wheel_dia 140");
    Line("/plot speed 100");
    ClearOutput();
    for (int i = 0; i < 4; ++i)
        chassis.wheel_rpm_meas[i] = 60.0f;
    Wait(10);
    assert(strstr(Output(), "plot:0.0,0.0,0.0,0.0,439.8,0.0\r\n"));
    Line("/plot off");
    memset(chassis.wheel_rpm_meas, 0, sizeof(chassis.wheel_rpm_meas));
}

static unsigned PlotLinesInOneSecond(void)
{
    ClearOutput();
    for (int i = 0; i < 200; ++i) // 200 × 5 ms
    {
        Wait(5);
        drain(&uart);
    }
    return CountOutput("plot:");
}

static void TestPlotRate(void)
{
    Begin();
    Line("/plot heading 50");
    assert(PlotLinesInOneSecond() == 50U);
    Line("/plot heading");
    assert(PlotLinesInOneSecond() == 50U); // 默认 50 Hz
    Line("/plot heading 100");
    assert(PlotLinesInOneSecond() == 100U);
    /* 周期不是节拍整数倍时，平均频率也要对。 */
    Line("/plot heading 30");
    unsigned lines = PlotLinesInOneSecond();
    assert(lines >= 29U && lines <= 31U);
    Line("/plot off");
    assert(PlotLinesInOneSecond() == 0U);
}

/* 开始绘图时回复列名，界面靠它给曲线命名。 */
static void TestPlotReportsColumns(void)
{
    Begin();
    ClearOutput();
    Line("/plot wheels 10");
    assert(strstr(Output(), "OK   plot wheels 10 Hz: target_lf,"));
    Line("/plot off");
    ClearOutput();
    Line("/plot off");
    assert(strstr(Output(), "OK   plot off"));
}

int main(void)
{
    FDCAN_HandleTypeDef can = {0};
    assert(Chassis_Init(&can) == HAL_OK);
    tick(1);
    assert(ChassisCmd_Init(&uart) == HAL_OK);
    ChassisCmd_Poll();
    ChassisTelemetry_SetRate(0);
    for (int i = 0; i < 4; ++i)
        tick(1);
    assert(chassis.state == CHASSIS_STATE_READY);

    TestHeadingLoopFollowsParameters();
    TestZeroKiClearsIntegral();
    TestDistanceLoopFollowsParameters();
    TestSettleTimeFollowsParameter();
    TestDistanceTolDefinesStationary();
    TestHeadingTolDefinesSettled();
    TestPathBrakesToWaypointWithTunedAcceleration();
    TestPathLooksAheadWithTunedAcceleration();
    TestCornerExitSpeedFollowsParameters();
    TestDistanceTolDefinesPathFinish();
    TestHeadingTolDefinesPathFinish();
    TestRampsFollowParameters();
    TestCornerSpeedFollowsParameters();
    TestCornerLimitCoversWholeCorner();
    TestCommandSpeedAppliesToNextPath();
    TestWheelDiameterScalesKinematics();
    TestWheelDiameterScalesPathPrediction();
    TestChassisKScalesRotation();
    TestStoppedOnlyParametersRefusedWhileMoving();
    TestBadInputChangesNothing();
    TestRangeLimitsInclusive();
    TestValuesRoundToDisplayedDecimals();
    TestResetRestoresDefaults();
    TestReplyFormatsForUi();
    TestStatusLineFormatForUi();
    TestList();
    TestCompetitionCommandsAbsentInCommandMode();
    TestConsoleLineDoesNotStopRobot();
    TestNormalCommandsStillWork();
    TestSlashOnlyCountsAtLineStart();
    TestCommandsCaseInsensitive();
    TestLineEndingVariants();
    TestTooLongLineRejected();
    TestReceiveOverflowDuringConsoleLine();
    TestHelpTexts();
    TestPlotChannels();
    TestPlotMeasuredSpeedUsesWheelDiameter();
    TestPlotRate();
    TestPlotReportsColumns();
    puts("PASS: console parameters drive chassis behaviour, bad input, stopped-only, rounding, "
         "reset, UI text formats, line endings, overflow, help, plot");
    return 0;
}
