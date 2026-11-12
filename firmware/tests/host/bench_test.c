/* 底盘单独调试功能：单轮测试、堵转保护恢复、故障原因文本、无陀螺仪航向推算。
   run_tests.py 会编两次：默认（带 IMU）和 -DCHASSIS_USE_IMU=0。 */
#define main original_chassis_tests
#include "chassis_test.c"
#undef main

static void run_ticks(int n)
{
    for (int i = 0; i < n; i++)
        plant();
}

int main(void)
{
    FDCAN_HandleTypeDef can = {0};
    UART_HandleTypeDef uart = {0};
    assert(DWT->LAR == 0U);
    assert(Chassis_Init(&can) == HAL_OK);
    assert(DWT->LAR == 0xC5ACCE55U); /* Cortex-M7 DWT unlocked, CYCCNT counts without debugger */
    tick(1);
    assert(chassis.state == CHASSIS_STATE_READY);
    assert(ChassisCmd_Init(&uart) == HAL_OK);
    drain(&uart);

#if !CHASSIS_USE_IMU
    /* 没接陀螺仪也能 READY；航向由四轮推算，转向/直行闭环都能完成。 */
    imu.valid = 0;
    imu.yaw = 0;
    tick(1);
    assert(chassis.state == CHASSIS_STATE_READY);
    command(&uart, "T90\n");
    assert(chassis_cmd_status == HAL_OK);
    complete();
    assert(fabsf(chassis.yaw - 90.0f) <= 0.15f);
    assert(fabsf(Math_WrapDeg(imu.yaw - chassis.yaw)) < 0.5f); /* plant's true yaw agrees */
    float x0 = chassis.pos_x, y0 = chassis.pos_y;
    command(&uart, "F300\n");
    complete();
    assert(fabsf(chassis.pos_x - x0) < 5 && fabsf(chassis.pos_y - y0 - 300) < 5);
    command(&uart, "A0\n");
    complete();
    puts("PASS: no-IMU build: READY without HWT101, wheel-odometry heading, turn + move");
    return 0;
#endif

    /* ---- 单轮测试：一次只转一个轮子，按 LF RF LR RR 顺序，结果只打印一次 ---- */
    tx_log[0] = 0;
    command(&uart, "W\n");
    assert(chassis_cmd_status == HAL_OK && chassis.mode == CHASSIS_MODE_WHEEL);
    unsigned seen_order[4] = {0}, seen = 0;
    int last_active = -1;
    for (int n = 0; n < 2000 && chassis.mode == CHASSIS_MODE_WHEEL; ++n)
    {
        plant();
        int spinning = 0, which = -1;
        for (int i = 0; i < 4; ++i)
            if (output[i] != 0)
                ++spinning, which = i;
        assert(spinning <= 1);
        if (spinning && which != last_active)
        {
            assert(output[which] > 29.9f && output[which] < 30.1f);
            seen_order[seen++] = (unsigned)which;
            last_active = which;
        }
        ChassisTelemetry_Poll();
        uart.gState = 0;
    }
    assert(chassis.mode == CHASSIS_MODE_VEL && seen == 4);
    for (unsigned i = 0; i < 4; ++i)
    {
        assert(seen_order[i] == i);
        assert(chassis_wheel_result.verdict[i] == CHASSIS_WT_OK);
        assert(fabsf(chassis_wheel_result.delta_deg[i] - 180.0f) < 10.0f);
    }
    drain(&uart);
    assert(strstr(tx_log, "WHEEL LF spinning now") && strstr(tx_log, "WHEEL RR spinning now"));
    assert(strstr(tx_log, "WHEEL RF encoder: expect +180deg") && strstr(tx_log, "-> OK"));
    assert(strstr(tx_log, "DONE wheel test: encoders all OK"));

    /* 单个轮子 W3；电机不动（位置不更新）判 NO MOTION */
    tx_log[0] = 0;
    command(&uart, "W3\n");
    assert(chassis_cmd_status == HAL_OK);
    for (int n = 0; n < 1000 && chassis.mode == CHASSIS_MODE_WHEEL; ++n)
    {
        float frozen = motors[2].position_deg;
        plant();
        motors[2].position_deg = frozen;
        motors[2].speed_rpm = 0;
    }
    assert(chassis_wheel_result.mask == 0x04 && chassis_wheel_result.verdict[2] == CHASSIS_WT_NONE);
    drain(&uart);
    assert(strstr(tx_log, "WHEEL LR encoder") && strstr(tx_log, "NO MOTION"));
    assert(strstr(tx_log, "WHEEL LF encoder") == NULL);

    /* W5 无效；S 中途打断不出结果 */
    command(&uart, "W5\n");
    assert(chassis_cmd_status == HAL_ERROR);
    uint8_t seq = chassis_wheel_result.seq;
    command(&uart, "W1\n");
    run_ticks(20);
    command(&uart, "S\n");
    run_ticks(400);
    assert(chassis.mode == CHASSIS_MODE_VEL && chassis_wheel_result.seq == seq);
    assert(output[0] == 0);

    /* ---- 堵转保护：报具体轮子；S 发解除保护并自动回 READY ---- */
    tx_log[0] = 0;
    motors[1].protected_flag = 1;
    plant();
    assert(chassis.state == CHASSIS_STATE_FAULT && chassis.fault == CHASSIS_FAULT_SENSOR);
    assert(chassis.fault_detail & CHASSIS_WAIT_STALL(1));
    drain(&uart);
    assert(strstr(tx_log, "-> FAULT") && strstr(tx_log, "RF:stall"));
    command(&uart, "W\n");
    assert(chassis_cmd_status == HAL_BUSY); /* not while faulted */
    unsigned clears = clear_calls;
    command(&uart, "S\n");
    assert(chassis.state == CHASSIS_STATE_FAULT); /* flag still latched in the driver */
    run_ticks(40);
    assert(clear_calls > clears && chassis.state == CHASSIS_STATE_READY);
    assert(chassis.fault == CHASSIS_FAULT_NONE);

    /* 掉使能的电机：S 重新使能 */
    motors[3].enabled = 0;
    tick(0);
    assert(chassis.state == CHASSIS_STATE_FAULT && (chassis.fault_detail & CHASSIS_WAIT_EN(3)));
    command(&uart, "S\n");
    /* (the fake drivers report enabled again as soon as they are asked) */
    for (int i = 0; i < 40 && chassis.state != CHASSIS_STATE_READY; ++i)
        tick(1);
    assert(chassis.state == CHASSIS_STATE_READY);

    /* 定距超时的故障原因 */
    tx_log[0] = 0;
    assert(Chassis_MoveDistance(100, 0) == HAL_OK);
    for (int i = 0; i < 6100; i++)
        tick(1); // fresh sensors, wheels never move
    assert(chassis.state == CHASSIS_STATE_FAULT && chassis.fault == CHASSIS_FAULT_MOVE_TIMEOUT);
    drain(&uart);
    assert(strstr(tx_log, "move/turn timeout"));
    command(&uart, "S\n");
    assert(chassis.state == CHASSIS_STATE_READY);

    /* 非法命令有可读的错误提示 */
    tx_log[0] = 0;
    command(&uart, "F0\n");
    command(&uart, "Q\n");
    drain(&uart);
    assert(strstr(tx_log, "ERR  "));
    puts("PASS: wheel test order/verdicts/abort, stall-protection + disabled-motor recovery, "
         "fault reasons, DWT unlock, error text");
    return 0;
}
