#define main original_chassis_tests
#include "chassis_test.c"
#undef main

/* 验证空闲结束、分片、无效行和紧急停车语义。 */
int main(void)
{
    FDCAN_HandleTypeDef can = {0};
    UART_HandleTypeDef uart = {0};
    assert(Chassis_Init(&can) == HAL_OK);
    tick(1);
    assert(ChassisCmd_Init(&uart) == HAL_OK);
    event_type = 0;
    command(&uart, "F5");
    assert(chassis.mode == CHASSIS_MODE_VEL);
    event_type = HAL_UART_RXEVENT_IDLE;
    command(&uart, "00 L300 T90 B200");
    assert(chassis_cmd_status == HAL_OK);
    assert(chassis.mode == CHASSIS_MODE_PATH);
    assert(fabsf(planned.x - 500) < .01f);
    assert(fabsf(planned.y - 100) < .01f);
    assert(fabsf(planned.heading - 90) < .01f);
    command(&uart, "S F500");
    assert(chassis.mode == CHASSIS_MODE_VEL && pending_count == 0);
    command(&uart, "F100 X9");
    assert(chassis_cmd_status == HAL_ERROR && chassis.mode == CHASSIS_MODE_VEL);
    command(&uart, "F20");
    assert(chassis_cmd_status == HAL_OK && chassis.mode == CHASSIS_MODE_PATH);
    /* 参考折线只有 257 点，连续追加超过容量后仍必须接受（旧实现会一直报 ERROR）。 */
    float before_x = planned.x, before_y = planned.y;
    for (unsigned i = 0; i < 300; ++i)
    {
        command(&uart, "F10");
        assert(chassis_cmd_status == HAL_OK);
    }
    assert(fabsf(hypotf(planned.x - before_x, planned.y - before_y) - 3000.0f) < 1.0f);
    command(&uart, "S");
    drain(&uart);
    tx_log[0] = 0;
    uart.gState = 1; /* UART busy: the summary is still captured once, sent later */
    chassis_path_result = (chassis_path_stats){123, 2, .1f, 456, 1};
    ChassisTelemetry_Poll();
    chassis_path_result.distance_mm = 999;
    drain(&uart);
    assert(
        strstr(tx_log, "DONE path: time=0.46s dist=123.0mm max_cross=2.00mm max_heading=0.10deg"));
    assert(strstr(tx_log, "999") == NULL);
    puts("PASS: idle commit, TC fragments, blended yaw, long append, S clearing, invalid atomic "
         "line, immutable completion stats");
    return 0;
}
