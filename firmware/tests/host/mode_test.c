#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "chassis.h"
#include "agv_config.h"
/* 巡检和机械臂模式不接受手动运动。 */
#define LOCKED (AGV_MODE == AGV_MODE_SURVEY || AGV_MODE == AGV_MODE_ARM)
static unsigned move_calls, stop_calls, latches;
chassis_t chassis;
void ChassisTelemetry_Init(UART_HandleTypeDef *u)
{
    (void)u;
}
void ChassisTelemetry_Ack(char c, HAL_StatusTypeDef r)
{
    (void)c;
    (void)r;
}
void ChassisTelemetry_Poll(void)
{
}
void ChassisTelemetry_Printf(const char *f, ...)
{
    (void)f;
}
void ChassisTelemetry_SetRate(unsigned hz)
{
    (void)hz;
}
void ChassisTelemetry_Help(void)
{
}
void ChassisTelemetry_Info(void)
{
}
static unsigned wheel_calls;
HAL_StatusTypeDef Chassis_WheelTest(uint8_t mask)
{
    (void)mask;
    ++wheel_calls;
    return HAL_OK;
}
void Chassis_SetMoveSpeedLimit(float x)
{
    (void)x;
}
HAL_StatusTypeDef Chassis_FollowPath(const chassis_waypoint *p, unsigned n, uint8_t s)
{
    (void)p;
    (void)n;
    (void)s;
    ++move_calls;
    return HAL_OK;
}
HAL_StatusTypeDef Chassis_AppendPath(const chassis_waypoint *p, unsigned n)
{
    (void)p;
    (void)n;
    ++move_calls;
    return HAL_OK;
}
void Chassis_Stop(void)
{
    ++stop_calls;
}
#if AGV_MODE == AGV_MODE_COMPETITION
static unsigned run_stops;
static uint8_t run_going;
void Comp_Stop(void)
{
    ++run_stops;
    run_going = 0U;
}
uint8_t Comp_Running(void)
{
    return run_going;
}
#endif
void Agv_Stop(void)
{
    ++latches;
    Chassis_Stop();
}
#if AGV_MODE == AGV_MODE_ARM
static unsigned arm_stops;
void Arm_Stop(void)
{
    ++arm_stops;
}
#endif
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef *u, uint8_t *b, uint16_t n)
{
    (void)u;
    (void)b;
    (void)n;
    return HAL_OK;
}
uint32_t HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *u)
{
    (void)u;
    return HAL_UART_RXEVENT_IDLE;
}
chassis_tuning chassis_tune = CHASSIS_TUNE_DEFAULTS;
static unsigned console_calls;
static char console_seen[64];
void Console_Line(char *line)
{
    assert(strlen(line) < sizeof(console_seen));
    strcpy(console_seen, line);
    ++console_calls;
}
#include "../../User/Comm/chassis_cmd.c"
int main(void)
{
    UART_HandleTypeDef u = {0};
    assert(ChassisCmd_Init(&u) == HAL_OK);
    /* 控制台行在三种模式下都可用，行里的 s 不触发停车。 */
    static const char console_text[] = "/set heading_tol 2\r\n";
    memcpy(rx_buf, console_text, sizeof(console_text) - 1U);
    ChassisCmd_RxEvent(&u, sizeof(console_text) - 1U);
    ChassisCmd_Poll();
    assert(console_calls == 1U && strcmp(console_seen, "set heading_tol 2") == 0);
    assert(stop_calls == 0U && latches == 0U);
    /* 巡检模式路线固定、机械臂模式底盘一直停着，不接受手动运动；比赛模式流程没跑时可以手动开车调试。
     */
    memcpy(rx_buf, "F100", 4);
    ChassisCmd_RxEvent(&u, 4);
    ChassisCmd_Poll();
    assert(move_calls == (LOCKED ? 0U : 1U));
    memcpy(rx_buf, "S", 1);
    ChassisCmd_RxEvent(&u, 1);
    ChassisCmd_Poll();
    assert(stop_calls == 1 && latches == (AGV_MODE == AGV_MODE_COMMAND ? 0U : 1U));
    for (unsigned i = 0; i < 300; ++i)
        Push('F');
    ChassisCmd_Poll();
    assert(stop_calls == 2 && latches == (AGV_MODE == AGV_MODE_COMMAND ? 0U : 2U));
    memcpy(rx_buf, "\nW\n", 3); /* first \n ends the line discarded after the overflow */
    ChassisCmd_RxEvent(&u, 3);
    ChassisCmd_Poll();
    assert(wheel_calls == (LOCKED ? 0U : 1U));
#if AGV_MODE == AGV_MODE_ARM
    assert(arm_stops == 2U); /* S 和接收溢出都停机械臂 */
#endif
#if AGV_MODE == AGV_MODE_COMPETITION
    /* 比赛流程运行中不接受手动运动和单轮测试，S 能停流程。 */
    unsigned moves = move_calls, wheels = wheel_calls, stops = run_stops;
    run_going = 1U;
    memcpy(rx_buf, "F100\n", 5);
    ChassisCmd_RxEvent(&u, 5);
    ChassisCmd_Poll();
    memcpy(rx_buf, "W\n", 2);
    ChassisCmd_RxEvent(&u, 2);
    ChassisCmd_Poll();
    assert(move_calls == moves && wheel_calls == wheels && run_going);
    memcpy(rx_buf, "S\n", 2);
    ChassisCmd_RxEvent(&u, 2);
    ChassisCmd_Poll();
    assert(run_stops == stops + 1U && !run_going);
    memcpy(rx_buf, "F100\n", 5);
    ChassisCmd_RxEvent(&u, 5);
    ChassisCmd_Poll();
    assert(move_calls == moves + 1U);
#endif
    printf("mode %d command gating/stop/overflow OK\n", AGV_MODE);
}
