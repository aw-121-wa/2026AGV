#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#undef assert
#define assert(x)                                                                                  \
    do                                                                                             \
    {                                                                                              \
        if (!(x))                                                                                  \
        {                                                                                          \
            fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #x);                                   \
            exit(1);                                                                               \
        }                                                                                          \
    } while (0)
#include "main.h"
#include "hwt101.h"
#include "zdt_x42s.h"
host_dwt host_clock;
host_debug host_core;
uint32_t SystemCoreClock = 480000000U;
static uint32_t ms;
imu_data imu;
static zdt_motor motors[4];
static float output[4]; /* RPM actually commanded to each wheel */
static unsigned sync_triggers;
static int fail_send;
static uint32_t event_type = HAL_UART_RXEVENT_IDLE;
static char tx_capture[512];
static char tx_log[1 << 16]; /* everything ever sent, for text assertions */
static const uint8_t *tx_active;
static uint16_t tx_length;
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u, uint8_t *b, uint16_t n)
{
    if (u->gState)
        return HAL_BUSY;
    assert(n < sizeof(tx_capture));
    memcpy(tx_capture, b, n);
    tx_capture[n] = 0;
    size_t used = strlen(tx_log);
    if (used + n < sizeof(tx_log))
        memcpy(tx_log + used, b, n + 1U), tx_log[used + n] = 0;
    tx_active = b;
    tx_length = n;
    u->gState = 1;
    return HAL_OK;
}
uint32_t HAL_GetTick(void)
{
    return ms;
}
HAL_StatusTypeDef ZDT_Init(FDCAN_HandleTypeDef *p)
{
    (void)p;
    return HAL_OK;
}
zdt_motor *ZDT_AddMotor(uint8_t a)
{
    motors[a - 1].address = a;
    return &motors[a - 1];
}
static float cached[4];
HAL_StatusTypeDef ZDT_SetSpeed(zdt_motor *m, int16_t rpm_x10, uint16_t acc, zdt_sync sync)
{
    (void)acc;
    if (fail_send)
        return HAL_ERROR;
    if (sync == ZDT_SYNC_CACHE)
        cached[m->address - 1] = rpm_x10 * 0.1f;
    else
        output[m->address - 1] = rpm_x10 * 0.1f;
    return HAL_OK;
}
HAL_StatusTypeDef ZDT_SyncTrigger(void)
{
    if (fail_send)
        return HAL_ERROR;
    memcpy(output, cached, sizeof(output));
    ++sync_triggers;
    return HAL_OK;
}
HAL_StatusTypeDef ZDT_Stop(zdt_motor *m, zdt_sync sync)
{
    (void)sync;
    if (fail_send)
        return HAL_ERROR;
    output[m->address - 1] = 0;
    return HAL_OK;
}
uint32_t ZDT_TxFreeLevel(void)
{
    return 16U;
}
HAL_StatusTypeDef ZDT_EnableMotor(zdt_motor *m, uint8_t en)
{
    m->enabled = en;
    return HAL_OK;
}
HAL_StatusTypeDef ZDT_RequestPosition(zdt_motor *m)
{
    (void)m;
    return HAL_OK;
}
HAL_StatusTypeDef ZDT_RequestSpeed(zdt_motor *m)
{
    (void)m;
    return HAL_OK;
}
HAL_StatusTypeDef ZDT_RequestStatus(zdt_motor *m)
{
    (void)m;
    return HAL_OK;
}
void ZDT_ProcessData(void)
{
}
void ZDT_Service(void)
{
}
static unsigned clear_calls;
HAL_StatusTypeDef ZDT_ClearProtection(zdt_motor *m)
{
    m->stalled = m->protected_flag = 0;
    ++clear_calls;
    return HAL_OK;
}
uint32_t ZDT_BusOffCount(void)
{
    return 0U;
}
uint32_t ZDT_RxDropped(void)
{
    return 0U;
}
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
    return event_type;
}
#include "../../User/Chassis/chassis.c"
#include "../../User/Comm/chassis_telemetry.c"
#include "../../User/Comm/chassis_cmd.c"
#include "../../User/Comm/console.c"
void HWT101_RxEvent(UART_HandleTypeDef *u, uint16_t n)
{
    (void)u;
    (void)n;
}
void HWT101_RxError(UART_HandleTypeDef *u)
{
    (void)u;
}
#include "../../User/Comm/uart_callbacks.c"

static void feedback(void)
{
    imu.valid = 1;
    imu.updated_ms = ms;
    for (int i = 0; i < 4; i++)
    {
        motors[i].enabled = 1;
        motors[i].status_valid = 1;
        motors[i].speed_valid = 1;
        motors[i].position_seq++;
        motors[i].position_ms = motors[i].status_ms = motors[i].speed_ms = ms;
    }
}
static void tick(int fresh)
{
    ms += 5;
    host_clock.CYCCNT += 2400000;
    if (fresh)
        feedback();
    Chassis_Update();
}
static void plant(void)
{
    /* Ideal wheels, actual output RPM; yaw follows inverse kinematics. */
    float v[4];
    for (int i = 0; i < 4; i++)
    {
        motors[i].speed_rpm = output[i];
        motors[i].position_deg += output[i] * 6.0f * 0.005f;
        v[i] = output[i] * 3.14159265f * CHASSIS_WHEEL_DIA_MM / 60.0f;
    }
    imu.yaw = Math_WrapDeg(imu.yaw + (-v[0] + v[1] - v[2] + v[3]) / (4 * CHASSIS_K_MM) *
                                         57.2957795f * 0.005f);
    tick(1);
}
static void complete(void)
{
    for (int i = 0; i < 5900 && chassis.mode != CHASSIS_MODE_VEL; i++)
        plant();
    assert(chassis.state == CHASSIS_STATE_READY);
    assert(chassis.mode == CHASSIS_MODE_VEL);
}
static void sampled_move(void)
{
    float position[4];
    for (int i = 0; i < 4; i++)
        position[i] = motors[i].position_deg;
    float x = chassis.pos_x;
    assert(Chassis_MoveDistance(500, 0) == HAL_OK);
    int steps = 0;
    while (chassis.mode != CHASSIS_MODE_VEL && steps++ < 5900)
    {
        float v[4];
        for (int i = 0; i < 4; i++)
        {
            position[i] += output[i] * 6.0f * 0.005f;
            v[i] = output[i] * 3.14159265f * CHASSIS_WHEEL_DIA_MM / 60.0f;
        }
        imu.yaw = Math_WrapDeg(imu.yaw + (-v[0] + v[1] - v[2] + v[3]) / (4 * CHASSIS_K_MM) *
                                             57.2957795f * 0.005f);
        ms += 5;
        host_clock.CYCCNT += 2400000;
        imu.updated_ms = ms;
        if (steps % 4 == 0)
        { // All four positions sampled together every 20 ms.
            for (int i = 0; i < 4; i++)
            {
                motors[i].position_deg = position[i];
                motors[i].speed_rpm = output[i];
                motors[i].position_seq++;
                motors[i].position_ms = motors[i].speed_ms = motors[i].status_ms = ms;
            }
        }
        Chassis_Update();
    }
    assert(chassis.mode == CHASSIS_MODE_VEL && chassis.state == CHASSIS_STATE_READY);
    assert(fabsf(chassis.pos_x - x - 500) < 4);
}
/* Let the fake UART finish every chunk so all queued text reaches tx_log. */
static void drain(UART_HandleTypeDef *u)
{
    for (int i = 0; i < 64; ++i)
    {
        u->gState = 0;
        ChassisTelemetry_Poll();
    }
    u->gState = 0;
}
static void command(UART_HandleTypeDef *u, const char *text)
{
    size_t n = strlen(text);
    assert(n <= CMD_BUF_SIZE);
    memcpy(rx_buf, text, n);
    HAL_UARTEx_RxEventCallback(u, (uint16_t)n);
    ChassisCmd_Poll();
}
int main(void)
{
    FDCAN_HandleTypeDef can = {0};
    UART_HandleTypeDef uart = {0};
    assert(Chassis_Init(&can) == HAL_OK);
    assert(Chassis_TurnTo(90) == HAL_BUSY);
    tick(1);
    assert(chassis.state == CHASSIS_STATE_READY);
    assert(ChassisCmd_Init(&uart) == HAL_OK);
    command(&uart, "S\n");
    /* While HAL owns the IT buffer, polling (and new text) must not overwrite it. */
    ms += 100;
    command(&uart, "I\n");
    ChassisCmd_Poll();
    assert(memcmp(tx_active, tx_capture, tx_length) == 0);
    drain(&uart);
    assert(strstr(tx_log, "2026AGV chassis firmware") && strstr(tx_log, "OK   stop"));
    assert(strstr(tx_log, "EVT  ") && strstr(tx_log, "-> READY"));
    assert(strstr(tx_log, "motor LF addr=1") && strstr(tx_log, "CAN: bus_off=0"));
    command(&uart, "?\n");
    drain(&uart);
    assert(strstr(tx_log, "---- commands"));
    ChassisTelemetry_SetRate(10);
    ms += 200;
    tx_log[0] = 0;
    drain(&uart);
    assert(strstr(tx_log, "] READY STOP x=") && strstr(tx_log, "rpm(t/m) LF"));
    command(&uart, "D11\n");
    assert(chassis_cmd_status == HAL_ERROR);
    command(&uart, "D0\n");
    assert(chassis_cmd_status == HAL_OK);
    ms += 2000;
    tx_log[0] = 0;
    drain(&uart);
    assert(strstr(tx_log, "] READY") == NULL); /* status line off */
    tick(1);
    Chassis_Stop(); /* time jumped without feedback above: clear and continue */
    tick(1);
    assert(chassis.state == CHASSIS_STATE_READY);
    /* Small turns must not be accepted while still outside 0.1 degree. */
    assert(Chassis_TurnTo(0.3f) == HAL_OK);
    complete();
    assert(fabsf(chassis.yaw - 0.3f) <= 0.1f);
    imu.yaw += 0.4f;
    for (int i = 0; i < 2000; i++)
        plant();
    assert(fabsf(chassis.yaw - 0.3f) <= 0.1f); // Normal completion maintains heading.
    Chassis_Stop();
    imu.yaw += 0.4f;
    tick(1);
    assert(chassis.target.wz == 0); // Explicit stop cancels holding.
    command(&uart, "A0.125\n");
    assert(chassis_cmd_status == HAL_OK);
    complete();
    assert(fabsf(chassis.yaw - 0.125f) <= 0.1f);
    command(&uart, "A1.2345\n");
    assert(chassis_cmd_status == HAL_ERROR);
    /* New IMU data remains visible even when a wheel is stale. */
    ms += 600;
    imu.updated_ms = ms;
    imu.yaw = 12.25f;
    host_clock.CYCCNT += 2400000;
    Chassis_Update();
    assert(fabsf(chassis.yaw - 12.25f) < 0.001f);
    assert(chassis.state == CHASSIS_STATE_FAULT);
    tick(1);
    Chassis_Stop();
    Chassis_SetVelocity(100, 0, 0);
    imu.yaw += 0.15f;
    for (int i = 0; i < 200; i++)
    {
        Chassis_SetVelocity(100, 0, 0);
        tick(1);
    }
    assert(chassis.target.wz < -0.3f && chassis.target.wz > -0.6f);
    assert(chassis.wheel_rpm_x10[0] !=
           chassis.wheel_rpm_x10[1]); /* sub-RPM correction reaches wheels */
    Chassis_Stop();
    chassis_vel combined = {1200, 1200, 240};
    int16_t limited[4];
    Chassis_Solve(&combined, limited);
    for (int i = 0; i < 4; i++)
        assert(abs(limited[i]) <= 4000); /* 0.1 RPM */
    imu.yaw = 179;
    tick(1);
    assert(Chassis_TurnTo(-179) == HAL_OK);
    tick(1);
    assert(chassis.heading_error > 0 && chassis.heading_error < 3);
    assert(chassis.target.wz > 0);
    Chassis_Stop();
    imu.yaw = 0;
    tick(1);
    assert(Chassis_TurnBy(180) == HAL_OK);
    tick(1);
    assert(chassis.target.wz > 0);
    Chassis_Stop();
    assert(Chassis_TurnBy(-180) == HAL_OK);
    tick(1);
    assert(chassis.target.wz < 0);
    Chassis_Stop();
    imu.yaw = 90;
    tick(1);
    assert(Chassis_TurnBy(180) == HAL_OK);
    tick(1);
    assert(chassis.target.wz > 0);
    Chassis_Stop();
    imu.yaw = 0;
    tick(1);
    float x = chassis.pos_x, y = chassis.pos_y;
    command(&uart, "F1000\n");
    assert(chassis_cmd_status == HAL_OK);
    command(&uart, "T90\n");
    assert(chassis_cmd_status == HAL_OK); /* 运动中的转向排队，不再拒绝 */
    for (int i = 0; i < 100; i++)
        plant();
    imu.yaw += 8; // external heading disturbance while translating
    complete();
    assert(fabsf(chassis.pos_x - x - 1000) < 4);
    assert(fabsf(chassis.pos_y - y) < 4);
    assert(fabsf(Math_WrapDeg(chassis.yaw - 90)) <= 0.1f); /* 追加的 T90 已经执行 */
    /* PATH 模式同样统计平移期航向误差，$H 里的两项才不是旧值。 */
    assert(chassis.heading_peak_dynamic >= 7.9f && chassis.heading_over_limit > 0);
    command(&uart, "A90\n");
    complete();
    assert(fabsf(chassis.yaw - 90) <= 0.1f);
    x = chassis.pos_x;
    y = chassis.pos_y;
    command(&uart, "F200\n");
    complete();
    assert(fabsf(chassis.pos_x - x) < 6);
    assert(fabsf(chassis.pos_y - y - 200) < 4);
    command(&uart, "T-90\n");
    complete();
    assert(fabsf(chassis.yaw) < 2.1f);
    command(&uart, "A0\n");
    assert(chassis_cmd_status == HAL_OK);
    complete();
    command(&uart, "F1.5\n");
    assert(chassis_cmd_status == HAL_ERROR);
    command(&uart, "T999999999999999999\n");
    assert(chassis_cmd_status == HAL_ERROR);
    assert(Chassis_MoveDistance(NAN, 0) == HAL_ERROR);
    command(&uart, "F100\n");
    tick(1);
    for (int i = 0; i < 102; i++)
        tick(0);
    assert(chassis.state == CHASSIS_STATE_FAULT);
    tick(1);
    assert(chassis.state == CHASSIS_STATE_FAULT);
    command(&uart, "S\n");
    assert(chassis.state == CHASSIS_STATE_READY);
    assert(output[0] == 0 && output[1] == 0 && output[2] == 0 && output[3] == 0);
    /* Wheel speeds are cached with the sync flag and released by one trigger. */
    unsigned triggers = sync_triggers;
    Chassis_SetVelocity(100, 0, 0);
    tick(1);
    assert(sync_triggers == triggers + 1 && output[0] > 0 && output[0] == output[1]);
    Chassis_SetVelocity(100, 0, 0);
    tick(1);
    fail_send = 1;
    Chassis_Stop();
    tick(1);
    assert(s_force_apply);
    fail_send = 0;
    tick(1);
    assert(output[0] == 0);
    assert(Chassis_MoveDistance(100, 0) == HAL_OK);
    for (int i = 0; i < 6100; i++)
        tick(1); // fresh sensors but stalled motion
    assert(chassis.state == CHASSIS_STATE_FAULT);
    command(&uart, "S\n");
    imu.yaw = 0;
    tick(1);
    sampled_move();
    for (int i = 0; i < 300; i++)
        Push('X');
    ChassisCmd_Poll();
    assert(chassis_cmd_status == HAL_ERROR);
    assert(chassis.mode == CHASSIS_MODE_VEL);
    /* 溢出后丢弃到下一个结束符，避免半条指令被执行。 */
    command(&uart, "\n");
    event_type = 0;
    command(&uart, "T-"); // Full-buffer callback does not terminate command.
    event_type = HAL_UART_RXEVENT_IDLE;
    command(&uart, "90\n");
    assert(chassis_cmd_status == HAL_OK);
    command(&uart, "S\n");
    assert(Chassis_TurnBy(90) == HAL_OK);
    uint32_t speed_time = motors[0].speed_ms;
    for (int i = 0; i < 102; i++)
    {
        ms += 5;
        host_clock.CYCCNT += 2400000;
        feedback();
        motors[0].speed_ms = speed_time;
        Chassis_Update();
    }
    assert(chassis.state == CHASSIS_STATE_FAULT);
    tick(1);
    command(&uart, "S\n");
    tick(1);
    assert(Chassis_MoveDistance(100, 0) == HAL_OK);
    tick(1);
    assert(chassis.mode == CHASSIS_MODE_MOVE);
    command(&uart, "S\n");
    Chassis_SetMoveSpeedLimit(150);
    assert(Chassis_MoveDistance(5000, 0) == HAL_OK);
    for (int i = 0; i < 6100; i++)
        tick(1);
    assert(chassis.mode == CHASSIS_MODE_MOVE && chassis.state == CHASSIS_STATE_READY);
    for (int i = 0; i < 10000; i++)
        tick(1);
    assert(chassis.state == CHASSIS_STATE_FAULT); // Extended timeout still terminates a stuck move.
    Chassis_Stop();
    Chassis_SetMoveSpeedLimit(400);
    uint32_t before = Chassis_Micros();
    host_clock.CYCCNT += 480U;
    assert(Chassis_Micros() - before == 1U);
    host_clock.CYCCNT = 0xffffff00U;
    (void)Chassis_Micros();
    before = Chassis_Micros();
    host_clock.CYCCNT += 480U;
    assert(Chassis_Micros() - before == 1U);
    puts("PASS: distance, heading disturbance, world coordinates, turns, parsing, feedback loss, "
         "timeout, stop retry, DWT wrap");
    return 0;
}
