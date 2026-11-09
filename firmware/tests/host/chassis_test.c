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
#include "chassis.h"
host_dwt host_clock;
host_debug host_core;
uint32_t SystemCoreClock = 480000000U;
static uint32_t ms;
imu_data imu;
static zdt_motor motors[4];
/* 假电机的读数和收到的命令都按电机自己的方向；车上右侧两台反装（CHASSIS_WHEEL_DIR_xx），
   output[] 和 WheelTurn/WheelSpeed 换算成轮子让车前进的方向，测试都按轮子写。 */
static const float wheel_dir[4] = {CHASSIS_WHEEL_DIR_LF, CHASSIS_WHEEL_DIR_RF, CHASSIS_WHEEL_DIR_LR,
                                   CHASSIS_WHEEL_DIR_RR};
static float output[4]; /* RPM actually commanded to each wheel, forward positive */
static void WheelTurn(int i, float wheel_deg)
{
    motors[i].position_deg += wheel_dir[i] * wheel_deg;
}
static void WheelSpeed(int i, float wheel_rpm)
{
    motors[i].speed_rpm = wheel_dir[i] * wheel_rpm;
}
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
    float wheel_rpm = wheel_dir[m->address - 1] * rpm_x10 * 0.1f;
    if (sync == ZDT_SYNC_CACHE)
        cached[m->address - 1] = wheel_rpm;
    else
        output[m->address - 1] = wheel_rpm;
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
static unsigned stop_frames; /* 发出去的急停帧 */
HAL_StatusTypeDef ZDT_Stop(zdt_motor *m, zdt_sync sync)
{
    (void)sync;
    if (fail_send)
        return HAL_ERROR;
    output[m->address - 1] = 0;
    ++stop_frames;
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
/* 这一拍发出的读取：种类（0 位置、1 转速、2 状态）和电机地址。 */
static uint8_t requests[16][2];
static unsigned request_count;
static HAL_StatusTypeDef Request(zdt_motor *m, uint8_t kind)
{
    if (request_count < 16U)
    {
        requests[request_count][0] = kind;
        requests[request_count][1] = m->address;
        ++request_count;
    }
    return HAL_OK;
}
HAL_StatusTypeDef ZDT_RequestPosition(zdt_motor *m)
{
    return Request(m, 0U);
}
HAL_StatusTypeDef ZDT_RequestSpeed(zdt_motor *m)
{
    return Request(m, 1U);
}
HAL_StatusTypeDef ZDT_RequestStatus(zdt_motor *m)
{
    return Request(m, 2U);
}
void ZDT_ProcessData(void)
{
}
void ZDT_Service(void)
{
}
static unsigned flushes;
void ZDT_Flush(void)
{
    ++flushes;
}
static uint8_t staged[4]; /* 假装这一拍已经给各台攒了几帧 */
uint8_t ZDT_Staged(const zdt_motor *m)
{
    return staged[m->address - 1];
}
static unsigned clear_calls;
HAL_StatusTypeDef ZDT_ClearProtection(zdt_motor *m)
{
    m->stalled = m->protected_flag = 0;
    ++clear_calls;
    return HAL_OK;
}
uint32_t ZDT_BusErrors(void)
{
    return 7U;
}
uint32_t ZDT_RxLost(void)
{
    return 2U;
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
#include "../../User/Comm/tune_log.c"
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
    TuneLog_Tick(); // 和固件的底盘任务同一个顺序
}
static void plant(void)
{
    /* Ideal wheels, actual output RPM; yaw follows inverse kinematics. */
    float v[4];
    for (int i = 0; i < 4; i++)
    {
        WheelSpeed(i, output[i]);
        WheelTurn(i, output[i] * 6.0f * 0.005f);
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
    float position[4]; /* 电机自己方向上的读数 */
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
            position[i] += wheel_dir[i] * output[i] * 6.0f * 0.005f;
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
                WheelSpeed(i, output[i]);
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
/* 每 4 拍里位置、转速、状态各一拍，每拍四个轮子各读一条，剩下一拍不读；发出去的条数记在诊断里。
   读取在拍末排在这一拍的命令后面，和命令一起发；已经有两条命令的那台这一拍不读。 */
static void TestFeedbackRequests(void)
{
    ChassisFeedbackDiag before, after;
    Chassis_FeedbackDiag(3U, &before);
    unsigned kinds = 0U, quiet = 0U;
    for (int t = 0; t < 8; ++t)
    {
        tick(1);
        request_count = 0U;
        unsigned f = flushes;
        Chassis_CycleEnd();
        assert(flushes == f + 1U);
        if (request_count == 0U)
        {
            ++quiet;
            continue;
        }
        assert(request_count == 4U);
        for (unsigned i = 0; i < 4U; ++i)
            assert(requests[i][0] == requests[0][0] && requests[i][1] == i + 1U);
        kinds |= 1U << requests[0][0];
    }
    assert(kinds == 7U && quiet == 2U);
    Chassis_FeedbackDiag(3U, &after);
    for (unsigned k = 0; k < 3U; ++k)
        assert(after.sent[k] == before.sent[k] + 2U && after.fail[k] == before.fail[k]);
    /* 右后轮这一拍已经有停车和零速两条：只读另外三台，没读的不算发出去。 */
    for (int t = 0; t < 4; ++t)
    {
        tick(1);
        staged[3] = 2U;
        request_count = 0U;
        Chassis_CycleEnd();
        staged[3] = 0U;
        if (request_count == 0U)
            continue;
        assert(request_count == 3U);
        for (unsigned i = 0; i < 3U; ++i)
            assert(requests[i][1] == i + 1U);
    }
    ChassisFeedbackDiag skipped;
    Chassis_FeedbackDiag(3U, &skipped);
    for (unsigned k = 0; k < 3U; ++k)
        assert(skipped.sent[k] == after.sent[k]);
}
/* 自己走完（定距、路径）只下发零速，不发急停帧：急停、零速和读取挤在同一拍，排在最后的右后轮会丢应答。
   S 才发急停。 */
static void TestFinishSendsNoStopFrames(UART_HandleTypeDef *u)
{
    unsigned before = stop_frames;
    assert(Chassis_MoveDistance(100, 0) == HAL_OK);
    complete();
    command(u, "F100\n");
    complete();
    assert(stop_frames == before);
    assert(output[0] == 0 && output[1] == 0 && output[2] == 0 && output[3] == 0);
    command(u, "S\n");
    assert(stop_frames == before + 4U);
}
/* 反馈断了进故障：零速发出去以后不再每拍重发，和静止时一样 100 ms 一次，总线留给读反馈。 */
static void TestFaultResendsSlowly(void)
{
    tick(1);
    for (int i = 0; i < 102; i++)
        tick(0);
    assert(chassis.state == CHASSIS_STATE_FAULT);
    unsigned triggers = sync_triggers;
    for (int i = 0; i < 100; i++)
        tick(0);
    assert(sync_triggers - triggers <= 100U / CHASSIS_RESEND_DIV + 1U);
    tick(1);
    Chassis_Stop();
    tick(1);
    assert(chassis.state == CHASSIS_STATE_READY);
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
    float hold = -(CHASSIS_HEADING_KP + CHASSIS_HEADING_KI * 1.0f) * 0.15f; // 比例 + 积分 1 s
    assert(chassis.target.wz < 0.7f * hold && chassis.target.wz > 1.3f * hold);
    assert(chassis.wheel_rpm_x10[0] !=
           chassis.wheel_rpm_x10[1]); /* sub-RPM correction reaches wheels */
    Chassis_Stop();
    chassis_vel combined = {1200, 1200, 240};
    int16_t limited[4];
    Chassis_Solve(&combined, limited);
    for (int i = 0; i < 4; i++)
        assert(abs(limited[i]) <= (int)CHASSIS_MAX_WHEEL_RPM * 10); /* 0.1 RPM */
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
    assert(chassis_feedback_fault.valid);
    assert(chassis_feedback_fault.bus_errors == 7U && chassis_feedback_fault.rx_lost == 2U);
    assert(chassis_feedback_fault.reason == CHASSIS_FAULT_MOVE_TIMEOUT);
    uint32_t captured = chassis_feedback_fault.at_ms;
    Chassis_Stop();
    assert(chassis_feedback_fault.valid && chassis_feedback_fault.at_ms == captured);
    ChassisFeedbackDiag diag;
    Chassis_FeedbackDiag(0U, &diag);
    assert(diag.valid[0] && diag.valid[1] && diag.valid[2]);
    tick(1);
    assert(chassis.state == CHASSIS_STATE_READY);
    TestFeedbackRequests();
    TestFaultResendsSlowly();
    TestFinishSendsNoStopFrames(&uart);
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
         "timeout, stop retry, feedback request schedule, fault resend rate, no stop frame on finish, "
         "DWT wrap");
    return 0;
}
