#include "chassis_telemetry.h"
#include "chassis.h"
#include "hwt101.h"
#include "agv_config.h"
#include "user_math.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#if AGV_MODE == AGV_MODE_SURVEY
#include "agv_run.h"
#elif AGV_MODE == AGV_MODE_COMPETITION
#include "competition.h"
#endif

/* 纯文本串口输出：串口助手直接可读，不依赖任何上位机程序。 */

#define TX_RING_SIZE 2048U // 环形发送缓冲；帮助文本约 1 KB，可整段排队
#define TX_CHUNK_MAX 256U  // 每次中断发送的最大字节数
#define LINE_MAX     256U  // 单次 Printf 的最大长度

static UART_HandleTypeDef *telemetry_uart;
static char tx_ring[TX_RING_SIZE];
static unsigned tx_head, tx_tail; // head 写入、tail 是正在/下一次发送的起点
static unsigned tx_inflight; // 已交给 HAL 正在发送的字节数，发送完成前不得覆盖
static unsigned long tx_dropped;
static char line[LINE_MAX];

static uint32_t report_period_ms = 1000U; // 0 = 关闭周期状态行
static uint32_t last_report;
static telemetry_plot plot_group;
static uint32_t plot_period_ms, last_plot;
static uint8_t last_state = 0xFFU, last_fault;
static uint8_t stats_seen, wheel_seq, wheel_active;
static uint8_t banner_done;
static telemetry_route_point route_point; // 非 NULL = 有路线没发完
static unsigned route_total, route_line;
static uint32_t route_ms;

static const char *const k_wheel_name[CHASSIS_WHEEL_NUM] = {"LF", "RF", "LR", "RR"};

/* ---------- 浮点转文本（newlib-nano 默认不带 printf 浮点） ---------- */
#define NUM_POOL 24U
static char num_pool[NUM_POOL][24];
static unsigned num_next;

/* 把 value 格式化成 decimals 位小数，返回池里的临时字符串；一次 Printf 最多用 NUM_POOL 个。 */
static const char *Num(float value, unsigned decimals)
{
    char *out = num_pool[num_next];
    num_next = (num_next + 1U) % NUM_POOL;
    if (!isfinite(value) || fabsf(value) > 1.0e7f)
    {
        (void)snprintf(out, sizeof(num_pool[0]), "nan");
        return out;
    }
    unsigned long scale = 1UL;
    for (unsigned i = 0; i < decimals; ++i)
        scale *= 10UL;
    long raw = lroundf(value * (float)scale);
    unsigned long mag = (unsigned long)(raw < 0 ? -raw : raw);
    if (decimals == 0U)
        (void)snprintf(out, sizeof(num_pool[0]), "%s%lu", raw < 0 ? "-" : "", mag);
    else
        (void)snprintf(out, sizeof(num_pool[0]), "%s%lu.%0*lu", raw < 0 ? "-" : "", mag / scale,
                       (int)decimals, mag % scale);
    return out;
}

/* ---------- 发送环形缓冲 ---------- */
static unsigned TxFree(void)
{
    return (tx_tail + TX_RING_SIZE - tx_head - 1U) % TX_RING_SIZE;
}

static void TxPut(const char *text, unsigned n)
{
    if (n > TxFree())
    {
        tx_dropped += n;
        return;
    }
    for (unsigned i = 0; i < n; ++i)
    {
        tx_ring[tx_head] = text[i];
        tx_head = (tx_head + 1U) % TX_RING_SIZE;
    }
}

void ChassisTelemetry_Printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    int n = vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    if (n <= 0)
        return;
    if ((unsigned)n >= sizeof(line))
        n = (int)sizeof(line) - 1;
    TxPut(line, (unsigned)n);
}

void ChassisTelemetry_Puts(const char *text)
{
    TxPut(text, (unsigned)strlen(text));
}

/* 串口空闲时：先确认上一块已发完再前移 tail，然后把下一段连续数据交给 HAL。 */
static void TxKick(void)
{
    if (telemetry_uart->gState != HAL_UART_STATE_READY)
        return;
    if (tx_inflight)
    {
        tx_tail = (tx_tail + tx_inflight) % TX_RING_SIZE;
        tx_inflight = 0U;
    }
    if (tx_tail == tx_head)
        return;
    unsigned n = tx_head > tx_tail ? tx_head - tx_tail : TX_RING_SIZE - tx_tail;
    if (n > TX_CHUNK_MAX)
        n = TX_CHUNK_MAX;
    if (HAL_UART_Transmit_IT(telemetry_uart, (uint8_t *)&tx_ring[tx_tail], (uint16_t)n) == HAL_OK)
        tx_inflight = n;
}

/* ---------- 文本化 ---------- */
static const char *StateName(uint8_t state)
{
    return state == CHASSIS_STATE_IDLE    ? "IDLE"
           : state == CHASSIS_STATE_READY ? "READY"
           : state == CHASSIS_STATE_FAULT ? "FAULT"
                                          : "?";
}

static const char *ModeName(uint8_t mode)
{
    return mode == CHASSIS_MODE_VEL     ? "STOP"
           : mode == CHASSIS_MODE_MOVE  ? "MOVE"
           : mode == CHASSIS_MODE_TURN  ? "TURN"
           : mode == CHASSIS_MODE_PATH  ? "PATH"
           : mode == CHASSIS_MODE_WHEEL ? "WHEEL"
                                        : "?";
}

/* 把 CHASSIS_WAIT_* 位图写成 "IMU LF:no-reply RF:disabled" 这类可读文本。 */
static const char *ReasonText(uint16_t reason)
{
    static char text[160];
    int n = 0;
    text[0] = 0;
    if (reason & CHASSIS_WAIT_IMU)
        n += snprintf(text + n, sizeof(text) - (unsigned)n, " IMU:no-data");
    for (unsigned i = 0; i < CHASSIS_WHEEL_NUM && n < (int)sizeof(text) - 24; ++i)
    {
        if (reason & CHASSIS_WAIT_FB(i))
            n += snprintf(text + n, sizeof(text) - (unsigned)n, " %s:no-reply", k_wheel_name[i]);
        if (reason & CHASSIS_WAIT_EN(i))
            n += snprintf(text + n, sizeof(text) - (unsigned)n, " %s:disabled", k_wheel_name[i]);
        if (reason & CHASSIS_WAIT_STALL(i))
            n += snprintf(text + n, sizeof(text) - (unsigned)n, " %s:stall", k_wheel_name[i]);
    }
    if ((reason & CHASSIS_WAIT_ODOM) && n < (int)sizeof(text) - 16)
        n += snprintf(text + n, sizeof(text) - (unsigned)n, " odometry:init");
    if (n == 0)
        (void)snprintf(text, sizeof(text), " none");
    return text;
}

static const char *FaultText(void)
{
    static char text[200];
    switch (chassis.fault)
    {
    case CHASSIS_FAULT_SENSOR:
        (void)snprintf(text, sizeof(text), "feedback lost/motor fault:%s",
                       ReasonText(chassis.fault_detail));
        break;
    case CHASSIS_FAULT_MOVE_TIMEOUT:
        (void)snprintf(text, sizeof(text), "move/turn timeout (blocked or slipping?)");
        break;
    case CHASSIS_FAULT_PATH_TIMEOUT:
        (void)snprintf(text, sizeof(text), "path segment timeout (60 s)");
        break;
    default:
        (void)snprintf(text, sizeof(text), "unknown");
        break;
    }
    return text;
}

static const char *VerdictText(uint8_t verdict)
{
    return verdict == CHASSIS_WT_OK ? "OK"
           : verdict == CHASSIS_WT_REVERSED
               ? "ENCODER SIGN FLIPPED -> check driver Dir/encoder menu"
           : verdict == CHASSIS_WT_NONE ? "NO MOTION -> check enable/wiring/address/power"
           : verdict == CHASSIS_WT_ODD  ? "WRONG ANGLE -> check load/slip/driver settings"
                                        : "skipped";
}

static void PrintStatus(uint32_t now)
{
    ChassisTelemetry_Printf(
        "[%7s] %s %s x=%s y=%s yaw=%s tgt=%s err=%s v=%s/%s/%s rpm(t/m) LF %d/%d RF %d/%d LR "
        "%d/%d RR %d/%d",
        Num((float)now * 0.001f, 1), StateName(chassis.state), ModeName(chassis.mode),
        Num(chassis.pos_x, 1), Num(chassis.pos_y, 1), Num(chassis.yaw, 2),
        Num(chassis.heading_target, 2), Num(chassis.heading_error, 2), Num(chassis.current.vx, 0),
        Num(chassis.current.vy, 0), Num(chassis.current.wz, 1),
        (int)lroundf(chassis.wheel_rpm_x10[0] * 0.1f), (int)lroundf(chassis.wheel_rpm_meas[0]),
        (int)lroundf(chassis.wheel_rpm_x10[1] * 0.1f), (int)lroundf(chassis.wheel_rpm_meas[1]),
        (int)lroundf(chassis.wheel_rpm_x10[2] * 0.1f), (int)lroundf(chassis.wheel_rpm_meas[2]),
        (int)lroundf(chassis.wheel_rpm_x10[3] * 0.1f), (int)lroundf(chassis.wheel_rpm_meas[3]));
    if (chassis.state == CHASSIS_STATE_IDLE)
        ChassisTelemetry_Printf(" | waiting:%s", ReasonText(Chassis_WaitReason()));
    else if (chassis.state == CHASSIS_STATE_FAULT)
        ChassisTelemetry_Printf(" | fault: %s (send S)", FaultText());
    ChassisTelemetry_Printf("\r\n");
#if AGV_MODE == AGV_MODE_SURVEY
    ChassisTelemetry_Printf("          AGV state=%u err=%u leg=%u covered=0x%03x field=(%s,%s) "
                            "residual=%smm\r\n",
                            (unsigned)agv.state, (unsigned)agv.error, (unsigned)agv.leg,
                            (unsigned)agv.covered, Num(agv.x, 1), Num(agv.y, 1),
                            Num(agv.home_residual_mm, 2));
#elif AGV_MODE == AGV_MODE_COMPETITION
    ChassisTelemetry_Printf("          POSE (%s,%s,%s)\r\n", Num(comp.x, 1), Num(comp.y, 1),
                            Num(comp.heading_deg, 2));
#endif
}

static void PrintBanner(void)
{
    ChassisTelemetry_Printf(
        "\r\n==== 2026AGV chassis firmware | mode=%s | IMU=%s | CAN 1M ====\r\n"
        "Type H (or ?) + Enter for help. Status line every %lu ms (D0 = off).\r\n",
        AGV_MODE_NAME, CHASSIS_USE_IMU ? "HWT101" : "OFF(wheel odometry)",
        (unsigned long)report_period_ms);
}

void ChassisTelemetry_SendRoute(unsigned count, telemetry_route_point point)
{
    route_point = point;
    route_total = count;
    route_line = 0U;
}

static void SendRouteLine(uint32_t now)
{
    if (!route_point || now == route_ms || TxFree() < TX_RING_SIZE / 2U)
        return;
    route_ms = now;
    if (route_line == 0U)
        ChassisTelemetry_Printf("ROUTE points %u\r\n", route_total);
    else if (route_line <= route_total)
    {
        MapPoint p = route_point(route_line - 1U);
        ChassisTelemetry_Printf("ROUTE %u %ld %ld\r\n", route_line - 1U, (long)p.x, (long)p.y);
    }
    else
    {
        ChassisTelemetry_Printf("ROUTE end\r\n");
        route_point = NULL;
    }
    ++route_line;
}

void ChassisTelemetry_Help(void)
{
    /* 帮助文本超过 Printf 的单次上限，用 Puts。 */
    ChassisTelemetry_Puts(
        "---- commands (case-insensitive, separate by space, end with Enter or just pause) ----\r\n"
        "  F<mm> B<mm> L<mm> R<mm>  move forward/back/left/right, 1..5000 mm, e.g. F300\r\n"
        "  T<deg>   turn relative, -180..180, up to 3 decimals, e.g. T90  T-45.5\r\n"
        "  A<deg>   turn to absolute heading, e.g. A0\r\n"
        "  S        stop now / clear fault (also drops the rest of the line)\r\n"
        "  W        wheel direction test LF,RF,LR,RR (LIFT THE WHEELS FIRST); W1..W4 one wheel\r\n"
        "  I        detailed info (IMU, each motor, CAN counters)\r\n"
        "  D<hz>    status line rate: D0 off, D1..D10 Hz\r\n"
        "  H or ?   this help\r\n"
        "  /help    console: change parameters live, plot data, bench tools\r\n"
        "  Several moves on one line run in order: F300 L200 B300 R200\r\n");
    ChassisTelemetry_Printf(
        "  Move speed %s mm/s. Replies: OK / BUSY / ERR; 'DONE' when a path finishes.\r\n",
        Num(chassis_tune.cmd_speed, 0));
}

void ChassisTelemetry_Info(void)
{
    uint32_t now = HAL_GetTick();
    ChassisTelemetry_Printf("---- info @ %s s ----\r\n", Num((float)now * 0.001f, 3));
    ChassisTelemetry_Printf("chassis: state=%s mode=%s speed_limit=%smm/s cycle=%lu jitter=%luus "
                            "loop=%luus dt=%sms\r\n",
                            StateName(chassis.state), ModeName(chassis.mode),
                            Num(Chassis_SpeedLimit(), 0), (unsigned long)chassis.cycle,
                            (unsigned long)chassis.jitter_us, (unsigned long)chassis.loop_peak_us,
                            Num(chassis.dt_actual * 1000.0f, 2));
    ChassisTelemetry_Printf("waiting/fault flags:%s\r\n", ReasonText(Chassis_WaitReason()));
    if (chassis.state == CHASSIS_STATE_FAULT)
        ChassisTelemetry_Printf("fault: %s\r\n", FaultText());
#if CHASSIS_USE_IMU
    ChassisTelemetry_Printf("IMU: valid=%u yaw=%s age=%lums\r\n", (unsigned)imu.valid,
                            Num(imu.yaw, 3), (unsigned long)(now - imu.updated_ms));
#else
    ChassisTelemetry_Printf("IMU: disabled (CHASSIS_USE_IMU=0), yaw from wheels=%s\r\n",
                            Num(chassis.yaw, 3));
#endif
    for (unsigned i = 0; i < CHASSIS_WHEEL_NUM; ++i)
    {
        const zdt_motor *m = Chassis_Wheel(i);
        if (m == NULL)
            continue;
        ChassisTelemetry_Printf(
            "motor %s addr=%u en=%u stall=%u prot=%u pos=%sdeg rpm=%s "
            "rx_ok=%lu rx_err=%lu age=%lums ack=0x%02X\r\n",
            k_wheel_name[i], (unsigned)m->address, (unsigned)m->enabled, (unsigned)m->stalled,
            (unsigned)m->protected_flag, Num(m->position_deg, 1), Num(m->speed_rpm, 1),
            (unsigned long)m->rx_frames, (unsigned long)m->rx_errors,
            (unsigned long)(m->position_seq ? now - m->position_ms : 0U), (unsigned)m->ack_code);
    }
    ChassisTelemetry_Printf(
        "CAN: bus_off=%lu rx_dropped=%lu tx_free=%lu | serial tx_dropped=%lu\r\n",
        (unsigned long)ZDT_BusOffCount(), (unsigned long)ZDT_RxDropped(),
        (unsigned long)ZDT_TxFreeLevel(), tx_dropped);
    ChassisTelemetry_Printf("heading: peak_dynamic=%sdeg over_0.2deg=%lu cycles\r\n",
                            Num(chassis.heading_peak_dynamic, 3),
                            (unsigned long)chassis.heading_over_limit);
}

void ChassisTelemetry_SetRate(unsigned hz)
{
    report_period_ms = hz ? 1000U / (hz > 10U ? 10U : hz) : 0U;
}

uint8_t ChassisTelemetry_Periodic(void)
{
    return report_period_ms != 0U;
}

void ChassisTelemetry_SetPlot(telemetry_plot group, unsigned hz)
{
    plot_group = group;
    plot_period_ms = hz ? 1000U / hz : 0U;
}

static void PrintPlot(void)
{
    if (plot_group == TELEMETRY_PLOT_HEADING)
    {
        ChassisTelemetry_Printf("plot:%s,%s,%s,%s,%s\r\n", Num(chassis.heading_target, 3),
                                Num(chassis.yaw, 3), Num(chassis.heading_error, 3),
                                Num(chassis.target.wz, 3), Num(chassis.current.wz, 3));
    }
    else if (plot_group == TELEMETRY_PLOT_SPEED)
    {
        /* 四轮实测转速按运动学正解换成车体速度，和里程计同一套符号。 */
        const float *rpm = chassis.wheel_rpm_meas;
        float mm_per_s = MATH_PI * chassis_tune.wheel_dia / (60.0f * CHASSIS_GEAR_RATIO);
        float vx = 0.25f * mm_per_s * (rpm[0] + rpm[1] + rpm[2] + rpm[3]);
        float vy = 0.25f * mm_per_s * (-rpm[0] + rpm[1] + rpm[2] - rpm[3]);
        ChassisTelemetry_Printf("plot:%s,%s,%s,%s,%s,%s\r\n", Num(chassis.target.vx, 1),
                                Num(chassis.target.vy, 1), Num(chassis.current.vx, 1),
                                Num(chassis.current.vy, 1), Num(vx, 1), Num(vy, 1));
    }
    else if (plot_group == TELEMETRY_PLOT_WHEELS)
    {
        ChassisTelemetry_Printf(
            "plot:%s,%s,%s,%s,%s,%s,%s,%s\r\n", Num(chassis.wheel_rpm_x10[0] * 0.1f, 1),
            Num(chassis.wheel_rpm_x10[1] * 0.1f, 1), Num(chassis.wheel_rpm_x10[2] * 0.1f, 1),
            Num(chassis.wheel_rpm_x10[3] * 0.1f, 1), Num(chassis.wheel_rpm_meas[0], 1),
            Num(chassis.wheel_rpm_meas[1], 1), Num(chassis.wheel_rpm_meas[2], 1),
            Num(chassis.wheel_rpm_meas[3], 1));
    }
}

const char *ChassisTelemetry_Num(float value, unsigned decimals)
{
    return Num(value, decimals);
}

void ChassisTelemetry_Init(UART_HandleTypeDef *uart)
{
    telemetry_uart = uart;
    tx_head = tx_tail = tx_inflight = 0U;
    tx_dropped = 0UL;
    last_report = 0U;
    last_state = 0xFFU;
    last_fault = 0U;
    stats_seen = 0U;
    wheel_seq = chassis_wheel_result.seq;
    wheel_active = 0U;
    banner_done = 0U;
    route_point = NULL;
}

void ChassisTelemetry_Ack(char command, HAL_StatusTypeDef result)
{
    if (command == 'P')
    {
        if (result == HAL_OK)
            ChassisTelemetry_Printf("OK   path queued (%s mm/s)\r\n", Num(Chassis_SpeedLimit(), 0));
        else if (result == HAL_BUSY)
            ChassisTelemetry_Printf("BUSY chassis not ready: %s%s\r\n", StateName(chassis.state),
                                    chassis.state == CHASSIS_STATE_FAULT ? " (send S to clear)"
                                                                         : "");
        else
            ChassisTelemetry_Printf("ERR  line rejected, nothing executed\r\n");
        return;
    }
    if (command == 'S')
    {
        ChassisTelemetry_Printf("OK   stop\r\n");
        return;
    }
    if (command == '!')
    {
        ChassisTelemetry_Printf("ERR  receive overflow, line discarded, chassis stopped\r\n");
        return;
    }
    if (command == 'W')
    {
        if (result == HAL_OK)
            ChassisTelemetry_Printf("OK   wheel test started, %s rpm %lu ms per wheel\r\n",
                                    Num(CHASSIS_WHEEL_TEST_RPM, 0),
                                    (unsigned long)CHASSIS_WHEEL_TEST_MS);
        else if (result == HAL_BUSY)
            ChassisTelemetry_Printf("BUSY wheel test needs READY and stopped (now %s %s)\r\n",
                                    StateName(chassis.state), ModeName(chassis.mode));
        else
            ChassisTelemetry_Printf("ERR  use W or W1..W4\r\n");
        return;
    }
    if (result == HAL_OK)
        ChassisTelemetry_Printf("OK   %c\r\n", command ? command : '?');
    else if (result == HAL_BUSY)
        ChassisTelemetry_Printf("BUSY %c ignored (only S works in this mode)\r\n",
                                command ? command : '?');
    else
        ChassisTelemetry_Printf("ERR  bad command '%c' (H for help)\r\n",
                                command >= ' ' && command < 127 ? command : '?');
}

void ChassisTelemetry_Poll(void)
{
    if (!telemetry_uart)
        return;
    uint32_t now = HAL_GetTick();
    if (!banner_done)
    {
        banner_done = 1U;
        PrintBanner();
    }
    /* 状态变化立即打印 */
    if (chassis.state != last_state ||
        (chassis.state == CHASSIS_STATE_FAULT && chassis.fault != last_fault))
    {
        if (chassis.state == CHASSIS_STATE_FAULT)
            ChassisTelemetry_Printf("EVT  %s -> FAULT: %s. Send S to clear.\r\n",
                                    StateName(last_state), FaultText());
        else if (chassis.state == CHASSIS_STATE_READY)
            ChassisTelemetry_Printf("EVT  %s -> READY%s\r\n", StateName(last_state),
                                    last_state == 0xFFU || last_state == CHASSIS_STATE_IDLE
                                        ? ": all 4 motors enabled, feedback OK"
                                        : "");
        else if (chassis.state == CHASSIS_STATE_IDLE)
            ChassisTelemetry_Printf("EVT  IDLE: enabling motors, waiting for feedback\r\n");
        last_state = chassis.state;
        last_fault = chassis.fault;
    }
    /* 路径完成统计：每条路径只打印一次 */
    if (!chassis_path_result.completed)
        stats_seen = 0U;
    else if (!stats_seen)
    {
        stats_seen = 1U;
        ChassisTelemetry_Printf(
            "DONE path: time=%ss dist=%smm max_cross=%smm max_heading=%sdeg jitter=%luus "
            "loop=%luus\r\n",
            Num((float)chassis_path_result.elapsed_ms * 0.001f, 2),
            Num(chassis_path_result.distance_mm, 1), Num(chassis_path_result.max_cross_mm, 2),
            Num(chassis_path_result.max_heading_deg, 2),
            (unsigned long)chassis_path_result.max_jitter_us,
            (unsigned long)chassis_path_result.max_loop_us);
    }
    /* 单轮测试：每换一个轮子提示一次，让现场目测方向 */
    if (chassis_wheel_result.active != wheel_active)
    {
        wheel_active = chassis_wheel_result.active;
        if (wheel_active)
            ChassisTelemetry_Printf("WHEEL %s spinning now (addr %u): it must roll FORWARD "
                                    "(top of the wheel moves toward the front). If it rolls "
                                    "backward set CHASSIS_WHEEL_DIR_%s to -1\r\n",
                                    k_wheel_name[wheel_active - 1U],
                                    (unsigned)Chassis_Wheel(wheel_active - 1U)->address,
                                    k_wheel_name[wheel_active - 1U]);
    }
    /* 单轮测试结果 */
    if (chassis_wheel_result.seq != wheel_seq)
    {
        wheel_seq = chassis_wheel_result.seq;
        float expect = CHASSIS_WHEEL_TEST_RPM * 6.0f * (float)CHASSIS_WHEEL_TEST_MS * 0.001f;
        uint8_t all_ok = 1U;
        for (unsigned i = 0; i < CHASSIS_WHEEL_NUM; ++i)
        {
            if (!(chassis_wheel_result.mask & (1U << i)))
                continue;
            ChassisTelemetry_Printf("WHEEL %s encoder: expect +%sdeg got %sdeg (%s rpm) -> %s\r\n",
                                    k_wheel_name[i], Num(expect, 0),
                                    Num(chassis_wheel_result.delta_deg[i], 1),
                                    Num(chassis_wheel_result.meas_rpm[i], 1),
                                    VerdictText(chassis_wheel_result.verdict[i]));
            all_ok = all_ok && chassis_wheel_result.verdict[i] == CHASSIS_WT_OK;
        }
        ChassisTelemetry_Printf("DONE wheel test: encoders %s. Direction = what you saw: every "
                                "wheel must have rolled forward.\r\n",
                                all_ok ? "all OK" : "NOT OK, see above");
    }
    /* 周期状态行 */
    if (report_period_ms && now - last_report >= report_period_ms)
    {
        last_report = now;
        PrintStatus(now);
    }
    if (plot_group != TELEMETRY_PLOT_OFF && now - last_plot >= plot_period_ms)
    {
        /* 保留相位：周期不是节拍整数倍时平均频率仍然准；落后再多也不补发。 */
        last_plot = now - (now - last_plot) % plot_period_ms;
        PrintPlot();
    }
    SendRouteLine(now);
    TxKick();
}
