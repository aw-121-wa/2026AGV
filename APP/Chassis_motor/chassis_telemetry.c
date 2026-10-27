#include "chassis_telemetry.h"
#include "chassis.h"
#include "agv_config.h"
#include <math.h>
#include <stdio.h>

#define TELEMETRY_PERIOD_MS 50U
#define ACK_CAPACITY        16U
static UART_HandleTypeDef *telemetry_uart;
static char tx[512];

static struct
{
    char command;
    HAL_StatusTypeDef result;
} acknowledgements[ACK_CAPACITY];

static unsigned ack_head, ack_tail;
static uint32_t ack_dropped, last_tx;
static uint8_t origin_valid;
static float origin_yaw, origin_x, origin_y;

/* 转为三位小数文本，避免链接 printf 浮点库。 */
static void Fixed(char out[24], float value)
{
    if (!isfinite(value) || fabsf(value) > 1000000.0f)
    {
        (void)snprintf(out, 24, "nan");
        return;
    }
    long raw = lroundf(value * 1000.0f);
    unsigned long magnitude = (unsigned long)(raw < 0 ? -raw : raw);
    (void)snprintf(out, 24, "%s%lu.%03lu", raw < 0 ? "-" : "", magnitude / 1000U,
                   magnitude % 1000U);
}

/* 初始化串口与回执队列；场地原点在首次 READY 时采集。 */
void ChassisTelemetry_Init(UART_HandleTypeDef *uart)
{
    telemetry_uart = uart;
    ack_head = ack_tail = 0;
    ack_dropped = last_tx = 0;
    origin_valid = 0;
}

/* 有界回执队列满时计数，不阻塞控制也不覆盖已排队结果。 */
void ChassisTelemetry_Ack(char command, HAL_StatusTypeDef result)
{
    unsigned next = (ack_head + 1U) % ACK_CAPACITY;
    if (next == ack_tail)
    {
        ++ack_dropped;
        return;
    }
    acknowledgements[ack_head].command = command ? command : '?';
    acknowledgements[ack_head].result = result;
    ack_head = next;
}

/* 发送回执或 20 Hz 遥测；HAL 完成前绝不修改发送缓冲区。 */
void ChassisTelemetry_Poll(void)
{
    if (!telemetry_uart)
        return;
    if (!origin_valid && chassis.state == CHASSIS_STATE_READY)
    {
        origin_x = chassis.pos_x;
        origin_y = chassis.pos_y;
        origin_yaw = chassis.yaw;
        origin_valid = 1;
    }
    if (telemetry_uart->gState != HAL_UART_STATE_READY)
        return;
    uint32_t now = HAL_GetTick();
    int n;
    uint8_t ack = ack_tail != ack_head;
    if (ack)
    {
        HAL_StatusTypeDef result = acknowledgements[ack_tail].result;
        const char *name = result == HAL_OK        ? "OK"
                           : result == HAL_BUSY    ? "BUSY"
                           : result == HAL_TIMEOUT ? "TIMEOUT"
                                                   : "ERROR";
        n = snprintf(tx, sizeof(tx), "$L,%s,ACK,%c,%s\n", result == HAL_OK ? "i" : "w",
                     acknowledgements[ack_tail].command, name);
    }
    else
    {
        if (now - last_tx < TELEMETRY_PERIOD_MS || !origin_valid)
            return;
        float a = origin_yaw * 0.01745329252f;
        float dx = chassis.pos_x - origin_x, dy = chassis.pos_y - origin_y;
        float x = 2250.0f + cosf(a) * dx + sinf(a) * dy;
        float y = (AGV_START_ZONE == 1 ? 2250.0f : 150.0f) - sinf(a) * dx + cosf(a) * dy;
        char v[11][24];
        Fixed(v[0], x);
        Fixed(v[1], y);
        Fixed(v[2], chassis.yaw);
        Fixed(v[3], chassis.heading_target);
        Fixed(v[4], chassis.heading_error);
        Fixed(v[5], chassis.current.vx);
        Fixed(v[6], chassis.current.vy);
        Fixed(v[7], chassis.current.wz);
        Fixed(v[8], chassis.dt_actual);
        Fixed(v[9], chassis.heading_peak_dynamic);
        Fixed(v[10], origin_yaw);
        n = snprintf(tx, sizeof(tx),
                     "$T,%lu,%s,%s,%s,%s,%s,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d\n"
                     "$V,%s,%s,%s,%s,%lu\n$H,%s,%lu,%lu,%lu\n$C,%s\n",
                     (unsigned long)now, v[0], v[1], v[2], v[3], v[4], chassis.state, chassis.mode,
                     chassis.wheel_rpm[0], chassis.wheel_rpm_meas[0], chassis.wheel_rpm[1],
                     chassis.wheel_rpm_meas[1], chassis.wheel_rpm[2], chassis.wheel_rpm_meas[2],
                     chassis.wheel_rpm[3], chassis.wheel_rpm_meas[3], v[5], v[6], v[7], v[8],
                     (unsigned long)chassis.cycle, v[9], (unsigned long)chassis.heading_over_limit,
                     (unsigned long)chassis.jitter_us, (unsigned long)ack_dropped, v[10]);
    }
    if (n <= 0 || (unsigned)n >= sizeof(tx))
        return;
    if (HAL_UART_Transmit_IT(telemetry_uart, (uint8_t *)tx, (uint16_t)n) == HAL_OK)
    {
        if (ack)
            ack_tail = (ack_tail + 1U) % ACK_CAPACITY;
        else
            last_tx = now;
    }
}
