#ifndef SER_PORT_H
#define SER_PORT_H

#include "main.h"

/* 比赛模式新增的三路串口（机械臂模式只用视觉那一路），寄存器级中断收发 + 环形缓冲，不用 DMA（避免 D-Cache 一致性问题，
   也不需要改 CubeMX 的 MPU 配置）。外设、引脚、时钟、NVIC 都在 SerPort_Init 里自己配置，
   不进 .ioc，CubeMX 重新生成代码不会覆盖。
     SER_LIDAR   = USART1  PA9  TX / PA10 RX  AF7   RPLIDAR C1
     SER_VISION  = USART2  PA2  TX / PA3  RX  AF7   视觉上位机
     SER_SCANNER = USART3  PB10 TX / PB11 RX  AF7   二维码扫码模块
   中断优先级 6（与 UART5/USART6 相同，在 FreeRTOS 可屏蔽范围内）。 */

typedef enum
{
    SER_LIDAR = 0,
    SER_VISION = 1,
    SER_SCANNER = 2,
    SER_COUNT
} SerId;

HAL_StatusTypeDef SerPort_Init(SerId id, uint32_t baud);

/* 从接收缓冲取最多 max 字节，返回实际字节数。只在一个任务里调用。 */
unsigned SerPort_Read(SerId id, uint8_t *dst, unsigned max);

/* 整段放入发送缓冲（放不下则整段丢弃并计数），返回 1 = 已排队。只在一个任务里调用。 */
uint8_t SerPort_Write(SerId id, const uint8_t *src, unsigned len);

/* 丢弃接收缓冲中的全部数据。 */
void SerPort_Flush(SerId id);

/* 开关接收中断：不再需要的外设关掉接收，它继续发数据也不占 CPU。打开时丢弃之前的数据。 */
void SerPort_EnableRx(SerId id, uint8_t on);

typedef struct
{
    uint32_t rx_bytes, rx_overflow, hw_errors, tx_dropped;
} SerPortStats;
const SerPortStats *SerPort_Stats(SerId id);

#endif
