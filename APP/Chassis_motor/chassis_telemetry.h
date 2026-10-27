#ifndef CHASSIS_TELEMETRY_H
#define CHASSIS_TELEMETRY_H

#include "usart.h"

/* 绑定与命令接收相同的串口，发送缓冲区由模块持有。 */
void ChassisTelemetry_Init(UART_HandleTypeDef *uart);

/* 将一条命令的接受结果入队，OK 不代表运动已经完成。 */
void ChassisTelemetry_Ack(char command, HAL_StatusTypeDef result);

/* 底盘任务周期调用，使用中断发送；串口忙时保留缓冲区并跳过。 */
void ChassisTelemetry_Poll(void);

#endif
