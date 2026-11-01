#ifndef CHASSIS_TELEMETRY_H
#define CHASSIS_TELEMETRY_H

#include "usart.h"

/* 串口文本输出（USART6，直接用串口助手看，不需要上位机）：
   - 每行都是可读的 ASCII 文本，以 \r\n 结尾；串口助手选 115200 8N1、ASCII/文本显示。
   - 周期状态行默认 1 Hz（命令 D0 关闭，D1..D10 设置频率）；状态变化、命令回执、
     路径完成、单轮测试结果都会立即单独打印一行。
   - 所有文本先进环形发送缓冲区，再用中断分块发送；缓冲区满时丢弃新文本并计数。 */

/* 绑定与命令接收相同的串口，并排队打印开机信息。 */
void ChassisTelemetry_Init(UART_HandleTypeDef *uart);

/* 打印一条命令的处理结果；OK 只表示已接受，走完看 DONE 行。 */
void ChassisTelemetry_Ack(char command, HAL_StatusTypeDef result);

/* 底盘任务周期调用：检测事件、生成周期状态行、启动中断发送。 */
void ChassisTelemetry_Poll(void);

/* 追加一行（或多行）文本，printf 格式；只能在底盘任务里调用。 */
void ChassisTelemetry_Printf(const char *format, ...) __attribute__((format(printf, 1, 2)));

/* 周期状态行频率：0 = 关闭，1..10 Hz。 */
void ChassisTelemetry_SetRate(unsigned hz);

/* 立即打印帮助 / 详细诊断信息。 */
void ChassisTelemetry_Help(void);
void ChassisTelemetry_Info(void);

/* 因发送缓冲区满而丢弃的字节数。 */
unsigned long ChassisTelemetry_Dropped(void);

#endif
