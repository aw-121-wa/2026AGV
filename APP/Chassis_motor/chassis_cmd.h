#ifndef __CHASSIS_CMD_H
#define __CHASSIS_CMD_H

#include "usart.h"

// 绑定命令串口（本工程用 USART6），在串口初始化之后、调度器启动之前调用一次。
// 命令格式：F1000/B1000/L1000/R1000(mm)，A90(绝对角)，T-90(相对角)，S(停车)。
// 多条命令以空格或换行分隔；兼容一帧一条且无换行。运动中拒绝新运动，S 总能停车。
HAL_StatusTypeDef ChassisCmd_Init(UART_HandleTypeDef *huart);
// 最近命令结果，供调试器观察；BUSY 表示尚未就绪或正在运动。
extern HAL_StatusTypeDef chassis_cmd_status;
void ChassisCmd_RxEvent(UART_HandleTypeDef *huart, uint16_t size);

// 在底盘任务里每周期调用，用于接收出错后重新开启接收。
void ChassisCmd_Poll(void);

#endif
