#ifndef HWT101_H
#define HWT101_H

#include "usart.h"

typedef struct
{
    float roll; // 保留字段，当前驱动只更新 yaw
    float pitch;
    float yaw; // 航向角，单位：度
    uint32_t updated_ms;
    uint8_t valid;
} imu_data;

extern imu_data imu;

// 调度前调用一次；UART RX DMA 必须已配置为循环模式。
// DMA 缓冲区的非缓存属性由 main.c MPU_Config 的 region1 提供（见 2026AGV.ioc）。
HAL_StatusTypeDef HWT101_Init(UART_HandleTypeDef *huart);
// 只在一个任务中周期调用，data 不可为空。
void HWT101_ProcessData(UART_HandleTypeDef *huart, imu_data *data);

// 由统一 UART 回调分发，其他 UART 自动忽略。
void HWT101_RxEvent(UART_HandleTypeDef *huart, uint16_t size);
void HWT101_RxError(UART_HandleTypeDef *huart);

#endif
