#ifndef HWT101_H
#define HWT101_H

#include "usart.h"

typedef struct {
    float roll;  // 保留字段，当前驱动只更新 yaw
    float pitch;
    float yaw;   // 航向角，单位：度
} imu_data;

extern imu_data imu;

// 在串口初始化后调用一次；返回 HAL_OK 表示接收已启动。
HAL_StatusTypeDef HWT101_Init(UART_HandleTypeDef *huart);
// 只在一个任务中周期调用，data 不可为空。
void HWT101_ProcessData(UART_HandleTypeDef *huart, imu_data *data);

#endif
