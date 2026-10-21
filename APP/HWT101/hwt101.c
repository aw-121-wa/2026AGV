#include "hwt101.h"
#include <string.h>

#define RX_SIZE 64U
#define FRAME_SIZE 11U

imu_data imu = {0};

static UART_HandleTypeDef *imu_uart;
static uint8_t rx_byte;
static uint8_t rx_buf[RX_SIZE];
static volatile uint16_t rx_head; // 中断写入位置
static volatile uint16_t rx_tail; // 任务读取位置
static uint8_t frame[FRAME_SIZE];
static uint8_t frame_index;

HAL_StatusTypeDef HWT101_Init(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return HAL_ERROR;
    }
    if (imu_uart != NULL)
    {
        return HAL_BUSY; // 单实例驱动，只初始化一次
    }

    rx_head = 0;
    rx_tail = 0;
    frame_index = 0;
    imu_uart = huart;
    HAL_StatusTypeDef status = HAL_UART_Receive_IT(huart, &rx_byte, 1);
    if (status != HAL_OK)
    {
        imu_uart = NULL;
    }
    return status;
}

static uint8_t HWT101_ReadByte(uint8_t *byte)
{
    if (rx_head == rx_tail)
    {
        return 0;
    }

    *byte = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1U) % RX_SIZE;
    return 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (imu_uart == NULL || huart != imu_uart)
    {
        return;
    }

    uint16_t next = (rx_head + 1U) % RX_SIZE;
    if (next != rx_tail)
    {
        rx_buf[rx_head] = rx_byte;
        rx_head = next;
    }
    // 满时丢弃新字节，仍继续接收。
    (void)HAL_UART_Receive_IT(huart, &rx_byte, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (imu_uart != NULL && huart == imu_uart)
    {
        // 溢出等错误可能让 HAL 停止接收，重新启动。
        (void)HAL_UART_Receive_IT(huart, &rx_byte, 1);
    }
}

void HWT101_ProcessData(UART_HandleTypeDef *huart, imu_data *data)
{
    if (imu_uart == NULL || huart != imu_uart || data == NULL)
    {
        return;
    }

    // 若中断中重新开启接收失败，任务中再次尝试。
    if (huart->RxState == HAL_UART_STATE_READY)
    {
        (void)HAL_UART_Receive_IT(huart, &rx_byte, 1);
    }

    uint8_t byte;
    while (HWT101_ReadByte(&byte))
    {
        if (frame_index == 0 && byte != 0x55)
        {
            continue;
        }

        frame[frame_index++] = byte;
        if (frame_index < FRAME_SIZE)
        {
            continue;
        }

        uint8_t sum = 0;
        for (uint8_t i = 0; i < FRAME_SIZE - 1U; ++i)
        {
            sum += frame[i];
        }

        if (sum == frame[FRAME_SIZE - 1U])
        {
            if (frame[1] == 0x53)
            {
                int16_t raw_yaw = (int16_t)(((uint16_t)frame[7] << 8) | frame[6]);
                data->yaw = raw_yaw / 32768.0f * 180.0f;
            }
            frame_index = 0;
        }
        else
        {
            // 坏帧中可能包含下一帧的帧头，保留它后面的字节。
            uint8_t start = 1;
            while (start < FRAME_SIZE && frame[start] != 0x55)
            {
                ++start;
            }
            frame_index = FRAME_SIZE - start;
            memmove(frame, frame + start, frame_index);
        }
    }
}
