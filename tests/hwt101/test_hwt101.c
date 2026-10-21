#include <assert.h>
#include <stdio.h>
#include "hwt101.h"

UART_HandleTypeDef huart1;
static uint8_t *receive_byte;

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart,
                                    uint8_t *byte, uint16_t size)
{
    assert(uart == &huart1 && size == 1);
    receive_byte = byte;
    return HAL_OK;
}

static void feed(const uint8_t *bytes, unsigned count)
{
    for (unsigned i = 0; i < count; ++i)
    {
        *receive_byte = bytes[i];
        HAL_UART_RxCpltCallback(&huart1);
    }
}

int main(void)
{
    imu_data result = {0};
    uint8_t positive[] = {0x55, 0x53, 0, 0, 0, 0, 0, 0x40, 0, 0, 0xe8};
    uint8_t negative[] = {0x55, 0x53, 0, 0, 0, 0, 0, 0xc0, 0, 0, 0x68};
    uint8_t noise[] = {0x12, 0x55, 0x01};

    assert(HWT101_Init(NULL) == HAL_ERROR);
    assert(HWT101_Init(&huart1) == HAL_OK);
    feed(positive, 5);
    HWT101_ProcessData(&huart1, &result);
    assert(result.yaw == 0.0f);
    feed(positive + 5, 6);
    HWT101_ProcessData(&huart1, &result);
    assert(result.yaw == 90.0f);

    /* Exercise wraparound, noise and recovery from a false header. */
    for (unsigned i = 0; i < 100; ++i)
    {
        feed(noise, sizeof(noise));
        feed(negative, sizeof(negative));
        HWT101_ProcessData(&huart1, &result);
        assert(result.yaw == -90.0f);
    }

    positive[10] ^= 1;
    feed(positive, sizeof(positive));
    HWT101_ProcessData(&huart1, &result);
    assert(result.yaw == -90.0f);
    positive[10] ^= 1;
    feed(positive, sizeof(positive));
    HWT101_ProcessData(&huart1, NULL);
    HWT101_ProcessData(&huart1, &result);
    assert(result.yaw == 90.0f);

    /* Valid angular-velocity frames must not change the angle. */
    positive[1] = 0x52;
    positive[10] = 0xe7;
    result.yaw = 12.0f;
    feed(positive, sizeof(positive));
    HWT101_ProcessData(&huart1, &result);
    assert(result.yaw == 12.0f);
    positive[1] = 0x53;
    positive[10] = 0xe8;

    /* A full ring drops new bytes without writing out of bounds. */
    for (unsigned i = 0; i < 100; ++i)
    {
        uint8_t junk = 0;
        feed(&junk, 1);
    }
    HWT101_ProcessData(&huart1, &result);
    feed(positive, sizeof(positive));
    HWT101_ProcessData(&huart1, &result);
    assert(result.yaw == 90.0f);
    puts("HWT101 tests passed");
    return 0;
}
