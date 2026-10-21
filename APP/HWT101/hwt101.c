#include "hwt101.h"
#include <string.h>

#define DMA_SIZE 256U
#define RX_SIZE 512U
#define FRAME_SIZE 11U

imu_data imu = {0};

static UART_HandleTypeDef *imu_uart;
/* DMA 专用缓冲区，由链接脚本放入 RAM_D2，不占用 DTCM。 */
static uint8_t dma_buf[DMA_SIZE] __attribute__((section(".imu_dma"), aligned(256)));
static uint16_t dma_pos;
static uint8_t rx_buf[RX_SIZE];
static uint32_t rx_time[RX_SIZE]; // 保留接收时间，避免积压数据被误判为新数据
static volatile uint16_t rx_head; // 中断写入位置
static volatile uint16_t rx_tail; // 任务读取位置
static uint8_t frame[FRAME_SIZE];
static uint8_t frame_index;
static volatile uint8_t rx_gap;     // 队列溢出，需要重新同步帧头
static volatile uint8_t restart_rx; // 接收异常，需要在任务中重启 DMA

static void HWT101_DmaMemoryInit(void)
{
    __HAL_RCC_D2SRAM1_CLK_ENABLE();
    MPU_Region_InitTypeDef region = {0};
    region.Enable = MPU_REGION_ENABLE;
    region.Number = MPU_REGION_NUMBER1;
    region.BaseAddress = (uint32_t)(uintptr_t)dma_buf;
    region.Size = MPU_REGION_SIZE_256B;
    region.TypeExtField = MPU_TEX_LEVEL1; // 普通内存，不启用缓存
    region.AccessPermission = MPU_REGION_FULL_ACCESS;
    region.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    region.IsShareable = MPU_ACCESS_SHAREABLE;
    region.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    region.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    HAL_MPU_Disable();
    HAL_MPU_ConfigRegion(&region);
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/* 初始化和错误恢复共用：清空旧数据，启动循环接收。 */
static HAL_StatusTypeDef HWT101_StartRx(void)
{
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    rx_head = rx_tail = dma_pos = 0U;
    frame_index = rx_gap = restart_rx = 0U;
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(imu_uart, dma_buf, DMA_SIZE);
    restart_rx = (status != HAL_OK);
    __set_PRIMASK(irq);
    return status;
}

HAL_StatusTypeDef HWT101_Init(UART_HandleTypeDef *huart)
{
    if (huart == NULL || huart->hdmarx == NULL || huart->hdmarx->Init.Mode != DMA_CIRCULAR)
    {
        return HAL_ERROR;
    }
    if (imu_uart != NULL)
    {
        return HAL_BUSY; // 单实例驱动，只初始化一次
    }

    imu.valid = 0U;
    HWT101_DmaMemoryInit();
    imu_uart = huart;
    HAL_StatusTypeDef status = HWT101_StartRx();
    if (status != HAL_OK)
    {
        imu_uart = NULL;
    }
    return status;
}

static uint8_t HWT101_ReadByte(uint8_t *byte, uint32_t *received_ms)
{
    if (rx_head == rx_tail)
    {
        return 0;
    }

    __DMB();
    *byte = rx_buf[rx_tail];
    *received_ms = rx_time[rx_tail];
    rx_tail = (rx_tail + 1U) % RX_SIZE;
    return 1;
}

void HWT101_RxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
    (void)size;
    if (imu_uart == NULL || huart != imu_uart || restart_rx)
    {
        return;
    }

    /* 按 DMA 实际写入位置取数，避免延迟或重复回调造成重复读取。
       UART5 与 DMA 中断保持相同抢占优先级；保留半满中断以接收连续数据。 */
    uint16_t remaining = (uint16_t)__HAL_DMA_GET_COUNTER(huart->hdmarx);
    if (remaining > DMA_SIZE) return;
    uint16_t pos = (DMA_SIZE - remaining) % DMA_SIZE;
    uint16_t count = (pos + DMA_SIZE - dma_pos) % DMA_SIZE;
    if (count == 0U) return; // 跳过重复的接收事件
    uint16_t free_bytes = (rx_tail + RX_SIZE - rx_head - 1U) % RX_SIZE;
    if (rx_gap || count > free_bytes) {
        rx_gap = 1U;
        dma_pos = pos;
        return;
    }
    uint16_t head = rx_head;
    uint32_t received_ms = HAL_GetTick();
    while (dma_pos != pos) {
        rx_buf[head] = dma_buf[dma_pos];
        rx_time[head] = received_ms;
        head = (head + 1U) % RX_SIZE;
        dma_pos = (dma_pos + 1U) % DMA_SIZE;
    }
    __DMB();
    rx_head = head;
}

void HWT101_RxError(UART_HandleTypeDef *huart)
{
    if (imu_uart != NULL && huart == imu_uart)
    {
        // 中断只标记错误，重启操作交给 IMU 任务。
        restart_rx = 1U;
        imu.valid = 0U;
    }
}

/* 逐字节组帧：仅校验通过的角度帧才能更新航向。 */
static void HWT101_ParseByte(uint8_t byte, uint32_t received_ms, imu_data *data)
{
    if (frame_index == 0U && byte != 0x55) return;
    frame[frame_index++] = byte;
    if (frame_index < FRAME_SIZE) return;

    uint8_t sum = 0U;
    for (uint8_t i = 0; i < FRAME_SIZE - 1U; ++i) sum += frame[i];
    if (sum != frame[FRAME_SIZE - 1U]) {
        // 坏帧内可能包含下一帧帧头，保留后续字节继续匹配。
        uint8_t start = 1U;
        while (start < FRAME_SIZE && frame[start] != 0x55) ++start;
        frame_index = FRAME_SIZE - start;
        memmove(frame, frame + start, frame_index);
        return;
    }

    frame_index = 0U;
    if (frame[1] != 0x53) return;
    int16_t raw_yaw = (int16_t)(((uint16_t)frame[7] << 8) | frame[6]);
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    if (!restart_rx && !rx_gap) {
        data->yaw = raw_yaw * (180.0f / 32768.0f);
        data->updated_ms = received_ms; // 使用帧尾的接收时间，不使用解析时间
        data->valid = 1U;
    }
    __set_PRIMASK(irq);
}

void HWT101_ProcessData(UART_HandleTypeDef *huart, imu_data *data)
{
    if (!imu_uart || huart != imu_uart || !data) return;

    if (restart_rx || huart->RxState == HAL_UART_STATE_READY) {
        data->valid = 0U;
        restart_rx = 1U;
        if (HAL_UART_AbortReceive(huart) == HAL_OK) (void)HWT101_StartRx();
        return; // 重启失败时保留标志，下次任务继续重试
    }
    if (rx_gap) {
        uint32_t irq = __get_PRIMASK();
        __disable_irq();
        rx_tail = rx_head; // 丢弃断流前的数据，重新寻找帧头
        rx_gap = 0U;
        __set_PRIMASK(irq);
        frame_index = 0U;
        data->valid = 0U;
    }

    // 每次最多消费一圈队列，避免连续接收时长期占用任务。
    uint8_t byte;
    uint32_t received_ms;
    for (uint16_t n = 0; n < RX_SIZE && !restart_rx && !rx_gap; ++n) {
        if (!HWT101_ReadByte(&byte, &received_ms)) break;
        HWT101_ParseByte(byte, received_ms, data);
    }
}
