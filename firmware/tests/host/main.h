#ifndef HOST_MAIN_H
#define HOST_MAIN_H
#include <stdint.h>
#include <stddef.h>
typedef enum
{
    HAL_OK,
    HAL_ERROR,
    HAL_BUSY,
    HAL_TIMEOUT
} HAL_StatusTypeDef;
typedef struct
{
    int dummy;
} FDCAN_HandleTypeDef;
typedef struct
{
    struct
    {
        uint32_t Mode;
    } Init;
    uint32_t remaining;
} DMA_HandleTypeDef;
typedef struct
{
    uint32_t RxState;
    DMA_HandleTypeDef *hdmarx;
    uint32_t gState;
} UART_HandleTypeDef;
typedef struct
{
    uint32_t Enable, Number, BaseAddress, Size, TypeExtField, AccessPermission;
    uint32_t DisableExec, IsShareable, IsCacheable, IsBufferable;
} MPU_Region_InitTypeDef;
#define DMA_CIRCULAR                   1U
#define MPU_REGION_ENABLE              1U
#define MPU_REGION_NUMBER1             1U
#define MPU_REGION_SIZE_256B           7U
#define MPU_TEX_LEVEL1                 1U
#define MPU_REGION_FULL_ACCESS         3U
#define MPU_INSTRUCTION_ACCESS_DISABLE 1U
#define MPU_ACCESS_SHAREABLE           1U
#define MPU_ACCESS_NOT_CACHEABLE       0U
#define MPU_ACCESS_NOT_BUFFERABLE      0U
#define MPU_PRIVILEGED_DEFAULT         4U
#define __HAL_DMA_GET_COUNTER(d)       ((d)->remaining)
#define __HAL_RCC_D2SRAM1_CLK_ENABLE() ((void)0)
void HAL_MPU_Disable(void);
void HAL_MPU_ConfigRegion(MPU_Region_InitTypeDef *);
void HAL_MPU_Enable(uint32_t);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
typedef struct
{
    uint32_t CYCCNT, CTRL, LAR;
} host_dwt;
typedef struct
{
    uint32_t DEMCR;
} host_debug;
extern host_dwt host_clock;
extern host_debug host_core;
extern uint32_t SystemCoreClock;
#define DWT                        (&host_clock)
#define CoreDebug                  (&host_core)
#define CoreDebug_DEMCR_TRCENA_Msk 1U
#define DWT_CTRL_CYCCNTENA_Msk     1U
#define HAL_UART_STATE_READY       0U
#define HAL_UART_RXEVENT_IDLE      1U
static inline uint32_t __get_PRIMASK(void)
{
    return 0;
}
static inline void __set_PRIMASK(uint32_t x)
{
    (void)x;
}
static inline void __disable_irq(void)
{
}
static inline void __DMB(void)
{
}
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
uint32_t HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *);
#endif
