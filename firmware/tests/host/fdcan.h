#ifndef HOST_FDCAN_H
#define HOST_FDCAN_H
#include "main.h"
typedef struct
{
    uint32_t Identifier, IdType, TxFrameType, DataLength, ErrorStateIndicator, BitRateSwitch;
    uint32_t FDFormat, TxEventFifoControl, MessageMarker;
} FDCAN_TxHeaderTypeDef;
typedef struct
{
    uint32_t Identifier, IdType, RxFrameType, DataLength;
} FDCAN_RxHeaderTypeDef;
typedef struct
{
    uint32_t BusOff;
} FDCAN_ProtocolStatusTypeDef;
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *, FDCAN_ProtocolStatusTypeDef *);
typedef struct
{
    uint32_t TxErrorCnt, RxErrorCnt, RxErrorPassive, ErrorLogging;
} FDCAN_ErrorCountersTypeDef;
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(const FDCAN_HandleTypeDef *, FDCAN_ErrorCountersTypeDef *);
#define FDCAN_EXTENDED_ID             1U
#define FDCAN_STANDARD_ID             0U
#define FDCAN_DATA_FRAME              0U
#define FDCAN_ESI_ACTIVE              0U
#define FDCAN_BRS_OFF                 0U
#define FDCAN_CLASSIC_CAN             0U
#define FDCAN_NO_TX_EVENTS            0U
#define FDCAN_ACCEPT_IN_RX_FIFO0      0U
#define FDCAN_REJECT_REMOTE           1U
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 1U
#define FDCAN_IT_RX_FIFO0_MESSAGE_LOST 8U
#define FDCAN_RX_FIFO0                0U
#define FDCAN_DLC_BYTES_0             0U
#define FDCAN_DLC_BYTES_1             1U
#define FDCAN_DLC_BYTES_2             2U
#define FDCAN_DLC_BYTES_3             3U
#define FDCAN_DLC_BYTES_4             4U
#define FDCAN_DLC_BYTES_5             5U
#define FDCAN_DLC_BYTES_6             6U
#define FDCAN_DLC_BYTES_7             7U
#define FDCAN_DLC_BYTES_8             8U
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *, uint32_t, uint32_t, uint32_t,
                                               uint32_t);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *, uint32_t, uint32_t);
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *, FDCAN_TxHeaderTypeDef *,
                                                uint8_t *);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *, uint32_t, FDCAN_RxHeaderTypeDef *,
                                         uint8_t *);
#endif
