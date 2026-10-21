#include "hwt101.h"
#include "chassis_cmd.h"

/* HAL 回调统一入口，各驱动按串口句柄判断是否处理。 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    HWT101_RxEvent(huart, size);
    ChassisCmd_RxEvent(huart, size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    HWT101_RxError(huart);
}
