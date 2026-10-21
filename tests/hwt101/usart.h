#ifndef TEST_USART_H
#define TEST_USART_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY } HAL_StatusTypeDef;
typedef struct { uint32_t RxState; } UART_HandleTypeDef;
#define HAL_UART_STATE_READY 0U
extern UART_HandleTypeDef huart1;
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
#endif
