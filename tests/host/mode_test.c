#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "chassis.h"
#include "agv_config.h"
static unsigned move_calls,stop_calls,latches;
void ChassisTelemetry_Init(UART_HandleTypeDef *u) { (void)u; }
void ChassisTelemetry_Ack(char c,HAL_StatusTypeDef r) { (void)c;(void)r; }
void ChassisTelemetry_Poll(void) {}
HAL_StatusTypeDef Chassis_MoveDistance(float a,float b) { (void)a;(void)b;++move_calls;return HAL_OK; }
HAL_StatusTypeDef Chassis_TurnTo(float a) { (void)a;++move_calls;return HAL_OK; }
HAL_StatusTypeDef Chassis_TurnBy(float a) { (void)a;++move_calls;return HAL_OK; }
void Chassis_Stop(void) { ++stop_calls; }
void Agv_Stop(void) { ++latches; Chassis_Stop(); }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef *u,uint8_t *b,uint16_t n) {
 (void)u;(void)b;(void)n;return HAL_OK;
}
uint32_t HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *u) { (void)u;return HAL_UART_RXEVENT_IDLE; }
#include "../../User/Comm/chassis_cmd.c"
int main(void) {
 UART_HandleTypeDef u={0}; assert(ChassisCmd_Init(&u)==HAL_OK);
 memcpy(rx_buf,"F100",4); ChassisCmd_RxEvent(&u,4); ChassisCmd_Poll();
 assert(move_calls==(AGV_MODE==AGV_MODE_COMMAND?1U:0U));
 memcpy(rx_buf,"S",1); ChassisCmd_RxEvent(&u,1); ChassisCmd_Poll();
 assert(stop_calls==1 && latches==(AGV_MODE==AGV_MODE_COMMAND?0U:1U));
 for(unsigned i=0;i<300;++i) Push('F');
 ChassisCmd_Poll();
 assert(stop_calls==2 && latches==(AGV_MODE==AGV_MODE_COMMAND?0U:2U));
 printf("mode %d command gating/stop/overflow OK\n",AGV_MODE);
}
