#ifndef AGV_RUN_H
#define AGV_RUN_H

#include "main.h"
#include <stdint.h>

typedef enum
{
    AGV_IDLE, // 巡检模式：等 /start 选出发区；其他模式一直是它
    AGV_WAIT,
    AGV_DOCK_OUT,
    AGV_PLAN,
    AGV_DRIVE,
    AGV_DOCK_IN,
    AGV_DONE,
    AGV_STOPPED,
    AGV_FAULT
} AgvState;

typedef enum
{
    AGV_ERR_NONE,
    AGV_ERR_READY,
    AGV_ERR_CHASSIS,
    AGV_ERR_PATH,
    AGV_ERR_POSE,
    AGV_ERR_TIMEOUT,
    AGV_ERR_MAP
} AgvError;

typedef struct
{
    AgvState state;
    AgvError error;
    uint16_t covered;
    uint16_t leg;
    float x, y;
    float home_residual_mm; // 里程计终点残差，不等于外部实测回库误差
    uint8_t zone;           // 这一趟的出发区 1/2，还没开始过是 0
} AgvStatus;

extern AgvStatus agv;
/* 上电调用一次。巡检模式也不自己出发，停在 AGV_IDLE 等 Agv_Start。 */
void Agv_Init(unsigned mode);
/* 从出发区 zone（1 或 2）开始跑一趟：车要放在那个出发区中心、车头朝 +X。没在跑（空闲、跑完、停了、
   故障）且底盘 READY、停着才接受，否则 HAL_BUSY；不是巡检模式或 zone 不对 HAL_ERROR。
   跑完、停了、故障以后把车放回出发区，可以再开始。 */
HAL_StatusTypeDef Agv_Start(unsigned zone);

void Agv_Tick(void);

void Agv_Stop(void);

/* 把整圈路线发给调试台（ROUTE 行）；还没开走时回 ROUTE none。开走时也会自动发一次。 */
void Agv_RouteReport(void);

#endif
