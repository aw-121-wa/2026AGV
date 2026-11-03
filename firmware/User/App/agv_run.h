#ifndef AGV_RUN_H
#define AGV_RUN_H

#include <stdint.h>

typedef enum
{
    AGV_IDLE,
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
} AgvStatus;

extern AgvStatus agv;
void Agv_Init(unsigned mode, unsigned zone);

void Agv_Tick(void);

void Agv_Stop(void);

/* 把整圈路线发给调试台（ROUTE 行）；还没开走时回 ROUTE none。开走时也会自动发一次。 */
void Agv_RouteReport(void);

#endif
