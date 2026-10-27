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
} AgvStatus;

extern AgvStatus agv;
void Agv_Init(unsigned mode, unsigned zone);

void Agv_Tick(void);

void Agv_Stop(void);

#endif
