#ifndef ASTAR_H
#define ASTAR_H

#include "map.h"

typedef enum
{
    ASTAR_IDLE,
    ASTAR_BUSY,
    ASTAR_FOUND,
    ASTAR_NO_PATH,
    ASTAR_INVALIDATED
} AStarStatus;

uint8_t AStar_Begin(MapCell start, MapCell goal, uint16_t r0, uint16_t c0, uint16_t r1,
                    uint16_t c1);

void AStar_Step(unsigned budget);

AStarStatus AStar_Status(void);

uint16_t AStar_PathLength(void);

uint8_t AStar_PathCell(uint16_t index, MapCell *p);

#endif
