#ifndef SURVEY_H
#define SURVEY_H

#include "map.h"

/* 生成从出发区角点开始并回到该点、覆盖全部 12 条通道的最短巡检回路。 */
void Survey_Init(unsigned zone);

/* 出发区对应的交叉点编号。 */
unsigned Survey_StartNode(unsigned zone);

unsigned Survey_Count(void);

uint8_t Survey_Leg(unsigned index, MapPoint *a, MapPoint *b, unsigned *edge);

MapPoint Survey_Gateway(unsigned zone);

#endif
