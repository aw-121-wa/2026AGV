#ifndef SURVEY_H
#define SURVEY_H

#include "map.h"

void Survey_Init(unsigned zone);

unsigned Survey_Count(void);

uint8_t Survey_Leg(unsigned index, MapPoint *a, MapPoint *b, unsigned *edge);

MapPoint Survey_Gateway(unsigned zone);

#endif
