#include "survey.h"

/* 3x3 intersections: 8 perimeter edges + 4 spokes. */
static const uint8_t edges[12][2] = {{0, 1}, {1, 2}, {3, 4}, {4, 5}, {6, 7}, {7, 8},
                                     {0, 3}, {3, 6}, {1, 4}, {4, 7}, {2, 5}, {5, 8}};
static const int32_t axis[3] = {355, 1205, 2055};
static uint8_t route[25], leg_edge[24];
static unsigned length, visited;

/* 把 0..8 的编号映射成 3×3 交叉点的世界坐标。 */
static MapPoint Point(unsigned n)
{
    return (MapPoint){axis[n % 3], axis[n / 3]};
}

/* 深度优先走遍 3×3 网格的所有边，生成覆盖式巡检路径。 */
static void Walk(unsigned n)
{
    for (unsigned e = 0; e < 12; ++e)
    {
        if ((visited & (1U << e)) || (edges[e][0] != n && edges[e][1] != n))
            continue;
        unsigned other = edges[e][0] == n ? edges[e][1] : edges[e][0];
        visited |= 1U << e;
        leg_edge[length - 1] = (uint8_t)e;
        route[length++] = (uint8_t)other;
        Walk(other);
        leg_edge[length - 1] = (uint8_t)e;
        route[length++] = (uint8_t)n;
    }
}

/* 初始化巡检路线：从指定区域的角点出发做一次覆盖遍历。 */
void Survey_Init(unsigned zone)
{
    length = 1;
    visited = 0;
    route[0] = (uint8_t)(zone == 1 ? 8 : 2);
    Walk(route[0]);
}

/* 返回路线包含的段数。 */
unsigned Survey_Count(void)
{
    return length - 1;
}

/* 取第 i 段的起点、终点和对应的网格边编号。 */
uint8_t Survey_Leg(unsigned i, MapPoint *a, MapPoint *b, unsigned *edge)
{
    if (i >= Survey_Count() || !a || !b || !edge)
        return 0;
    *a = Point(route[i]);
    *b = Point(route[i + 1]);
    *edge = leg_edge[i];
    return 1;
}

/* 返回指定区域的出入口点。 */
MapPoint Survey_Gateway(unsigned zone)
{
    return Point(zone == 1 ? 8 : 2);
}
