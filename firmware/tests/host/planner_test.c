#include <assert.h>
#include <stdio.h>
#include "astar.h"
static void finish(void)
{
    unsigned ticks = 0;
    while (AStar_Status() == ASTAR_BUSY && ++ticks < 20000)
        AStar_Step(32);
    assert(ticks < 20000);
}
int main(void)
{
    Map_Init();
    MapCell a = {120, 35}, b = {120, 205};
    assert(AStar_Begin(a, b, 0, 0, MAP_ROWS - 1, MAP_COLS - 1));
    finish();
    assert(AStar_Status() == ASTAR_FOUND && AStar_PathLength() == 171);
    for (unsigned i = 1; i < AStar_PathLength(); ++i)
    {
        MapCell x, y;
        assert(AStar_PathCell(i - 1, &x) && AStar_PathCell(i, &y));
        int dr = (int)x.row - y.row, dc = (int)x.col - y.col;
        assert(dr * dr + dc * dc == 1 && Map_CanStand(y.row, y.col));
    }
    assert(!AStar_Begin(a, (MapCell){75, 75}, 0, 0, 239, 239));
    assert(AStar_Begin(a, b, 100, 16, 139, 223));
    assert(Map_AddObstacle(1205, 1205, 200));
    finish();
    assert(AStar_Status() == ASTAR_INVALIDATED);
    assert(AStar_Begin(a, b, 100, 16, 139, 223));
    finish();
    assert(AStar_Status() == ASTAR_NO_PATH);
    puts("planner OK");
}
