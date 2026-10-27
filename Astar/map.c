#include "map.h"
#include "agv_config.h"
#include <math.h>
#include <string.h>

typedef struct
{
    float x0, y0, x1, y1;
} Rect;

typedef struct
{
    int32_t x, y;
    uint16_t radius;
} Obstacle;

static uint8_t map_grid[MAP_ROWS][MAP_COLS] AGV_RAM;
static Obstacle obstacles[MAP_MAX_OBSTACLES];
static uint8_t obstacle_count, initialized;
static uint32_t revision;
static const Rect forbidden[] = {
    {MAP_ISLAND_LOW, MAP_ISLAND_LOW, MAP_ISLAND_LOW + MAP_ISLAND_SIZE,
     MAP_ISLAND_LOW + MAP_ISLAND_SIZE},
    {MAP_ISLAND_HIGH, MAP_ISLAND_LOW, MAP_ISLAND_HIGH + MAP_ISLAND_SIZE,
     MAP_ISLAND_LOW + MAP_ISLAND_SIZE},
    {MAP_ISLAND_LOW, MAP_ISLAND_HIGH, MAP_ISLAND_LOW + MAP_ISLAND_SIZE,
     MAP_ISLAND_HIGH + MAP_ISLAND_SIZE},
    {MAP_ISLAND_HIGH, MAP_ISLAND_HIGH, MAP_ISLAND_HIGH + MAP_ISLAND_SIZE,
     MAP_ISLAND_HIGH + MAP_ISLAND_SIZE},
    {0, MAP_STATION_BEGIN, MAP_STATION_DEPTH, MAP_STATION_END},
    {MAP_STATION_BEGIN, 0, MAP_STATION_END, MAP_STATION_DEPTH},
    {0, MAP_HEIGHT_MM - MAP_TOP_RESERVE_MM, 2100, MAP_HEIGHT_MM}};

/* 判断一个矩形区域是否在场内且不压障碍、不碰禁区。 */
static uint8_t BoxFree(Rect b)
{
    /* 1 micrometre tolerance only for floating-point coordinate roundoff. */
    if (b.x0 < -0.001f || b.y0 < -0.001f || b.x1 > MAP_WIDTH_MM + 0.001f ||
        b.y1 > MAP_HEIGHT_MM + 0.001f)
        return 0;
    for (unsigned i = 0; i < sizeof(forbidden) / sizeof(forbidden[0]); ++i)
    {
        Rect q = forbidden[i];
        if (b.x0 < q.x1 && b.x1 > q.x0 && b.y0 < q.y1 && b.y1 > q.y0)
            return 0;
    }
    for (unsigned i = 0; i < obstacle_count; ++i)
    {
        float x = fmaxf(b.x0, fminf((float)obstacles[i].x, b.x1));
        float y = fmaxf(b.y0, fminf((float)obstacles[i].y, b.y1));
        float dx = x - obstacles[i].x, dy = y - obstacles[i].y;
        if (dx * dx + dy * dy <= (float)obstacles[i].radius * obstacles[i].radius)
            return 0;
    }
    return 1;
}

/* 按 10 mm 栅格逐格重建整张占据地图。 */
static void BuildGrid(void)
{
    for (unsigned r = 0; r < MAP_ROWS; ++r)
        for (unsigned c = 0; c < MAP_COLS; ++c)
        {
            Rect b = {(float)c * MAP_CELL_MM, (float)r * MAP_CELL_MM, (float)(c + 1) * MAP_CELL_MM,
                      (float)(r + 1) * MAP_CELL_MM};
            map_grid[r][c] = BoxFree(b) ? MAP_FREE : MAP_BLOCKED;
        }
}

/* 清空障碍并重建栅格地图，同时递增地图版本号。 */
void Map_Init(void)
{
    obstacle_count = 0;
    memset(obstacles, 0, sizeof(obstacles));
    BuildGrid();
    initialized = 1;
    ++revision;
}

/* 返回地图版本号，供 A* 判断地图是否被改过。 */
uint32_t Map_Revision(void)
{
    return revision;
}

/* 查询某个栅格是否可通行。 */
uint8_t Map_IsFree(uint16_t row, uint16_t col)
{
    return initialized && row < MAP_ROWS && col < MAP_COLS && map_grid[row][col] == MAP_FREE;
}

/* 用车体外接矩形判断该位置放不放得下车（不考虑朝向）。 */
uint8_t Map_CanPose(float x, float y, float margin)
{
    if (!initialized || !isfinite(x) || !isfinite(y) || !isfinite(margin) || margin < 0)
        return 0;
    float hx = ROBOT_LENGTH_MM * 0.5f + margin, hy = ROBOT_WIDTH_MM * 0.5f + margin;
    Rect b = {x - hx, y - hy, x + hx, y + hy};
    return BoxFree(b);
}

/* 判断某个栅格中心按给定余量能否停车。 */
uint8_t Map_CanStand(uint16_t row, uint16_t col)
{
    if (row >= MAP_ROWS || col >= MAP_COLS)
        return 0;
    return Map_CanPose(col * MAP_CELL_MM + MAP_CELL_MM * 0.5f,
                       row * MAP_CELL_MM + MAP_CELL_MM * 0.5f, MAP_MARGIN_MM);
}

/* 带朝向的车体包络检查，用轴对齐外框做保守估计。 */
uint8_t Map_CanPoseHeading(float x, float y, float heading_deg, float margin)
{
    if (!initialized || !isfinite(x) || !isfinite(y) || !isfinite(heading_deg) ||
        !isfinite(margin) || margin < 0)
        return 0;
    float a = heading_deg * 0.01745329252f, c = fabsf(cosf(a)), s = fabsf(sinf(a));
    float hx = (ROBOT_LENGTH_MM * c + ROBOT_WIDTH_MM * s) * 0.5f + margin;
    float hy = (ROBOT_LENGTH_MM * s + ROBOT_WIDTH_MM * c) * 0.5f + margin;
    /* Axis-aligned envelope of rotated rectangle: conservative. */
    return BoxFree((Rect){x - hx, y - hy, x + hx, y + hy});
}

/* 世界坐标（mm）转栅格行列号。 */
uint8_t Map_WorldToGrid(int32_t x, int32_t y, uint16_t *row, uint16_t *col)
{
    if (!row || !col || x < 0 || y < 0 || x >= MAP_WIDTH_MM || y >= MAP_HEIGHT_MM)
        return 0;
    *row = (uint16_t)(y / MAP_CELL_MM);
    *col = (uint16_t)(x / MAP_CELL_MM);
    return 1;
}

/* 栅格行列号转该格中心的世界坐标。 */
uint8_t Map_GridToWorld(uint16_t row, uint16_t col, MapPoint *p)
{
    if (!p || row >= MAP_ROWS || col >= MAP_COLS)
        return 0;
    p->x = col * MAP_CELL_MM + MAP_CELL_MM / 2;
    p->y = row * MAP_CELL_MM + MAP_CELL_MM / 2;
    return 1;
}

/* 判断沿轴向的一段直线扫过的区域是否无碰撞。 */
uint8_t Map_CanSegment(MapPoint a, MapPoint b, float margin)
{
    if (!initialized || margin < 0 || !isfinite(margin) || (a.x != b.x && a.y != b.y))
        return 0;
    float hx = ROBOT_LENGTH_MM * 0.5f + margin, hy = ROBOT_WIDTH_MM * 0.5f + margin;
    Rect sweep = {fminf(a.x, b.x) - hx, fminf(a.y, b.y) - hy, fmaxf(a.x, b.x) + hx,
                  fmaxf(a.y, b.y) + hy};
    return BoxFree(sweep);
}

/* 添加一个圆形障碍并重建地图、递增版本号。 */
uint8_t Map_AddObstacle(int32_t x, int32_t y, uint16_t radius)
{
    if (!initialized || obstacle_count >= MAP_MAX_OBSTACLES || !radius || radius > MAP_WIDTH_MM ||
        x < 0 || y < 0 || x >= MAP_WIDTH_MM || y >= MAP_HEIGHT_MM)
        return 0;
    obstacles[obstacle_count++] = (Obstacle){x, y, radius};
    BuildGrid();
    ++revision;
    return 1;
}

/* 返回 1 号或 2 号启动区的起始点坐标。 */
uint8_t Map_StartPoint(uint8_t zone, MapPoint *p)
{
    if (!p || (zone != 1 && zone != 2))
        return 0;
    *p = (MapPoint){2250, zone == 1 ? 2250 : 150};
    return 1;
}
