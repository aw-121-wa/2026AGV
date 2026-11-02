#include "map.h"
#include "agv_config.h"
#include "user_math.h"
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

/* 构型空间位图：bit=1 表示车身（含 MAP_MARGIN_MM）能以该栅格中心停放。
   7.2 KB，替代原来规划用不上的 57.6 KB 原始占据栅格。 */
static uint8_t stand_bits[(MAP_CELLS + 7U) / 8U] AGV_RAM;
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

static Rect RobotBox(float x, float y, float margin)
{
    float hx = ROBOT_LENGTH_MM * 0.5f + margin, hy = ROBOT_WIDTH_MM * 0.5f + margin;
    return (Rect){x - hx, y - hy, x + hx, y + hy};
}

static float CellCentre(uint16_t index)
{
    return index * MAP_CELL_MM + MAP_CELL_MM * 0.5f;
}

/* 重新计算 [r0,r1]x[c0,c1] 内的可停放位；only_clear 时只把原来可停放的格子置为不可停放。 */
static void BuildStand(uint16_t r0, uint16_t c0, uint16_t r1, uint16_t c1, uint8_t only_clear)
{
    for (uint16_t r = r0; r <= r1; ++r)
        for (uint16_t c = c0; c <= c1; ++c)
        {
            unsigned id = (unsigned)r * MAP_COLS + c;
            uint8_t mask = (uint8_t)(1U << (id & 7U));
            if (only_clear && !(stand_bits[id >> 3] & mask))
                continue;
            if (BoxFree(RobotBox(CellCentre(c), CellCentre(r), MAP_MARGIN_MM)))
                stand_bits[id >> 3] |= mask;
            else
                stand_bits[id >> 3] &= (uint8_t)~mask;
        }
}

/* 清空障碍并重建栅格地图，同时递增地图版本号。 */
void Map_Init(void)
{
    obstacle_count = 0;
    memset(obstacles, 0, sizeof(obstacles));
    BuildStand(0, 0, MAP_ROWS - 1U, MAP_COLS - 1U, 0U);
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
    if (!initialized || row >= MAP_ROWS || col >= MAP_COLS)
        return 0;
    return BoxFree((Rect){(float)col * MAP_CELL_MM, (float)row * MAP_CELL_MM,
                          (float)(col + 1U) * MAP_CELL_MM, (float)(row + 1U) * MAP_CELL_MM});
}

/* 查表判断栅格中心能否容纳车身和默认余量，O(1)。 */
uint8_t Map_CanStand(uint16_t row, uint16_t col)
{
    if (!initialized || row >= MAP_ROWS || col >= MAP_COLS)
        return 0;
    unsigned id = (unsigned)row * MAP_COLS + col;
    return (stand_bits[id >> 3] >> (id & 7U)) & 1U;
}

/* 带朝向的车体包络检查，用轴对齐外框做保守估计。 */
uint8_t Map_CanPoseHeading(float x, float y, float heading_deg, float margin)
{
    if (!initialized || !isfinite(x) || !isfinite(y) || !isfinite(heading_deg) ||
        !isfinite(margin) || margin < 0)
        return 0;
    float a = heading_deg * MATH_DEG_TO_RAD, c = fabsf(cosf(a)), s = fabsf(sinf(a));
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
    /* 新障碍只会减少可停放区域，只重算它能影响到的矩形范围。 */
    int32_t reach_x = (int32_t)(ROBOT_LENGTH_MM / 2 + MAP_MARGIN_MM + radius + MAP_CELL_MM);
    int32_t reach_y = (int32_t)(ROBOT_WIDTH_MM / 2 + MAP_MARGIN_MM + radius + MAP_CELL_MM);
    int32_t c0 = (x - reach_x) / MAP_CELL_MM, c1 = (x + reach_x) / MAP_CELL_MM;
    int32_t r0 = (y - reach_y) / MAP_CELL_MM, r1 = (y + reach_y) / MAP_CELL_MM;
    BuildStand((uint16_t)(r0 < 0 ? 0 : r0), (uint16_t)(c0 < 0 ? 0 : c0),
               (uint16_t)(r1 >= MAP_ROWS ? MAP_ROWS - 1 : r1),
               (uint16_t)(c1 >= MAP_COLS ? MAP_COLS - 1 : c1), 1U);
    ++revision;
    return 1;
}

/* 返回 1 号或 2 号启动区的起始点坐标。 */
uint8_t Map_StartPoint(uint8_t zone, MapPoint *p)
{
    if (!p || (zone != 1 && zone != 2))
        return 0;
    *p = (MapPoint){AGV_START_X_MM, AGV_START_Y_MM(zone)};
    return 1;
}
