#ifndef MAP_H
#define MAP_H

#include <stdint.h>
#define MAP_WIDTH_MM  2400
#define MAP_HEIGHT_MM 2400
#define MAP_CELL_MM   10

#define MAP_COLS  (MAP_WIDTH_MM / MAP_CELL_MM)
#define MAP_ROWS  (MAP_HEIGHT_MM / MAP_CELL_MM)
#define MAP_CELLS (MAP_COLS * MAP_ROWS)

#define ROBOT_LENGTH_MM 270
#define ROBOT_WIDTH_MM  270
#define MAP_MARGIN_MM   10

#define ROBOT_HALF_COLS                                                                            \
    (((ROBOT_LENGTH_MM + 1) / 2 + MAP_MARGIN_MM + MAP_CELL_MM - 1) / MAP_CELL_MM)
#define ROBOT_HALF_ROWS (((ROBOT_WIDTH_MM + 1) / 2 + MAP_MARGIN_MM + MAP_CELL_MM - 1) / MAP_CELL_MM)

/* Nominal symmetric placement: calibrate against actual field. */
#define MAP_ISLAND_LOW  550
#define MAP_ISLAND_HIGH 1400
#define MAP_ISLAND_SIZE 450

#define MAP_STATION_BEGIN 910
#define MAP_STATION_END   1490
#define MAP_STATION_DEPTH 150

/* Conservative reservation for raw table intrusion (75-85 mm). */
#define MAP_TOP_RESERVE_MM 85

#define MAP_MAX_OBSTACLES 16

typedef enum
{
    MAP_FREE = 0,
    MAP_BLOCKED = 1
} MapCellType;

typedef struct
{
    uint16_t row, col;
} MapCell;

typedef struct
{
    int32_t x, y;
} MapPoint;

/* 清空动态障碍并构建原始栅格地图。 */
void Map_Init(void);

/* 查询原始栅格是否空闲，不包含车身包络。 */
uint8_t Map_IsFree(uint16_t row, uint16_t col);

/* 判断栅格中心能否容纳车身和默认余量。 */
uint8_t Map_CanStand(uint16_t row, uint16_t col);

/* 将场地毫米坐标换算为行列号。 */
uint8_t Map_WorldToGrid(int32_t x, int32_t y, uint16_t *row, uint16_t *col);

/* 返回栅格中心的场地毫米坐标。 */
uint8_t Map_GridToWorld(uint16_t row, uint16_t col, MapPoint *p);

/* 检查轴对齐车身和指定额外余量。 */
uint8_t Map_CanPose(float x, float y, float margin);

/* 检查带朝向车身的保守外接矩形。 */
uint8_t Map_CanPoseHeading(float x, float y, float heading_deg, float margin);

/* 检查轴向移动段的完整车身扫掠区域。 */
uint8_t Map_CanSegment(MapPoint a, MapPoint b, float margin);

/* 添加圆形障碍并递增地图版本。 */
uint8_t Map_AddObstacle(int32_t x, int32_t y, uint16_t radius);

/* 读取当前地图版本，用于使旧路径失效。 */
uint32_t Map_Revision(void);

/* 获取指定出发区中心。 */
uint8_t Map_StartPoint(uint8_t zone, MapPoint *p);

#endif
