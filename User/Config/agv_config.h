#ifndef AGV_CONFIG_H
#define AGV_CONFIG_H

#define AGV_MODE_COMMAND     0
#define AGV_MODE_SURVEY      1
#define AGV_MODE_COMPETITION 2

#ifndef AGV_MODE
#define AGV_MODE AGV_MODE_COMMAND
#endif

#ifndef AGV_START_ZONE
#define AGV_START_ZONE 1
#endif

#if AGV_MODE < 0 || AGV_MODE > 2 || AGV_START_ZONE < 1 || AGV_START_ZONE > 2
#error Invalid AGV configuration
#endif

/* At startup front faces field +X. IMU zero is captured automatically. */
#define AGV_START_DELAY_MS   3000U
#define AGV_READY_TIMEOUT_MS 15000U
#define AGV_LEG_TIMEOUT_MS   30000U

/* 出发区中心（场地坐标，mm）。 */
#define AGV_START_X_MM       2250
#define AGV_START_Y_MM(zone) ((zone) == 1 ? 2250 : 150)

#define AGV_POSITION_TOL_MM 5.0f
#define AGV_CROSS_TRACK_MM  8.0f
/* 航向安全限：持续超过 AGV_HEADING_FAULT_MS 才判故障，避免起步瞬态误停车。
   0.2° 精度指标由底盘 heading_peak_dynamic / heading_over_limit 统计，不再用来停车。 */
#define AGV_HEADING_FAULT_DEG 3.0f
#define AGV_HEADING_FAULT_MS  200U

#define AGV_SURVEY_SPEED_MM_S 150.0f

#define AGV_SEARCH_BUDGET  32U // 每拍 A* 最多扩展的节点数
#define AGV_CORRIDOR_CELLS 15U // 单段规划走廊半宽（栅格）

#if defined(__arm__) || defined(__thumb__)
#define AGV_RAM __attribute__((section(".agv_ram"), aligned(4)))
#else
#define AGV_RAM
#endif

#endif
