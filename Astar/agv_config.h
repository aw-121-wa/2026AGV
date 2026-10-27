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

#define AGV_POSITION_TOL_MM 5.0f
#define AGV_CROSS_TRACK_MM  8.0f
#define AGV_HEADING_TOL_DEG 0.2f // 跑图期间超出动态精度要求即停车

#define AGV_SURVEY_SPEED_MM_S 150.0f

#define AGV_SEARCH_BUDGET 32U

#if defined(__arm__) || defined(__thumb__)
#define AGV_RAM __attribute__((section(".agv_ram"), aligned(4)))
#else
#define AGV_RAM
#endif

#endif
