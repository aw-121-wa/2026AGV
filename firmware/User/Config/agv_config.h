#ifndef AGV_CONFIG_H
#define AGV_CONFIG_H

/* 带 #ifndef 的配置允许在 CMake 命令行用 -D 覆盖，例如
   cmake --preset Release -DAGV_MODE=1 -DAGV_SURVEY_LAPS=2。
   #ifndef 只负责给默认值：命令行传了就不重复定义（否则会重定义告警）；
   紧跟的 #if/#error 在编译期卡住非法组合，避免出现 0 圈、9 圈或逐段停车开关写成 2 还能编过。 */
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

#define AGV_MODE_NAME                                                                              \
    (AGV_MODE == AGV_MODE_COMMAND  ? "COMMAND"                                                     \
     : AGV_MODE == AGV_MODE_SURVEY ? "SURVEY"                                                      \
                                   : "COMPETITION")

/* At startup front faces field +X. IMU zero is captured automatically. */
#define AGV_START_DELAY_MS   3000U
#define AGV_READY_TIMEOUT_MS 15000U // 等底盘 READY / 等 FollowPath 接受
#define AGV_LEG_TIMEOUT_MS   30000U // 整圈总超时的兜底基准：本值 + 1.5 倍预计行驶时间

/* 出发区中心（场地坐标，mm）。 */
#define AGV_START_X_MM       2250
#define AGV_START_Y_MM(zone) ((zone) == 1 ? 2250 : 150)

/* 回库定位目标值：残差只作为统计量（$A 帧）上报，不单独判故障。 */
#define AGV_POSITION_TOL_MM 5.0f
#define AGV_CROSS_TRACK_MM  8.0f
/* 航向安全限：持续超过 AGV_HEADING_FAULT_MS 才判故障，避免起步瞬态误停车。
   0.2° 精度指标由底盘 heading_peak_dynamic / heading_over_limit 统计，不再用来停车。 */
#define AGV_HEADING_FAULT_DEG 3.0f
#define AGV_HEADING_FAULT_MS  200U

/* 巡检路线的编译期开关，同样可被 -D 覆盖。 */
#ifndef AGV_SURVEY_LAPS
#define AGV_SURVEY_LAPS 1U // 完整覆盖圈数，仅最后一圈入库
#endif
#ifndef AGV_STOP_EACH_SEGMENT
#define AGV_STOP_EACH_SEGMENT 0U // 1 用于逐段停车对照
#endif
#if AGV_SURVEY_LAPS < 1 || AGV_SURVEY_LAPS > 8 || AGV_STOP_EACH_SEGMENT > 1
#error Invalid survey configuration
#endif
#ifndef AGV_SURVEY_SPEED_MM_S
#define AGV_SURVEY_SPEED_MM_S 400.0f // 巡检速度 mm/s；运行时钳位到 10~CHASSIS_MOVE_MAX_SPEED
#endif

/* 串口指令模式（F/B/L/R/T/A）的移动速度 mm/s。 */
#ifndef CHASSIS_COMMAND_SPEED_MM_S
#define CHASSIS_COMMAND_SPEED_MM_S 1000.0f
#endif

#define AGV_SEARCH_BUDGET  32U // 每拍 A* 最多扩展的节点数
#define AGV_CORRIDOR_CELLS 15U // 单段规划走廊半宽（栅格）

/* 大数组（A* 的 cost/heap/position、地图构型空间位图、调参日志）放 .agv_ram：链接脚本把它单独
   放进 512 KB 的 RAM 段，避免撑爆只有 128 KB 的 DTCM；主机测试里该宏展开为空。 */
#if defined(__arm__) || defined(__thumb__)
#define AGV_RAM __attribute__((section(".agv_ram"), aligned(4)))
#else
#define AGV_RAM
#endif

#endif
