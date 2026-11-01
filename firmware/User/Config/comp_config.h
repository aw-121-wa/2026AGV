#ifndef COMP_CONFIG_H
#define COMP_CONFIG_H

/* 比赛模式（AGV_MODE_COMPETITION）专用配置。
   指令模式 / 跑图模式不初始化、不调用这里的任何外设和模块。 */

/* ---------------- RPLIDAR C1 ----------------
   USART1：PA9 = MCU TX → 雷达 RX，PA10 = MCU RX ← 雷达 TX；460800 8N1，3.3V 电平。
   雷达 5V 单独供电（电机 + 测距核心约 230 mA），GND 与 MCU 共地。 */
#define COMP_LIDAR_BAUD 460800U

/* 安装位置（车体系：原点 = 车体中心，x 向车头，y 向车左，mm）。
   COMP_LIDAR_YAW_DEG：雷达 0° 方向相对车头的逆时针夹角。雷达扫描平面必须低于 100 mm
   （障碍物高度），否则看不到障碍物。 */
#ifndef COMP_LIDAR_X_MM
#define COMP_LIDAR_X_MM 0.0f
#endif
#ifndef COMP_LIDAR_Y_MM
#define COMP_LIDAR_Y_MM 0.0f
#endif
#ifndef COMP_LIDAR_YAW_DEG
#define COMP_LIDAR_YAW_DEG 0.0f
#endif

#define COMP_LIDAR_MIN_MM       160.0f // 更近的点视为车体 / 夹爪自身遮挡
#define COMP_LIDAR_MAX_MM       1800.0f // C1 精度 ±30 mm，更远的点不参与识别
#define COMP_LIDAR_BOOT_MS      600U   // 上电后等雷达自检完成再发命令
#define COMP_LIDAR_STOP_MS      20U    // STOP 之后隔多久再发 SCAN
#define COMP_LIDAR_SILENT_MS    1000U  // 超过此时间没有新采样就重新发 STOP + SCAN

/* ---------------- 障碍物识别 ---------------- */
#define COMP_OBS_RADIUS_MM    25.0f // 实际障碍物 Ø50
#define COMP_OBS_GAP_MM       45.0f // 同一簇相邻点最大间距
#define COMP_OBS_MAX_WIDTH_MM 95.0f // 簇首尾弦长上限（Ø50 + 测距噪声），更宽的是围挡/其他物体
#define COMP_OBS_WALL_MM      60.0f // 离场地边界更近的点视为围挡
#define COMP_OBS_STATIC_MM    40.0f // 离已知静态结构 / 已登记障碍更近的点忽略
#define COMP_OBS_MATCH_MM     60.0f // 跨帧关联距离
#define COMP_OBS_CONFIRM      4U    // 连续命中多少圈才确认
#define COMP_OBS_FORGET       20U   // 未确认目标多少圈没再看到就丢弃
#define COMP_OBS_MAP_RADIUS   35U   // 写入地图的半径 = 25 + 测量误差余量
#define COMP_OBS_TRACKS       24U

/* ---------------- 视觉上位机 ----------------
   USART2：PA2 = MCU TX → 上位机 RX，PA3 = MCU RX ← 上位机 TX；115200 8N1，3.3V 电平。 */
#define COMP_VISION_BAUD       115200U
#define COMP_VISION_TIMEOUT_MS 500U // 超过此时间没收到上位机心跳判离线
#define COMP_POSE_PERIOD_MS    100U // 下发位姿周期
#define COMP_MODE_PERIOD_MS    200U // 重复下发视觉模式（上位机重启后能自动同步）
#define COMP_REPORT_PERIOD_MS  1000U // USART6 文本状态行周期

/* ---------------- 二维码扫码模块 ----------------
   USART3：PB10 = MCU TX → 模块 RX，PB11 = MCU RX ← 模块 TX；8N1，3.3V 电平。
   模块要设成连续或感应读码（固件不发触发命令），并关掉补光灯：规则只允许垂直向下补光。 */
#define COMP_SCANNER_BAUD    9600U // 常见模块的出厂值，以模块手册为准
#define COMP_SCANNER_IDLE_MS 30U   // 模块没有行尾符时，超过此时间没有新字节就算一行结束

#endif
