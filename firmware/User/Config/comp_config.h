#ifndef COMP_CONFIG_H
#define COMP_CONFIG_H

/* 比赛模式（AGV_MODE_COMPETITION）专用配置。
   指令模式 / 跑图模式不初始化、不调用这里的任何外设和模块。 */

/* ---------------- RPLIDAR C1 ----------------
   USART1：PA9 = MCU TX → 雷达 RX，PA10 = MCU RX ← 雷达 TX；460800 8N1，3.3V 电平。
   雷达用单独一路 5V（电机 + 测距核心约 230 mA），但要从同一块电池稳压来，规则只允许一个电源；
   GND 与 MCU 共地。 */
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

#define COMP_LIDAR_MIN_MM       200.0f // 更近的点视为车体自身：270 mm 车身的半对角线是 191 mm
#define COMP_LIDAR_MAX_MM       3400.0f // 在启停区要看到整个场地：2400 mm 见方的对角线
#define COMP_LIDAR_BOOT_MS      600U   // 上电后等雷达自检完成再发命令
#define COMP_LIDAR_STOP_MS      20U    // STOP 之后隔多久再发 SCAN
#define COMP_LIDAR_SILENT_MS    1000U  // 超过此时间没有新采样就重新发 STOP + SCAN

/* ---------------- 开局全局扫描 ----------------
   车停着扫，开动时雷达不处理。一个位置看不到被别的障碍物挡住的东西，所以在几个位置各扫一次：
   启停区、二维码板前、去场地中央的路上。全部扫完输出结果，关掉雷达，之后不再处理雷达数据。
   随机摆放 3~5 个障碍物时，漏检的布局：扫一处 3%~11%，两处 0.1%~0.2%，三处 1000 组里没有。 */
#define COMP_SCAN_PLACES      3U     // 扫够几个不同的位置
#define COMP_SCAN_REVS        20U    // C1 约 10 圈/秒，一次约 2 秒
#define COMP_SCAN_TIMEOUT_MS  10000U // 这么久没扫完（雷达没接、不出点）就放弃
#define COMP_SCAN_MIN_MOVE_MM 300.0f // 离上次扫描的位置不到这么远，视角没变，不用再扫

/* ---------------- 开局流程的停车点 ---------------- */
#define COMP_QR_STOP_X_MM  2205   // 二维码板在 x = 2400 处：实装后按扫码模块的读码距离改
#define COMP_QR_STOP_Y_MM  1205   // 板子中心 y 在 1100~1300 之间随机
#define COMP_CENTER_MM     1205   // 场地中央十字路口
#define COMP_QR_TIMEOUT_MS 10000U // 到了二维码板前这么久还没读到任务码就停下

/* ---------------- 障碍物识别 ---------------- */
#define COMP_OBS_RADIUS_MM    25.0f // 实际障碍物 Ø50
#define COMP_OBS_GAP_MM       45.0f // 同一簇相邻点最大间距
#define COMP_OBS_MAX_WIDTH_MM 95.0f // 簇首尾弦长上限（Ø50 + 测距噪声），更宽的是围挡/其他物体
#define COMP_OBS_WALL_MM      60.0f // 离场地边界更近的点视为围挡
#define COMP_OBS_STATIC_MM    40.0f // 离转盘更近的点忽略
#define COMP_OBS_MATCH_MM     60.0f // 跨圈关联距离
#define COMP_OBS_CONFIRM      8U    // 一次扫描里至少多少圈看到才算障碍物
#define COMP_OBS_FORGET       20U   // 没到圈数的目标多少圈没再看到就丢弃
#define COMP_OBS_MAP_RADIUS   45U   // 写入地图的半径 = 25 + 定位误差：只测这一次，全场仿真最坏约 25 mm
#define COMP_OBS_TRACKS       24U

/* ---------------- 路线（/route） ---------------- */
#define COMP_ROUTE_MAX          32U  // 起点、拐点、终点总数；A* 的路线只有几个拐点
#define COMP_ROUTE_START_TOL_MM 5.0f // 开走时车离规划起点超过这个距离，说明规划后车动过

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
