#ifndef COMP_CONFIG_H
#define COMP_CONFIG_H

/* 比赛模式（AGV_MODE_COMPETITION）专用配置。
   指令模式 / 跑图模式不初始化、不调用这里的任何外设和模块。 */

/* ---------------- RPLIDAR C1 ----------------
   USART1：PA9 = MCU TX → 雷达 RX，PA10 = MCU RX ← 雷达 TX；460800 8N1，3.3V 电平。
   雷达用单独一路 5V（电机 + 测距核心约 230 mA），但要从同一块电池稳压来，规则只允许一个电源；
   GND 与 MCU 共地。 */
#define COMP_LIDAR_BAUD 460800U

/* 伸出去扫描时的安装位置（车体系：原点 = 车体中心，x 向车头，y 向车左，mm）。
   雷达装在舵机上，从车身左后角伸出去，车身只挡住朝车身那 90°。默认按 C1（外径 55.6 mm）外沿贴着
   车角算，装好以后量了改（/set lidar_x、lidar_y、lidar_yaw）。
   COMP_LIDAR_YAW_DEG：雷达 0° 方向相对车头的逆时针夹角。雷达扫描平面必须低于 100 mm
   （障碍物高度），否则看不到障碍物。 */
#ifndef COMP_LIDAR_X_MM
#define COMP_LIDAR_X_MM -155.0f
#endif
#ifndef COMP_LIDAR_Y_MM
#define COMP_LIDAR_Y_MM 155.0f
#endif
#ifndef COMP_LIDAR_YAW_DEG
#define COMP_LIDAR_YAW_DEG 0.0f
#endif

/* 伸出去的雷达按这个半径的圆算进规划（C1 外径 55.6 mm），舵机支架另算时改大。边走边扫时雷达一直
   伸着，Map_CanStand、A* 都要它和车身一起放得下。 */
#define COMP_LIDAR_RADIUS_MM    28.0f
#define COMP_LIDAR_MIN_MM       50.0f   // C1 的最小量程，更近的是噪声
#define COMP_LIDAR_BODY_MM      10.0f   // 车身轮廓外扩这么多以内的点是车体自己：雷达在车角外，打得到车身
#define COMP_LIDAR_MAX_MM       3400.0f // 在启停区要看到整个场地：2400 mm 见方的对角线
#define COMP_LIDAR_BOOT_MS      600U   // 上电后等雷达自检完成再发命令
#define COMP_LIDAR_STOP_MS      20U    // STOP 之后隔多久再发 SCAN
#define COMP_LIDAR_SILENT_MS    1000U  // 超过此时间没有新采样就重新发 STOP + SCAN

/* ---------------- 雷达舵机 ----------------
   TIM3_CH1：PA6 出 50 Hz 信号，脉宽 500~2500 us 对应 0~180°（270° 的舵机把 COMP_SERVO_RANGE_DEG
   改成 270）。舵机电源单独稳压（堵转电流大），和 MCU 共地。
   出发前外形不能超过 300 mm 见方（规则，出发后展开的不限），雷达平时收在车身里。扫描时转到
   servo_scan 把雷达伸出去，等 COMP_LIDAR_SERVO_MS 才开始读点；开局边走边扫时一直伸着，规划把它
   （COMP_LIDAR_RADIUS_MM 的圆）算进车身，原地转之前收回来，扫描结束转回 servo_stow 收进来。
   两个角度装好以后按实际改，现场先用 /set 试。 */
#define COMP_SERVO_MIN_US         500.0f
#define COMP_SERVO_MAX_US         2500.0f
#define COMP_SERVO_RANGE_DEG      180.0f
#define COMP_LIDAR_SERVO_STOW_DEG 0.0f
#define COMP_LIDAR_SERVO_SCAN_DEG 90.0f
#define COMP_LIDAR_SERVO_MS       800U // 舵机没有位置反馈：转一次给这么久
/* ---------------- 开局全局扫描 ----------------
   一个位置看不到被别的障碍物挡住的东西，车身还挡住 90°，所以要从几个位置看。开局先在启停区停着
   扫一轮（第一段路要看清了才走），之后边走边扫：每 COMP_SCAN_REVS 圈算一轮，够圈数的写进地图，
   这一轮结束时车在哪就记一个位置（和记过的隔 COMP_SCAN_MIN_MOVE_MM 以上才算新的）。换过
   COMP_SCAN_STABLE_PLACES 个位置、又连续 COMP_SCAN_STABLE_REVS 圈没有新目标和待确认的，或者记满
   COMP_SCAN_PLACES 个位置、开局过了 COMP_SCAN_TOTAL_MS，就输出结果、收雷达，之后不再处理雷达数据。 */
#define COMP_SCAN_PLACES        5U     // 最多记几个位置
#define COMP_SCAN_STABLE_PLACES 3U     // 至少换过几个位置才能提前结束
#define COMP_SCAN_STABLE_REVS   30U    // 连续这么多圈没有新目标、没有待确认的，算看全了
#define COMP_SCAN_TOTAL_MS      30000U // 边走边扫最多这么久
#define COMP_SCAN_REVS          20U    // 一轮的圈数：C1 约 10 圈/秒，一轮约 2 秒
#define COMP_SCAN_TIMEOUT_MS    10000U // 一轮这么久没扫完（雷达没接、不出点）就放弃
#define COMP_SCAN_MIN_MOVE_MM   300.0f // 两个位置至少隔这么远才算换了视角

/* ---------------- 开局流程 ----------------
   读码时扫码模块要对着二维码板所在的那面墙（+X）：按扫码模块装在车的右侧算，车头朝 +Y。
   雷达在左后角，开局摆车要让这个角朝场地里面：
   - 启停区 1（板子上方）车头朝 +Y，左后角朝左下，去板子是后退，扫码模块已经对着板子，一路不转；
   - 启停区 2（板子下方）车头朝 +X，左后角朝左上，去板子是往左平移；扫码模块这时朝下，
     到了板前收起雷达原地转到 +Y 再读码，转完雷达伸出去接着扫。
   扫码模块装在车头时 COMP_QR_HEADING_DEG 改成 0，两个启停区的朝向也要重新选。 */
#define COMP_START_HEADING_DEG(zone) ((zone) == 2 ? 0.0f : 90.0f)
#define COMP_QR_HEADING_DEG          90.0f
/* 走的方向偏离雷达看得到的那两边不到这么多也算看清了：多扫过的一窄条紧挨着车，7° 时最宽 33 mm。 */
#define COMP_SEEN_TOL_DEG 7.0f

/* 二维码板在 x = 2400 处，实装后按扫码模块的读码距离改。原地转时车角扫过半径 191 mm 的圆，
   2195 离场地边还剩 14 mm；读码时右侧离板子 70 mm。放在地图格子中间：停车差 1 mm 也还在同一格，
   沿板找码时判断能不能站和停车点一致。 */
#define COMP_QR_STOP_X_MM 2195
/* 板子中心 y 在 1100~1300 之间随机：车先到离它来的方向近的那头（启停区 1 从上面来是 1300，启停区 2
   是 1100），没读到就沿着板子慢慢走到另一头，到头还没读到就掉头，来回找。 */
#define COMP_QR_Y_MIN_MM   1100
#define COMP_QR_Y_MAX_MM   1300
#define COMP_QR_SWEEP_MM_S 50.0f  // 沿板找码的速度：实装后按扫码模块边走边读的能力改
#define COMP_CENTER_MM     1205   // 场地中央十字路口
#define COMP_QR_TIMEOUT_MS 10000U // 开始沿板找码后这么久还没读到任务码就停下

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
