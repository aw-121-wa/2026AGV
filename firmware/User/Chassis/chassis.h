#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "agv_config.h"
#include "fdcan.h"
#include "zdt_x42s.h"

/* 机械参数 */
/* 卡尺量的是 77.0（含辊子包络），但按 77.0 跑 F2400 实走 2436 mm（+1.5%）：固件的里程计和下发的
   转速都用这个值换算，实测偏小会让车一直多走。按 2400/2436 标定成 78.16，里程和速度一起修正。
   现场还可以停机用 /set wheel_dia 微调。 */
#define CHASSIS_WHEEL_DIA_MM 78.16f
#define CHASSIS_LX_MM        210.0f // 轴距：前后轮中心距
#define CHASSIS_LY_MM        230.4f // 轮距：左右轮中心距
#define CHASSIS_GEAR_RATIO   1.0f   // 电机转数/轮转数，直驱填 1.0
#define CHASSIS_K_MM         ((CHASSIS_LX_MM + CHASSIS_LY_MM) * 0.5f)

/* 节拍 */
#define CHASSIS_PERIOD_US 5000U
#define CHASSIS_PERIOD_S  0.005f

/* 输入限幅与斜坡 */
#define CHASSIS_MAX_LINEAR    2000.0f // mm/s
#define CHASSIS_MAX_ANGULAR   240.0f  // deg/s
#define CHASSIS_ACC_LINEAR    2000.0f // mm/s^2，前向和横向共用，暂不猜测横向摩擦系数
#define CHASSIS_MAX_WHEEL_RPM 750U    // 底盘轮速上限，独立于驱动器协议极限
#define CHASSIS_ACC_ANGULAR   720.0f // deg/s^2

/* 驱动器下发 */
#define CHASSIS_DRV_ACC      0U // ZDT 加速度 RPM/s，0 = 驱动器不额外限制，交给软件斜坡
#define CHASSIS_FEEDBACK_DIV 4U // 每 4 周期（20 ms）位置、转速、状态各读一次，每次四轮一起读
#define CHASSIS_ENABLE_DIV   20U // 每 20 周期（100 ms）重发使能
#define CHASSIS_RESEND_DIV   20U // 每 20 周期（100 ms）强制重发速度，兜底丢帧

/* 自主运动参数：现场从低速开始校准方向和增益。 */
#define CHASSIS_MOVE_MAX_MM         5000.0f
#define CHASSIS_MOVE_TIMEOUT_US     30000000U
#define CHASSIS_FEEDBACK_TIMEOUT_MS 500U
#define CHASSIS_MOVE_MAX_SPEED      2000.0f // mm/s
#define CHASSIS_DISTANCE_KP         1.5f   // 1/s
#define CHASSIS_DISTANCE_TOL_MM     1.0f   // 定距到位容差；3 mm 时 F2400 会正好停在容差边上
#define CHASSIS_HEADING_KP          9.34f // (deg/s)/deg
#define CHASSIS_TURN_MAX_SPEED      180.0f // deg/s
#define CHASSIS_HEADING_TOL_DEG     0.1f  // 静态验收误差上限（仍需外部基准实测）
#define CHASSIS_DYNAMIC_TOL_DEG     0.2f  // 平移期间航向误差验收上限
#define CHASSIS_HEADING_KI          0.19f  // 初始积分增益，实车低速标定
#define CHASSIS_HEADING_I_MAX       7.0f  // 积分输出上限，deg/s
#define CHASSIS_HEADING_I_ZONE      8.0f  // 小误差积分，避免大角度转向积累
/* 下面三个由自动调参量出来（调试台“自动调参”，README 11.5），0 = 不用，和原来一样。
   刹车曲线：离目标 e 度时比例项最多转 sqrt(2·a·e)，就是按减速度 a 正好停在目标上的转速。没有它，
   大角度转到快到位时比例项降得比角加速度斜坡快，车跟不上就冲过头，kp 只能往小里调。
   平移前馈：平移时车身会跟着转（轮径、辊子打滑不对称），每 1 m/s 转多少 deg/s 量得出来，
   下发转速时先反着补上，航向环只管剩下的。 */
#define CHASSIS_HEADING_BRAKE       280.0f // 转向刹车曲线的减速度，deg/s^2
#define CHASSIS_HEADING_FF_X        0.91f // 前后平移每 1 m/s 补多少转速，deg/s
#define CHASSIS_HEADING_FF_Y        0.0f // 左右平移每 1 m/s 补多少转速，deg/s
#define CHASSIS_SETTLE_US           200000U
#define CHASSIS_YAW_SIGN            1.0f // 校准后使逆时针为正

/* ---------- 底盘单独调试用开关（都可以用 CMake -D 覆盖） ---------- */
/* 1 = 航向用 HWT101 陀螺仪；0 = 不接陀螺仪，航向由四轮里程计推算（只用于台架/底盘单测，
   麦轮打滑会让航向漂移，正式跑图必须用 1）。 */
#ifndef CHASSIS_USE_IMU
#define CHASSIS_USE_IMU 1
#endif
/* 每个轮子的电机安装方向：+1 = 电机正转时该轮让车前进，-1 = 反装。
   左右两侧电机通常是镜像安装的。用串口命令 W 做轮向测试（轮子离地）：每个轮子会按
   “让车前进”的方向转 1 秒，眼睛看它是不是往前滚；往后滚的轮子把这里改成 -1 再编译。
   （电机编码器只能看到自己转了多少，看不出装反，所以方向必须目测。） */
#ifndef CHASSIS_WHEEL_DIR_LF
#define CHASSIS_WHEEL_DIR_LF 1
#endif
#ifndef CHASSIS_WHEEL_DIR_RF
#define CHASSIS_WHEEL_DIR_RF -1
#endif
#ifndef CHASSIS_WHEEL_DIR_LR
#define CHASSIS_WHEEL_DIR_LR 1
#endif
#ifndef CHASSIS_WHEEL_DIR_RR
#define CHASSIS_WHEEL_DIR_RR -1
#endif
/* 单轮测试：每个轮子按“前进方向”转 CHASSIS_WHEEL_TEST_RPM，持续 CHASSIS_WHEEL_TEST_MS，再停 300 ms
 * 读位置。 */
#define CHASSIS_WHEEL_TEST_RPM  30.0f
#define CHASSIS_WHEEL_TEST_MS   1000U
#define CHASSIS_WHEEL_SETTLE_MS 300U
#define CHASSIS_RECOVER_MS      3000U // S 清故障时，最多花这么久解除堵转保护/重新使能

typedef enum
{
    CHASSIS_WHEEL_LF = 0,
    CHASSIS_WHEEL_RF,
    CHASSIS_WHEEL_LR,
    CHASSIS_WHEEL_RR,
    CHASSIS_WHEEL_NUM,
} chassis_wheel;

typedef struct
{
    float vx; // mm/s，向前为正
    float vy; // mm/s，向左为正
    float wz; // deg/s，逆时针为正
} chassis_vel;

typedef enum
{
    CHASSIS_STATE_IDLE = 0, // 上电未使能，正在重试
    CHASSIS_STATE_READY,    // 四轴使能确认，正常接受速度指令
    CHASSIS_STATE_FAULT,    // 有轴堵转或保护，已断输出
} chassis_state;

typedef enum
{
    CHASSIS_MODE_VEL = 0, // 速度模式，每周期下发轮速
    CHASSIS_MODE_MOVE,    // 编码器定距 + 航向保持
    CHASSIS_MODE_TURN,    // 指定航向转向
    CHASSIS_MODE_PATH,    // 连续航点跟踪
    CHASSIS_MODE_WHEEL,   // 单轮方向测试（串口 W 命令）
} chassis_mode;

/* 未就绪/故障原因位图（Chassis_WaitReason / chassis.fault_detail）。 */
#define CHASSIS_WAIT_IMU      0x0001U          // 陀螺仪无数据或超时
#define CHASSIS_WAIT_FB(i)    (0x0002U << (i)) // 第 i 轮反馈（位置/转速/状态）缺失或超时
#define CHASSIS_WAIT_EN(i)    (0x0020U << (i)) // 第 i 轮未使能
#define CHASSIS_WAIT_ODOM     0x0200U          // 里程计还没拿到第一组四轮位置
#define CHASSIS_WAIT_STALL(i) (0x0400U << (i)) // 第 i 轮堵转 / 堵转保护

typedef enum
{
    CHASSIS_FAULT_NONE = 0,
    CHASSIS_FAULT_SENSOR, // 运行中反馈丢失/电机掉使能/堵转，详情见 fault_detail
    CHASSIS_FAULT_MOVE_TIMEOUT, // 定距/转向超时未完成
    CHASSIS_FAULT_PATH_TIMEOUT, // 路径单段 60 s 未完成
} chassis_fault;

typedef struct
{
    uint8_t seq;        // 每完成一次测试加 1，遥测据此只打印一次
    uint8_t mask;       // 本次测试的轮子位图
    uint8_t active;     // 正在转的轮子编号 + 1，0 = 没有在测
    float delta_deg[4]; // 实测转角（已乘安装方向），度
    float meas_rpm[4];  // 运行中实测转速（已乘安装方向）
    uint8_t verdict[4]; // 见 CHASSIS_WT_*
} chassis_wheel_test;
#define CHASSIS_WT_SKIP     0U
#define CHASSIS_WT_OK       1U
#define CHASSIS_WT_REVERSED 2U // 编码器读数与命令反号：驱动器方向/编码器参数被改过
#define CHASSIS_WT_NONE     3U // 几乎没转：查使能/接线/地址
#define CHASSIS_WT_ODD      4U // 转了但角度不对：查轮径/细分/打滑
extern chassis_wheel_test chassis_wheel_result;

typedef struct
{
    chassis_vel target;                       // 上位给的目标速度
    chassis_vel current;                      // 斜坡后的实际下发速度
    int16_t wheel_rpm_x10[CHASSIS_WHEEL_NUM]; // 解算后的目标轮速，0.1 RPM
    float wheel_rpm_meas[CHASSIS_WHEEL_NUM];  // 反馈测得的轮速，RPM
    float pos_x;                              // 里程计：世界坐标 X 位移，mm
    float pos_y;                              // 里程计：世界坐标 Y 位移，mm
    float heading_target;                     // deg
    float heading_error;                      // 最短角误差，deg
    float heading_peak_dynamic;               // 本次平移观测到的最大航向误差，deg
    uint32_t heading_over_limit;              // 本次平移超出 0.2° 的控制周期数
    float yaw;                                // 航向角，deg
    float dt_actual;                          // 本周期 DWT 实测步长，s
    uint32_t jitter_us;                       // 观测到的最大节拍偏差，µs
    uint32_t loop_peak_us;                    // 观测到的单拍最长占用，µs；超过周期下一拍才会迟到
    uint8_t state;
    uint8_t mode;          // 当前模式，取值见 chassis_mode
    uint32_t cycle;        // 周期计数，用于分频
    uint8_t fault;         // 进入 FAULT 的原因，取值见 chassis_fault
    uint16_t fault_detail; // SENSOR 故障时的 CHASSIS_WAIT_* 位图
} chassis_t;

extern chassis_t chassis;

#define CHASSIS_PATH_CAPACITY     256U
#define CHASSIS_PATH_DEVIATION_MM 5.0f
/* 拐角圆弧的向心加速度预算：cap = sqrt(acc_linear × 该系数 × 半径)。
   0.25 = 原来按 1/4 线加速度算，90° 弯半径约 8 mm → 63 mm/s；1.0 = 按满线加速度算，约 126 mm/s。
   可用 -DCHASSIS_PATH_CORNER_FACTOR=... 覆盖。 */
#ifndef CHASSIS_PATH_CORNER_FACTOR
#define CHASSIS_PATH_CORNER_FACTOR 1.0f
#endif

/* 运行时可调的参数，字段名就是串口控制台里的参数名（console.c）。上面的宏是开机默认值，
   定稿的值写回宏里。只在底盘任务里读写（控制台也在底盘任务里运行），不需要加锁。 */
typedef struct
{
    float heading_kp, heading_ki, heading_i_max, heading_i_zone;
    float heading_brake, heading_ff_x, heading_ff_y;
    float turn_max_speed;
    float distance_kp, distance_tol, heading_tol, settle_ms;
    float acc_linear, acc_angular;
    float cmd_speed;
    float wheel_dia, chassis_k, path_deviation;
} chassis_tuning;

#define CHASSIS_TUNE_DEFAULTS                                                                      \
    {                                                                                              \
        .heading_kp = CHASSIS_HEADING_KP,                                                          \
        .heading_ki = CHASSIS_HEADING_KI,                                                          \
        .heading_i_max = CHASSIS_HEADING_I_MAX,                                                    \
        .heading_i_zone = CHASSIS_HEADING_I_ZONE,                                                  \
        .heading_brake = CHASSIS_HEADING_BRAKE,                                                    \
        .heading_ff_x = CHASSIS_HEADING_FF_X,                                                      \
        .heading_ff_y = CHASSIS_HEADING_FF_Y,                                                      \
        .turn_max_speed = CHASSIS_TURN_MAX_SPEED,                                                  \
        .distance_kp = CHASSIS_DISTANCE_KP,                                                        \
        .distance_tol = CHASSIS_DISTANCE_TOL_MM,                                                   \
        .heading_tol = CHASSIS_HEADING_TOL_DEG,                                                    \
        .settle_ms = CHASSIS_SETTLE_US / 1000.0f,                                                  \
        .acc_linear = CHASSIS_ACC_LINEAR,                                                          \
        .acc_angular = CHASSIS_ACC_ANGULAR,                                                        \
        .cmd_speed = CHASSIS_COMMAND_SPEED_MM_S,                                                   \
        .wheel_dia = CHASSIS_WHEEL_DIA_MM,                                                         \
        .chassis_k = CHASSIS_K_MM,                                                                 \
        .path_deviation = CHASSIS_PATH_DEVIATION_MM,                                               \
    }

extern chassis_tuning chassis_tune;
extern const chassis_tuning chassis_tune_default;

typedef struct
{
    float x, y, heading;
} chassis_waypoint;
typedef struct
{
    float distance_mm, max_cross_mm, max_heading_deg;
    uint32_t elapsed_ms;
    uint32_t max_jitter_us, max_loop_us; // 这条路径期间的最大节拍偏差、单拍最长占用
    uint8_t completed;
} chassis_path_stats;
extern chassis_path_stats chassis_path_result;
/* 提交世界坐标航点；stop_each 为逐段停车对照模式。 */
HAL_StatusTypeDef Chassis_FollowPath(const chassis_waypoint *points, unsigned count,
                                     uint8_t stop_each);
/* 将世界坐标航点追加到正在运行的路径。 */
HAL_StatusTypeDef Chassis_AppendPath(const chassis_waypoint *points, unsigned count);
/* 正在走向第几个航点：1 是 Chassis_FollowPath 给的第一个，没有路径时返回 0。 */
unsigned Chassis_PathTarget(void);

// 在调度器启动前调用一次；内部完成 DWT、CAN 总线、四轴注册。
HAL_StatusTypeDef Chassis_Init(FDCAN_HandleTypeDef *hfdcan);
// 以下运动 API 仅由底盘任务调用（MoveRequest 除外）。
// 写目标速度即可返回，不阻塞；由 Chassis_Update 负责解算下发。
void Chassis_SetVelocity(float vx, float vy, float wz);
// 立即清零目标并断输出。
void Chassis_Stop(void);
// 只在一个任务里按 CHASSIS_PERIOD_US 周期调用。
void Chassis_Update(void);
// 底盘任务做完这一拍的所有事后调用：排上反馈读取，把这一拍要发的 CAN 帧一起发出去，统计单拍占用。
void Chassis_CycleEnd(void);
// 单调递增的微秒时基，供任务层对齐节拍。
// 注意：内部有状态（会推进累加器），只在底盘任务里调用；回绕周期约 71.6 分钟。
// 算差值一律用无符号减法 uint32_t elapsed = now - last; 不要写 if (now > last)。
uint32_t Chassis_Micros(void);

// 定距移动：起始车体系 forward_mm 向前、left_mm 向左。保持起始航向。
// READY 且空闲、反馈新鲜才接受，否则 BUSY；距离向量超过 5000 mm 返回 ERROR。
HAL_StatusTypeDef Chassis_MoveDistance(float forward_mm, float left_mm);
// 同上，但移动中保持指定的绝对航向 heading_deg（用于跑图时锁定场地航向）。
HAL_StatusTypeDef Chassis_MoveDistanceHold(float forward_mm, float left_mm, float heading_deg);
// 空闲时设置下一次移动限速，超出 10~2000 mm/s 会钳位到边界；运行中保持本次限速和超时一致。
void Chassis_SetMoveSpeedLimit(float mm_per_second);

// 单轮方向测试：mask bit0..3 = LF/RF/LR/RR；READY 且空闲才接受。轮子要离地！
HAL_StatusTypeDef Chassis_WheelTest(uint8_t mask);
// 当前未就绪原因（CHASSIS_WAIT_* 位图），0 = 全部正常。
uint16_t Chassis_WaitReason(void);
// 只读访问第 i 个轮子的驱动状态，给串口诊断用。
const zdt_motor *Chassis_Wheel(unsigned i);
/* 一个轮子的反馈诊断，数组顺序是位置、转速、状态：多久没更新（valid 为 0 时还没收到过），
   发出去几条读取、收到几条回复、几条没能入队，都从上电累计。 */
typedef struct
{
    uint32_t age_ms[3], sent[3], got[3], fail[3];
    uint8_t valid[3];
} ChassisFeedbackDiag;
void Chassis_FeedbackDiag(unsigned i, ChassisFeedbackDiag *out);
// 最近一次进故障那一刻的四轮反馈诊断，留到下次进故障。
typedef struct
{
    uint32_t at_ms;
    uint32_t bus_off, bus_errors, rx_lost, rx_dropped, tx_free;
    uint16_t detail; // 同 chassis.fault_detail
    uint8_t valid, reason;
    ChassisFeedbackDiag wheel[CHASSIS_WHEEL_NUM];
} ChassisFeedbackFault;
extern ChassisFeedbackFault chassis_feedback_fault;
// 当前自主运动限速 mm/s。
float Chassis_SpeedLimit(void);

// 绝对航向 / 相对转角，单位度，范围 [-180, 180]；逆时针为正。
HAL_StatusTypeDef Chassis_TurnTo(float degrees);
HAL_StatusTypeDef Chassis_TurnBy(float degrees);

#endif
