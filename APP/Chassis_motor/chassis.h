#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "fdcan.h"
#include "zdt_x42s.h"

/* 机械参数 */
#define CHASSIS_WHEEL_DIA_MM 77.0f  // 麦轮外径（含辊子包络），卡尺量
#define CHASSIS_LX_MM        210.0f // 轴距：前后轮中心距
#define CHASSIS_LY_MM        230.4f // 轮距：左右轮中心距
#define CHASSIS_GEAR_RATIO   1.0f   // 电机转数/轮转数，直驱填 1.0
#define CHASSIS_K_MM         ((CHASSIS_LX_MM + CHASSIS_LY_MM) * 0.5f)

/* 节拍 */
#define CHASSIS_PERIOD_US 5000U
#define CHASSIS_PERIOD_S  0.005f

/* 输入限幅与斜坡 */
#define CHASSIS_MAX_LINEAR    1200.0f // mm/s
#define CHASSIS_MAX_ANGULAR   240.0f  // deg/s
#define CHASSIS_ACC_LINEAR    2000.0f // mm/s^2
#define CHASSIS_ACC_FWD       CHASSIS_ACC_LINEAR
#define CHASSIS_ACC_LAT       CHASSIS_ACC_LINEAR // 实测后独立标定，暂不猜测横向摩擦系数
#define CHASSIS_MAX_WHEEL_RPM 400U               // 底盘轮速上限，独立于驱动器协议极限
#define CHASSIS_ACC_ANGULAR   720.0f             // deg/s^2

/* 驱动器下发 */
#define CHASSIS_DRV_ACC      0U  // ZDT 加速度档位，0 = 不额外限制，交给软件斜坡
#define CHASSIS_FEEDBACK_DIV 4U  // 每 4 周期轮询一个轴，4 轴一圈 80 ms
#define CHASSIS_ENABLE_DIV   20U // 每 20 周期（100 ms）重发使能
#define CHASSIS_RESEND_DIV   20U // 每 20 周期（100 ms）强制重发速度，兜底丢帧

/* 自主运动参数：现场从低速开始校准方向和增益。 */
#define CHASSIS_MOVE_MAX_MM         5000.0f
#define CHASSIS_MOVE_TIMEOUT_US     30000000U
#define CHASSIS_FEEDBACK_TIMEOUT_MS 500U
#define CHASSIS_MOVE_MAX_SPEED      400.0f // mm/s
#define CHASSIS_DISTANCE_KP         1.5f   // 1/s
#define CHASSIS_DISTANCE_TOL_MM     3.0f
#define CHASSIS_HEADING_KP          2.0f  // (deg/s)/deg
#define CHASSIS_TURN_MAX_SPEED      90.0f // deg/s
#define CHASSIS_HEADING_TOL_DEG     0.1f  // 静态验收误差上限（仍需外部基准实测）
#define CHASSIS_DYNAMIC_TOL_DEG     0.2f  // 平移期间航向误差验收上限
#define CHASSIS_HEADING_KI          1.0f  // 初始积分增益，实车低速标定
#define CHASSIS_HEADING_I_MAX       5.0f  // 积分输出上限，deg/s
#define CHASSIS_HEADING_I_ZONE      8.0f  // 小误差积分，避免大角度转向积累
#define CHASSIS_SETTLE_US           200000U
#define CHASSIS_YAW_SIGN            1.0f // 校准后使逆时针为正

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
} chassis_mode;

typedef struct
{
    chassis_vel target;                        // 上位给的目标速度
    chassis_vel current;                       // 斜坡后的实际下发速度
    int16_t wheel_rpm[CHASSIS_WHEEL_NUM];      // 解算后的目标轮速
    int16_t wheel_rpm_meas[CHASSIS_WHEEL_NUM]; // 反馈测得的轮速
    float pos_x;                               // 里程计：世界坐标 X 位移，mm
    float pos_y;                               // 里程计：世界坐标 Y 位移，mm
    float heading_target;                      // deg
    float heading_error;                       // 最短角误差，deg
    float heading_peak_dynamic;                // 本次平移观测到的最大航向误差，deg
    uint32_t heading_over_limit;               // 本次平移超出 0.2° 的控制周期数
    float yaw;                                 // 航向角，deg
    float dt_actual;                           // 本周期 DWT 实测步长，s
    uint32_t jitter_us;                        // 观测到的最大节拍偏差，µs
    uint8_t state;
    uint8_t mode;   // 当前模式，取值见 chassis_mode
    uint32_t cycle; // 周期计数，用于分频
} chassis_t;

extern chassis_t chassis;

// 在调度器启动前调用一次；内部完成 DWT、CAN 总线、四轴注册。
HAL_StatusTypeDef Chassis_Init(FDCAN_HandleTypeDef *hfdcan);
// 以下运动 API 仅由底盘任务调用（MoveRequest 除外）。
// 写目标速度即可返回，不阻塞；由 Chassis_Update 负责解算下发。
void Chassis_SetVelocity(float vx, float vy, float wz);
// 立即清零目标并断输出。
void Chassis_Stop(void);
// 只在一个任务里按 CHASSIS_PERIOD_US 周期调用。
void Chassis_Update(void);
// 单调递增的微秒时基，供任务层对齐节拍。
// 注意：内部有状态（会推进累加器），只在底盘任务里调用；回绕周期约 71.6 分钟。
// 算差值一律用无符号减法 uint32_t elapsed = now - last; 不要写 if (now > last)。
uint32_t Chassis_Micros(void);

// 定距移动：起始车体系 forward_mm 向前、left_mm 向左。保持起始航向。
// READY 且空闲、反馈新鲜才接受，否则 BUSY；距离向量超过 5000 mm 返回 ERROR。
HAL_StatusTypeDef Chassis_MoveDistance(float forward_mm, float left_mm);
// 空闲时设置下一次移动限速，范围 10~400 mm/s；运行中保持本次限速和超时一致。
void Chassis_SetMoveSpeedLimit(float mm_per_second);
// 单槽移动请求，可在中断里调用；已有待处理请求时保留旧请求。
// 消费时若正在运动则丢弃；需要获取 BUSY/ERROR 的任务应使用 MoveDistance。
void Chassis_MoveRequest(float forward_mm, float left_mm);

// 绝对航向 / 相对转角，单位度，范围 [-180, 180]；逆时针为正。
HAL_StatusTypeDef Chassis_TurnTo(float degrees);
HAL_StatusTypeDef Chassis_TurnBy(float degrees);

#endif
