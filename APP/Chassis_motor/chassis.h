#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "fdcan.h"
#include "zdt_x42s.h"

/* 机械参数 */
#define CHASSIS_WHEEL_DIA_MM  77.0f    // 麦轮外径（含辊子包络），卡尺量
#define CHASSIS_LX_MM         210.0f   // 轴距：前后轮中心距
#define CHASSIS_LY_MM         230.4f   // 轮距：左右轮中心距
#define CHASSIS_GEAR_RATIO    1.0f     // 电机转数/轮转数，直驱填 1.0
#define CHASSIS_K_MM          ((CHASSIS_LX_MM + CHASSIS_LY_MM) * 0.5f)

/* 节拍 */
#define CHASSIS_PERIOD_US     5000U
#define CHASSIS_PERIOD_S      0.005f

/* 输入限幅与斜坡 */
#define CHASSIS_MAX_LINEAR    1200.0f  // mm/s
#define CHASSIS_MAX_ANGULAR   240.0f   // deg/s
#define CHASSIS_ACC_LINEAR    2000.0f  // mm/s^2
#define CHASSIS_ACC_ANGULAR   720.0f   // deg/s^2

/* 驱动器下发 */
#define CHASSIS_DRV_ACC       0U       // ZDT 加速度档位，0 = 不额外限制，交给软件斜坡
#define CHASSIS_FEEDBACK_DIV  4U       // 每 4 周期轮询一个轴，4 轴一圈 80 ms
#define CHASSIS_ENABLE_DIV    20U      // 每 20 周期（100 ms）重发使能
#define CHASSIS_RESEND_DIV    20U      // 每 20 周期（100 ms）强制重发速度，兜底丢帧

#define CHASSIS_YAW_SIGN      1.0f     //  // 1.0f = 正向为顺时针减小，-1.0f = 正向为逆时针减小

typedef enum {
    CHASSIS_WHEEL_LF = 0,
    CHASSIS_WHEEL_RF,
    CHASSIS_WHEEL_LR,
    CHASSIS_WHEEL_RR,
    CHASSIS_WHEEL_NUM,
} chassis_wheel;

typedef struct {
    float vx;   // mm/s，向前为正
    float vy;   // mm/s，向左为正
    float wz;   // deg/s，逆时针为正
} chassis_vel;

typedef enum {
    CHASSIS_STATE_IDLE = 0, // 上电未使能，正在重试
    CHASSIS_STATE_READY,    // 四轴使能确认，正常接受速度指令
    CHASSIS_STATE_FAULT,    // 有轴堵转或保护，已断输出
} chassis_state;

typedef struct {
    chassis_vel target;                        // 上位给的目标速度
    chassis_vel current;                       // 斜坡后的实际下发速度
    int16_t wheel_rpm[CHASSIS_WHEEL_NUM];      // 解算后的目标轮速
    int16_t wheel_rpm_meas[CHASSIS_WHEEL_NUM]; // 反馈测得的轮速
    float pos_x;        // 里程计：沿车头累计位移，mm
    float pos_y;        // 里程计：横向累计位移，mm
    float yaw;          // 航向角，deg
    float dt_actual;    // 本周期 DWT 实测步长，s
    uint32_t jitter_us; // 观测到的最大节拍偏差，µs
    uint8_t state;
    uint32_t cycle;     // 周期计数，用于分频
} chassis_t;

extern chassis_t chassis;

// 在调度器启动前调用一次；内部完成 DWT、CAN 总线、四轴注册。
HAL_StatusTypeDef Chassis_Init(FDCAN_HandleTypeDef *hfdcan);
// 写目标速度即可返回，不阻塞；由 Chassis_Update 负责解算下发。
void Chassis_SetVelocity(float vx, float vy, float wz);
// 立即清零目标并断输出。
void Chassis_Stop(void);
// 只在一个任务里按 CHASSIS_PERIOD_US 周期调用。
void Chassis_Update(void);
// DWT 微秒时基，供任务层对齐节拍。
uint32_t Chassis_Micros(void);

#endif
