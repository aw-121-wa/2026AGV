#include "main.h"
#include "chassis.h"
#include "hwt101.h"
#include <string.h>

/* ---------- DWT 微秒时基 ---------- */
static void Chassis_DwtInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 打开跟踪单元时钟
    DWT->CYCCNT = 0U;                               // 清计数器
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;           // 开始每个 CPU 周期自增
}

uint32_t Chassis_Micros(void)
{
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

/* 电机地址表，顺序必须与 chassis_wheel 一致 */
static const uint8_t k_addr[CHASSIS_WHEEL_NUM] = {
    ZDT_ADDR_LF, ZDT_ADDR_RF, ZDT_ADDR_LR, ZDT_ADDR_RR
};

/* X 型符号矩阵：行=轮，列=(vx, vy, K·ω)。M·Mᵀ=4I，正解即 Mᵀ/4 */
static const int8_t k_sign[CHASSIS_WHEEL_NUM][3] = {
    { +1, -1, -1 },   // LF
    { +1, +1, +1 },   // RF
    { +1, +1, -1 },   // LR
    { +1, -1, +1 },   // RR
};

chassis_t chassis;
static zdt_motor *wheel[CHASSIS_WHEEL_NUM];
static uint32_t   s_last_us;                            // 上一拍的时间戳
static int16_t    s_last_rpm[CHASSIS_WHEEL_NUM];        // 上次成功下发的轮速


static float Chassis_Clamp(float x, float lim)
{
    if (x >  lim) return  lim;
    if (x < -lim) return -lim;
    return x;
}

static float Chassis_Ramp(float cur, float tgt, float step)
{
    float diff = tgt - cur;
    if (diff >  step) return cur + step;
    if (diff < -step) return cur - step;
    return tgt;
}

/* 逆运动学：车体速度 -> 四轮 RPM，并做整体等比限幅 */
static void Chassis_Solve(const chassis_vel *v, int16_t out[CHASSIS_WHEEL_NUM])
{
    const float rpm_per_mmps = 60.0f / (3.14159265f * CHASSIS_WHEEL_DIA_MM); //轮子转一圈移动的距离 = πD，mm/s -> RPM
    const float w_rad = v->wz * 0.01745329f;   // deg/s -> rad/s
    float raw[CHASSIS_WHEEL_NUM];
    float peak = 0.0f;

    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; ++i) {
        float v_wheel = (float)k_sign[i][0] * v->vx
                      + (float)k_sign[i][1] * v->vy
                      + (float)k_sign[i][2] * CHASSIS_K_MM * w_rad;
        raw[i] = v_wheel * rpm_per_mmps * CHASSIS_GEAR_RATIO;

        if (raw[i] > peak)        peak =  raw[i];
        else if (-raw[i] > peak)  peak = -raw[i];
    }

    /* 超限时四轮同乘一个系数，合速度方向不变。
       若改成逐个 clamp，超限的那个轮子被截短，车会歪着走。 */
    float scale = 1.0f;
    if (peak > (float)ZDT_MAX_RPM) {
        scale = (float)ZDT_MAX_RPM / peak;
    }

    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; ++i) {
        out[i] = (int16_t)(raw[i] * scale);
    }
}

/* 只在轮速变化或到强制重发周期时才发帧 */
static void Chassis_Apply(void)
{
    uint8_t force = ((chassis.cycle % CHASSIS_RESEND_DIV) == 0U);

    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; ++i) {
        if (wheel[i] == NULL) continue;
        if (!force && (chassis.wheel_rpm[i] == s_last_rpm[i])) continue;

        if (ZDT_SetSpeed(wheel[i], chassis.wheel_rpm[i], CHASSIS_DRV_ACC) == HAL_OK) {
            s_last_rpm[i] = chassis.wheel_rpm[i];
        }
        /* 返回 ERROR 时不更新 s_last_rpm，下个周期自动重试 */
    }
}

static uint8_t Chassis_HasFault(void)
{
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; ++i) {
        if (wheel[i] == NULL) continue;
        if (wheel[i]->stalled || wheel[i]->protected_flag) return 1U;
    }
    return 0U;
}

static uint8_t Chassis_AllEnabled(void)
{
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; ++i) {
        if ((wheel[i] == NULL) || (wheel[i]->enabled == 0U)) return 0U;
    }
    return 1U;
}

static void Chassis_EnableAll(void)
{
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; ++i) {
        if (wheel[i] != NULL) (void)ZDT_EnableMotor(wheel[i], 1U);
    }
}

static void Chassis_StopAll(void)
{
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; ++i) {
        if (wheel[i] != NULL) (void)ZDT_Stop(wheel[i]);
    }
}

/* 每轮只问一个轴，避免 12 条反馈命令挤爆深度 8 的 TX FIFO */
static void Chassis_RequestOne(void)
{
    uint8_t idx = (uint8_t)((chassis.cycle / CHASSIS_FEEDBACK_DIV) % CHASSIS_WHEEL_NUM);
    if (wheel[idx] != NULL) {
        (void)ZDT_RequestFeedback(wheel[idx]);
    }
}

static void Chassis_UpdateOdometry(float dt)
{
    const float mmps_per_rpm = 3.14159265f * CHASSIS_WHEEL_DIA_MM / 60.0f;
    float v[CHASSIS_WHEEL_NUM];

    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; ++i) {
        float rpm = (wheel[i] != NULL) ? wheel[i]->speed_rpm : 0.0f;
        v[i] = rpm * mmps_per_rpm / CHASSIS_GEAR_RATIO;
        chassis.wheel_rpm_meas[i] = (int16_t)rpm;
    }

    /* 正运动学：符号矩阵正交，正解 = Mᵀ/4，不用求逆 */
    float vx = 0.25f * ( v[0] + v[1] + v[2] + v[3]);
    float vy = 0.25f * (-v[0] + v[1] + v[2] - v[3]);

    chassis.pos_x += vx * dt;
    chassis.pos_y += vy * dt;
    chassis.yaw = CHASSIS_YAW_SIGN * imu.yaw;
}

void Chassis_Update(void)
{
    /* ---- 0. DWT 量真实步长 ---- */
    uint32_t now_us  = Chassis_Micros();
    uint32_t elapsed = now_us - s_last_us;   // 无符号减法，跨 CYCCNT 回绕也成立
    s_last_us = now_us;

    float dt = (float)elapsed * 0.000001f;
    if ((elapsed < 500U) || (elapsed > 50000U)) {
        dt = CHASSIS_PERIOD_S;               // 首拍、调试暂停、异常调度都走这里
    }
    chassis.dt_actual = dt;

    uint32_t dev = (elapsed > CHASSIS_PERIOD_US) ? (elapsed - CHASSIS_PERIOD_US)
                                                 : (CHASSIS_PERIOD_US - elapsed);
    if (dev > chassis.jitter_us) chassis.jitter_us = dev;

    chassis.cycle++;

    /* ---- 1. 先把中断收到的帧解析掉 ---- */
    ZDT_ProcessData();

    /* ---- 2. 运行中出现堵转/保护：断输出并锁定 ---- */
    if ((chassis.state == CHASSIS_STATE_READY) && Chassis_HasFault()) {
        Chassis_Stop();
        chassis.state = CHASSIS_STATE_FAULT;
    }

    /* ---- 3. 使能阶段 ---- */
    if (chassis.state == CHASSIS_STATE_IDLE) {
        if ((chassis.cycle % CHASSIS_ENABLE_DIV) == 0U) Chassis_EnableAll();
        if (!Chassis_AllEnabled()) return;
        chassis.state = CHASSIS_STATE_READY;
    }

    /* ---- 4. 故障锁定：继续读状态，故障位清零后自动恢复 ---- */
    if (chassis.state == CHASSIS_STATE_FAULT) {
        if ((chassis.cycle % CHASSIS_FEEDBACK_DIV) == 0U) {
            Chassis_RequestOne();
        }
        if (Chassis_HasFault()) return;
        chassis.state = CHASSIS_STATE_READY;
    }

    /* ---- 5. 目标速度斜坡，步长用实测 dt ---- */
    float step_lin = CHASSIS_ACC_LINEAR * dt;
    float step_ang = CHASSIS_ACC_ANGULAR * dt;
    chassis.current.vx = Chassis_Ramp(chassis.current.vx, chassis.target.vx, step_lin);
    chassis.current.vy = Chassis_Ramp(chassis.current.vy, chassis.target.vy, step_lin);
    chassis.current.wz = Chassis_Ramp(chassis.current.wz, chassis.target.wz, step_ang);

    /* ---- 6. 解算 + 限幅 ---- */
    Chassis_Solve(&chassis.current, chassis.wheel_rpm);

    /* ---- 7. 下发 ---- */
    Chassis_Apply();

    /* ---- 8. 轮询一个轴的反馈 ---- */
    if ((chassis.cycle % CHASSIS_FEEDBACK_DIV) == 0U) {
        Chassis_RequestOne();
    }

    /* ---- 9. 里程计，积分步长同样用实测 dt ---- */
    Chassis_UpdateOdometry(dt);
}

void Chassis_SetVelocity(float vx, float vy, float wz)
{
    chassis.target.vx = Chassis_Clamp(vx, CHASSIS_MAX_LINEAR);
    chassis.target.vy = Chassis_Clamp(vy, CHASSIS_MAX_LINEAR);
    chassis.target.wz = Chassis_Clamp(wz, CHASSIS_MAX_ANGULAR);
}

void Chassis_Stop(void)
{
    memset(&chassis.target,  0, sizeof(chassis.target));
    memset(&chassis.current, 0, sizeof(chassis.current));
    memset(chassis.wheel_rpm, 0, sizeof(chassis.wheel_rpm));
    memset(s_last_rpm, 0, sizeof(s_last_rpm));
    Chassis_StopAll();
}
