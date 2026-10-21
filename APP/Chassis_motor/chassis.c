#include "main.h"
#include "chassis.h"
#include "hwt101.h"
#include <string.h>
#include <math.h>

/* ---------- DWT 微秒时基 ---------- */
/*
 * CYCCNT 是 32 位「CPU 周期数」，480 MHz 下 8.95 秒回绕一次。
 * 注意：不能写成「CYCCNT / 分频」再相减。除法会把模数从 2^32 改成
 * 2^32/480 ≈ 8.95e6，无符号减法在回绕点会算出一个约 +8.4e6 µs 的假增量，
 * 结果是任务每 8.95 秒才走一拍。
 * 正确做法：先在周期域做无符号减法（模 2^32 天然成立），再把增量换算成微秒
 * 累加到一个普通 32 位微秒计数器里，回绕周期变成 71.6 分钟。
 */
#define CHASSIS_CYC_PER_US (SystemCoreClock / 1000000U)

static uint32_t s_last_cyc;    // 上一次读到的 CYCCNT
static uint32_t s_cycle_remainder;
static uint32_t s_uptime_us;   // 软件累计微秒数，只在这里推进

static void Chassis_DwtInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 打开跟踪单元时钟
    DWT->CYCCNT = 0U;                               // 清计数器
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;           // 开始每个 CPU 周期自增
    s_last_cyc  = 0U;
    s_cycle_remainder = 0U;
    s_uptime_us = 0U;
}

uint32_t Chassis_Micros(void)
{
    uint32_t now_cyc   = DWT->CYCCNT;
    uint32_t delta_cyc = now_cyc - s_last_cyc;   // 周期域减法，回绕自动正确
    s_last_cyc = now_cyc;
    uint64_t cycles = (uint64_t)delta_cyc + s_cycle_remainder;
    s_uptime_us += (uint32_t)(cycles / CHASSIS_CYC_PER_US);
    s_cycle_remainder = (uint32_t)(cycles % CHASSIS_CYC_PER_US);
    return s_uptime_us;
}

/* 电机地址表，顺序必须与 chassis_wheel 一致 */
static const uint8_t k_addr[CHASSIS_WHEEL_NUM] = {
    ZDT_ADDR_LF, ZDT_ADDR_RF, ZDT_ADDR_LR, ZDT_ADDR_RR
};

/* X 型符号矩阵：行=轮，列=(vx, vy, K·ω)。Mᵀ·M=4I，正解即 Mᵀ/4 */
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
static uint8_t    s_force_apply;                        // 1 = 下一拍无条件重发全部轮速
static volatile uint8_t s_move_req;                     // 1 = 有未处理的定距移动请求
static volatile float   s_move_forward;                 // 请求的纵向距离，mm
static volatile float   s_move_left;                    // 请求的横向距离，mm
static uint32_t s_move_start_us;
static uint32_t s_settle_us;
static float s_goal_x, s_goal_y;
static float s_prev_deg[CHASSIS_WHEEL_NUM];
static uint32_t s_prev_seq[CHASSIS_WHEEL_NUM];
static uint8_t s_odom_valid;

static float Chassis_Wrap(float angle)
{
    angle = fmodf(angle, 360.0f);
    if (angle > 180.0f) angle -= 360.0f;
    if (angle < -180.0f) angle += 360.0f;
    return angle;
}

static uint8_t Chassis_FeedbackReady(void)
{
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    imu_data sample = imu;
    __set_PRIMASK(irq);
    uint32_t now = HAL_GetTick();
    if (!sample.valid || !isfinite(sample.yaw) ||
        (now - sample.updated_ms > CHASSIS_FEEDBACK_TIMEOUT_MS)) return 0U;
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; ++i) {
        if (!wheel[i] || !wheel[i]->position_seq || !wheel[i]->status_valid ||
            !wheel[i]->speed_valid || now - wheel[i]->speed_ms > CHASSIS_FEEDBACK_TIMEOUT_MS ||
            now - wheel[i]->position_ms > CHASSIS_FEEDBACK_TIMEOUT_MS ||
            now - wheel[i]->status_ms > CHASSIS_FEEDBACK_TIMEOUT_MS) return 0U;
    }
    return 1U;
}

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
    uint8_t failed = 0U;
    uint8_t force = s_force_apply ||
                    ((chassis.cycle % CHASSIS_RESEND_DIV) == 0U);

    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; ++i) {
        if (wheel[i] == NULL) continue;
        if (!force && (chassis.wheel_rpm[i] == s_last_rpm[i])) continue;

        if (ZDT_SetSpeed(wheel[i], chassis.wheel_rpm[i], CHASSIS_DRV_ACC) == HAL_OK) {
            s_last_rpm[i] = chassis.wheel_rpm[i];
        } else failed = 1U;
        /* 返回 ERROR 时不更新 s_last_rpm，下个周期自动重试 */
    }

    s_force_apply = failed;
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

static void Chassis_UpdateOdometry(void)
{
    if (!Chassis_FeedbackReady()) return;
    chassis.yaw = Chassis_Wrap(CHASSIS_YAW_SIGN * imu.yaw);
    /* 四轮位置均更新后才计算增量，不对旧速度反复积分。 */
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; ++i) {
        chassis.wheel_rpm_meas[i] = (int16_t)wheel[i]->speed_rpm;
        if (!wheel[i]->position_seq || wheel[i]->position_seq == s_prev_seq[i]) return;
    }
    float d[CHASSIS_WHEEL_NUM];
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; ++i) {
        d[i] = (wheel[i]->position_deg - s_prev_deg[i]) *
               (3.14159265f * CHASSIS_WHEEL_DIA_MM / (360.0f * CHASSIS_GEAR_RATIO));
        s_prev_deg[i] = wheel[i]->position_deg;
        s_prev_seq[i] = wheel[i]->position_seq;
    }
    if (!s_odom_valid) { s_odom_valid = 1U; return; }
    float dx = 0.25f * (d[0] + d[1] + d[2] + d[3]);
    float dy = 0.25f * (-d[0] + d[1] + d[2] - d[3]);
    float rad = chassis.yaw * 0.01745329252f;
    chassis.pos_x += cosf(rad) * dx - sinf(rad) * dy;
    chassis.pos_y += sinf(rad) * dx + cosf(rad) * dy;
}

static void Chassis_AutoControl(float dt, uint32_t now)
{
    float ex = s_goal_x - chassis.pos_x;
    float ey = s_goal_y - chassis.pos_y;
    float distance = hypotf(ex, ey);
    chassis.heading_error = Chassis_Wrap(chassis.heading_target - chassis.yaw);
    float vx = 0.0f, vy = 0.0f;
    if (chassis.mode == CHASSIS_MODE_MOVE && distance > CHASSIS_DISTANCE_TOL_MM) {
        float speed = fminf(CHASSIS_DISTANCE_KP * distance, CHASSIS_MOVE_MAX_SPEED);
        float rad = chassis.yaw * 0.01745329252f;
        vx = speed * (cosf(rad) * ex + sinf(rad) * ey) / distance;
        vy = speed * (-sinf(rad) * ex + cosf(rad) * ey) / distance;
    }
    chassis.target.vx = vx;
    chassis.target.vy = vy;
    chassis.target.wz = Chassis_Clamp(CHASSIS_HEADING_KP * chassis.heading_error,
                                      CHASSIS_TURN_MAX_SPEED);
    uint8_t settled = fabsf(chassis.heading_error) <= CHASSIS_HEADING_TOL_DEG &&
        (chassis.mode == CHASSIS_MODE_TURN || distance <= CHASSIS_DISTANCE_TOL_MM);
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; ++i)
        if (fabsf(wheel[i]->speed_rpm) > 2.0f) settled = 0U;
    if (settled) s_settle_us += (uint32_t)(dt * 1000000.0f);
    else s_settle_us = 0U;
    if (now - s_move_start_us > CHASSIS_MOVE_TIMEOUT_US) {
        Chassis_Stop();
        chassis.state = CHASSIS_STATE_FAULT;
    } else if (s_settle_us >= CHASSIS_SETTLE_US) {
        Chassis_Stop();
    }
}

HAL_StatusTypeDef Chassis_Init(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == NULL) {
        return HAL_ERROR;
    }
    if (wheel[CHASSIS_WHEEL_LF] != NULL) {
        return HAL_BUSY;   // 单实例，只初始化一次
    }

    Chassis_DwtInit();     // 必须在第一次调用 Chassis_Micros 之前打开 TRCENA

    if (ZDT_Init(hfdcan) != HAL_OK) {
        return HAL_ERROR;
    }

    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; ++i) {
        wheel[i] = ZDT_AddMotor(k_addr[i]);
        if (wheel[i] == NULL) {
            return HAL_ERROR;
        }
    }

    memset(&chassis, 0, sizeof(chassis));
    memset(s_last_rpm, 0, sizeof(s_last_rpm));
    chassis.state = CHASSIS_STATE_IDLE;   // 由 Chassis_Update 负责使能后再进 READY
    chassis.mode  = CHASSIS_MODE_VEL;
    s_force_apply = 0U;
    s_move_req = 0U;
    s_last_us = Chassis_Micros();
    return HAL_OK;
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
    } else if (chassis.cycle > 0U) {
        // 只在正常范围内统计抖动，且跳过首拍：从 Chassis_Init 到任务第一次运行之间
        // 隔着 HAL 初始化 + 启动调度器，那段间隔不是抖动，计进来会永久污染最大值。
        uint32_t dev = (elapsed > CHASSIS_PERIOD_US) ? (elapsed - CHASSIS_PERIOD_US)
                                                     : (CHASSIS_PERIOD_US - elapsed);
        if (dev > chassis.jitter_us) chassis.jitter_us = dev;
    }
    chassis.dt_actual = dt;

    chassis.cycle++;

    /* ---- 1. 先把中断收到的帧解析掉 ---- */
    ZDT_ProcessData();

    if ((chassis.cycle % CHASSIS_FEEDBACK_DIV) == 0U) Chassis_RequestOne();
    Chassis_UpdateOdometry();
    if (chassis.state == CHASSIS_STATE_IDLE) {
        if ((chassis.cycle % CHASSIS_ENABLE_DIV) == 0U) Chassis_EnableAll();
        if (!Chassis_AllEnabled() || !Chassis_FeedbackReady() || !s_odom_valid) return;
        chassis.state = CHASSIS_STATE_READY;
    }
    if (chassis.state == CHASSIS_STATE_READY &&
        (Chassis_HasFault() || !Chassis_AllEnabled() || !Chassis_FeedbackReady())) {
        Chassis_Stop();
        chassis.state = CHASSIS_STATE_FAULT;
    }
    if (chassis.state == CHASSIS_STATE_FAULT) {
        /* 停车帧入队失败时，继续重发零速。 */
        s_force_apply = 1U;
        Chassis_Apply();
        return;
    }
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    uint8_t pending = s_move_req;
    float forward = s_move_forward, left = s_move_left;
    s_move_req = 0U;
    __set_PRIMASK(irq);
    if (pending) (void)Chassis_MoveDistance(forward, left);
    if (chassis.mode != CHASSIS_MODE_VEL) Chassis_AutoControl(dt, Chassis_Micros());

    /* ---- 7. 目标速度斜坡，步长用实测 dt ---- */
    float step_lin = CHASSIS_ACC_LINEAR * dt;
    float step_ang = CHASSIS_ACC_ANGULAR * dt;
    chassis.current.vx = Chassis_Ramp(chassis.current.vx, chassis.target.vx, step_lin);
    chassis.current.vy = Chassis_Ramp(chassis.current.vy, chassis.target.vy, step_lin);
    chassis.current.wz = Chassis_Ramp(chassis.current.wz, chassis.target.wz, step_ang);

    /* ---- 8. 解算 + 限幅 ---- */
    Chassis_Solve(&chassis.current, chassis.wheel_rpm);

    /* ---- 9. 下发 ---- */
    Chassis_Apply();

}

void Chassis_SetVelocity(float vx, float vy, float wz)
{
    if (!isfinite(vx) || !isfinite(vy) || !isfinite(wz) ||
        chassis.state != CHASSIS_STATE_READY || chassis.mode != CHASSIS_MODE_VEL) return;
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
    s_move_req = 0U;                    // 丢掉还没执行的移动请求
    chassis.mode = CHASSIS_MODE_VEL;
    s_force_apply = 1U;
    s_settle_us = 0U;
    Chassis_StopAll();
    if (chassis.state == CHASSIS_STATE_FAULT && Chassis_FeedbackReady() &&
        Chassis_AllEnabled() && !Chassis_HasFault()) chassis.state = CHASSIS_STATE_READY;
}

/* 所有自主运动共用入口：检查反馈、记录目标、重置计时。 */
static HAL_StatusTypeDef Chassis_StartMotion(chassis_mode mode, float heading)
{
    if (chassis.state != CHASSIS_STATE_READY || chassis.mode != CHASSIS_MODE_VEL ||
        !Chassis_FeedbackReady() || !s_odom_valid) return HAL_BUSY;
    chassis.heading_target = heading;
    s_move_start_us = Chassis_Micros();
    s_settle_us = 0U;
    chassis.mode = mode;
    return HAL_OK;
}

HAL_StatusTypeDef Chassis_MoveDistance(float forward_mm, float left_mm)
{
    if (!isfinite(forward_mm) || !isfinite(left_mm) ||
        hypotf(forward_mm, left_mm) > CHASSIS_MOVE_MAX_MM ||
        (forward_mm == 0.0f && left_mm == 0.0f)) return HAL_ERROR;
    HAL_StatusTypeDef status = Chassis_StartMotion(CHASSIS_MODE_MOVE, chassis.yaw);
    if (status != HAL_OK) return status;
    float rad = chassis.yaw * 0.01745329252f;
    s_goal_x = chassis.pos_x + cosf(rad) * forward_mm - sinf(rad) * left_mm;
    s_goal_y = chassis.pos_y + sinf(rad) * forward_mm + cosf(rad) * left_mm;
    return HAL_OK;
}

HAL_StatusTypeDef Chassis_TurnTo(float degrees)
{
    if (!isfinite(degrees) || degrees < -180.0f || degrees > 180.0f) return HAL_ERROR;
    return Chassis_StartMotion(CHASSIS_MODE_TURN, degrees);
}

HAL_StatusTypeDef Chassis_TurnBy(float degrees)
{
    if (!isfinite(degrees) || fabsf(degrees) > 180.0f) return HAL_ERROR;
    /* 暂不归一化目标，保留恰好 +/-180 度时用户指定的方向。 */
    return Chassis_StartMotion(CHASSIS_MODE_TURN, chassis.yaw + degrees);
}

void Chassis_MoveRequest(float forward_mm, float left_mm)
{
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    if (!s_move_req) {
        s_move_forward = forward_mm;
        s_move_left = left_mm;
        s_move_req = 1U;
    }
    __set_PRIMASK(irq);
}
