#include "main.h"
#include "critical.h"
#include "chassis.h"
#include "hwt101.h"
#include "user_math.h"
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

static uint32_t s_last_cyc; // 上一次读到的 CYCCNT
static uint32_t s_cycle_remainder;
static uint32_t s_uptime_us; // 软件累计微秒数，只在这里推进

/* 打开 DWT 周期计数器，给 5 ms 底盘节拍提供高精度时基。 */
static void Chassis_DwtInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 打开跟踪单元时钟
    /* Cortex-M7（STM32H7）的 DWT 带软件锁：不写 LAR 解锁，没接调试器时 CYCCNT 不计数，
       Chassis_Micros 恒为 0，定距/转向超时永远不会触发（接着调试器时又一切正常）。 */
    DWT->LAR = 0xC5ACCE55U;
    DWT->CYCCNT = 0U;                    // 清计数器
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // 开始每个 CPU 周期自增
    s_last_cyc = 0U;
    s_cycle_remainder = 0U;
    s_uptime_us = 0U;
}

/* 返回单调递增的微秒计数；内部推进累加器，只能在底盘任务里调用。 */
uint32_t Chassis_Micros(void)
{
    uint32_t now_cyc = DWT->CYCCNT;
    uint32_t delta_cyc = now_cyc - s_last_cyc; // 周期域减法，回绕自动正确
    s_last_cyc = now_cyc;
    uint64_t cycles = (uint64_t)delta_cyc + s_cycle_remainder;
    s_uptime_us += (uint32_t)(cycles / CHASSIS_CYC_PER_US);
    s_cycle_remainder = (uint32_t)(cycles % CHASSIS_CYC_PER_US);
    return s_uptime_us;
}

/* 电机地址表，顺序必须与 chassis_wheel 一致 */
static const uint8_t k_addr[CHASSIS_WHEEL_NUM] = {ZDT_ADDR_LF, ZDT_ADDR_RF, ZDT_ADDR_LR,
                                                  ZDT_ADDR_RR};

/* 电机安装方向，顺序与 chassis_wheel 一致 */
static const float k_dir[CHASSIS_WHEEL_NUM] = {CHASSIS_WHEEL_DIR_LF, CHASSIS_WHEEL_DIR_RF,
                                               CHASSIS_WHEEL_DIR_LR, CHASSIS_WHEEL_DIR_RR};
#if (CHASSIS_WHEEL_DIR_LF * CHASSIS_WHEEL_DIR_LF != 1) ||                                          \
    (CHASSIS_WHEEL_DIR_RF * CHASSIS_WHEEL_DIR_RF != 1) ||                                          \
    (CHASSIS_WHEEL_DIR_LR * CHASSIS_WHEEL_DIR_LR != 1) ||                                          \
    (CHASSIS_WHEEL_DIR_RR * CHASSIS_WHEEL_DIR_RR != 1)
#error CHASSIS_WHEEL_DIR_xx must be 1 or -1
#endif

/* X 型符号矩阵：行=轮，列=(vx, vy, K·ω)。Mᵀ·M=4I，正解即 Mᵀ/4 */
static const int8_t k_sign[CHASSIS_WHEEL_NUM][3] = {
    {+1, -1, -1}, // LF
    {+1, +1, +1}, // RF
    {+1, +1, -1}, // LR
    {+1, -1, +1}, // RR
};

chassis_t chassis;
chassis_tuning chassis_tune = CHASSIS_TUNE_DEFAULTS;
const chassis_tuning chassis_tune_default = CHASSIS_TUNE_DEFAULTS;
static zdt_motor *wheel[CHASSIS_WHEEL_NUM];
static uint32_t s_last_us;                    // 上一拍的时间戳
static int16_t s_last_rpm[CHASSIS_WHEEL_NUM]; // 上次成功下发的轮速，0.1 RPM
static uint8_t s_force_apply;                 // 1 = 下一拍无条件重发全部轮速
static uint32_t s_move_start_us;
static uint32_t s_move_timeout_us = CHASSIS_MOVE_TIMEOUT_US;
static uint32_t s_settle_us;
static float s_goal_x, s_goal_y;
static float s_move_speed_limit = CHASSIS_MOVE_MAX_SPEED;
static float s_heading_integral;
static uint8_t s_heading_stationary; // 上一拍航向环是按“没在平移”算的
static uint8_t s_heading_hold;
static uint8_t s_recover;     // 1 = S 已请求清故障，正在解除保护/重新使能
static uint32_t s_recover_ms; // 清故障开始时间
chassis_wheel_test chassis_wheel_result;
static uint8_t s_wt_mask, s_wt_index, s_wt_settling;
static uint32_t s_wt_start_ms;
static float s_wt_start_deg;

static void Chassis_Finish(void);

/* 车体坐标系下的轮子位置/转速：驱动器读数乘安装方向。 */
static float Chassis_WheelDeg(unsigned i)
{
    return k_dir[i] * wheel[i]->position_deg;
}
static float Chassis_WheelRpm(unsigned i)
{
    return k_dir[i] * wheel[i]->speed_rpm;
}

const zdt_motor *Chassis_Wheel(unsigned i)
{
    return i < CHASSIS_WHEEL_NUM ? wheel[i] : NULL;
}

float Chassis_SpeedLimit(void)
{
    return s_move_speed_limit;
}

/* 每个轮子三类反馈（位置、转速、状态）发出去几条读取、几条没能入队。 */
static uint32_t s_fb_sent[CHASSIS_WHEEL_NUM][3], s_fb_fail[CHASSIS_WHEEL_NUM][3];
ChassisFeedbackFault chassis_feedback_fault;

void Chassis_FeedbackDiag(unsigned i, ChassisFeedbackDiag *out)
{
    if (out == NULL || i >= CHASSIS_WHEEL_NUM)
        return;
    memset(out, 0, sizeof(*out));
    memcpy(out->sent, s_fb_sent[i], sizeof(out->sent));
    memcpy(out->fail, s_fb_fail[i], sizeof(out->fail));
    const zdt_motor *m = wheel[i];
    if (m == NULL)
        return;
    const uint32_t got[3] = {m->position_seq, m->speed_seq, m->status_seq};
    const uint32_t at[3] = {m->position_ms, m->speed_ms, m->status_ms};
    const uint8_t valid[3] = {m->position_seq != 0U, m->speed_valid, m->status_valid};
    uint32_t now = HAL_GetTick();
    for (unsigned k = 0; k < 3U; ++k)
    {
        out->got[k] = got[k];
        out->valid[k] = valid[k];
        out->age_ms[k] = now - at[k];
    }
}

static void Chassis_Fault(chassis_fault reason, uint16_t detail)
{
    if (chassis.state != CHASSIS_STATE_FAULT)
    {
        chassis_feedback_fault.valid = 0U;
        chassis_feedback_fault.at_ms = HAL_GetTick();
        chassis_feedback_fault.bus_off = ZDT_BusOffCount();
        chassis_feedback_fault.bus_errors = ZDT_BusErrors();
        chassis_feedback_fault.rx_lost = ZDT_RxLost();
        chassis_feedback_fault.rx_dropped = ZDT_RxDropped();
        chassis_feedback_fault.tx_free = ZDT_TxFreeLevel();
        chassis_feedback_fault.reason = (uint8_t)reason;
        chassis_feedback_fault.detail = detail;
        for (unsigned i = 0; i < CHASSIS_WHEEL_NUM; ++i)
            Chassis_FeedbackDiag(i, &chassis_feedback_fault.wheel[i]);
        chassis_feedback_fault.valid = 1U;
    }
    Chassis_Stop();
    chassis.state = CHASSIS_STATE_FAULT;
    chassis.fault = (uint8_t)reason;
    chassis.fault_detail = detail;
    s_recover = 0U;
}

/* 设置自主移动的速度上限：非有限值忽略，其余钳位到 10~CHASSIS_MOVE_MAX_SPEED。
   只在 VEL 模式接受新值，路径/定距执行中保持本次限速，避免和已排好的限速表打架。 */
void Chassis_SetMoveSpeedLimit(float mm_per_second)
{
    if (chassis.mode != CHASSIS_MODE_VEL || !isfinite(mm_per_second))
        return;
    if (mm_per_second < 10.0f)
        mm_per_second = 10.0f;
    if (mm_per_second > CHASSIS_MOVE_MAX_SPEED)
        mm_per_second = CHASSIS_MOVE_MAX_SPEED;
    s_move_speed_limit = mm_per_second;
}

static float s_prev_deg[CHASSIS_WHEEL_NUM];
static uint32_t s_prev_seq[CHASSIS_WHEEL_NUM];
static uint32_t s_odom_sample_ms[CHASSIS_WHEEL_NUM];
static uint8_t s_odom_valid;

/* 反馈新鲜度检查：返回缺失项位图（CHASSIS_WAIT_IMU / CHASSIS_WAIT_FB(i)），0 = 全部新鲜。 */
static uint16_t Chassis_FeedbackMissing(void)
{
    uint16_t missing = 0U;
    uint32_t now = HAL_GetTick();
#if CHASSIS_USE_IMU
    critical_state irq = Critical_Enter();
    imu_data sample = imu;
    Critical_Exit(irq);
    if (!sample.valid || !isfinite(sample.yaw) ||
        (now - sample.updated_ms > CHASSIS_FEEDBACK_TIMEOUT_MS))
        missing |= CHASSIS_WAIT_IMU;
#endif
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; ++i)
    {
        if (!wheel[i] || !wheel[i]->position_seq || !wheel[i]->status_valid ||
            !wheel[i]->speed_valid || now - wheel[i]->speed_ms > CHASSIS_FEEDBACK_TIMEOUT_MS ||
            now - wheel[i]->position_ms > CHASSIS_FEEDBACK_TIMEOUT_MS ||
            now - wheel[i]->status_ms > CHASSIS_FEEDBACK_TIMEOUT_MS)
            missing |= (uint16_t)CHASSIS_WAIT_FB(i);
    }
    return missing;
}

/* 判断陀螺仪与四轮反馈是否新鲜：全部有效且未超时才返回 1。 */
static uint8_t Chassis_FeedbackReady(void)
{
    return Chassis_FeedbackMissing() == 0U;
}

/* 每周期最多变化 step，把当前值朝目标值做斜坡逼近。 */
static float Chassis_Ramp(float cur, float tgt, float step)
{
    float diff = tgt - cur;
    if (diff > step)
        return cur + step;
    if (diff < -step)
        return cur - step;
    return tgt;
}

/* 逆运动学：车体速度 -> 四轮转速（0.1 RPM），并做整体等比限幅。 */
static void Chassis_Solve(const chassis_vel *v, int16_t out[CHASSIS_WHEEL_NUM])
{
    /* 轮子转一圈移动 πD，mm/s -> 0.1 RPM */
    const float rpm_x10_per_mmps = 600.0f * CHASSIS_GEAR_RATIO / (MATH_PI * chassis_tune.wheel_dia);
    const float w_mm = chassis_tune.chassis_k * v->wz * MATH_DEG_TO_RAD;
    float raw[CHASSIS_WHEEL_NUM];
    float peak = 0.0f;

    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; ++i)
    {
        float v_wheel =
            (float)k_sign[i][0] * v->vx + (float)k_sign[i][1] * v->vy + (float)k_sign[i][2] * w_mm;
        raw[i] = v_wheel * rpm_x10_per_mmps;
        if (fabsf(raw[i]) > peak)
            peak = fabsf(raw[i]);
    }

    /* 超限时四轮同乘一个系数，合速度方向不变；逐个 clamp 会让车走歪。 */
    const float limit = (float)CHASSIS_MAX_WHEEL_RPM * 10.0f;
    float scale = peak > limit ? limit / peak : 1.0f;
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; ++i)
        out[i] = (int16_t)lroundf(raw[i] * scale);
}

/* 四轮速度带同步标志入队，再广播一次触发，使四轮同时变速。
   只在轮速变化或到强制重发周期时发送；任一轮入队失败就不触发，下拍整组重发。 */
static void Chassis_Apply(void)
{
    uint8_t changed = s_force_apply || ((chassis.cycle % CHASSIS_RESEND_DIV) == 0U);
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM && !changed; ++i)
        changed = chassis.wheel_rpm_x10[i] != s_last_rpm[i];
    if (!changed)
        return;

    s_force_apply = 1U;
    if (ZDT_TxFreeLevel() < CHASSIS_WHEEL_NUM + 1U)
        return;
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; ++i)
        if (ZDT_SetSpeed(
                wheel[i],
                (int16_t)(k_dir[i] < 0.0f ? -chassis.wheel_rpm_x10[i] : chassis.wheel_rpm_x10[i]),
                CHASSIS_DRV_ACC, ZDT_SYNC_CACHE) != HAL_OK)
            return;
    if (ZDT_SyncTrigger() != HAL_OK)
        return;
    memcpy(s_last_rpm, chassis.wheel_rpm_x10, sizeof(s_last_rpm));
    s_force_apply = 0U;
}

/* 检查四轮是否有堵转或驱动器保护标志。 */
static uint8_t Chassis_HasFault(void)
{
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; ++i)
    {
        if (wheel[i] == NULL)
            continue;
        if (wheel[i]->stalled || wheel[i]->protected_flag)
            return 1U;
    }
    return 0U;
}

/* 未就绪原因：反馈缺失 + 未使能 + 堵转 + 里程计未初始化。 */
uint16_t Chassis_WaitReason(void)
{
    uint16_t reason = Chassis_FeedbackMissing();
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; ++i)
    {
        if (wheel[i] == NULL || !wheel[i]->enabled)
            reason |= (uint16_t)CHASSIS_WAIT_EN(i);
        if (wheel[i] != NULL && (wheel[i]->stalled || wheel[i]->protected_flag))
            reason |= (uint16_t)CHASSIS_WAIT_STALL(i);
    }
    if (!s_odom_valid)
        reason |= CHASSIS_WAIT_ODOM;
    return reason;
}

/* 四轮是否都已确认使能。 */
static uint8_t Chassis_AllEnabled(void)
{
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; ++i)
    {
        if ((wheel[i] == NULL) || (wheel[i]->enabled == 0U))
            return 0U;
    }
    return 1U;
}

/* 给四轮各发一次使能命令，未使能时周期重发。 */
static void Chassis_EnableAll(void)
{
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; ++i)
    {
        if (wheel[i] != NULL)
            (void)ZDT_EnableMotor(wheel[i], 1U);
    }
}

/* 给四轮各发一次立即停止命令。 */
static void Chassis_StopAll(void)
{
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; ++i)
    {
        if (wheel[i] != NULL)
            (void)ZDT_Stop(wheel[i], ZDT_SYNC_NOW);
    }
}

/* 每 CHASSIS_FEEDBACK_DIV 拍里位置、转速、状态各占一拍，每拍给四个轮子各发一条同样的读取，四轮位置同一拍读，
   里程计用的是同一时刻的数据。拍末排在这一拍所有命令（速度、停车、使能、解除保护，串口命令和流程发的也在内）
   后面：一台驱动器一拍最多收两帧，这一拍已经有两条命令的（停车加零速、解除保护加零速）就不读，下一轮再读。
   一拍收三帧，它对第一帧的应答正好撞上第三帧（ZDT_Flush）；原来一拍给一台驱动器连读转速和状态再加速度命令，
   左前、右前的状态 6 秒没回过，走完一段又加一条急停时右后轮丢应答。 */
#if CHASSIS_FEEDBACK_DIV < 3
#error CHASSIS_FEEDBACK_DIV must leave one tick for each of position, speed and status
#endif
static void Chassis_RequestFeedback(void)
{
    static HAL_StatusTypeDef (*const request[3])(zdt_motor *) = {
        ZDT_RequestPosition, ZDT_RequestSpeed, ZDT_RequestStatus};
    uint32_t kind = chassis.cycle % CHASSIS_FEEDBACK_DIV;
    if (kind >= 3U)
        return;
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; ++i)
    {
        if (ZDT_Staged(wheel[i]) >= 2U)
            continue;
        if (request[kind](wheel[i]) == HAL_OK)
            ++s_fb_sent[i][kind];
        else
            ++s_fb_fail[i][kind];
    }
}

/* 独立采集新鲜 IMU 航向，轮子反馈失效仍由上层故障保护处理。 */
static void Chassis_UpdateYaw(void)
{
#if CHASSIS_USE_IMU
    critical_state irq = Critical_Enter();
    imu_data sample = imu;
    Critical_Exit(irq);
#else
    int sample = 0;
#endif
#if CHASSIS_USE_IMU
    if (sample.valid && isfinite(sample.yaw) &&
        HAL_GetTick() - sample.updated_ms <= CHASSIS_FEEDBACK_TIMEOUT_MS)
        chassis.yaw = Math_WrapDeg(CHASSIS_YAW_SIGN * sample.yaw);
#else
    (void)sample; // 不接陀螺仪：航向在 Chassis_UpdateOdometry 里由轮子推算
#endif
    chassis.heading_error = Math_WrapDeg(chassis.heading_target - chassis.yaw);
}

/* 四轮位置均更新后才累计位移，航向单独跟随新鲜 IMU。 */
static void Chassis_UpdateOdometry(void)
{
    if (!Chassis_FeedbackReady())
        return;
    /* 四轮位置均更新后才计算增量，不对旧速度反复积分。 */
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; ++i)
    {
        chassis.wheel_rpm_meas[i] = Chassis_WheelRpm(i);
        if (!wheel[i]->position_seq || wheel[i]->position_seq == s_prev_seq[i])
            return;
    }
    float d[CHASSIS_WHEEL_NUM];
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; ++i)
    {
        float deg = Chassis_WheelDeg(i);
        d[i] = (deg - s_prev_deg[i]) *
               (MATH_PI * chassis_tune.wheel_dia / (360.0f * CHASSIS_GEAR_RATIO));
        s_prev_deg[i] = deg;
        s_prev_seq[i] = wheel[i]->position_seq;
        s_odom_sample_ms[i] = wheel[i]->position_ms;
    }
    if (!s_odom_valid)
    {
        s_odom_valid = 1U;
        return;
    }
    float dx = 0.25f * (d[0] + d[1] + d[2] + d[3]);
    float dy = 0.25f * (-d[0] + d[1] + d[2] - d[3]);
#if CHASSIS_USE_IMU
    float rad = chassis.yaw * MATH_DEG_TO_RAD;
#else
    /* 无陀螺仪：dθ = Σ(k_sign[i][2]·d[i]) / (4K)，位移按中点航向旋转到世界系。 */
    float dtheta = 0.25f * (-d[0] + d[1] - d[2] + d[3]) / chassis_tune.chassis_k; // rad
    float rad = chassis.yaw * MATH_DEG_TO_RAD + 0.5f * dtheta;
    chassis.yaw = Math_WrapDeg(chassis.yaw + dtheta * MATH_RAD_TO_DEG);
#endif
    chassis.pos_x += cosf(rad) * dx - sinf(rad) * dy;
    chassis.pos_y += sinf(rad) * dx + cosf(rad) * dy;
}

/* 小误差 PI 修正并限制积分；进入静态容差后撤去修正以便停稳。
   heading_brake > 0 时比例项不超过刹车曲线（见 chassis.h），积分不受它限制。 */
static float Chassis_HeadingControl(float dt, uint8_t stationary)
{
    float error = chassis.heading_error;
    /* 平移刚停下的那一拍清积分：积分里攒的是顶平移扰动的量，车一停扰动就没了，留着它和比例项
       对着干，车头要好几秒才回得来（1.5 m 横移后实测仿真 4 s）。 */
    if (stationary && !s_heading_stationary)
        s_heading_integral = 0.0f;
    s_heading_stationary = stationary;
    if (stationary && fabsf(error) <= chassis_tune.heading_tol)
    {
        s_heading_integral = 0.0f;
        return 0.0f;
    }
    /* KI 调成 0 时积分一起清掉，旧的修正量不留在输出里。 */
    if (fabsf(error) < chassis_tune.heading_i_zone && chassis_tune.heading_ki > 0.0f)
        s_heading_integral = Math_Clamp(s_heading_integral + chassis_tune.heading_ki * error * dt,
                                        chassis_tune.heading_i_max);
    else
        s_heading_integral = 0.0f;
    float p = chassis_tune.heading_kp * error;
    if (chassis_tune.heading_brake > 0.0f)
        p = Math_Clamp(p, sqrtf(2.0f * chassis_tune.heading_brake * fabsf(error)));
    return Math_Clamp(p + s_heading_integral, chassis_tune.turn_max_speed);
}

#include "chassis_path.inc"

/* 定距/转向闭环：距离 P 环和航向 PI 环，持续停稳后完成，超时判故障。 */
static void Chassis_AutoControl(float dt, uint32_t now)
{
    float ex = s_goal_x - chassis.pos_x;
    float ey = s_goal_y - chassis.pos_y;
    float distance = hypotf(ex, ey);
    chassis.heading_error = Math_WrapDeg(chassis.heading_target - chassis.yaw);
    float vx = 0.0f, vy = 0.0f;
    if (chassis.mode == CHASSIS_MODE_MOVE && distance > chassis_tune.distance_tol)
    {
        float speed = fminf(chassis_tune.distance_kp * distance, s_move_speed_limit);
        float rad = chassis.yaw * MATH_DEG_TO_RAD;
        vx = speed * (cosf(rad) * ex + sinf(rad) * ey) / distance;
        vy = speed * (-sinf(rad) * ex + cosf(rad) * ey) / distance;
    }
    chassis.target.vx = vx;
    chassis.target.vy = vy;
    uint8_t stationary = chassis.mode == CHASSIS_MODE_TURN || distance <= chassis_tune.distance_tol;
    chassis.target.wz = Chassis_HeadingControl(dt, stationary);
    if (!stationary)
    {
        float error = fabsf(chassis.heading_error);
        if (error > chassis.heading_peak_dynamic)
            chassis.heading_peak_dynamic = error;
        if (error > CHASSIS_DYNAMIC_TOL_DEG)
            ++chassis.heading_over_limit;
    }
    uint8_t settled = fabsf(chassis.heading_error) <= chassis_tune.heading_tol &&
                      (chassis.mode == CHASSIS_MODE_TURN || distance <= chassis_tune.distance_tol);
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; ++i)
        if (fabsf(Chassis_WheelRpm(i)) > 2.0f)
            settled = 0U;
    if (settled)
        s_settle_us += (uint32_t)(dt * 1000000.0f);
    else
        s_settle_us = 0U;
    if (now - s_move_start_us > s_move_timeout_us)
        Chassis_Fault(CHASSIS_FAULT_MOVE_TIMEOUT, 0U);
    else if (s_settle_us >= (uint32_t)(chassis_tune.settle_ms * 1000.0f))
    {
        Chassis_Finish();
        s_heading_hold = 1U; // 正常完成后保持原目标；显式 S/故障停止仍完全撤去控制
    }
}

/* 单轮方向测试：逐个轮子按“前进方向”转固定时间，停稳后读实际转角并判定方向。 */
static void Chassis_WheelControl(void)
{
    uint32_t now = HAL_GetTick();
    memset(&chassis.target, 0, sizeof(chassis.target));
    memset(&chassis.current, 0, sizeof(chassis.current));
    memset(chassis.wheel_rpm_x10, 0, sizeof(chassis.wheel_rpm_x10));
    unsigned i = s_wt_index;
    if (!s_wt_settling)
    {
        if (now - s_wt_start_ms < CHASSIS_WHEEL_TEST_MS)
        {
            chassis.wheel_rpm_x10[i] = (int16_t)lroundf(CHASSIS_WHEEL_TEST_RPM * 10.0f);
            if (now - s_wt_start_ms > CHASSIS_WHEEL_TEST_MS / 2U)
                chassis_wheel_result.meas_rpm[i] = Chassis_WheelRpm(i);
            return;
        }
        s_wt_settling = 1U;
        s_wt_start_ms = now;
        return;
    }
    if (now - s_wt_start_ms < CHASSIS_WHEEL_SETTLE_MS)
        return;
    float expect = CHASSIS_WHEEL_TEST_RPM * 6.0f * (float)CHASSIS_WHEEL_TEST_MS * 0.001f;
    float delta = Chassis_WheelDeg(i) - s_wt_start_deg;
    float ratio = delta / expect;
    chassis_wheel_result.delta_deg[i] = delta;
    chassis_wheel_result.verdict[i] = fabsf(ratio) < 0.2f              ? CHASSIS_WT_NONE
                                      : ratio < -0.5f                  ? CHASSIS_WT_REVERSED
                                      : (ratio > 0.5f && ratio < 1.5f) ? CHASSIS_WT_OK
                                                                       : CHASSIS_WT_ODD;
    /* 下一个要测的轮子 */
    while (++s_wt_index < CHASSIS_WHEEL_NUM && !(s_wt_mask & (1U << s_wt_index)))
    {
    }
    if (s_wt_index >= CHASSIS_WHEEL_NUM)
    {
        chassis_wheel_result.active = 0U;
        ++chassis_wheel_result.seq;
        Chassis_Finish();
        return;
    }
    s_wt_settling = 0U;
    s_wt_start_ms = now;
    s_wt_start_deg = Chassis_WheelDeg(s_wt_index);
    chassis_wheel_result.active = (uint8_t)(s_wt_index + 1U);
}

HAL_StatusTypeDef Chassis_WheelTest(uint8_t mask)
{
    mask &= 0x0FU;
    if (!mask)
        return HAL_ERROR;
    if (chassis.state != CHASSIS_STATE_READY || chassis.mode != CHASSIS_MODE_VEL ||
        !Chassis_FeedbackReady())
        return HAL_BUSY;
    uint8_t seq = chassis_wheel_result.seq;
    memset(&chassis_wheel_result, 0, sizeof(chassis_wheel_result));
    chassis_wheel_result.seq = seq;
    chassis_wheel_result.mask = mask;
    s_wt_mask = mask;
    s_wt_index = 0U;
    while (!(mask & (1U << s_wt_index)))
        ++s_wt_index;
    s_wt_settling = 0U;
    s_wt_start_ms = HAL_GetTick();
    s_wt_start_deg = Chassis_WheelDeg(s_wt_index);
    s_heading_hold = 0U;
    chassis_wheel_result.active = (uint8_t)(s_wt_index + 1U);
    chassis.mode = CHASSIS_MODE_WHEEL;
    return HAL_OK;
}

/* S 清故障：解除堵转保护、重新使能，全部恢复后回 READY；超时放弃（仍保持 FAULT）。 */
static void Chassis_Recover(void)
{
    if (Chassis_FeedbackReady() && Chassis_AllEnabled() && !Chassis_HasFault())
    {
        chassis.state = CHASSIS_STATE_READY;
        chassis.fault = CHASSIS_FAULT_NONE;
        chassis.fault_detail = 0U;
        s_recover = 0U;
        return;
    }
    if (HAL_GetTick() - s_recover_ms > CHASSIS_RECOVER_MS)
    {
        s_recover = 0U;
        chassis.fault_detail = Chassis_WaitReason();
        return;
    }
    if ((chassis.cycle % CHASSIS_ENABLE_DIV) != 0U)
        return;
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; ++i)
    {
        if (wheel[i]->stalled || wheel[i]->protected_flag)
            (void)ZDT_ClearProtection(wheel[i]);
        else if (!wheel[i]->enabled)
            (void)ZDT_EnableMotor(wheel[i], 1U);
    }
}

/* 初始化底盘：DWT 时基、CAN 总线、四轴注册、状态复位。 */
HAL_StatusTypeDef Chassis_Init(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == NULL)
    {
        return HAL_ERROR;
    }
    if (wheel[CHASSIS_WHEEL_LF] != NULL)
    {
        return HAL_BUSY; // 单实例，只初始化一次
    }

    Chassis_DwtInit(); // 必须在第一次调用 Chassis_Micros 之前打开 TRCENA

    if (ZDT_Init(hfdcan) != HAL_OK)
    {
        return HAL_ERROR;
    }

    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; ++i)
    {
        wheel[i] = ZDT_AddMotor(k_addr[i]);
        if (wheel[i] == NULL)
        {
            return HAL_ERROR;
        }
    }

    memset(&chassis, 0, sizeof(chassis));
    memset(s_last_rpm, 0, sizeof(s_last_rpm));
    chassis.state = CHASSIS_STATE_IDLE; // 由 Chassis_Update 负责使能后再进 READY
    chassis.mode = CHASSIS_MODE_VEL;
    s_force_apply = 0U;
    s_last_us = Chassis_Micros();
    return HAL_OK;
}

/* 底盘 5 ms 主循环：量时基、解析反馈、跑状态机、斜坡、逆解、下发轮速。 */
void Chassis_Update(void)
{
    /* ---- 0. DWT 量真实步长 ---- */
    uint32_t now_us = Chassis_Micros();
    uint32_t elapsed = now_us - s_last_us; // 无符号减法，跨 CYCCNT 回绕也成立
    s_last_us = now_us;

    float dt = (float)elapsed * 0.000001f;
    if ((elapsed < 500U) || (elapsed > 50000U))
    {
        dt = CHASSIS_PERIOD_S; // 首拍、调试暂停、异常调度都走这里
    }
    else if (chassis.cycle > 0U)
    {
        // 只在正常范围内统计抖动，且跳过首拍：从 Chassis_Init 到任务第一次运行之间
        // 隔着 HAL 初始化 + 启动调度器，那段间隔不是抖动，计进来会永久污染最大值。
        uint32_t dev = (elapsed > CHASSIS_PERIOD_US) ? (elapsed - CHASSIS_PERIOD_US)
                                                     : (CHASSIS_PERIOD_US - elapsed);
        if (dev > chassis.jitter_us)
            chassis.jitter_us = dev;
        if (chassis.mode == CHASSIS_MODE_PATH && dev > chassis_path_result.max_jitter_us)
            chassis_path_result.max_jitter_us = dev;
    }
    chassis.dt_actual = dt;

    chassis.cycle++;

    /* ---- 1. 先把中断收到的帧解析掉 ---- */
    ZDT_Service(); // CAN Bus-Off 自动恢复
    ZDT_ProcessData();

    Chassis_UpdateYaw();
    Chassis_UpdateOdometry();
    chassis.heading_error = Math_WrapDeg(chassis.heading_target - chassis.yaw);
    if (chassis.state == CHASSIS_STATE_IDLE)
    {
        if ((chassis.cycle % CHASSIS_ENABLE_DIV) == 0U)
            Chassis_EnableAll();
        if (!Chassis_AllEnabled() || !Chassis_FeedbackReady() || !s_odom_valid)
            return;
        chassis.state = CHASSIS_STATE_READY;
    }
    if (chassis.state == CHASSIS_STATE_READY &&
        (Chassis_HasFault() || !Chassis_AllEnabled() || !Chassis_FeedbackReady()))
        Chassis_Fault(CHASSIS_FAULT_SENSOR, (uint16_t)(Chassis_WaitReason() & ~CHASSIS_WAIT_ODOM));
    if (chassis.state == CHASSIS_STATE_FAULT)
    {
        if (s_recover)
            Chassis_Recover();
        /* 进故障时 Chassis_Stop 已经要求发零速，入队失败的下拍重发，发出去以后和静止时一样 100 ms
           重发一次：每拍都发会一直占着总线，读反馈更难回来。 */
        Chassis_Apply();
        return;
    }
    if (chassis.mode == CHASSIS_MODE_WHEEL)
    {
        Chassis_WheelControl(); // 直接给单轮转速，不经过斜坡和运动学
        Chassis_Apply();
        return;
    }
    if (chassis.mode == CHASSIS_MODE_PATH)
        Chassis_PathControl(dt);
    else if (chassis.mode != CHASSIS_MODE_VEL)
        Chassis_AutoControl(dt, Chassis_Micros());
    else if (s_heading_hold)
    {
        uint8_t stationary = chassis.target.vx == 0.0f && chassis.target.vy == 0.0f;
        chassis.target.wz = Chassis_HeadingControl(dt, stationary);
    }

    /* ---- 7. 目标速度斜坡，步长用实测 dt ---- */
    float step_lin = chassis_tune.acc_linear * dt;
    float step_ang = chassis_tune.acc_angular * dt;
    chassis.current.vx = Chassis_Ramp(chassis.current.vx, chassis.target.vx, step_lin);
    chassis.current.vy = Chassis_Ramp(chassis.current.vy, chassis.target.vy, step_lin);
    chassis.current.wz = Chassis_Ramp(chassis.current.wz, chassis.target.wz, step_ang);

    /* ---- 8. 平移前馈 + 解算 + 限幅 ---- */
    chassis_vel out = chassis.current;
    out.wz += (chassis_tune.heading_ff_x * out.vx + chassis_tune.heading_ff_y * out.vy) * 0.001f;
    Chassis_Solve(&out, chassis.wheel_rpm_x10);

    /* ---- 9. 下发 ---- */
    Chassis_Apply();
}

void Chassis_CycleEnd(void)
{
    Chassis_RequestFeedback(); // 这一拍的命令都排好了（串口命令、流程发的停车也在内）再排读取
    ZDT_Flush();
    uint32_t busy = Chassis_Micros() - s_last_us;
    if (busy > chassis.loop_peak_us)
        chassis.loop_peak_us = busy;
    if (chassis.mode == CHASSIS_MODE_PATH && busy > chassis_path_result.max_loop_us)
        chassis_path_result.max_loop_us = busy;
}

/* 速度模式入口：限幅后写入目标速度；非 READY 或非速度模式时忽略。 */
void Chassis_SetVelocity(float vx, float vy, float wz)
{
    if (!isfinite(vx) || !isfinite(vy) || !isfinite(wz) || chassis.state != CHASSIS_STATE_READY ||
        chassis.mode != CHASSIS_MODE_VEL)
        return;
    chassis.target.vx = Math_Clamp(vx, CHASSIS_MAX_LINEAR);
    chassis.target.vy = Math_Clamp(vy, CHASSIS_MAX_LINEAR);
    chassis.target.wz = Math_Clamp(wz, CHASSIS_MAX_ANGULAR);
    if (wz != 0.0f)
    {
        s_heading_hold = 0U;
        s_heading_integral = 0.0f;
    }
    else if (!s_heading_hold)
    {
        chassis.heading_target = chassis.yaw;
        s_heading_hold = 1U;
        s_heading_integral = 0.0f;
    }
}

/* 停下来：清目标和斜坡、丢弃待执行请求，下发零速。自己做完的（定距、转向、路径、单轮测试）只用这个：
   轮子已经停稳，不用再发急停帧，那一拍照常读反馈（有急停帧的那一拍不读，见 Chassis_RequestFeedback）。 */
static void Chassis_Finish(void)
{
    s_path_count = 0U;
    s_path_index = 0U;
    memset(&chassis.target, 0, sizeof(chassis.target));
    memset(&chassis.current, 0, sizeof(chassis.current));
    memset(chassis.wheel_rpm_x10, 0, sizeof(chassis.wheel_rpm_x10));
    memset(s_last_rpm, 0, sizeof(s_last_rpm));
    chassis.mode = CHASSIS_MODE_VEL;
    s_force_apply = 1U;
    s_settle_us = 0U;
    s_heading_hold = 0U;
    s_heading_integral = 0.0f;
    chassis_wheel_result.active = 0U; // 中途停车：本次单轮测试作废，不出结果
}

/* 立即停车：Chassis_Finish 之外四轮再各发一条急停；故障解除后自动退回 READY。 */
void Chassis_Stop(void)
{
    Chassis_Finish();
    Chassis_StopAll();
    if (chassis.state != CHASSIS_STATE_FAULT)
        return;
    if (Chassis_FeedbackReady() && Chassis_AllEnabled() && !Chassis_HasFault())
    {
        chassis.state = CHASSIS_STATE_READY;
        chassis.fault = CHASSIS_FAULT_NONE;
        chassis.fault_detail = 0U;
        s_recover = 0U;
    }
    else if (!s_recover)
    {
        /* 堵转保护/掉使能不会自己消失：由主循环发解除保护和使能命令，恢复后自动回 READY。 */
        s_recover = 1U;
        s_recover_ms = HAL_GetTick();
    }
}

/* 所有自主运动共用入口：检查反馈、记录目标、重置计时。 */
static HAL_StatusTypeDef Chassis_StartMotion(chassis_mode mode, float heading)
{
    if (chassis.state != CHASSIS_STATE_READY || chassis.mode != CHASSIS_MODE_VEL ||
        !Chassis_FeedbackReady() || !s_odom_valid)
        return HAL_BUSY;
    chassis.heading_target = heading;
    s_heading_integral = 0.0f;
    s_heading_hold = 0U;
    chassis.heading_peak_dynamic = 0.0f;
    chassis.heading_over_limit = 0U;
    s_move_start_us = Chassis_Micros();
    s_move_timeout_us = CHASSIS_MOVE_TIMEOUT_US;
    s_settle_us = 0U;
    chassis.mode = mode;
    return HAL_OK;
}

/* 定距移动：把车体系位移换算成世界系目标点，移动中保持 heading_deg。 */
HAL_StatusTypeDef Chassis_MoveDistanceHold(float forward_mm, float left_mm, float heading_deg)
{
    if (!isfinite(forward_mm) || !isfinite(left_mm) || !isfinite(heading_deg) ||
        hypotf(forward_mm, left_mm) > CHASSIS_MOVE_MAX_MM ||
        (forward_mm == 0.0f && left_mm == 0.0f))
        return HAL_ERROR;
    HAL_StatusTypeDef status = Chassis_StartMotion(CHASSIS_MODE_MOVE, heading_deg);
    if (status != HAL_OK)
        return status;
    float rad = chassis.yaw * MATH_DEG_TO_RAD;
    s_goal_x = chassis.pos_x + cosf(rad) * forward_mm - sinf(rad) * left_mm;
    s_goal_y = chassis.pos_y + sinf(rad) * forward_mm + cosf(rad) * left_mm;
    /* 按本次实际限速估算，另留减速/航向收敛余量；最短仍保留 30 秒。 */
    float seconds = 2.0f * hypotf(forward_mm, left_mm) / s_move_speed_limit + 10.0f;
    s_move_timeout_us = (uint32_t)fmaxf((float)CHASSIS_MOVE_TIMEOUT_US, seconds * 1000000.0f);
    return HAL_OK;
}

/* 定距移动并锁定当前航向。 */
HAL_StatusTypeDef Chassis_MoveDistance(float forward_mm, float left_mm)
{
    return Chassis_MoveDistanceHold(forward_mm, left_mm, chassis.yaw);
}

/* 转向到绝对航向角，输入范围 [-180, 180] 度。 */
HAL_StatusTypeDef Chassis_TurnTo(float degrees)
{
    if (!isfinite(degrees) || degrees < -180.0f || degrees > 180.0f)
        return HAL_ERROR;
    return Chassis_StartMotion(CHASSIS_MODE_TURN, degrees);
}

/* 在当前航向上相对转动指定角度。 */
HAL_StatusTypeDef Chassis_TurnBy(float degrees)
{
    if (!isfinite(degrees) || fabsf(degrees) > 180.0f)
        return HAL_ERROR;
    /* 暂不归一化目标，保留恰好 +/-180 度时用户指定的方向。 */
    return Chassis_StartMotion(CHASSIS_MODE_TURN, chassis.yaw + degrees);
}

