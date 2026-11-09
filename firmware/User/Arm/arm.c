#include "arm.h"
#include "user_math.h"
#include <math.h>
#include <string.h>

#define ARM_DT             0.005f // 底盘任务节拍
#define ARM_RESEND_DIV     20U    // 停着时每 20 拍（100 ms）重发一次位置，兜底丢帧
#define ARM_RETRY_DIV      20U    // 使能、松开、清故障每 100 ms 重发
#define ARM_SPEED_MARGIN   1.25f  // 下发的限速比规划速度高一点，电机赶得上下一个点
#define ARM_MIN_RPM_X10    10U    // 限速至少 1 RPM，最后一点误差也能走完
#define ARM_STOP_SETTLE_MS 30U    // 急停后等这么久再读位置对齐：电机停稳了
#define ARM_HISTORY        32U    // 关节角历史（20 ms 一个，0.64 s），查拍照那一刻臂在哪
#define ARM_DEADBAND_MM    0.5f   // 跟随时目标挪动不到这么多就不重新规划
#define ARM_PATH_STEP      1.0f   // 查路径时关节空间每 1° 查一次离死点多远
#define ARM_SAME_MM        0.5f   // 逆解的四种组合，正解回去离目标这么近才算同一种装配方式
#define ARM_REACH_STEPS    12U    // 物料够不着：沿着朝它的方向二分找能走到的最远处

/* 挂着等发的命令。一台驱动器一拍最多收两帧（见 ZDT_Flush），命令一拍只发一条。 */
enum
{
    CMD_STOP = 0x01U,
    CMD_CLEAR = 0x02U,
    CMD_DISABLE = 0x04U,
    CMD_ENABLE = 0x08U,
};

arm_tuning arm_tune = ARM_TUNE_DEFAULTS;
const arm_tuning arm_tune_default = ARM_TUNE_DEFAULTS;
arm_status arm;

static zdt_motor *joint[2];
static uint32_t cycle;
static float base[2];          // 关节角为 0 时的电机读数（多圈）
static float vel[2];           // 规划速度，°/s
static float vmax[2], amax[2]; // 这次运动分给各关节的限速和加速度：按距离分，两个关节同时到
static uint8_t pending[2];     // 每台电机挂着的 CMD_*
static uint8_t dirty;          // 规划位置变了还没发出去
static uint8_t move_pending;   // 有一次 Arm_MoveJoints 还没走完，走完 arm.moves 加 1
static uint8_t want_free, recovering;
static uint32_t recover_ms;
static uint8_t need_sync; // 1 = 规划等新的位置读数对齐，期间不下发位置
static uint32_t sync_seq[2], sync_ms;
static uint32_t used_seq[2];
static float calib[4]; // 上一拍的零点和方向，变了就重新对齐
static struct
{
    uint32_t ms;
    float q[2];
} history[ARM_HISTORY];
static unsigned history_head, history_count;
static arm_point points[ARM_POINTS];
static unsigned point_count;
static uint32_t seen_ms;

/* arm_calib.h 的点位表，最后一项是名字为空的结束标记。 */
#define ARM_POINT(name, q1, q2) {#name, {q1, q2}},
static const arm_point default_points[] = {ARM_POINT_TABLE{"", {0.0f, 0.0f}}};
#undef ARM_POINT

static float Dir(unsigned i)
{
    return (i ? arm_tune.flip2 : arm_tune.flip1) >= 0.5f ? -1.0f : 1.0f;
}

/* 电机读数 → 关节角：按零点折到 ±180°，电机多转了几圈也不影响。 */
static float JointAngle(unsigned i)
{
    return Dir(i) * Math_WrapDeg(joint[i]->position_deg - (i ? arm_tune.zero2 : arm_tune.zero1));
}

static uint8_t Problems(uint32_t now, uint8_t locked)
{
    uint8_t why = 0U;
    for (unsigned i = 0; i < 2U; ++i)
    {
        if (joint[i]->position_seq == 0U || now - joint[i]->position_ms > ARM_FEEDBACK_TIMEOUT_MS)
            why |= (uint8_t)ARM_WHY_FEEDBACK(i);
        if (joint[i]->stalled || joint[i]->protected_flag)
            why |= (uint8_t)ARM_WHY_STALL(i);
        if (locked && !joint[i]->enabled)
            why |= (uint8_t)ARM_WHY_ENABLE(i);
    }
    return why;
}

/* 规划从实测位置重新开始：速度清零，目标就是这里。 */
static void Sync(void)
{
    for (unsigned i = 0; i < 2U; ++i)
    {
        float q = JointAngle(i);
        base[i] = joint[i]->position_deg - Dir(i) * q;
        arm.sp[i] = arm.goal[i] = q;
        vel[i] = 0.0f;
    }
    arm.moving = 0U;
    need_sync = 0U;
    dirty = 1U;
}

/* 等两台电机都来了 settle_ms 以后的新位置再对齐（急停、重新锁住、改零点以后），这期间不下发位置，
   电机停在原地。 */
static void RequestSync(uint32_t settle_ms)
{
    for (unsigned i = 0; i < 2U; ++i)
        sync_seq[i] = joint[i]->position_seq;
    sync_ms = HAL_GetTick() + settle_ms;
    need_sync = 1U;
    arm.moving = 0U;
    move_pending = 0U;
    vel[0] = vel[1] = 0.0f;
}

static uint8_t SyncReady(void)
{
    for (unsigned i = 0; i < 2U; ++i)
        if (joint[i]->position_seq == sync_seq[i] || (int32_t)(joint[i]->position_ms - sync_ms) < 0)
            return 0U;
    return 1U;
}

/* 新的位置帧：更新实测关节角、末端位置，记进历史。 */
static void Measure(void)
{
    if (joint[0]->position_seq == used_seq[0] && joint[1]->position_seq == used_seq[1])
        return;
    used_seq[0] = joint[0]->position_seq;
    used_seq[1] = joint[1]->position_seq;
    if (!used_seq[0] || !used_seq[1])
        return;
    arm.q[0] = JointAngle(0);
    arm.q[1] = JointAngle(1);
    if (!Arm_Forward(arm.q[0], arm.q[1], &arm.x, &arm.y))
        arm.x = arm.y = NAN; // 按现在的尺寸合不拢：尺寸或零点不对
    history[history_head].ms = joint[0]->position_ms;
    history[history_head].q[0] = arm.q[0];
    history[history_head].q[1] = arm.q[1];
    history_head = (history_head + 1U) % ARM_HISTORY;
    if (history_count < ARM_HISTORY)
        ++history_count;
}

/* 零点、方向被 /set 或 /arm zero 改了：实测关节角马上按新的算，锁着的话规划重新对齐，臂不动。 */
static void CheckCalibration(void)
{
    const float now[4] = {arm_tune.zero1, arm_tune.zero2, arm_tune.flip1, arm_tune.flip2};
    if (memcmp(now, calib, sizeof(calib)) == 0)
        return;
    memcpy(calib, now, sizeof(calib));
    used_seq[0] = used_seq[1] = 0U;
    history_count = 0U; // 旧零点下的姿态不能拿来换算视觉
    Measure();
    if (arm.state == ARM_READY)
    {
        arm.follow = 0U;
        RequestSync(0U);
    }
}

/* 拍照那一刻（t）臂的关节角：在历史里线性插值，比最早的还早就用最早的。 */
static void PoseAt(uint32_t t, float q[2])
{
    q[0] = arm.q[0];
    q[1] = arm.q[1];
    for (unsigned k = 0; k < history_count; ++k)
    {
        unsigned at = (history_head + ARM_HISTORY - 1U - k) % ARM_HISTORY;
        if ((int32_t)(t - history[at].ms) < 0)
            continue;
        const float *a = history[at].q;
        if (k == 0U)
        {
            q[0] = a[0];
            q[1] = a[1];
            return;
        }
        unsigned next = (at + 1U) % ARM_HISTORY;
        float span = (float)(history[next].ms - history[at].ms);
        float f = span > 0.0f ? (float)(t - history[at].ms) / span : 0.0f;
        q[0] = a[0] + f * (history[next].q[0] - a[0]);
        q[1] = a[1] + f * (history[next].q[1] - a[1]);
        return;
    }
    if (history_count)
    {
        unsigned oldest = (history_head + ARM_HISTORY - history_count) % ARM_HISTORY;
        q[0] = history[oldest].q[0];
        q[1] = history[oldest].q[1];
    }
}

static void Want(unsigned i, uint8_t cmd)
{
    if (cmd & (CMD_ENABLE | CMD_DISABLE)) // 后要的为准
        pending[i] &= (uint8_t)~(CMD_ENABLE | CMD_DISABLE);
    pending[i] |= cmd;
}

static void Fault(uint8_t why)
{
    arm.state = ARM_FAULT;
    arm.detail = why;
    arm.moving = 0U;
    arm.follow = 0U;
    move_pending = 0U;
    recovering = 0U;
    vel[0] = vel[1] = 0.0f;
    pending[0] = pending[1] = CMD_STOP;
}

/* 一个关节走一拍：按离目标的距离算出“从现在开始刹车刚好停在目标”的速度，速度和加速度都不超限。
   起步、巡航用这次分到的加速度，刹车总能用满加速度。离散时间下刹车距离是 a·dt²·m(m+1)/2，
   解出来就是下面 stop 的式子，按它走不会冲过目标。返回 1 = 还在走。 */
static uint8_t PlanJoint(unsigned i)
{
    float e = arm.goal[i] - arm.sp[i], v = vel[i];
    float a = amax[i], full = fmaxf(arm_tune.acc, a);
    if (fabsf(e) < 1e-4f && fabsf(v) <= full * ARM_DT)
    {
        if (e != 0.0f || v != 0.0f)
            dirty = 1U;
        arm.sp[i] = arm.goal[i];
        vel[i] = 0.0f;
        return 0U;
    }
    float stop = sqrtf(0.25f * a * a * ARM_DT * ARM_DT + 2.0f * a * fabsf(e)) - 0.5f * a * ARM_DT;
    float want = copysignf(fminf(stop, vmax[i]), e);
    float limit = (want * v < 0.0f || fabsf(want) < fabsf(v)) ? full : a;
    float dv = want - v;
    if (dv > limit * ARM_DT)
        dv = limit * ARM_DT;
    else if (dv < -limit * ARM_DT)
        dv = -limit * ARM_DT;
    v += dv;
    float step = v * ARM_DT;
    dirty = 1U;
    /* 离目标不到 a·dt² 时按 stop 走会越过一点，这时速度最多 a·dt：直接停在目标上，减速度也不超。 */
    if (step * e > 0.0f && fabsf(step) >= fabsf(e) && fabsf(v) <= 1.001f * full * ARM_DT)
    {
        arm.sp[i] = arm.goal[i];
        vel[i] = 0.0f;
        return 0U;
    }
    arm.sp[i] += step;
    vel[i] = v;
    return 1U;
}

/* together = 1：两个关节按距离比例分限速和加速度，同时出发同时到，关节空间里走直线，
   也就是 PathClear 查过的那条线。 */
static void SetGoal(const float q[2], uint8_t together)
{
    float d0 = fabsf(q[0] - arm.sp[0]), d1 = fabsf(q[1] - arm.sp[1]), d = fmaxf(d0, d1);
    for (unsigned i = 0; i < 2U; ++i)
    {
        float share = together && d > 0.01f ? fmaxf((i ? d1 : d0) / d, 0.02f) : 1.0f;
        vmax[i] = arm_tune.speed * share;
        amax[i] = arm_tune.acc * share;
        arm.goal[i] = q[i];
    }
    arm.moving = 1U;
}

/* 位置命令（两帧）：这一拍这台已经攒了命令就不发，下一个发送拍再发。 */
static void SendSetpoints(void)
{
    uint8_t sent = 1U;
    for (unsigned i = 0; i < 2U; ++i)
    {
        long speed = lroundf(fabsf(vel[i]) / 6.0f * ARM_SPEED_MARGIN * 10.0f); // °/s → 0.1 RPM
        if (speed < (long)ARM_MIN_RPM_X10)
            speed = ARM_MIN_RPM_X10;
        if (speed > (long)ZDT_MAX_RPM_X10)
            speed = ZDT_MAX_RPM_X10;
        long angle = lroundf((base[i] + Dir(i) * arm.sp[i]) * 10.0f);
        if (ZDT_Staged(joint[i]) != 0U ||
            ZDT_MovePosition(joint[i], (int32_t)angle, (uint16_t)speed, ZDT_POS_ABSOLUTE,
                             ZDT_SYNC_NOW) != HAL_OK)
            sent = 0U;
    }
    if (sent)
        dirty = 0U;
}

static void SendCommand(unsigned i)
{
    uint8_t cmd = pending[i];
    if (!cmd || ZDT_Staged(joint[i]) >= 2U)
        return;
    HAL_StatusTypeDef status;
    uint8_t bit = cmd & CMD_STOP      ? CMD_STOP
                  : cmd & CMD_CLEAR   ? CMD_CLEAR
                  : cmd & CMD_DISABLE ? CMD_DISABLE
                                      : CMD_ENABLE;
    if (bit == CMD_STOP)
        status = ZDT_Stop(joint[i], ZDT_SYNC_NOW);
    else if (bit == CMD_CLEAR)
        status = ZDT_ClearProtection(joint[i]);
    else
        status = ZDT_EnableMotor(joint[i], bit == CMD_ENABLE);
    if (status == HAL_OK)
        pending[i] &= (uint8_t)~bit;
}

/* 每 4 拍：0、2 拍发位置（走的时候 10 ms 一次），1 拍读位置，3 拍读状态；挂着的命令哪一拍都能先发。
 */
static void Send(void)
{
    uint32_t slot = cycle % 4U;
    for (unsigned i = 0; i < 2U; ++i)
        SendCommand(i);
    if ((slot == 0U || slot == 2U) && arm.state == ARM_READY && !need_sync &&
        (dirty || cycle % ARM_RESEND_DIV == 0U))
        SendSetpoints();
    if (slot == 1U || slot == 3U)
        for (unsigned i = 0; i < 2U; ++i)
            if (ZDT_Staged(joint[i]) < 2U)
                (void)(slot == 1U ? ZDT_RequestPosition(joint[i]) : ZDT_RequestStatus(joint[i]));
}

static void Recover(uint32_t now)
{
    uint8_t why = Problems(now, !want_free);
    if (!why)
    {
        arm.state = ARM_WAIT; // 等待态按 want_free 锁住或松开
        recovering = 0U;
        return;
    }
    if (now - recover_ms > ARM_RECOVER_MS)
    {
        recovering = 0U;
        arm.detail = why;
        return;
    }
    if (cycle % ARM_RETRY_DIV != 0U)
        return;
    for (unsigned i = 0; i < 2U; ++i)
    {
        if (joint[i]->stalled || joint[i]->protected_flag)
            Want(i, CMD_CLEAR);
        else if (!want_free && !joint[i]->enabled)
            Want(i, CMD_ENABLE);
    }
}

void Arm_Tick(void)
{
    if (joint[0] == NULL)
        return;
    uint32_t now = HAL_GetTick();
    ++cycle;
    CheckCalibration();
    Measure();
    uint8_t retry = cycle % ARM_RETRY_DIV == 0U;
    switch (arm.state)
    {
    case ARM_WAIT:
        arm.detail = Problems(now, !want_free);
        for (unsigned i = 0; i < 2U && retry; ++i)
            if (joint[i]->enabled == want_free)
                Want(i, want_free ? CMD_DISABLE : CMD_ENABLE);
        if (want_free && !(arm.detail & (ARM_WHY_FEEDBACK(0) | ARM_WHY_FEEDBACK(1))))
        {
            arm.state = ARM_FREE;
            arm.detail = 0U;
        }
        else if (!want_free && !arm.detail)
        {
            arm.state = ARM_READY;
            RequestSync(0U);
        }
        break;
    case ARM_READY:
    {
        uint8_t why = Problems(now, 1U);
        if (why)
        {
            Fault(why);
            break;
        }
        if (want_free)
        {
            Want(0, CMD_STOP | CMD_DISABLE);
            Want(1, CMD_STOP | CMD_DISABLE);
            arm.state = ARM_FREE;
            arm.moving = move_pending = 0U;
            break;
        }
        if (need_sync && SyncReady())
            Sync();
        if (!need_sync)
        {
            uint8_t moving = PlanJoint(0);
            moving |= PlanJoint(1);
            if (!moving && move_pending)
            {
                move_pending = 0U;
                ++arm.moves;
            }
            arm.moving = moving;
        }
        if (arm.follow && arm.seen && now - seen_ms > ARM_FOLLOW_LOST_MS)
            arm.seen = 0U;
        break;
    }
    case ARM_FREE:
    {
        uint8_t why = Problems(now, 0U) & (ARM_WHY_FEEDBACK(0) | ARM_WHY_FEEDBACK(1));
        if (why)
        {
            Fault(why);
            break;
        }
        for (unsigned i = 0; i < 2U; ++i)
        {
            arm.sp[i] = arm.goal[i] = arm.q[i];
            if (retry && joint[i]->enabled)
                Want(i, CMD_DISABLE);
        }
        if (!want_free)
        {
            Want(0, CMD_ENABLE);
            Want(1, CMD_ENABLE);
            arm.state = ARM_WAIT;
        }
        break;
    }
    default:
        if (recovering)
            Recover(now);
        break;
    }
    Send();
}

HAL_StatusTypeDef Arm_Init(void)
{
    if (joint[0] != NULL)
        return HAL_BUSY;
    joint[0] = ZDT_AddMotor(ARM_ADDR_J1);
    joint[1] = ZDT_AddMotor(ARM_ADDR_J2);
    if (joint[0] == NULL || joint[1] == NULL)
    {
        joint[0] = joint[1] = NULL;
        return HAL_ERROR;
    }
    memset(&arm, 0, sizeof(arm));
    arm.state = ARM_WAIT;
    arm.reach = 1U;
    cycle = 0U;
    dirty = move_pending = want_free = recovering = need_sync = 0U;
    used_seq[0] = used_seq[1] = 0U;
    history_head = history_count = 0U;
    calib[0] = arm_tune.zero1;
    calib[1] = arm_tune.zero2;
    calib[2] = arm_tune.flip1;
    calib[3] = arm_tune.flip2;
    pending[0] = pending[1] = CMD_ENABLE;
    point_count = 0U;
    for (const arm_point *p = default_points; p->name[0]; ++p)
        (void)Arm_SetPoint(p->name, p->q[0], p->q[1]);
    return HAL_OK;
}

uint8_t Arm_InLimits(float q1, float q2)
{
    return isfinite(q1) && isfinite(q2) && q1 >= arm_tune.q1_min && q1 <= arm_tune.q1_max &&
           q2 >= arm_tune.q2_min && q2 <= arm_tune.q2_max;
}

void Arm_Elbows(float q1, float q2, float elbow[4])
{
    elbow[0] = arm_tune.j1_x + arm_tune.a1 * cosf(q1 * MATH_DEG_TO_RAD);
    elbow[1] = arm_tune.j1_y + arm_tune.a1 * sinf(q1 * MATH_DEG_TO_RAD);
    elbow[2] = arm_tune.j2_x + arm_tune.a2 * cosf(q2 * MATH_DEG_TO_RAD);
    elbow[3] = arm_tune.j2_y + arm_tune.a2 * sinf(q2 * MATH_DEG_TO_RAD);
}

/* 正解：两根小臂从两个肘出发交在末端，两圆的两个交点按 arm_mode 选一个。 */
uint8_t Arm_Forward(float q1, float q2, float *x, float *y)
{
    float e[4], b1 = arm_tune.b1, b2 = arm_tune.b2;
    Arm_Elbows(q1, q2, e);
    float dx = e[2] - e[0], dy = e[3] - e[1], d = hypotf(dx, dy);
    if (!(d > fabsf(b1 - b2) && d < b1 + b2 && d > 1e-3f)) // 写成这样 NAN 也合不拢
        return 0U;
    float along = (b1 * b1 - b2 * b2 + d * d) / (2.0f * d);
    float h = sqrtf(fmaxf(b1 * b1 - along * along, 0.0f)) * (arm_tune.mode >= 0.5f ? -1.0f : 1.0f);
    *x = e[0] + (along * dx - h * dy) / d;
    *y = e[1] + (along * dy + h * dx) / d;
    return 1U;
}

/* 两个向量的夹角离 0° 或 180° 还有多少度。 */
static float Corner(float ax, float ay, float bx, float by)
{
    float angle = fabsf(atan2f(ax * by - ay * bx, ax * bx + ay * by)) * MATH_RAD_TO_DEG;
    return fminf(angle, 180.0f - angle);
}

/* 死点是两根小臂在末端成一条线：这时两台电机管不住末端，机构会翻到另一种装配方式，两台电机还会
   互相顶。一边的大小臂拉直或折叠不算：那只是这根臂够得着的边，关节直着转过去肘换一边就行，
   两个工作区之间就是这么摆过去的。 */
float Arm_Clearance(float q1, float q2)
{
    float e[4], x, y;
    if (!Arm_InLimits(q1, q2) || !Arm_Forward(q1, q2, &x, &y))
        return -1.0f;
    Arm_Elbows(q1, q2, e);
    return Corner(e[0] - x, e[1] - y, e[2] - x, e[3] - y);
}

/* 逆解：每根大臂两支，四种组合里合得拢到 (x, y)（装配方式对）、离死点至少 arm_margin 的，最多 4
 * 个。 */
static unsigned Candidates(float x, float y, float out[4][2])
{
    const float jx[2] = {arm_tune.j1_x, arm_tune.j2_x}, jy[2] = {arm_tune.j1_y, arm_tune.j2_y};
    const float a[2] = {arm_tune.a1, arm_tune.a2}, b[2] = {arm_tune.b1, arm_tune.b2};
    float branch[2][2];
    for (unsigned i = 0; i < 2U; ++i)
    {
        float dx = x - jx[i], dy = y - jy[i], r = hypotf(dx, dy);
        float c = (a[i] * a[i] + r * r - b[i] * b[i]) / (2.0f * a[i] * r);
        if (!(r > 1e-3f && c >= -1.0f && c <= 1.0f))
            return 0U;
        float toward = atan2f(dy, dx), spread = acosf(c);
        branch[i][0] = Math_WrapDeg((toward + spread) * MATH_RAD_TO_DEG);
        branch[i][1] = Math_WrapDeg((toward - spread) * MATH_RAD_TO_DEG);
    }
    unsigned n = 0U;
    for (unsigned k = 0; k < 4U; ++k)
    {
        float q1 = branch[0][k >> 1], q2 = branch[1][k & 1U], px, py;
        // 另一种装配方式的姿态正解回去是另一个交点，不是 (x, y)
        if (Arm_Clearance(q1, q2) < arm_tune.margin || !Arm_Forward(q1, q2, &px, &py) ||
            hypotf(px - x, py - y) > ARM_SAME_MM)
            continue;
        out[n][0] = q1;
        out[n][1] = q2;
        ++n;
    }
    return n;
}

static float Distance(const float a[2], const float b[2])
{
    return fabsf(a[0] - b[0]) + fabsf(a[1] - b[1]);
}

uint8_t Arm_Inverse(float x, float y, const float near[2], float q[2])
{
    float cand[4][2];
    unsigned n = Candidates(x, y, cand), best = 0U;
    for (unsigned k = 1; k < n; ++k)
        if (Distance(cand[k], near) < Distance(cand[best], near))
            best = k;
    if (n)
    {
        q[0] = cand[best][0];
        q[1] = cand[best][1];
    }
    return n > 0U;
}

/* 关节空间走直线（两个关节同时到就是走直线），每 1° 查一次：一路上离死点的余量不能比 arm_margin、
   起点、终点里最小的那个还小。收纳姿态离死点近一点也能走出去、走回来，但不会半路更近。 */
static uint8_t PathClear(const float from[2], const float to[2])
{
    float start = Arm_Clearance(from[0], from[1]), end = Arm_Clearance(to[0], to[1]);
    if (start <= 0.0f || end <= 0.0f)
        return 0U;
    float floor = fminf(arm_tune.margin, fminf(start, end)) - 0.5f;
    float d0 = to[0] - from[0], d1 = to[1] - from[1];
    unsigned steps = (unsigned)ceilf(fmaxf(fabsf(d0), fabsf(d1)) / ARM_PATH_STEP);
    for (unsigned k = 1; k < steps; ++k)
    {
        float f = (float)k / (float)steps;
        if (Arm_Clearance(from[0] + f * d0, from[1] + f * d1) < floor)
            return 0U;
    }
    return 1U;
}

static arm_result Check(const float q[2])
{
    if (!Arm_InLimits(q[0], q[1]))
        return ARM_LIMIT;
    if (Arm_Clearance(q[0], q[1]) <= 0.0f || Arm_Clearance(arm.sp[0], arm.sp[1]) <= 0.0f)
        return ARM_REACH;
    return PathClear(arm.sp, q) ? ARM_OK : ARM_PATH;
}

arm_result Arm_MoveJoints(float q1, float q2)
{
    if (joint[0] == NULL || arm.state != ARM_READY || need_sync)
        return ARM_BUSY;
    const float q[2] = {q1, q2};
    arm_result result = Check(q);
    if (result != ARM_OK)
        return result;
    arm.follow = 0U;
    SetGoal(q, 1U);
    move_pending = 1U;
    return ARM_OK;
}

/* 末端去 (x, y)：几组逆解里挑从现在的规划位置直着走过去不经过死点、离得最近的。离得最近的那组
   不一定走得过去（比如从车右边摆回车左边，近的那组要穿过死点，远一点的那组绕得开）。 */
static arm_result Solve(float x, float y, float q[2])
{
    float cand[4][2];
    unsigned n = Candidates(x, y, cand);
    uint8_t found = 0U;
    for (unsigned k = 0; k < n; ++k)
        if (PathClear(arm.sp, cand[k]) &&
            (!found || Distance(cand[k], arm.sp) < Distance(q, arm.sp)))
        {
            q[0] = cand[k][0];
            q[1] = cand[k][1];
            found = 1U;
        }
    return found ? ARM_OK : n ? ARM_PATH : ARM_REACH;
}

arm_result Arm_MoveTo(float x, float y)
{
    float q[2];
    if (joint[0] == NULL || arm.state != ARM_READY || need_sync)
        return ARM_BUSY;
    arm_result result = Solve(x, y, q);
    return result == ARM_OK ? Arm_MoveJoints(q[0], q[1]) : result;
}

void Arm_Stop(void)
{
    if (joint[0] == NULL)
        return;
    arm.follow = 0U;
    if (arm.state == ARM_READY)
    {
        Want(0, CMD_STOP);
        Want(1, CMD_STOP);
        RequestSync(ARM_STOP_SETTLE_MS);
    }
    else if (arm.state == ARM_FAULT)
    {
        recovering = 1U;
        recover_ms = HAL_GetTick();
    }
    else if (arm.state == ARM_WAIT)
    {
        for (unsigned i = 0; i < 2U; ++i)
            if (arm.detail & ARM_WHY_STALL(i))
                Want(i, CMD_CLEAR);
    }
}

HAL_StatusTypeDef Arm_SetFree(uint8_t free)
{
    if (joint[0] == NULL)
        return HAL_ERROR;
    if (arm.state == ARM_FAULT)
        return HAL_BUSY;
    want_free = free ? 1U : 0U;
    if (want_free)
        arm.follow = 0U;
    return HAL_OK;
}

HAL_StatusTypeDef Arm_ZeroFor(float q1, float q2, float zero[2])
{
    if (joint[0] == NULL || (arm.state != ARM_READY && arm.state != ARM_FREE) || arm.moving ||
        need_sync)
        return HAL_BUSY;
    if (!isfinite(q1) || !isfinite(q2) || fabsf(q1) > 180.0f || fabsf(q2) > 180.0f)
        return HAL_ERROR;
    const float q[2] = {q1, q2};
    for (unsigned i = 0; i < 2U; ++i)
        zero[i] = Math_WrapDeg(joint[i]->position_deg - Dir(i) * q[i]);
    return HAL_OK;
}

/* 相机装在哪根小臂上（1、2），固定在车上是 0。 */
static unsigned CameraLink(void)
{
    long link = lroundf(arm_tune.cam);
    return link == 1L || link == 2L ? (unsigned)link : 0U;
}

/* 那根小臂（肘指向末端）在姿态 q 时朝哪，°。 */
static uint8_t LinkDirection(unsigned link, const float q[2], float *x, float *y, float *deg)
{
    float e[4];
    if (!Arm_Forward(q[0], q[1], x, y))
        return 0U;
    Arm_Elbows(q[0], q[1], e);
    *deg = atan2f(*y - e[2U * link - 1U], *x - e[2U * link - 2U]) * MATH_RAD_TO_DEG;
    return 1U;
}

/* 物料帧的偏差是相对画面 TARGET_PX、按画面的前/左给的：装在小臂上时按那一刻小臂的朝向转到车体系，
   原点是末端；固定在车上时原点是 cam_x/cam_y。 */
static uint8_t ToBody(float fwd, float left, const float q[2], float *x, float *y)
{
    float ox = arm_tune.cam_x, oy = arm_tune.cam_y, rot = arm_tune.cam_rot;
    unsigned link = CameraLink();
    if (link)
    {
        if (!LinkDirection(link, q, &ox, &oy, &rot))
            return 0U;
        rot -= arm_tune.cam_rot;
    }
    float c = cosf(rot * MATH_DEG_TO_RAD), s = sinf(rot * MATH_DEG_TO_RAD);
    *x = ox + c * fwd - s * left;
    *y = oy + s * fwd + c * left;
    return 1U;
}

uint8_t Arm_CameraToBody(float fwd, float left, float *x, float *y)
{
    return ToBody(fwd, left, arm.q, x, y);
}

uint8_t Arm_CameraDirection(float *deg)
{
    float x, y;
    unsigned link = CameraLink();
    return link && LinkDirection(link, arm.q, &x, &y, deg);
}

static uint8_t Reachable(float x, float y, float q[2])
{
    return Solve(x, y, q) == ARM_OK;
}

/* 跟随：够得着就过去；够不着（或者路上有死点）就沿着从现在的目标朝物料的方向，二分找能走到的
   最远处停在那。 */
static void Track(float x, float y)
{
    float q[2], gx, gy;
    arm.reach = Reachable(x, y, q);
    if (!arm.reach)
    {
        if (!Arm_Forward(arm.goal[0], arm.goal[1], &gx, &gy))
            return;
        float lo = 0.0f, hi = 1.0f, test[2];
        uint8_t any = 0U;
        for (unsigned k = 0; k < ARM_REACH_STEPS; ++k)
        {
            float mid = 0.5f * (lo + hi);
            if (Reachable(gx + (x - gx) * mid, gy + (y - gy) * mid, test))
            {
                lo = mid;
                q[0] = test[0];
                q[1] = test[1];
                any = 1U;
            }
            else
                hi = mid;
        }
        if (!any)
            return;
    }
    SetGoal(q, 1U); // 同时到：走的就是查过的那条直线
}

HAL_StatusTypeDef Arm_Follow(uint8_t color)
{
    if (joint[0] == NULL || color > COLOR_LIGHT_BLUE)
        return HAL_ERROR;
    if (color && arm.state != ARM_READY)
        return HAL_BUSY;
    arm.follow = color;
    arm.seen = 0U;
    arm.reach = 1U;
    move_pending = 0U;
    return HAL_OK;
}

/* 每帧物料：挑要跟的颜色里离末端最近的一个，按拍照那一刻臂的姿态换算到车体系，作为新目标。 */
void Arm_SeeMaterials(const VisionMaterial *materials, unsigned count, uint32_t frame_ms)
{
    if (joint[0] == NULL || !arm.follow)
        return;
    float q[2], best = 0.0f, bx = 0.0f, by = 0.0f;
    uint8_t found = 0U;
    PoseAt(frame_ms - (uint32_t)arm_tune.cam_lag, q);
    for (unsigned k = 0; k < count; ++k)
    {
        if (materials[k].color != arm.follow)
            continue;
        float x, y;
        if (!ToBody(materials[k].fwd_mm, materials[k].left_mm, q, &x, &y))
            continue;
        float d = hypotf(x - arm.x, y - arm.y);
        if (!found || d < best)
        {
            best = d;
            bx = x;
            by = y;
            found = 1U;
        }
    }
    if (!found)
        return;
    seen_ms = frame_ms;
    arm.seen = 1U;
    arm.follow_x = bx;
    arm.follow_y = by;
    if (arm.state != ARM_READY || need_sync)
        return;
    float gx, gy;
    if (arm.reach && Arm_Forward(arm.goal[0], arm.goal[1], &gx, &gy) &&
        hypotf(bx - gx, by - gy) < ARM_DEADBAND_MM)
        return;
    Track(bx, by);
}

uint8_t Arm_ValidName(const char *name)
{
    if (name == NULL || name[0] < 'a' || name[0] > 'z')
        return 0U;
    for (unsigned n = 0; name[n]; ++n)
    {
        char c = name[n];
        if (n >= ARM_NAME_MAX || !((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_'))
            return 0U;
    }
    return 1U;
}

const arm_point *Arm_FindPoint(const char *name)
{
    for (unsigned i = 0; i < point_count; ++i)
        if (strcmp(points[i].name, name) == 0)
            return &points[i];
    return NULL;
}

HAL_StatusTypeDef Arm_SetPoint(const char *name, float q1, float q2)
{
    if (!Arm_ValidName(name) || !Arm_InLimits(q1, q2))
        return HAL_ERROR;
    arm_point *p = (arm_point *)Arm_FindPoint(name);
    if (p == NULL)
    {
        if (point_count >= ARM_POINTS)
            return HAL_BUSY;
        p = &points[point_count++];
        strcpy(p->name, name);
    }
    p->q[0] = q1;
    p->q[1] = q2;
    return HAL_OK;
}

HAL_StatusTypeDef Arm_DeletePoint(const char *name)
{
    const arm_point *p = Arm_FindPoint(name);
    if (p == NULL)
        return HAL_ERROR;
    unsigned at = (unsigned)(p - points);
    memmove(&points[at], &points[at + 1U], (point_count - at - 1U) * sizeof(points[0]));
    --point_count;
    return HAL_OK;
}

unsigned Arm_PointCount(void)
{
    return point_count;
}

const arm_point *Arm_PointAt(unsigned index)
{
    return index < point_count ? &points[index] : NULL;
}
