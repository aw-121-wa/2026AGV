/* 机械臂（AGV_MODE_ARM）的主机测试：真实的 arm.c、ZDT 驱动、控制台命令、视觉协议和机械臂模式，
   CAN 总线上换成六台假的 X 固件电机（1..4 是轮子，只回读取；5、6 是两个关节，按位置命令转），
   视觉上位机换成“相机这一刻该看到什么”算出来的物料帧。 */
#include "arm.h"
#include "arm_app.h"
#include "chassis.h"
#include "chassis_telemetry.h"
#include "console.h"
#include "ser_port.h"
#include "tune_log.h"
#include "user_math.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#undef assert
#define assert(x)                                                                                  \
    do                                                                                             \
    {                                                                                              \
        if (!(x))                                                                                  \
        {                                                                                          \
            fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #x);                                   \
            exit(1);                                                                               \
        }                                                                                          \
    } while (0)

static uint32_t ms = 1000U;
uint32_t HAL_GetTick(void)
{
    return ms;
}

/* ---------- 控制台和遥测要的底盘、文本输出（只替换，不测） ---------- */
chassis_t chassis;
chassis_tuning chassis_tune = CHASSIS_TUNE_DEFAULTS;
const chassis_tuning chassis_tune_default = CHASSIS_TUNE_DEFAULTS;
HAL_StatusTypeDef TuneLog_Send(void)
{
    return HAL_ERROR;
}
void TuneLog_SetAuto(uint8_t on)
{
    (void)on;
}
void ChassisTelemetry_SetPlot(telemetry_plot group, unsigned hz)
{
    (void)group;
    (void)hz;
}
const char *ChassisTelemetry_Num(float value, unsigned decimals)
{
    static char pool[24][24];
    static unsigned next;
    char *out = pool[next];
    next = (next + 1U) % 24U;
    snprintf(out, sizeof(pool[0]), "%.*f", (int)decimals, (double)value);
    return out;
}
static char printed[1 << 16];
static size_t printed_len;
void ChassisTelemetry_Printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    int n = vsnprintf(printed + printed_len, sizeof(printed) - printed_len, format, args);
    va_end(args);
    assert(n >= 0 && (size_t)n < sizeof(printed) - printed_len);
    printed_len += (size_t)n;
}
void ChassisTelemetry_Puts(const char *text)
{
    ChassisTelemetry_Printf("%s", text);
}
static void ClearPrinted(void)
{
    printed_len = 0U;
    printed[0] = '\0';
}

/* ---------- 视觉串口：测试往里塞上位机的帧，单片机发出来的 MODE 帧记下来 ---------- */
static uint8_t vis_rx[8192];
static unsigned vis_head, vis_tail;
static uint8_t mode_sent = 0xFFU, mode_color = 0xFFU; // 最近一帧 MODE
static unsigned mode_frames;
HAL_StatusTypeDef SerPort_Init(SerId id, uint32_t baud)
{
    assert(id == SER_VISION && baud == 115200U);
    return HAL_OK;
}
unsigned SerPort_Read(SerId id, uint8_t *dst, unsigned max)
{
    assert(id == SER_VISION);
    unsigned n = 0U;
    while (n < max && vis_tail != vis_head)
        dst[n++] = vis_rx[vis_tail++ % sizeof(vis_rx)];
    return n;
}
uint8_t SerPort_Write(SerId id, const uint8_t *src, unsigned len)
{
    assert(id == SER_VISION && len >= 6U && src[0] == VL_SOF0 && src[1] == VL_SOF1);
    assert(VisionLink_Crc8(src + 2, 3U + src[2]) == src[5U + src[2]]);
    if (src[3] == VL_MCU_MODE)
    {
        mode_sent = src[5];
        mode_color = src[6];
        ++mode_frames;
    }
    return 1U;
}
static void HostFrame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    static uint8_t seq;
    uint8_t f[VL_FRAME_MAX] = {VL_SOF0, VL_SOF1, len, type, seq++};
    memcpy(f + 5, payload, len);
    f[5U + len] = VisionLink_Crc8(f + 2, 3U + len);
    for (unsigned i = 0; i < 6U + len; ++i)
        vis_rx[vis_head++ % sizeof(vis_rx)] = f[i];
}
static void PutI16(uint8_t *p, float v)
{
    long r = lroundf(v);
    p[0] = (uint8_t)(uint16_t)(int16_t)r;
    p[1] = (uint8_t)((uint16_t)(int16_t)r >> 8);
}

/* ---------- 假 CAN 总线上的电机：按地址收命令、回应答，5、6 按直通限速位置模式转 ---------- */
typedef struct
{
    uint8_t enabled, stalled, offline;
    float pos, target, speed; // 电机自己的读数，°；限速 °/s
    uint8_t code, body[16], len;
    unsigned fb, enables, disables, stops, clears;
    float hand; // 松开时每拍被手掰动多少度
} sim_motor;
static sim_motor sim[7]; // 下标就是地址
typedef struct
{
    uint32_t id;
    uint8_t len, data[8];
} frame_t;
static frame_t replies[128];
static unsigned reply_count, reply_next, fifo_used, frames_to[7];
static FDCAN_HandleTypeDef hcan;

static void Reply(uint8_t addr, const uint8_t *data, uint8_t len)
{
    assert(reply_count < 128U);
    frame_t *f = &replies[reply_count++];
    f->id = (uint32_t)addr << 8;
    f->len = len;
    memcpy(f->data, data, len);
}
static void Ack(uint8_t addr, uint8_t code, uint8_t status)
{
    const uint8_t d[3] = {code, status, 0x6BU};
    Reply(addr, d, 3U);
}
static uint32_t Be(const uint8_t *p, unsigned n)
{
    uint32_t v = 0U;
    for (unsigned i = 0; i < n; ++i)
        v = (v << 8) | p[i];
    return v;
}
/* 一条命令收齐了：按 X 固件执行，回应答。 */
static void Execute(uint8_t addr, sim_motor *m)
{
    const uint8_t *b = m->body;
    switch (m->code)
    {
    case 0xF3:
        m->enabled = b[1];
        if (b[1])
            ++m->enables;
        else
            ++m->disables;
        Ack(addr, 0xF3, 0x02);
        break;
    case 0xFB:
        if (!m->enabled)
        {
            Ack(addr, 0xFB, 0xE2); // 没使能时位置命令条件不满足
            break;
        }
        assert(b[7] == 1U && b[8] == 0U); // 绝对位置、立即执行
        m->target = (float)Be(b + 3, 4U) * (b[0] ? -0.1f : 0.1f);
        m->speed = (float)Be(b + 1, 2U) * 0.1f * 6.0f;
        ++m->fb;
        Ack(addr, 0xFB, 0x02);
        break;
    case 0xFE:
        m->target = m->pos;
        ++m->stops;
        Ack(addr, 0xFE, 0x02);
        break;
    case 0x0E:
        m->stalled = 0U;
        ++m->clears;
        Ack(addr, 0x0E, 0x02);
        break;
    case 0xF6:
        Ack(addr, 0xF6, 0x02);
        break;
    case 0x36:
    {
        long p = lroundf(m->pos * 10.0f);
        uint32_t u = (uint32_t)labs(p);
        const uint8_t d[7] = {
            0x36,       p < 0, (uint8_t)(u >> 24), (uint8_t)(u >> 16), (uint8_t)(u >> 8),
            (uint8_t)u, 0x6B};
        Reply(addr, d, 7U);
        break;
    }
    case 0x3A:
    {
        const uint8_t d[3] = {
            0x3A, (uint8_t)((m->enabled ? 0x01U : 0U) | (m->stalled ? 0x0CU : 0U)), 0x6B};
        Reply(addr, d, 3U);
        break;
    }
    default:
        assert(0);
    }
}
static unsigned BodyLength(uint8_t code)
{
    switch (code)
    {
    case 0xF3:
        return 4U;
    case 0xF6:
        return 7U;
    case 0xFB:
        return 10U;
    case 0xFE:
        return 3U;
    case 0x0E:
    case 0xFF:
        return 2U;
    default:
        return 1U; // 读取只有校验字节
    }
}

HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *h, uint32_t a, uint32_t b,
                                               uint32_t c, uint32_t d)
{
    (void)h, (void)a, (void)b, (void)c, (void)d;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *h)
{
    (void)h;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *h)
{
    (void)h;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *h,
                                              FDCAN_ProtocolStatusTypeDef *p)
{
    (void)h;
    p->BusOff = 0U;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(const FDCAN_HandleTypeDef *h,
                                             FDCAN_ErrorCountersTypeDef *c)
{
    (void)h;
    memset(c, 0, sizeof(*c));
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *h, uint32_t a, uint32_t b)
{
    (void)h, (void)a, (void)b;
    return HAL_OK;
}
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *h)
{
    (void)h;
    return 16U - fifo_used;
}
/* 交给硬件就算上了总线：按地址拼包，收齐一条就执行。 */
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *h, FDCAN_TxHeaderTypeDef *hd,
                                                uint8_t *d)
{
    (void)h;
    assert(fifo_used < 16U && hd->IdType == FDCAN_EXTENDED_ID);
    ++fifo_used;
    uint8_t addr = (uint8_t)(hd->Identifier >> 8), packet = (uint8_t)hd->Identifier;
    if (addr == 0U)
        return HAL_OK; // 广播同步：这里没有缓存的命令
    assert(addr <= 6U);
    ++frames_to[addr];
    sim_motor *m = &sim[addr];
    if (m->offline)
        return HAL_OK;
    if (packet == 0U)
    {
        m->code = d[0];
        m->len = 0U;
    }
    assert(d[0] == m->code);
    memcpy(m->body + m->len, d + 1, hd->DataLength - 1U);
    m->len = (uint8_t)(m->len + hd->DataLength - 1U);
    if (m->len == BodyLength(m->code))
    {
        assert(m->body[m->len - 1U] == 0x6BU);
        Execute(addr, m);
    }
    return HAL_OK;
}
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *h, uint32_t fifo)
{
    (void)h, (void)fifo;
    return reply_count - reply_next;
}
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *h, uint32_t fifo,
                                         FDCAN_RxHeaderTypeDef *hd, uint8_t *d)
{
    (void)h, (void)fifo;
    const frame_t *f = &replies[reply_next++];
    hd->Identifier = f->id | 0U;
    hd->IdType = FDCAN_EXTENDED_ID;
    hd->RxFrameType = FDCAN_DATA_FRAME;
    hd->DataLength = f->len;
    memcpy(d, f->data, f->len);
    return HAL_OK;
}

#include "../../User/Drivers/ZDT_X42S/zdt_x42s.c"
#include "../../User/Arm/arm.c"

/* ---------- 一拍：和固件的底盘任务同样的顺序 ---------- */
static zdt_motor *wheels[4];
static float true_q[400][2]; // 每拍按电机真实位置算的关节角（相机看到的是它），环形
static uint32_t true_ms[400];
static unsigned true_at;
static uint8_t camera_on;
static unsigned cam_link_true = 1U; // 相机实际装在哪根小臂上，0 = 固定在车上
static float cam_rot_true, lag_true = 60.0f;
static float mat_x, mat_y;
static uint8_t mat_color = COLOR_RED;
static void (*mat_path)(float t, float *x, float *y); // 物料在动：t 秒时在哪；NULL = 停在 mat_x/y
static uint32_t next_frame;

static void MaterialAt(uint32_t t, float *x, float *y)
{
    if (mat_path)
        mat_path((float)t * 0.001f, x, y);
    else
    {
        *x = mat_x;
        *y = mat_y;
    }
}

/* 转盘：绕 (10, 170) 半径 35 mm，30°/s，物料线速度 18 mm/s。 */
static void Turntable(float t, float *x, float *y)
{
    float a = t * 30.0f * MATH_DEG_TO_RAD;
    *x = 10.0f + 35.0f * cosf(a);
    *y = 170.0f + 35.0f * sinf(a);
}

static void TrueJoints(float q[2])
{
    q[0] = (arm_tune.flip1 >= 0.5f ? -1.0f : 1.0f) * Math_WrapDeg(sim[5].pos - arm_tune.zero1);
    q[1] = (arm_tune.flip2 >= 0.5f ? -1.0f : 1.0f) * Math_WrapDeg(sim[6].pos - arm_tune.zero2);
}

/* 拍照那一刻相机看到的物料偏差（同 vision/main.py 的 to_body：相对 TARGET_PX，画面的前/左）。
   相机装在小臂上时画面跟着那根小臂转，TARGET_PX 是末端正下方。 */
static void CameraSees(uint32_t t, float *fwd, float *left)
{
    unsigned at = true_at;
    for (unsigned k = 0; k < 400U; ++k)
    {
        at = (true_at + 400U - 1U - k) % 400U;
        if ((int32_t)(t - true_ms[at]) >= 0)
            break;
    }
    float ox = arm_tune.cam_x, oy = arm_tune.cam_y, rot = cam_rot_true, e[4];
    if (cam_link_true)
    {
        assert(Arm_Forward(true_q[at][0], true_q[at][1], &ox, &oy));
        Arm_Elbows(true_q[at][0], true_q[at][1], e);
        rot = atan2f(oy - e[2U * cam_link_true - 1U], ox - e[2U * cam_link_true - 2U]) *
                  MATH_RAD_TO_DEG -
              cam_rot_true;
    }
    float c = cosf(rot * MATH_DEG_TO_RAD), s = sinf(rot * MATH_DEG_TO_RAD), mx, my;
    MaterialAt(t, &mx, &my); // 照片里是那一刻的物料
    float dx = mx - ox, dy = my - oy;
    *fwd = c * dx + s * dy;
    *left = -s * dx + c * dy;
}

static void CameraTick(void)
{
    if (!camera_on || (int32_t)(ms - next_frame) < 0)
        return;
    next_frame = ms + 33U; // 30 帧/秒
    const uint8_t beat[2] = {0U, 30U};
    HostFrame(VL_HOST_HEARTBEAT, beat, 2U);
    float fwd, left;
    CameraSees(ms - (uint32_t)lag_true, &fwd, &left);
    uint8_t p[6] = {1U, mat_color};
    PutI16(p + 2, fwd * 10.0f);
    PutI16(p + 4, left * 10.0f);
    HostFrame(VL_HOST_MATERIALS, p, 6U);
}

static unsigned fb_ticks;     // 给关节发了位置命令的拍数（统计频率用）
static float worst_clearance; // 一路上离死点最近的时候（按电机真实位置算）
static void Tick(void)
{
    ms += 5U;
    HAL_FDCAN_RxFifo0Callback(&hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE); // 上一拍的应答
    reply_count = reply_next = 0U;
    ZDT_ProcessData();
    fifo_used = 0U;
    memset(frames_to, 0, sizeof(frames_to));
    unsigned fb_before = sim[5].fb + sim[6].fb;
    ArmApp_Tick();
    for (unsigned i = 0; i < 4U; ++i) // 底盘这一拍也在读轮子（Chassis_RequestFeedback）
        if (ZDT_Staged(wheels[i]) < 2U)
            (void)ZDT_RequestPosition(wheels[i]);
    ZDT_Flush();
    for (unsigned a = 1; a <= 6U; ++a)
        assert(frames_to[a] <= 2U); // 一台驱动器一拍最多收两帧
    if (sim[5].fb + sim[6].fb != fb_before)
        ++fb_ticks;
    for (unsigned a = 5; a <= 6U; ++a)
    {
        sim_motor *m = &sim[a];
        if (!m->enabled)
            m->pos += m->hand;
        else if (!m->stalled)
        {
            float step = m->speed * 0.005f, e = m->target - m->pos;
            m->pos += fabsf(e) <= step ? e : copysignf(step, e);
        }
    }
    TrueJoints(true_q[true_at]);
    worst_clearance = fminf(worst_clearance, Arm_Clearance(true_q[true_at][0], true_q[true_at][1]));
    true_ms[true_at] = ms;
    true_at = (true_at + 1U) % 400U;
    CameraTick();
}

#define RUN_UNTIL(cond, limit)                                                                     \
    do                                                                                             \
    {                                                                                              \
        unsigned n_ = 0U;                                                                          \
        while (!(cond))                                                                            \
        {                                                                                          \
            assert(++n_ <= (limit));                                                               \
            Tick();                                                                                \
        }                                                                                          \
    } while (0)

static void Run(unsigned ticks)
{
    while (ticks--)
        Tick();
}

static void Cmd(const char *text)
{
    char line[64];
    assert(strlen(text) < sizeof(line));
    strcpy(line, text);
    Console_Line(line);
}

static unsigned Count(const char *needle)
{
    unsigned n = 0U;
    for (const char *p = strstr(printed, needle); p; p = strstr(p + 1, needle))
        ++n;
    return n;
}

static void Near(float got, float want, float tol)
{
    if (!(fabsf(got - want) <= tol))
    {
        fprintf(stderr, "got %f want %f (tol %f)\n", (double)got, (double)want, (double)tol);
        assert(fabsf(got - want) <= tol);
    }
}

/* 两台关节电机的读数是多圈的任意值：J1 转过三圈多。零点按“现在就是 q = (120, 60)”标好：
   两台电机在车身中线上（J1 在后、J2 在前），两根大臂朝左后、左前张开，末端在车左边 (0, 188.6)。 */
static void Setup(void)
{
    memset(sim, 0, sizeof(sim));
    sim[5].pos = sim[5].target = 1234.5f;
    sim[6].pos = sim[6].target = -20.0f;
    reply_count = reply_next = 0U;
    vis_head = vis_tail = 0U;
    camera_on = 0U;
    cam_link_true = 1U;
    mat_path = NULL;
    mat_color = COLOR_RED;
    cam_rot_true = 0.0f;
    mode_sent = mode_color = 0xFFU;
    ClearPrinted();
    zdt_can = NULL;
    joint[0] = joint[1] = NULL;
    arm_tune = arm_tune_default;
    /* 测试用固定的几何，不跟 arm_calib.h 的占位尺寸走：两台电机在中线上相距 120，臂 100 / 150。 */
    arm_tune.j1_x = -60.0f;
    arm_tune.j2_x = 60.0f;
    arm_tune.j1_y = arm_tune.j2_y = 0.0f;
    arm_tune.a1 = arm_tune.a2 = 100.0f;
    arm_tune.b1 = arm_tune.b2 = 150.0f;
    arm_tune.mode = 0.0f;
    arm_tune.speed = 90.0f; // 速度、平滑、过死点也用固定的，arm_calib.h 调了不用跟着改
    arm_tune.acc = 360.0f;
    arm_tune.smooth = 0.0f;
    arm_tune.swing_speed = 300.0f;
    arm_tune.swing_run = 100.0f;
    arm_tune.cam_x = 0.0f;
    arm_tune.cam_y = 190.0f;
    arm_tune.zero1 = 34.5f;  // 1234.5 - 3 × 360 - 120
    arm_tune.zero2 = -80.0f; // -20 - (-80) = 60
    assert(ZDT_Init(&hcan) == HAL_OK);
    for (unsigned i = 0; i < 4U; ++i)
        wheels[i] = ZDT_AddMotor((uint8_t)(i + 1U));
    assert(ArmApp_Init() == HAL_OK);
    for (unsigned k = 0; k < 400U; ++k)
        true_ms[k] = 0U;
    worst_clearance = 180.0f;
}

static void Ready(void)
{
    Setup();
    RUN_UNTIL(arm.state == ARM_READY && !need_sync, 100U);
}

/* arm_calib.h 的默认尺寸（电机间距 200 实物，大臂 140、小臂 235 按设计俯视图）：图上那个姿态
   (90, 90) 末端在两个肘连线靠车右的一侧（side 1）、中线右边 73 mm，离死点够远。 */
static void TestDrawingDefaults(void)
{
    float x, y;
    arm_tune = arm_tune_default;
    arm.side = 1U;
    assert(arm_tune.mode == 1.0f && Arm_Forward(90.0f, 90.0f, &x, &y));
    Near(x, 0.0f, 0.05f);
    Near(y, -72.66f, 0.05f);
    assert(Arm_Clearance(90.0f, 90.0f) > 20.0f);
    /* 这一侧：两根大臂一起顺时针摆过去（谁也不越过谁），够得着车右外面和车头前面。车左的料盘在
       另一侧，要过死点，见 TestSwingFromPowerOnPose。 */
    float q[2];
    arm.sp[0] = arm.sp[1] = 90.0f;
    assert(Solve(0.0f, -280.0f, q) == ARM_OK && Solve(250.0f, 0.0f, q) == ARM_OK);
    arm.side = 0U;
}

/* 初始位置：上电前臂摆成图上那个姿态，驱动器的位置读数从 0 开始。按 arm_calib.h 的默认零点，固件
   一上电认的就是 (90, 90)、末端在中线右边 73 mm，锁在原地不动。 */
static void TestPowerOnPose(void)
{
    Setup();
    arm_tune = arm_tune_default;
    sim[5].pos = sim[6].pos = 0.0f;
    RUN_UNTIL(arm.state == ARM_READY && !need_sync, 100U);
    Near(arm.q[0], 90.0f, 0.01f);
    Near(arm.q[1], 90.0f, 0.01f);
    Near(arm.x, 0.0f, 0.05f);
    Near(arm.y, -72.66f, 0.05f);
    RUN_UNTIL(sim[5].fb && sim[6].fb, 10U);
    Near(sim[5].target, 0.0f, 0.05f);
    Near(sim[6].target, 0.0f, 0.05f);
}

/* 五连杆的几何：正解、逆解互相对得上；装配方式只认一种；死点附近不给。 */
static void TestKinematics(void)
{
    Setup();
    float x, y, q[2], e[4];
    assert(Arm_Forward(120.0f, 60.0f, &x, &y));
    Near(x, 0.0f, 0.01f);
    Near(y, 188.58f, 0.01f);
    Arm_Elbows(120.0f, 60.0f, e);
    Near(e[0], -110.0f, 0.01f);
    Near(e[3], 86.60f, 0.01f);
    arm.side = 1U; // 末端在两个肘连线的另一侧
    assert(Arm_Forward(120.0f, 60.0f, &x, &y));
    Near(y, 86.60f - 101.98f, 0.02f);
    arm.side = 0U;
    /* 两根小臂拉成一条线（两个肘正好隔 300 mm）：合不拢。 */
    assert(!Arm_Forward(154.158f, 25.842f, &x, &y) || Arm_Clearance(154.158f, 25.842f) < 1.0f);
    assert(Arm_Clearance(120.0f, 60.0f) > 70.0f);
    assert(Arm_Clearance(175.0f, 60.0f) < 0.0f); // 超限位
    unsigned solved = 0U;
    const float near[2] = {120.0f, 60.0f};
    for (float gx = -250.0f; gx <= 250.0f; gx += 20.0f)
        for (float gy = -300.0f; gy <= 300.0f; gy += 20.0f)
        {
            if (!Arm_Inverse(gx, gy, near, q))
                continue;
            ++solved;
            assert(Arm_Forward(q[0], q[1], &x, &y));
            Near(x, gx, 0.05f);
            Near(y, gy, 0.05f);
            assert(Arm_InLimits(q[0], q[1]) && Arm_Clearance(q[0], q[1]) >= arm_tune.margin);
        }
    assert(solved > 50U);
    assert(!Arm_Inverse(0.0f, 600.0f, near, q));
}

/* 上电：先使能，读到位置才 READY；在那之前一条位置命令都不发，第一条就是电机现在的位置。 */
static void TestStartupHoldsInPlace(void)
{
    Setup();
    Tick();
    assert(arm.state == ARM_WAIT && strstr(printed, "ARM state WAIT | waiting:"));
    RUN_UNTIL(arm.state == ARM_READY && !need_sync, 100U);
    assert(sim[5].enables == 1U && sim[6].enables == 1U);
    Near(arm.q[0], 120.0f, 0.01f);
    Near(arm.q[1], 60.0f, 0.01f);
    Near(arm.x, 0.0f, 0.05f);
    Near(arm.y, 188.58f, 0.05f);
    RUN_UNTIL(sim[5].fb && sim[6].fb, 10U);
    Near(sim[5].target, 1234.5f, 0.05f); // 多圈读数照样对得上，不会转回零圈
    Near(sim[6].target, -20.0f, 0.05f);
    /* 停着每 100 ms 重发一次位置，兜底丢帧；不会更频繁。 */
    unsigned before = sim[5].fb;
    Run(200);
    assert(sim[5].fb - before >= 9U && sim[5].fb - before <= 11U);
    assert(mode_sent == VISION_MATERIAL && mode_color == COLOR_ANY); // 不跟随时让视觉找全部物料
}

/* 关节空间走直线：两个关节同时到，速度、加速度都不超限，不冲过目标；10 ms 发一次位置。 */
static void TestJointMoveTogether(void)
{
    Ready();
    float start[2] = {arm.sp[0], arm.sp[1]}, last[2] = {start[0], start[1]}, v_last[2] = {0, 0};
    float vpeak = 0.0f, apeak = 0.0f, worst = 0.0f;
    assert(Arm_MoveJoints(150.0f, 70.0f) == ARM_OK); // J1 走 30°，J2 走 10°
    unsigned ticks = 0U, done[2] = {0U, 0U};
    fb_ticks = 0U;
    while (arm.moving && ticks < 2000U)
    {
        Tick();
        ++ticks;
        for (unsigned i = 0; i < 2U; ++i)
        {
            float v = (arm.sp[i] - last[i]) / 0.005f;
            vpeak = fmaxf(vpeak, fabsf(v));
            if (arm.sp[i] != arm.goal[i]) // 到达那一拍只走剩下不到 a·dt² 的一点，不算
                apeak = fmaxf(apeak, fabsf(v - v_last[i]) / 0.005f);
            v_last[i] = v;
            last[i] = arm.sp[i];
            if (!done[i] && arm.sp[i] == arm.goal[i])
                done[i] = ticks;
            float s = (arm.sp[i] - start[i]) / (arm.goal[i] - start[i]); // 关节空间里走到哪了
            worst = fmaxf(worst, fabsf(s - (arm.sp[0] - start[0]) / (arm.goal[0] - start[0])));
        }
    }
    assert(!arm.moving && arm.moves == 1U);
    assert(vpeak <= arm_tune.speed * 1.001f && apeak <= arm_tune.acc * 1.01f);
    assert(done[0] && done[1] && abs((int)done[0] - (int)done[1]) <= 2); // 一起到
    assert(worst < 0.02f);                                               // 走的是直线
    assert(fb_ticks >= ticks / 2U - 2U && fb_ticks <= ticks / 2U + 2U);  // 100 Hz
    printf("joint move: %u ticks, peak %.1f deg/s %.0f deg/s^2, off-line %.4f\n", ticks,
           (double)vpeak, (double)apeak, (double)worst);
    Run(40);
    Near(arm.q[0], 150.0f, 0.06f);
    Near(arm.q[1], 70.0f, 0.06f);
    Near(sim[5].pos, 1234.5f + 30.0f, 0.06f);
    assert(strstr(printed, "ARM done q=150.00,70.00"));
}

/* 半路改目标、往回走：速度连续，加速度不超满值，最后停在新目标上。 */
static void TestRetargetMidway(void)
{
    Ready();
    assert(Arm_MoveJoints(150.0f, 80.0f) == ARM_OK);
    Run(40);
    assert(arm.moving && vel[0] > 10.0f);
    assert(Arm_MoveJoints(105.0f, 55.0f) == ARM_OK);
    float last = arm.sp[0], v_last = vel[0], apeak = 0.0f;
    unsigned ticks = 0U;
    while (arm.moving && ticks++ < 3000U)
    {
        Tick();
        float v = (arm.sp[0] - last) / 0.005f;
        if (arm.sp[0] != arm.goal[0])
            apeak = fmaxf(apeak, fabsf(v - v_last) / 0.005f);
        v_last = v;
        last = arm.sp[0];
    }
    assert(!arm.moving && apeak <= arm_tune.acc * 1.01f);
    Near(arm.sp[0], 105.0f, 1e-4f);
    Run(40);
    Near(arm.q[0], 105.0f, 0.06f);
}

/* 末端走到 (x, y)；够不着、超限位、路上有死点都拒绝，臂不动。 */
static void TestCartesianMoves(void)
{
    Ready();
    assert(Arm_MoveTo(30.0f, 160.0f) == ARM_OK);
    RUN_UNTIL(!arm.moving, 2000U);
    Run(40);
    Near(arm.x, 30.0f, 0.3f);
    Near(arm.y, 160.0f, 0.3f);
    assert(Arm_MoveTo(0.0f, 600.0f) == ARM_REACH);
    assert(Arm_MoveJoints(0.0f, 175.0f) == ARM_LIMIT);
    /* 找一个本身离死点够远、但从这里直着走过去要经过死点的姿态：拒绝。 */
    uint8_t blocked = 0U;
    for (float t1 = -160.0f; t1 <= 160.0f && !blocked; t1 += 10.0f)
        for (float t2 = -160.0f; t2 <= 160.0f && !blocked; t2 += 10.0f)
        {
            const float to[2] = {t1, t2};
            if (Arm_Clearance(t1, t2) >= arm_tune.margin + 2.0f && !PathClear(arm.sp, to))
            {
                assert(Arm_MoveJoints(t1, t2) == ARM_PATH);
                blocked = 1U;
            }
        }
    assert(blocked && !arm.moving);
    /* 两个工作区：同一种装配方式，从车左边一下摆到车右边（两个肘交叉过去），路上离死点够远。 */
    assert(Arm_MoveTo(0.0f, -180.0f) == ARM_OK);
    RUN_UNTIL(!arm.moving, 4000U);
    Run(40);
    Near(arm.x, 0.0f, 0.3f);
    Near(arm.y, -180.0f, 0.3f);
    assert(Arm_MoveTo(0.0f, 180.0f) == ARM_OK); // 再摆回来
    RUN_UNTIL(!arm.moving, 4000U);
    assert(worst_clearance >= arm_tune.margin - 0.5f); // 真走过的路上离死点都够远
}

/* 松开示教：先停再松开，松开期间不发位置；手掰过以后重新锁住，就锁在掰到的地方，不弹回去。 */
static void TestFreeTeachAndRelock(void)
{
    Ready();
    Run(8);
    unsigned fb = sim[5].fb;
    assert(Arm_SetFree(1U) == HAL_OK);
    RUN_UNTIL(!sim[5].enabled && !sim[6].enabled, 10U);
    assert(arm.state == ARM_FREE && sim[5].stops == 1U && sim[5].disables == 1U);
    sim[5].hand = 0.1f; // 手掰 J1：100 拍 10°
    Run(100);
    sim[5].hand = 0.0f;
    Run(8);
    Near(arm.q[0], 130.0f, 0.1f);
    assert(sim[5].fb == fb); // 松开期间一条位置命令都没有
    assert(Arm_MoveJoints(120.0f, 60.0f) == ARM_BUSY);
    assert(Arm_SetFree(0U) == HAL_OK);
    RUN_UNTIL(arm.state == ARM_READY && !need_sync && sim[5].fb > fb, 100U);
    Near(sim[5].target, sim[5].pos, 0.06f); // 第一条位置命令就是掰到的地方
    Near(arm.goal[0], 130.0f, 0.1f);
}

/* 急停：马上发停止，等电机停稳、读到新位置才对齐，之后锁在停下的地方。 */
static void TestStopDuringMove(void)
{
    Ready();
    assert(Arm_MoveJoints(150.0f, 80.0f) == ARM_OK);
    Run(40);
    unsigned fb = sim[5].fb;
    Arm_Stop();
    Tick();
    assert(sim[5].stops == 1U && sim[6].stops == 1U);
    assert(sim[5].fb == fb && arm.moves == 0U);
    RUN_UNTIL(!need_sync, 20U);
    Run(4);
    Near(sim[5].target, sim[5].pos, 0.06f);
    assert(arm.state == ARM_READY && !arm.moving);
    assert(Arm_MoveJoints(120.0f, 60.0f) == ARM_OK);
    RUN_UNTIL(!arm.moving, 2000U);
    assert(arm.moves == 1U); // 被打断的那次不算走完
}

/* 堵转：停下、不再发位置；S 清保护以后回到 READY，从原地开始。掉线 300 ms 判故障。 */
static void TestFaults(void)
{
    Ready();
    sim[5].stalled = 1U;
    RUN_UNTIL(arm.state == ARM_FAULT, 10U);
    assert(arm.detail == ARM_WHY_STALL(0));
    Run(3);
    assert(sim[5].stops == 1U && sim[6].stops == 1U);
    unsigned fb = sim[5].fb;
    Run(100);
    assert(sim[5].fb == fb && arm.state == ARM_FAULT);
    assert(strstr(printed, "ARM state FAULT | fault: J1 stall (send S)"));
    assert(Arm_MoveJoints(130.0f, 60.0f) == ARM_BUSY && Arm_SetFree(1U) == HAL_BUSY);
    Arm_Stop();
    RUN_UNTIL(arm.state == ARM_READY && !need_sync, 100U);
    assert(sim[5].clears == 1U);
    Run(4);
    Near(sim[5].target, sim[5].pos, 0.06f);

    sim[6].offline = 1U;
    unsigned ticks = 0U;
    while (arm.state == ARM_READY && ticks < 200U)
    {
        Tick();
        ++ticks;
    }
    assert(arm.state == ARM_FAULT && arm.detail & ARM_WHY_FEEDBACK(1));
    // 最后一帧位置最多是 20 ms 前读的
    assert(ticks * 5U + 20U >= ARM_FEEDBACK_TIMEOUT_MS &&
           ticks * 5U <= ARM_FEEDBACK_TIMEOUT_MS + 40U);
}

/* 改零点、改方向：读数马上按新的算，臂不动；方向反了以后同样的关节命令电机反着转。 */
static void TestCalibrationChangesDoNotMove(void)
{
    Ready();
    Run(8);
    float pos = sim[5].pos;
    arm_tune.zero1 += 10.0f;
    Run(20);
    Near(arm.q[0], 110.0f, 0.06f);
    Near(sim[5].target, pos, 0.06f);
    arm_tune.flip1 = 1.0f;
    arm_tune.zero1 = Math_WrapDeg(sim[5].pos + 120.0f); // 反装：电机读数 = 零点 - 关节角
    Run(20);
    Near(arm.q[0], 120.0f, 0.06f);
    Near(sim[5].target, pos, 0.06f);
    assert(Arm_MoveJoints(130.0f, 60.0f) == ARM_OK);
    RUN_UNTIL(!arm.moving, 1000U);
    Run(20);
    Near(sim[5].pos, pos - 10.0f, 0.06f); // 关节 +10°，反装的电机 -10°
}

static void TestPoints(void)
{
    Ready();
    assert(Arm_SetPoint("home", 120.0f, 60.0f, 0U) == HAL_OK);
    assert(Arm_SetPoint("home", 130.0f, 70.0f, 1U) == HAL_OK && Arm_PointCount() == 1U); // 覆盖
    assert(Arm_FindPoint("home")->q[0] == 130.0f && Arm_FindPoint("home")->side == 1U);
    assert(Arm_SetPoint("9lives", 0.0f, 0.0f, 0U) == HAL_ERROR);
    assert(Arm_SetPoint("bad-name", 0.0f, 0.0f, 0U) == HAL_ERROR);
    assert(Arm_SetPoint("toolongname1", 0.0f, 0.0f, 0U) == HAL_ERROR);
    assert(Arm_SetPoint("far", 0.0f, 175.0f, 0U) == HAL_ERROR);
    char name[12];
    for (unsigned i = 1; i < ARM_POINTS; ++i)
    {
        snprintf(name, sizeof(name), "p%u", i);
        assert(Arm_SetPoint(name, (float)i, 0.0f, 0U) == HAL_OK);
    }
    assert(Arm_SetPoint("onemore", 0.0f, 0.0f, 0U) == HAL_BUSY);
    assert(Arm_DeletePoint("p3") == HAL_OK && Arm_PointCount() == ARM_POINTS - 1U);
    assert(Arm_FindPoint("p3") == NULL && Arm_FindPoint("p4")->q[0] == 4.0f);
    assert(Arm_DeletePoint("p3") == HAL_ERROR);
}

/* 平滑：发给电机的位置是规划位置最近 arm_smooth 毫秒的平均。梯形速度的加速度是一拍跳上去的，
   平均以后分 8 拍爬上去；两个关节还在同一条直线上，不冲过目标，最后正好停在目标上，发出去的
   位置追上了才算走完。 */
static void TestSmoothing(void)
{
    Ready();
    arm_tune.smooth = 40.0f; // 8 拍
    const float start[2] = {arm.sp[0], arm.sp[1]}, goal[2] = {150.0f, 70.0f};
    float was[2][2] = {{out[0], out[1]}, {arm.sp[0], arm.sp[1]}}; // 发出去的、规划的
    float v_was[2][2] = {{0}}, a_was[2][2] = {{0}}, jerk[2] = {0.0f, 0.0f}, vpeak = 0.0f,
          apeak = 0.0f;
    float off = 0.0f;
    unsigned ticks = 0U, planned = 0U;
    ClearPrinted();
    assert(Arm_MoveJoints(goal[0], goal[1]) == ARM_OK);
    while (arm.moving)
    {
        Tick();
        assert(++ticks < 2000U);
        if (!planned && arm.sp[0] == goal[0] && arm.sp[1] == goal[1])
            planned = ticks;
        const float now[2][2] = {{out[0], out[1]}, {arm.sp[0], arm.sp[1]}};
        for (unsigned k = 0; k < 2U; ++k) // 只看走得远的 J1：它用满速度和加速度
        {
            float v = (now[k][0] - was[k][0]) / 0.005f, a = (v - v_was[k][0]) / 0.005f;
            jerk[k] = fmaxf(jerk[k], fabsf(a - a_was[k][0]));
            if (k == 0U)
            {
                vpeak = fmaxf(vpeak, fabsf(v));
                apeak = fmaxf(apeak, fabsf(a));
            }
            was[k][0] = now[k][0];
            v_was[k][0] = v;
            a_was[k][0] = a;
        }
        float s0 = (out[0] - start[0]) / (goal[0] - start[0]);
        float s1 = (out[1] - start[1]) / (goal[1] - start[1]);
        off = fmaxf(off, fabsf(s0 - s1));
        assert(s0 <= 1.00001f && s1 <= 1.00001f);
    }
    printf("smoothing 40 ms: done %u ticks after the plan, peak %.1f deg/s %.0f deg/s^2, "
           "acceleration steps %.0f -> %.0f deg/s^2 per tick, off-line %.5f\n",
           ticks - planned, (double)vpeak, (double)apeak, (double)jerk[1], (double)jerk[0],
           (double)off);
    assert(out[0] == goal[0] && out[1] == goal[1] && planned);
    assert(ticks >= planned + 4U && ticks <= planned + 7U); // 窗口里全是目标了才算走完
    /* 加速度是位置的两次差分，单精度下有几 °/s² 的噪声。 */
    assert(off < 1e-3f && vpeak <= 90.0f * 1.001f && apeak <= 360.0f * 1.03f);
    assert(jerk[0] < 0.3f * jerk[1]); // 加速度一拍的跳变小了一大截
    Run(40);
    assert(Count("ARM done ") == 1U);
    Near(arm.q[0], goal[0], 0.06f);
    Near(arm.q[1], goal[1], 0.06f);
    /* 停着改窗口：不动。 */
    float held[2] = {sim[5].pos, sim[6].pos};
    arm_tune.smooth = 100.0f;
    Run(40);
    arm_tune.smooth = 0.0f;
    Run(40);
    Near(sim[5].pos, held[0], 0.01f);
    Near(sim[6].pos, held[1], 0.01f);
    assert(!arm.moving && out[0] == goal[0]);
}

/* 过死点：末端沿两台电机连线的中垂线从两个肘中间穿过去。先走到助跑点，再走那条直线：规划的
   关节角一路连续，两个肘的距离从不超过两根小臂的总长（不会让两台电机互相顶），到死点正好张到头、
   末端是 arm_swing_speed，那一拍换侧。两头都是末端离两个肘的连线 arm_swing_run 的地方：关节角
   一样，末端关于连线对称，所以甩过去、甩回来电机带末端的劲一样（实物上按离死点的距离取两头，
   甩得过去、甩不回来）。 */
static void TestSwing(void)
{
    Ready(); // q = (120, 60)：末端在两个肘连线的外侧 (0, 188.6)，side 0
    assert(arm.side == 0U && !arm.swinging);
    /* 两头：两个肘隔 2 × sqrt(150² - 100²) = 223.6，两根大臂在 121.2°、58.8°，肘连线在 y = 85.5，
       末端在它两边各 100：外侧 (0, 185.5)，里侧 (0, -14.5)。死点在 y = 43.6，所以外侧那一段长
       142、里侧只有 58。 */
    const float run = arm_tune.swing_run, half = sqrtf(150.0f * 150.0f - run * run);
    const float q_end = acosf((60.0f - half) / 100.0f) * MATH_RAD_TO_DEG;
    const float line_y = 100.0f * sinf(q_end * MATH_DEG_TO_RAD), dead_y = 43.589f;
    ClearPrinted();
    assert(Arm_Swing() == ARM_OK && arm.moving && arm.swinging && swing.phase == SWING_APPROACH);
    assert(Arm_Swing() == ARM_BUSY && Arm_MoveJoints(110.0f, 70.0f) == ARM_BUSY);
    assert(Arm_MoveTo(0.0f, 150.0f) == ARM_BUSY && Arm_Follow(COLOR_RED) == HAL_BUSY);
    float last[2] = {arm.sp[0], arm.sp[1]}, widest = 0.0f, cross_v = 0.0f, cross_gap = 1e9f;
    unsigned ticks = 0U, run_ticks = 0U;
    while (arm.moving)
    {
        uint8_t side = arm.side, running = swing.phase == SWING_RUN;
        Tick();
        assert(++ticks < 3000U);
        float e[4];
        Arm_Elbows(arm.sp[0], arm.sp[1], e);
        float apart = hypotf(e[2] - e[0], e[3] - e[1]);
        widest = fmaxf(widest, apart);
        assert(apart <= 300.0f + 0.002f);
        for (unsigned i = 0; i < 2U; ++i)
            assert(fabsf(arm.sp[i] - last[i]) <= 720.0f * 0.005f); // 关节角不跳
        last[0] = arm.sp[0];
        last[1] = arm.sp[1];
        if (running) // 末端在中垂线上：两个肘左右对称
        {
            ++run_ticks;
            Near(e[0] + e[2], 0.0f, 0.02f);
            Near(e[1], e[3], 0.02f);
        }
        if (arm.side != side) // 换侧那一拍：两个肘张到头，末端正全速穿过
        {
            cross_v = swing.v;
            cross_gap = 300.0f - apart;
        }
    }
    assert(arm.side == 1U && !arm.swinging && swing.phase == SWING_OFF);
    Near(cross_v, arm_tune.swing_speed, 0.5f);
    assert(cross_gap >= -0.002f && cross_gap < 0.03f);
    Near(widest, 300.0f, 0.03f);
    /* 两头的关节角一样；这一段长 2 × 100，死点前 142、死点后 58。 */
    Near(swing.q_in[0], q_end, 0.01f);
    Near(swing.q_in[1], 180.0f - q_end, 0.01f);
    Near(swing.q_out[0], swing.q_in[0], 0.01f);
    Near(swing.q_out[1], swing.q_in[1], 0.01f);
    Near(swing.length, 2.0f * run, 0.01f);
    Near(swing.cross, line_y + run - dead_y, 0.05f);
    /* 助跑那一段走一半加到全速，过了死点在后一半距离里收住：一共 1.5 × 200 / 300 = 1 s。 */
    assert(run_ticks >= 197U && run_ticks <= 205U);
    float x, y;
    assert(Arm_Forward(arm.sp[0], arm.sp[1], &x, &y));
    Near(x, 0.0f, 0.05f);
    Near(y, line_y - run, 0.05f);
    Run(40); // 电机跟上
    Near(arm.x, 0.0f, 0.3f);
    Near(arm.y, line_y - run, 0.3f);
    assert(strstr(printed, "ARM side 1\r\n") && Count("ARM done ") == 1U);
    printf("swing: %u ticks on the line, elbows at most %.3f mm apart (forearms 300), %.0f mm/s "
           "through the dead point\n",
           run_ticks, (double)widest, (double)cross_v);

    /* 再过一次就回来：已经在助跑点上，不用先走过去。这回死点前只有 58、死点后 142。 */
    assert(Arm_Swing() == ARM_OK && swing.phase == SWING_RUN);
    Near(swing.cross, dead_y - (line_y - run), 0.05f);
    RUN_UNTIL(!arm.moving, 1000U);
    assert(arm.side == 0U && Arm_Forward(arm.sp[0], arm.sp[1], &x, &y));
    Near(y, line_y + run, 0.05f);

    /* 半路急停：停在原地不再走，还没过死点就还在这一侧；对齐以后再过，先走回助跑点。 */
    assert(Arm_Swing() == ARM_OK);
    Run(30);
    assert(swing.phase == SWING_RUN && !swing.crossed);
    Arm_Stop();
    assert(!arm.moving && !arm.swinging && swing.phase == SWING_OFF && arm.side == 0U);
    Run(60);
    float held[2] = {sim[5].pos, sim[6].pos};
    Run(100);
    Near(sim[5].pos, held[0], 0.01f);
    Near(sim[6].pos, held[1], 0.01f);
    assert(Arm_Swing() == ARM_OK && swing.phase == SWING_APPROACH);
    RUN_UNTIL(!arm.moving, 3000U);
    assert(arm.side == 1U);

    /* 过不了：限位不让大臂张到死点；两根小臂加起来比两个肘能拉开的还长，根本没有死点。 */
    arm_tune.q1_max = 150.0f;
    assert(Arm_Swing() == ARM_LIMIT && !arm.moving);
    arm_tune.q1_max = 170.0f;
    arm_tune.b1 = arm_tune.b2 = 165.0f; // 两个肘最远 120 + 2 × 100 = 320 < 330
    assert(Arm_Swing() == ARM_REACH && !arm.moving);
}

/* 点位记着末端在哪一侧：去另一侧的点先过死点再走过去，走完才报一次 done；/arm j 带侧也一样。
   没甩过去时把固件记的那一侧改回来，臂不动。 */
static void TestSwingPoints(void)
{
    Ready();
    ClearPrinted();
    Cmd("arm point in 110 70 1");
    assert(strcmp(printed, "ARM point in 110.00 70.00 1\r\n") == 0);
    ClearPrinted();
    Cmd("arm point out 110 70"); // 不写侧：现在这一侧
    assert(strcmp(printed, "ARM point out 110.00 70.00 0\r\n") == 0);
    ClearPrinted();
    Cmd("arm go out");
    assert(strcmp(printed, "OK   arm to q=110.00,70.00\r\n") == 0 && !arm.swinging);
    RUN_UNTIL(!arm.moving, 2000U);
    ClearPrinted();
    Cmd("arm go in");
    assert(strcmp(printed, "OK   arm swing, then to q=110.00,70.00\r\n") == 0 && arm.swinging);
    RUN_UNTIL(!arm.moving, 3000U);
    assert(arm.side == 1U && arm.sp[0] == 110.0f && arm.sp[1] == 70.0f);
    Run(40);
    assert(Count("ARM done ") == 1U && strstr(printed, "ARM side 1\r\n"));
    Near(arm.x, 0.0f, 0.3f);
    Near(arm.y, 93.97f - 116.73f, 0.3f); // 同一组关节角，末端在两个肘连线的另一侧
    ClearPrinted();
    Cmd("arm");
    assert(strstr(printed, " move=0 side=1 follow=off\r\n"));
    ClearPrinted();
    Cmd("arm j 120 60 0");
    assert(strcmp(printed, "OK   arm swing, then to q=120.00,60.00\r\n") == 0);
    RUN_UNTIL(!arm.moving, 3000U);
    Run(40);
    assert(arm.side == 0U);
    Near(arm.y, 188.58f, 0.3f);
    ClearPrinted();
    Cmd("arm save here");
    assert(strstr(printed, "ARM point here ") && strstr(printed, " 0\r\n") &&
           Arm_FindPoint("here")->side == 0U);

    /* 没甩过去：改固件记的那一侧，臂不动，末端算到另一边。 */
    float held[2] = {sim[5].pos, sim[6].pos};
    ClearPrinted();
    Cmd("arm side 1");
    assert(strstr(printed, "OK   arm side 1\r\n") && arm.side == 1U);
    Run(20);
    assert(strstr(printed, "ARM side 1\r\n"));
    Near(arm.y, 86.60f - 101.98f, 0.05f);
    Near(sim[5].pos, held[0], 0.01f);
    Near(sim[6].pos, held[1], 0.01f);
    ClearPrinted();
    Cmd("arm side 2");
    assert(strstr(printed, "ERR  usage: /arm"));
    /* 标定时改 arm_mode（上电那一侧）：也是在说臂现在就在那一侧。 */
    Cmd("set arm_mode 1");
    Run(2);
    assert(arm.side == 1U);
    Cmd("set arm_mode 0");
    Run(2);
    assert(arm.side == 0U);
    RUN_UNTIL(!need_sync, 100U);
    ClearPrinted();
    Cmd("arm swing");
    assert(strcmp(printed, "OK   arm swing\r\n") == 0);
    ClearPrinted();
    Cmd("arm swing");
    assert(strcmp(printed, "BUSY arm moving, wait for ARM done\r\n") == 0);
    RUN_UNTIL(!arm.moving, 3000U);
    assert(arm.side == 1U);
    arm_tune.q1_max = 150.0f;
    ClearPrinted();
    Cmd("arm swing");
    assert(strcmp(printed, "ERR  arm: the swing or the point is outside the joint limits\r\n") ==
           0);
}

/* 死点标定：两根小臂掰成一条线，两个肘的距离就是两根小臂的总长，按现在的关节角算出来。 */
static void TestDeadPointCalibration(void)
{
    Ready();
    ClearPrinted();
    Cmd("arm deadpoint"); // 两个肘隔 220，离 300 远：没掰直
    assert(strcmp(printed, "ERR  arm: the forearms are not in one line (the elbows are not about "
                           "arm_b1 + arm_b2 apart)\r\n") == 0);
    /* 实物的小臂比填的长 2 mm：松开，掰直，两根大臂张到 156.9°、23.1°（电机读数只到 0.1°），
       两个肘隔 2 × (60 + 100 × cos 23.1°) = 303.96。 */
    assert(Arm_SetFree(1U) == HAL_OK);
    RUN_UNTIL(arm.state == ARM_FREE, 100U);
    sim[5].pos += 156.9f - 120.0f;
    sim[6].pos += 23.1f - 60.0f;
    Run(10);
    ClearPrinted();
    Cmd("arm deadpoint");
    assert(strcmp(printed, "OK   arm_b1 150.00 -> 151.98\r\nOK   arm_b2 150.00 -> 151.98\r\n") ==
           0);
    Near(arm_tune.b1, 151.98f, 0.006f);
    Near(arm_tune.b2, 151.98f, 0.006f);
}

/* 实物的尺寸、图上的初始位置：末端在两个肘连线靠车右的一侧，车左的料盘要在另一侧才够得着。
   过死点时末端在中线左边 37 mm，两个肘伸到车头车尾外面 x = ±235；过去以后三个料盘都够得着；
   回初始位置先过回来。 */
static void TestSwingFromPowerOnPose(void)
{
    Setup();
    arm_tune = arm_tune_default;
    sim[5].pos = sim[6].pos = 0.0f;
    RUN_UNTIL(arm.state == ARM_READY && !need_sync, 100U);
    assert(arm.side == 1U);
    assert(Arm_MoveTo(0.0f, 140.0f) == ARM_SIDE);
    assert(Arm_Swing() == ARM_OK);
    uint8_t crossed = 0U;
    unsigned ticks = 0U;
    float peak = 0.0f, was[2] = {out[0], out[1]}, widest = 0.0f;
    while (arm.moving)
    {
        uint8_t side = arm.side;
        Tick();
        ++ticks;
        peak = fmaxf(peak, fmaxf(fabsf(out[0] - was[0]), fabsf(out[1] - was[1])) / 0.005f);
        was[0] = out[0];
        was[1] = out[1];
        float e[4]; // 发给电机的位置（平滑开着，过死点那一段不平均）：两个肘照样张到头，不超
        Arm_Elbows(out[0], out[1], e);
        widest = fmaxf(widest, hypotf(e[2] - e[0], e[3] - e[1]));
        if (arm.side != side)
        {
            Near(e[0], -235.0f, 0.5f);
            Near(e[2], 235.0f, 0.5f);
            Near(e[1], 37.16f, 0.5f);
            crossed = 1U;
        }
    }
    assert(crossed && arm.side == 0U && widest <= 470.002f && widest > 469.97f);
    Run(40);
    /* 停在两个肘连线外侧 100 mm：大臂在 143.6°、36.4°，连线在 y = 83.1。 */
    Near(arm.x, 0.0f, 0.5f);
    Near(arm.y, 83.11f + 100.0f, 0.5f);
    Near(arm.q[0], 143.58f, 0.2f);
    Near(arm.q[1], 36.42f, 0.2f);
    printf("swing from the power-on pose: %.2f s in all, joints up to %.0f deg/s\n",
           (double)ticks * 0.005, (double)peak);
    static const float cups[3][2] = {{0.0f, 140.0f}, {-100.0f, 140.0f}, {100.0f, 140.0f}};
    for (unsigned k = 0; k < 3U; ++k)
    {
        assert(Arm_MoveTo(cups[k][0], cups[k][1]) == ARM_OK);
        RUN_UNTIL(!arm.moving, 6000U);
        Run(40);
        Near(arm.x, cups[k][0], 0.5f);
        Near(arm.y, cups[k][1], 0.5f);
    }
    assert(Arm_MoveJointsSide(90.0f, 90.0f, 1U) == ARM_OK); // 回初始位置：先过回去
    RUN_UNTIL(!arm.moving, 8000U);
    Run(40);
    assert(arm.side == 1U);
    Near(arm.x, 0.0f, 0.5f);
    Near(arm.y, -72.66f, 0.5f);
}

/* 跟随（相机装在 J1 那根小臂上，装歪了 30°，照片晚 60 ms 到）：静止的物料收敛到 1 mm 以内；
   转盘上的物料跟得住；一路上离死点都够远。 */
static void TestFollowArmCamera(void)
{
    Ready();
    cam_rot_true = arm_tune.cam_rot = 30.0f;
    mat_x = 30.0f;
    mat_y = 150.0f;
    camera_on = 1U;
    next_frame = ms;
    assert(Arm_Follow(COLOR_RED) == HAL_OK);
    Tick();
    assert(mode_sent == VISION_MATERIAL && mode_color == COLOR_RED); // 马上告诉上位机只找红的
    float after = 0.0f;
    for (unsigned t = 0; t < 600U; ++t)
    {
        Tick();
        if (t > 400U)
            after = fmaxf(after, hypotf(arm.x - mat_x, arm.y - mat_y));
    }
    printf("follow (camera on a forearm): within %.2f mm after 2 s\n", (double)after);
    assert(after < 1.0f && arm.seen && arm.reach);
    assert(strstr(printed, "ARM follow red:looking") && strstr(printed, "ARM follow red:tracking"));
    assert(arm.moves == 0U); // 跟随不打印 DONE
    mat_path = Turntable;
    float worst = 0.0f;
    for (unsigned t = 0; t < 1200U; ++t)
    {
        Tick();
        float x, y;
        MaterialAt(ms, &x, &y);
        if (t > 400U)
            worst = fmaxf(worst, hypotf(arm.x - x, arm.y - y));
    }
    printf("follow a turntable at 18 mm/s: worst %.2f mm, closest to a dead point %.1f deg\n",
           (double)worst, (double)worst_clearance);
    assert(worst < 8.0f && arm.reach);
    assert(worst_clearance >= arm_tune.margin - 0.5f);
    /* 物料看不到了：300 ms 后算丢了，臂停在最后看到的地方。 */
    camera_on = 0U;
    Run(70);
    assert(!arm.seen && Count("ARM follow red:looking") == 2U);
    float x = arm.x;
    Run(100);
    Near(arm.x, x, 0.5f);
    assert(Arm_Follow(0U) == HAL_OK);
    Tick();
    assert(mode_color == COLOR_ANY);
}

/* 相机固定在车上；物料够不着时朝它走到能走的最远处，报 far，也不靠近死点。 */
static void TestFollowFixedCameraAndReach(void)
{
    Ready();
    cam_link_true = 0U;
    arm_tune.cam = 0.0f;
    mat_x = -30.0f;
    mat_y = 150.0f;
    camera_on = 1U;
    next_frame = ms;
    assert(Arm_Follow(COLOR_RED) == HAL_OK);
    Run(600);
    Near(arm.x, mat_x, 1.0f);
    Near(arm.y, mat_y, 1.0f);
    mat_x = 0.0f; // 大臂加小臂全伸直也只有 250 mm
    mat_y = 600.0f;
    Run(600);
    assert(!arm.reach && strstr(printed, "ARM follow red:far"));
    printf("out of reach: stopped at (%.1f, %.1f), clearance %.1f deg\n", (double)arm.x,
           (double)arm.y, (double)Arm_Clearance(arm.q[0], arm.q[1]));
    /* 停在从 (-30, 150) 朝 (0, 600) 那条线上能走到的最远处：再往前 3 mm 就没有合得拢的姿态了。 */
    assert(arm.y > 210.0f);
    Near(arm.x, -30.0f + 30.0f * (arm.y - 150.0f) / 450.0f, 1.0f);
    float further[2], k = 3.0f / hypotf(30.0f, 450.0f);
    assert(!Arm_Inverse(arm.x + 30.0f * k, arm.y + 450.0f * k, arm.q, further));
    assert(worst_clearance >= arm_tune.margin - 0.5f);
    /* 别的颜色不跟。 */
    float y = arm.y;
    mat_color = COLOR_BLUE;
    mat_x = 30.0f;
    mat_y = 150.0f;
    Run(200);
    assert(!arm.seen);
    Near(arm.y, y, 0.5f);
}

/* 控制台命令和回复格式（调试台按这些解析）。 */
static void TestConsole(void)
{
    Setup();
    ClearPrinted();
    Cmd("arm j 130 70");
    assert(strcmp(printed, "BUSY arm waiting for the motors\r\n") == 0);
    RUN_UNTIL(arm.state == ARM_READY && !need_sync, 100U);

    ClearPrinted();
    Cmd("arm");
    assert(strncmp(printed, "          ARM READY q=120.00,60.00 xy=", 38) == 0);
    assert(strstr(
        printed, ",188.6 goal=120.00,60.00 move=0 side=0 follow=off\r\n          VIS OFF n=0\r\n"));
    ClearPrinted();
    Cmd("arm j 130 70");
    assert(strcmp(printed, "OK   arm to q=130.00,70.00\r\n") == 0);
    ClearPrinted();
    Cmd("arm j 500 0");
    assert(strcmp(printed, "ERR  arm: q1 must be -170.0..170.0, q2 -170.0..170.0\r\n") == 0);
    ClearPrinted();
    Cmd("arm xy 1000 0");
    assert(strstr(printed, "ERR  arm: out of reach"));
    RUN_UNTIL(!arm.moving, 1000U);
    Run(20);
    ClearPrinted();
    Cmd("arm jog j1 5");
    Cmd("arm jog j1 5"); // 从目标累加，不等上一下走完
    assert(Count("OK   arm to q=") == 2U && arm.goal[0] == 140.0f);
    RUN_UNTIL(!arm.moving, 1000U);
    Run(20);
    ClearPrinted();
    Cmd("arm jog y -10");
    float gx, gy, x0 = arm.x, y0 = arm.y;
    assert(Arm_Forward(arm.goal[0], arm.goal[1], &gx, &gy));
    Near(gx, x0, 0.2f);
    Near(gy, y0 - 10.0f, 0.2f);
    Cmd("arm jog y 300");
    assert(strstr(printed, "ERR  arm: out of reach"));
    RUN_UNTIL(!arm.moving, 1000U);
    Run(20);

    ClearPrinted();
    Cmd("arm save pick_1");
    assert(strstr(printed, "ARM point pick_1 ") && Arm_FindPoint("pick_1"));
    ClearPrinted();
    Cmd("arm point slot2 125.5 65");
    assert(strcmp(printed, "ARM point slot2 125.50 65.00 0\r\n") == 0);
    ClearPrinted();
    Cmd("arm point 2bad 0 0");
    assert(strstr(printed, "ERR  arm: point name is 1..11"));
    ClearPrinted();
    Cmd("arm point bad 0 175");
    assert(strstr(printed, "ERR  arm: q1 must be"));
    ClearPrinted();
    Cmd("arm points");
    assert(Count("ARM point ") == 2U && strstr(printed, "ARM points 2\r\n"));
    ClearPrinted();
    Cmd("arm go slot2");
    assert(strcmp(printed, "OK   arm to q=125.50,65.00\r\n") == 0);
    ClearPrinted();
    Cmd("arm save p");
    assert(strcmp(printed, "BUSY arm moving, wait for ARM done\r\n") == 0);
    RUN_UNTIL(!arm.moving, 2000U);
    ClearPrinted();
    Cmd("arm go nope");
    assert(strcmp(printed, "ERR  arm: no point 'nope' (/arm points)\r\n") == 0);
    ClearPrinted();
    Cmd("arm del slot2");
    assert(strcmp(printed, "OK   arm point slot2 deleted\r\n") == 0 && Arm_PointCount() == 1U);

    /* 零点：现在的姿态定为 (90, 90)，回复和 /set 一样；臂不动。 */
    Run(20);
    float pos5 = sim[5].pos;
    ClearPrinted();
    Cmd("arm zero 90 90");
    assert(Count("OK   arm_zero1 ") == 1U && Count("OK   arm_zero2 ") == 1U);
    Run(20);
    Near(arm.q[0], 90.0f, 0.02f);
    Near(arm.q[1], 90.0f, 0.02f);
    Near(sim[5].pos, pos5, 0.06f);
    ClearPrinted();
    Cmd("arm zero 200 0");
    assert(strcmp(printed, "ERR  arm: zero angles must be -180..180\r\n") == 0);
    ClearPrinted();
    Cmd("arm camdir"); // 相机在 J1 的小臂上：设成那根小臂现在的方向
    assert(strstr(printed, "OK   arm_cam_rot 0.0 -> "));
    float deg;
    assert(Arm_CameraDirection(&deg));
    Near(arm_tune.cam_rot, deg, 0.06f);
    Cmd("set arm_cam 0");
    ClearPrinted();
    Cmd("arm camdir");
    assert(strcmp(printed, "ERR  arm: camera is not on a forearm (arm_cam 1 or 2)\r\n") == 0);

    ClearPrinted();
    Cmd("list arm_");
    assert(Count("\r\n") == 28U &&
           strstr(printed, "arm_j1_x        = -60.0  [-300.0..300.0] default -100.0"));
    ClearPrinted();
    Cmd("set arm_speed 180");
    assert(strcmp(printed, "OK   arm_speed 90 -> 180\r\n") == 0);

    ClearPrinted();
    Cmd("arm follow blue");
    assert(strcmp(printed, "OK   arm follow blue\r\n") == 0 && arm.follow == COLOR_BLUE);
    Cmd("arm follow 1");
    assert(arm.follow == COLOR_RED);
    Cmd("arm follow lightblue");
    assert(arm.follow == COLOR_LIGHT_BLUE);
    ClearPrinted();
    Cmd("arm follow pink");
    assert(strstr(printed, "ERR  usage: /arm"));
    Cmd("arm follow off");
    assert(arm.follow == 0U);

    ClearPrinted();
    Cmd("arm free");
    assert(strcmp(printed, "OK   arm free: move it by hand\r\n") == 0);
    Run(10);
    assert(strstr(printed, "ARM state FREE\r\n"));
    ClearPrinted();
    Cmd("arm j 120 60");
    assert(strcmp(printed, "BUSY arm free, /arm hold first\r\n") == 0);
    Cmd("arm hold");
    RUN_UNTIL(arm.state == ARM_READY, 100U);
    ClearPrinted();
    Cmd("arm wiggle");
    assert(strstr(printed, "ERR  usage: /arm"));
    Cmd("arm point a b c d e f");
    assert(strstr(printed, "ERR  usage: /arm"));

    /* 视觉看到的物料换算到车体系，列在状态里（相机固定在车上，画面中心对着 cam_x/cam_y）。
       上面 camdir 把 arm_cam_rot 设成了小臂的方向，固定相机要转回 0。 */
    Cmd("set arm_cam_rot 0");
    camera_on = 1U;
    cam_link_true = 0U;
    mat_x = 200.0f;
    mat_y = -30.0f;
    next_frame = ms;
    Run(12);
    ClearPrinted();
    Cmd("arm");
    assert(strstr(printed, "          VIS ON n=1 red:200.0,-30.0\r\n"));
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0); // 断言失败时前面打的行还在
    TestDrawingDefaults();
    TestPowerOnPose();
    TestKinematics();
    TestStartupHoldsInPlace();
    TestJointMoveTogether();
    TestRetargetMidway();
    TestCartesianMoves();
    TestFreeTeachAndRelock();
    TestStopDuringMove();
    TestFaults();
    TestCalibrationChangesDoNotMove();
    TestPoints();
    TestSmoothing();
    TestSwing();
    TestSwingPoints();
    TestDeadPointCalibration();
    TestSwingFromPowerOnPose();
    TestFollowArmCamera();
    TestFollowFixedCameraAndReach();
    TestConsole();
    printf(
        "PASS: five-bar kinematics and dead points, startup hold, synced joint moves, retarget, "
        "cartesian/path checks, free/teach/relock, stop, faults, zero/flip without motion, points, "
        "setpoint smoothing, "
        "dead-point swing (line through the elbows, side tracking, points on the other side, "
        "calibration, real sizes), follow (forearm and fixed camera, turntable, lost, out of "
        "reach), "
        "console formats\n");
    return 0;
}
