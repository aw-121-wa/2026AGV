#include "tune_log.h"
#include "chassis.h"
#include "chassis_telemetry.h"
#include <math.h>

#define LINES_PER_TICK 4U

typedef enum
{
    LOG_EMPTY = 0,
    LOG_RECORD, // 动作进行中
    LOG_TAIL,   // 动作结束了，再记一小段
    LOG_READY,  // 记完了，可以发
} tune_log_phase;

typedef struct
{
    int32_t target, yaw; // 0.001°
    int16_t pid, cmd;    // 0.01 deg/s
    int16_t vx, vy;      // mm/s
} tune_sample;

/* 34 KB，放 .agv_ram（DTCM 不够）。那里上电不清零，没关系：只读写过的下标。
   log_modes 是 chassis_mode，平移时（航向环没有死区）加 0x10。 */
static tune_sample log_samples[TUNE_LOG_CAPACITY] AGV_RAM;
static uint8_t log_modes[TUNE_LOG_CAPACITY] AGV_RAM;
static unsigned log_count, log_tail_left, log_line;
static tune_log_phase log_phase;
static uint8_t log_motion, log_fault, log_done, log_truncated;
static uint8_t log_auto, log_pending, log_sending;
static chassis_tuning log_params; // 动作开始时的参数：日志是在这组参数下记的

static uint8_t LogInMotion(void)
{
    return chassis.mode == CHASSIS_MODE_PATH || chassis.mode == CHASSIS_MODE_TURN ||
           chassis.mode == CHASSIS_MODE_MOVE;
}

static int16_t LogFixed16(float value, float scale)
{
    return (int16_t)lroundf(fmaxf(-32767.0f, fminf(32767.0f, value * scale)));
}

static void LogRecord(void)
{
    if (log_count == TUNE_LOG_CAPACITY)
    {
        log_truncated = 1U;
        return;
    }
    log_samples[log_count] = (tune_sample){
        .target = lroundf(chassis.heading_target * 1000.0f),
        .yaw = lroundf(chassis.yaw * 1000.0f),
        .pid = LogFixed16(chassis.target.wz, 100.0f),
        .cmd = LogFixed16(chassis.current.wz, 100.0f),
        .vx = LogFixed16(chassis.current.vx, 1.0f),
        .vy = LogFixed16(chassis.current.vy, 1.0f),
    };
    uint8_t moving = chassis.target.vx != 0.0f || chassis.target.vy != 0.0f;
    log_modes[log_count++] = (uint8_t)(chassis.mode | (moving ? 0x10U : 0U));
}

/* 动作刚结束的这一拍：路径看有没有走完；原地转和定距正常结束时航向一定在容差里，S 停的一般不在。 */
static void LogNoteEnd(void)
{
    log_fault = chassis.state == CHASSIS_STATE_FAULT;
    log_done = !log_fault && (log_motion == CHASSIS_MODE_PATH
                                  ? chassis_path_result.completed
                                  : fabsf(chassis.heading_error) <= chassis_tune.heading_tol);
}

static const char *LogModeText(uint8_t mode)
{
    return mode == CHASSIS_MODE_PATH ? "path" : mode == CHASSIS_MODE_TURN ? "turn" : "move";
}

static void LogSendLine(void)
{
    const chassis_tuning *p = &log_params;
    if (log_line == 0U)
        ChassisTelemetry_Printf("LOG begin n=%u mode=%s end=%s trunc=%u imu=%u\r\n", log_count,
                                LogModeText(log_motion),
                                log_fault  ? "fault"
                                : log_done ? "done"
                                           : "stop",
                                (unsigned)log_truncated, (unsigned)CHASSIS_USE_IMU);
    else if (log_line == 1U)
        ChassisTelemetry_Printf(
            "LOG param heading_kp=%s heading_ki=%s heading_i_max=%s "
            "heading_i_zone=%s turn_max_speed=%s\r\n",
            ChassisTelemetry_Num(p->heading_kp, 3), ChassisTelemetry_Num(p->heading_ki, 3),
            ChassisTelemetry_Num(p->heading_i_max, 2), ChassisTelemetry_Num(p->heading_i_zone, 2),
            ChassisTelemetry_Num(p->turn_max_speed, 1));
    else if (log_line == 2U)
        ChassisTelemetry_Printf(
            "LOG param acc_angular=%s heading_tol=%s settle_ms=%s "
            "acc_linear=%s chassis_k=%s wheel_dia=%s distance_tol=%s\r\n",
            ChassisTelemetry_Num(p->acc_angular, 0), ChassisTelemetry_Num(p->heading_tol, 3),
            ChassisTelemetry_Num(p->settle_ms, 0), ChassisTelemetry_Num(p->acc_linear, 0),
            ChassisTelemetry_Num(p->chassis_k, 2), ChassisTelemetry_Num(p->wheel_dia, 2),
            ChassisTelemetry_Num(p->distance_tol, 2));
    else if (log_line - 3U < log_count)
    {
        unsigned i = log_line - 3U;
        const tune_sample *s = &log_samples[i];
        ChassisTelemetry_Printf("L %u %u %u %ld %ld %d %d %d %d\r\n", i, log_modes[i] & 0x0FU,
                                (unsigned)(log_modes[i] >> 4), (long)s->target, (long)s->yaw,
                                s->pid, s->cmd, s->vx, s->vy);
    }
    else
    {
        ChassisTelemetry_Printf("LOG end n=%u\r\n", log_count);
        log_sending = 0U;
    }
    ++log_line;
}

static void LogStartSend(void)
{
    log_pending = 0U;
    log_sending = 1U;
    log_line = 0U;
}

void TuneLog_Tick(void)
{
    uint8_t motion = LogInMotion();
    if (motion && log_phase != LOG_RECORD)
    {
        if (log_phase != LOG_TAIL) // 尾巴里又动了就接着记成一段，否则从头记
        {
            if (log_sending)
                ChassisTelemetry_Printf("LOG abort\r\n");
            log_sending = 0U;
            log_count = 0U;
            log_truncated = 0U;
            log_motion = chassis.mode;
            log_params = chassis_tune;
            log_pending = log_auto;
        }
        log_phase = LOG_RECORD;
    }
    if (log_phase == LOG_RECORD || log_phase == LOG_TAIL)
    {
        LogRecord();
        if (log_phase == LOG_RECORD && !motion)
        {
            LogNoteEnd();
            log_phase = LOG_TAIL;
            log_tail_left = TUNE_LOG_TAIL;
        }
        else if (log_phase == LOG_TAIL && --log_tail_left == 0U)
        {
            log_phase = LOG_READY;
            if (log_pending)
                LogStartSend();
        }
    }
    for (unsigned n = 0U; log_sending && n < LINES_PER_TICK && ChassisTelemetry_HasRoom(); ++n)
        LogSendLine();
}

void TuneLog_SetAuto(uint8_t on)
{
    log_auto = on ? 1U : 0U;
    if (log_phase == LOG_RECORD || log_phase == LOG_TAIL)
        log_pending = log_auto; // 正在记的这段也照新设置办
}

HAL_StatusTypeDef TuneLog_Send(void)
{
    if (log_phase == LOG_EMPTY)
        return HAL_ERROR;
    if (log_phase != LOG_READY)
    {
        log_pending = 1U;
        return HAL_BUSY;
    }
    LogStartSend();
    return HAL_OK;
}
