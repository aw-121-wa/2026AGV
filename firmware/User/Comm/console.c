#include "console.h"
#include "agv_config.h"
#include "chassis.h"
#include "chassis_telemetry.h"
#include "tune_log.h"
#include <math.h>
#include <stddef.h>
#include <string.h>
#if AGV_MODE == AGV_MODE_COMPETITION
#include "comp_config.h"
#include "competition.h"
#elif AGV_MODE == AGV_MODE_SURVEY
#include "agv_run.h"
#elif AGV_MODE == AGV_MODE_ARM
#include "arm.h"
#include "arm_app.h"
#endif

#define CONSOLE_ARGS 5U // 最长的命令 /arm point <名字> <q1> <q2>
#define STOPPED_ONLY 1U // 运动中一改速度或路径就会跳，只能停车改

typedef struct
{
    const char *name;
    float *value;
    const float *def;
    float min, max;
    uint8_t decimals, flags;
} Param;

/* 参数名就是结构体字段名。加一个参数：在 chassis_tuning / LidarTuning 里加字段，这里加一行，
   界面会自动列出来。 */
#define CHASSIS_PARAM(field, lo, hi, dec, flags)                                                   \
    {#field, &chassis_tune.field, &chassis_tune_default.field, lo, hi, dec, flags}
#define LIDAR_PARAM(field, lo, hi, dec)                                                            \
    {#field, &lidar_tune.field, &lidar_tune_default.field, lo, hi, dec, 0U}
#define ARM_PARAM(field, lo, hi, dec)                                                              \
    {"arm_" #field, &arm_tune.field, &arm_tune_default.field, lo, hi, dec, 0U}

static const Param params[] = {
    CHASSIS_PARAM(heading_kp, 0.0f, 20.0f, 3U, 0U),
    CHASSIS_PARAM(heading_ki, 0.0f, 20.0f, 3U, 0U),
    CHASSIS_PARAM(heading_i_max, 0.0f, 60.0f, 2U, 0U),
    CHASSIS_PARAM(heading_i_zone, 0.0f, 180.0f, 2U, 0U),
    CHASSIS_PARAM(heading_brake, 0.0f, 3000.0f, 0U, 0U),
    CHASSIS_PARAM(heading_ff_x, -30.0f, 30.0f, 2U, 0U),
    CHASSIS_PARAM(heading_ff_y, -30.0f, 30.0f, 2U, 0U),
    CHASSIS_PARAM(turn_max_speed, 1.0f, 240.0f, 1U, 0U),
    CHASSIS_PARAM(distance_kp, 0.1f, 10.0f, 3U, 0U),
    CHASSIS_PARAM(distance_tol, 0.5f, 50.0f, 2U, 0U),
    CHASSIS_PARAM(heading_tol, 0.01f, 10.0f, 3U, 0U),
    CHASSIS_PARAM(settle_ms, 0.0f, 2000.0f, 0U, 0U),
    CHASSIS_PARAM(acc_linear, 100.0f, 5000.0f, 0U, 0U),
    CHASSIS_PARAM(acc_angular, 30.0f, 3000.0f, 0U, 0U),
    CHASSIS_PARAM(cmd_speed, 10.0f, CHASSIS_MOVE_MAX_SPEED, 1U, 0U),
    CHASSIS_PARAM(wheel_dia, 40.0f, 150.0f, 2U, STOPPED_ONLY),
    CHASSIS_PARAM(chassis_k, 100.0f, 400.0f, 2U, STOPPED_ONLY),
    CHASSIS_PARAM(path_deviation, 0.5f, 50.0f, 2U, STOPPED_ONLY),
#if AGV_MODE == AGV_MODE_COMPETITION
    LIDAR_PARAM(lidar_x, -200.0f, 200.0f, 1U),
    LIDAR_PARAM(lidar_y, -200.0f, 200.0f, 1U),
    LIDAR_PARAM(lidar_yaw, -180.0f, 180.0f, 2U),
    LIDAR_PARAM(lidar_min, 50.0f, 500.0f, 0U),
    LIDAR_PARAM(lidar_max, 500.0f, 6000.0f, 0U),
    LIDAR_PARAM(obs_gap, 5.0f, 200.0f, 1U),
    LIDAR_PARAM(obs_width, 20.0f, 300.0f, 1U),
    LIDAR_PARAM(obs_match, 10.0f, 300.0f, 1U),
    LIDAR_PARAM(obs_confirm, 1.0f, 50.0f, 0U),
    LIDAR_PARAM(servo_scan, 0.0f, COMP_SERVO_RANGE_DEG, 1U),
    LIDAR_PARAM(servo_stow, 0.0f, COMP_SERVO_RANGE_DEG, 1U),
#elif AGV_MODE == AGV_MODE_ARM
    /* 机械臂自己处理参数变化（零点、方向改了从当前位置重新规划），不用停车才能改。 */
    ARM_PARAM(j1_x, -300.0f, 300.0f, 1U),
    ARM_PARAM(j1_y, -300.0f, 300.0f, 1U),
    ARM_PARAM(j2_x, -300.0f, 300.0f, 1U),
    ARM_PARAM(j2_y, -300.0f, 300.0f, 1U),
    ARM_PARAM(a1, 20.0f, 500.0f, 1U),
    ARM_PARAM(b1, 20.0f, 500.0f, 1U),
    ARM_PARAM(a2, 20.0f, 500.0f, 1U),
    ARM_PARAM(b2, 20.0f, 500.0f, 1U),
    ARM_PARAM(mode, 0.0f, 1.0f, 0U),
    ARM_PARAM(margin, 0.0f, 45.0f, 1U),
    ARM_PARAM(zero1, -180.0f, 180.0f, 2U),
    ARM_PARAM(zero2, -180.0f, 180.0f, 2U),
    ARM_PARAM(flip1, 0.0f, 1.0f, 0U),
    ARM_PARAM(flip2, 0.0f, 1.0f, 0U),
    ARM_PARAM(q1_min, -180.0f, 180.0f, 1U),
    ARM_PARAM(q1_max, -180.0f, 180.0f, 1U),
    ARM_PARAM(q2_min, -180.0f, 180.0f, 1U),
    ARM_PARAM(q2_max, -180.0f, 180.0f, 1U),
    ARM_PARAM(speed, 5.0f, 720.0f, 0U),
    ARM_PARAM(acc, 10.0f, 5000.0f, 0U),
    ARM_PARAM(cam, 0.0f, 2.0f, 0U),
    ARM_PARAM(cam_rot, -180.0f, 180.0f, 1U),
    ARM_PARAM(cam_x, -500.0f, 500.0f, 1U),
    ARM_PARAM(cam_y, -500.0f, 500.0f, 1U),
    ARM_PARAM(cam_lag, 0.0f, 300.0f, 0U),
#endif
};
#define PARAM_COUNT (sizeof(params) / sizeof(params[0]))

static uint8_t IsStopped(void)
{
    return chassis.mode == CHASSIS_MODE_VEL && chassis.target.vx == 0.0f &&
           chassis.target.vy == 0.0f && chassis.current.vx == 0.0f && chassis.current.vy == 0.0f;
}

/* 十进制小数，不支持指数、nan、inf。不用 strtof：它要占好几 KB Flash，而这里用不上那些写法。 */
static uint8_t ParseNumber(const char *text, float *out)
{
    uint8_t negative = *text == '-', dot = 0U;
    uint32_t digits = 0U, scale = 1U;
    unsigned count = 0U;
    if (*text == '-' || *text == '+')
        ++text;
    for (; *text; ++text)
    {
        if (*text == '.' && !dot)
        {
            dot = 1U;
            continue;
        }
        if (*text < '0' || *text > '9' || ++count > 9U)
            return 0;
        digits = digits * 10U + (uint32_t)(*text - '0');
        if (dot)
            scale *= 10U;
    }
    if (!count)
        return 0;
    float value = (float)digits / (float)scale;
    *out = negative ? -value : value;
    return 1;
}

static uint8_t ParseWhole(const char *text, float *out)
{
    return ParseNumber(text, out) && *out == floorf(*out);
}

static void CmdHelp(void)
{
    ChassisTelemetry_Puts("---- console: lines starting with / ----\r\n"
                          "  /list [prefix]  /set <name> <value>  /reset (all, while stopped)\r\n"
                          "  /plot <heading|speed|wheels> [1..100 Hz]  /plot off  /mode\r\n"
                          "  /log [on|off]  heading-loop log: last motion, or after every one\r\n"
#if AGV_MODE == AGV_MODE_COMPETITION
                          "  /start  /run  /task [<code>|clear]  /vision <mode 0..3> [color 0..6]\r\n"
                          "  /obs  /scan  /pose <x> <y> <heading>  /route [<x> <y>|go]\r\n"
#elif AGV_MODE == AGV_MODE_SURVEY
                          "  /route\r\n"
#elif AGV_MODE == AGV_MODE_ARM
                          "  /arm  /arm hold|free  /arm j <q1> <q2>  /arm xy <x> <y>\r\n"
                          "  /arm jog <j1|j2|x|y> <step>  /arm zero [<q1> <q2>]  /arm camdir\r\n"
                          "  /arm save|go|del <name>  /arm point <name> <q1> <q2>  /arm points\r\n"
                          "  /arm follow <red|yellow|blue|green|black|lightblue|off>\r\n"
#endif
    );
}

static void CmdList(const char *prefix)
{
    size_t n = strlen(prefix);
    unsigned shown = 0U;
    for (unsigned i = 0; i < PARAM_COUNT; ++i)
    {
        const Param *p = &params[i];
        if (strncmp(p->name, prefix, n) != 0)
            continue;
        ChassisTelemetry_Printf(
            "%-15s = %s  [%s..%s] default %s%s\r\n", p->name,
            ChassisTelemetry_Num(*p->value, p->decimals), ChassisTelemetry_Num(p->min, p->decimals),
            ChassisTelemetry_Num(p->max, p->decimals), ChassisTelemetry_Num(*p->def, p->decimals),
            (p->flags & STOPPED_ONLY) ? " (stopped only)" : "");
        ++shown;
    }
    if (!shown)
        ChassisTelemetry_Printf("ERR  no parameter starts with '%s'\r\n", prefix);
}

static void CmdSet(const char *name, const char *text)
{
    const Param *p = params;
    while (p < params + PARAM_COUNT && strcmp(p->name, name) != 0)
        ++p;
    float value;
    if (p == params + PARAM_COUNT)
        ChassisTelemetry_Printf("ERR  unknown parameter '%s' (/list)\r\n", name);
    else if (!ParseNumber(text, &value))
        ChassisTelemetry_Printf("ERR  %s: bad number '%s'\r\n", p->name, text);
    else if (value < p->min || value > p->max)
        ChassisTelemetry_Printf("ERR  %s must be %s..%s\r\n", p->name,
                                ChassisTelemetry_Num(p->min, p->decimals),
                                ChassisTelemetry_Num(p->max, p->decimals));
    else if ((p->flags & STOPPED_ONLY) && !IsStopped())
        ChassisTelemetry_Printf("BUSY %s can only change while stopped\r\n", p->name);
    else
    {
        /* 存进去的值按显示的小数位取整，看到的就是实际用的。 */
        float scale = 1.0f, old = *p->value;
        for (unsigned i = 0; i < p->decimals; ++i)
            scale *= 10.0f;
        *p->value = (float)lroundf(value * scale) / scale;
        ChassisTelemetry_Printf("OK   %s %s -> %s\r\n", p->name,
                                ChassisTelemetry_Num(old, p->decimals),
                                ChassisTelemetry_Num(*p->value, p->decimals));
    }
}

static void CmdReset(void)
{
    if (!IsStopped())
    {
        ChassisTelemetry_Printf("BUSY /reset can only run while stopped\r\n");
        return;
    }
    for (unsigned i = 0; i < PARAM_COUNT; ++i)
        *params[i].value = *params[i].def;
    ChassisTelemetry_Printf("OK   all parameters back to defaults\r\n");
}

static void CmdPlot(const char *name, const char *hz_text)
{
    /* 下标与 telemetry_plot 相同。 */
    static const char *const names[] = {"off", "heading", "speed", "wheels"};
    static const char *const columns[] = {
        "",
        "target,yaw,error,wz_target,wz",
        "vx_target,vy_target,vx,vy,vx_meas,vy_meas",
        "target_lf,target_rf,target_lr,target_rr,meas_lf,meas_rf,meas_lr,meas_rr",
    };
    unsigned group = 0U;
    float hz = 50.0f;
    while (group < 4U && strcmp(name, names[group]) != 0)
        ++group;
    if (group == 4U || (hz_text && (group == TELEMETRY_PLOT_OFF || !ParseWhole(hz_text, &hz) ||
                                    hz < 1.0f || hz > 100.0f)))
    {
        ChassisTelemetry_Printf("ERR  usage: /plot <heading|speed|wheels> [1..100], /plot off\r\n");
        return;
    }
    ChassisTelemetry_SetPlot((telemetry_plot)group, (unsigned)hz);
    if (group == TELEMETRY_PLOT_OFF)
        ChassisTelemetry_Printf("OK   plot off\r\n");
    else
        ChassisTelemetry_Printf("OK   plot %s %u Hz: %s\r\n", names[group], (unsigned)hz,
                                columns[group]);
}

/* 调参日志：/log 发最近一段，/log on|off 每段记完自动发（格式见 tune_log.h）。 */
static void CmdLog(const char *arg)
{
    if (!arg)
    {
        HAL_StatusTypeDef status = TuneLog_Send(); // 开始发了就不另外回复，日志本身就是回复
        if (status == HAL_ERROR)
            ChassisTelemetry_Printf("LOG none\r\n");
        else if (status == HAL_BUSY)
            ChassisTelemetry_Printf("OK   log after this motion\r\n");
    }
    else if (strcmp(arg, "on") == 0 || strcmp(arg, "off") == 0)
    {
        TuneLog_SetAuto(arg[1] == 'n');
        ChassisTelemetry_Printf("OK   log auto %s\r\n", arg);
    }
    else
        ChassisTelemetry_Printf("ERR  usage: /log [on|off]\r\n");
}

#if AGV_MODE == AGV_MODE_ARM
static void ArmBusy(void)
{
    static const char *const why[] = {"waiting for the motors", "settling, try again",
                                      "free, /arm hold first", "in fault, send S"};
    ChassisTelemetry_Printf("BUSY arm %s\r\n", arm.state == ARM_READY && arm.moving
                                                   ? "moving, wait for ARM done"
                                                   : why[arm.state]);
}

static void ArmLimits(void)
{
    ChassisTelemetry_Printf(
        "ERR  arm: q1 must be %s..%s, q2 %s..%s\r\n", ChassisTelemetry_Num(arm_tune.q1_min, 1),
        ChassisTelemetry_Num(arm_tune.q1_max, 1), ChassisTelemetry_Num(arm_tune.q2_min, 1),
        ChassisTelemetry_Num(arm_tune.q2_max, 1));
}

static void ArmMoved(arm_result result)
{
    if (result == ARM_BUSY)
        ArmBusy();
    else if (result == ARM_LIMIT)
        ArmLimits();
    else if (result == ARM_REACH)
        ChassisTelemetry_Printf("ERR  arm: out of reach (no pose there clears the dead points by "
                                "arm_margin)\r\n");
    else if (result == ARM_PATH)
        ChassisTelemetry_Printf("ERR  arm: the way there passes a dead point, go through another "
                                "point first\r\n");
    else
        ChassisTelemetry_Printf("OK   arm to q=%s,%s\r\n", ChassisTelemetry_Num(arm.goal[0], 2),
                                ChassisTelemetry_Num(arm.goal[1], 2));
}

static void ArmPoint(const arm_point *p)
{
    ChassisTelemetry_Printf("ARM point %s %s %s\r\n", p->name, ChassisTelemetry_Num(p->q[0], 2),
                            ChassisTelemetry_Num(p->q[1], 2));
}

/* 存点位：名字不对和超限位都是 ERROR，分开说。 */
static void ArmSavePoint(const char *name, float q1, float q2)
{
    HAL_StatusTypeDef result = Arm_SetPoint(name, q1, q2);
    if (result == HAL_OK)
        ArmPoint(Arm_FindPoint(name));
    else if (result == HAL_BUSY)
        ChassisTelemetry_Printf("BUSY arm: point table full (%u)\r\n", ARM_POINTS);
    else if (!Arm_ValidName(name))
        ChassisTelemetry_Printf(
            "ERR  arm: point name is 1..%u of a-z 0-9 _, starting with a letter\r\n", ARM_NAME_MAX);
    else
        ArmLimits();
}

/* 机械臂：标定、点动、点位、跟随（格式见 arm_app.h）。零点和相机方向通过 CmdSet 改参数，回复和
   /set 一样，调试台照常记下来。 */
static void CmdArm(unsigned argc, char *argv[])
{
    const char *sub = argc >= 2U ? argv[1] : "";
    float a = 0.0f, b = 0.0f;
    uint8_t pair = argc == 4U && ParseNumber(argv[2], &a) && ParseNumber(argv[3], &b);
    if (argc == 1U)
        ArmApp_PrintStatus();
    else if ((strcmp(sub, "hold") == 0 || strcmp(sub, "free") == 0) && argc == 2U)
    {
        if (Arm_SetFree(sub[0] == 'f') != HAL_OK)
            ArmBusy();
        else
            ChassisTelemetry_Printf("OK   arm %s\r\n",
                                    sub[0] == 'f' ? "free: move it by hand" : "hold");
    }
    else if (strcmp(sub, "j") == 0 && pair)
        ArmMoved(Arm_MoveJoints(a, b));
    else if (strcmp(sub, "xy") == 0 && pair)
        ArmMoved(Arm_MoveTo(a, b));
    else if (strcmp(sub, "jog") == 0 && argc == 4U && ParseNumber(argv[3], &b) &&
             (strcmp(argv[2], "j1") == 0 || strcmp(argv[2], "j2") == 0 ||
              strcmp(argv[2], "x") == 0 || strcmp(argv[2], "y") == 0))
    {
        /* 从目标算起：连着点几下会累加，不用等上一下走完。 */
        float x, y;
        if (argv[2][0] == 'j')
            ArmMoved(Arm_MoveJoints(arm.goal[0] + (argv[2][1] == '1' ? b : 0.0f),
                                    arm.goal[1] + (argv[2][1] == '2' ? b : 0.0f)));
        else if (!Arm_Forward(arm.goal[0], arm.goal[1], &x, &y))
            ArmMoved(ARM_REACH);
        else
            ArmMoved(
                Arm_MoveTo(x + (argv[2][0] == 'x' ? b : 0.0f), y + (argv[2][0] == 'y' ? b : 0.0f)));
    }
    else if (strcmp(sub, "zero") == 0 && (argc == 2U || pair))
    {
        float zero[2];
        HAL_StatusTypeDef result = Arm_ZeroFor(a, b, zero);
        if (result == HAL_BUSY)
            ArmBusy();
        else if (result != HAL_OK)
            ChassisTelemetry_Printf("ERR  arm: zero angles must be -180..180\r\n");
        else
        {
            CmdSet("arm_zero1", ChassisTelemetry_Num(zero[0], 2));
            CmdSet("arm_zero2", ChassisTelemetry_Num(zero[1], 2));
        }
    }
    else if (strcmp(sub, "camdir") == 0 && argc == 2U)
    {
        float deg;
        if (arm.state != ARM_READY && arm.state != ARM_FREE)
            ArmBusy();
        else if (!Arm_CameraDirection(&deg))
            ChassisTelemetry_Printf("ERR  arm: camera is not on a forearm (arm_cam 1 or 2)\r\n");
        else
            CmdSet("arm_cam_rot", ChassisTelemetry_Num(deg, 1));
    }
    else if (strcmp(sub, "follow") == 0 && argc == 3U &&
             (strcmp(argv[2], "off") == 0 || ArmApp_ParseColor(argv[2])))
    {
        uint8_t color = ArmApp_ParseColor(argv[2]);
        if (Arm_Follow(color) != HAL_OK)
            ArmBusy();
        else
            ChassisTelemetry_Printf("OK   arm follow %s\r\n",
                                    color ? ArmApp_ColorName(color) : "off");
    }
    else if (strcmp(sub, "points") == 0 && argc == 2U)
    {
        for (unsigned i = 0; i < Arm_PointCount(); ++i)
            ArmPoint(Arm_PointAt(i));
        ChassisTelemetry_Printf("ARM points %u\r\n", Arm_PointCount());
    }
    else if (strcmp(sub, "point") == 0 && argc == 5U && ParseNumber(argv[3], &a) &&
             ParseNumber(argv[4], &b))
        ArmSavePoint(argv[2], a, b);
    else if (strcmp(sub, "save") == 0 && argc == 3U)
    {
        if ((arm.state != ARM_READY && arm.state != ARM_FREE) || arm.moving)
            ArmBusy();
        else
            ArmSavePoint(argv[2], arm.q[0], arm.q[1]);
    }
    else if ((strcmp(sub, "go") == 0 || strcmp(sub, "del") == 0) && argc == 3U)
    {
        const arm_point *p = Arm_FindPoint(argv[2]);
        if (p == NULL)
            ChassisTelemetry_Printf("ERR  arm: no point '%s' (/arm points)\r\n", argv[2]);
        else if (sub[0] == 'g')
            ArmMoved(Arm_MoveJoints(p->q[0], p->q[1]));
        else
        {
            (void)Arm_DeletePoint(argv[2]);
            ChassisTelemetry_Printf("OK   arm point %s deleted\r\n", argv[2]);
        }
    }
    else
        ChassisTelemetry_Printf("ERR  usage: /arm [hold|free|points|camdir], /arm j|xy <a> <b>, "
                                "/arm jog j1|j2|x|y <step>, /arm zero [<q1> <q2>], "
                                "/arm save|go|del <name>, /arm point <name> <q1> <q2>, "
                                "/arm follow <color|off>\r\n");
}
#endif

#if AGV_MODE == AGV_MODE_COMPETITION
static void CmdVision(const char *mode_text, const char *color_text)
{
    float mode, color = 0.0f;
    if (!ParseWhole(mode_text, &mode) || mode < VISION_OFF || mode > VISION_RING ||
        (color_text &&
         (!ParseWhole(color_text, &color) || color < COLOR_ANY || color > COLOR_LIGHT_BLUE)))
    {
        ChassisTelemetry_Printf("ERR  usage: /vision <0..3> [color 0..6]\r\n");
        return;
    }
    Comp_SetVisionMode((VisionMode)mode, (uint8_t)color);
    ChassisTelemetry_Printf("OK   vision mode %u color %u\r\n", (unsigned)mode, (unsigned)color);
}

static void CmdTask(const char *arg)
{
    if (!arg)
        Comp_TaskReport();
    else if (strcmp(arg, "clear") == 0)
    {
        Comp_ClearTask();
        ChassisTelemetry_Printf("OK   task cleared, reading again\r\n");
    }
    else
        Comp_InjectTask(arg, (unsigned)strlen(arg));
}

static void CmdObs(void)
{
    MapPoint list[MAP_MAX_OBSTACLES];
    unsigned n = LidarObs_List(list, MAP_MAX_OBSTACLES);
    for (unsigned i = 0; i < n; ++i)
        ChassisTelemetry_Printf("  #%u (%ld,%ld)\r\n", i + 1U, (long)list[i].x, (long)list[i].y);
    ChassisTelemetry_Printf("OK   %u obstacles, scan %s\r\n", n,
                            comp.lidar_state == COMP_LIDAR_DONE     ? "done"
                            : comp.lidar_state == COMP_LIDAR_FAILED ? "failed"
                            : Comp_Running() || !IsStopped()        ? "running"
                                                                    : "idle");
}

static void CmdScan(void)
{
    if (!IsStopped() || Comp_Rescan() != HAL_OK)
        ChassisTelemetry_Printf("BUSY /scan needs the chassis stopped and the pose set\r\n");
    else
        ChassisTelemetry_Printf("OK   scan restarted\r\n");
}

static void CmdStart(void)
{
    if (!IsStopped() || Comp_Start() != HAL_OK)
        ChassisTelemetry_Printf("BUSY /start needs the chassis ready and stopped, pose set\r\n");
}

/* 流程在跑时只能看，不能改：规则禁止运行中遥控。S 不走这里，随时能停。 */
static uint8_t RunLocked(const char *cmd, unsigned argc)
{
    if (!Comp_Running())
        return 0;
    static const char *const allowed[] = {"help", "list", "mode", "plot", "log", "obs", "run"};
    for (unsigned i = 0; i < sizeof(allowed) / sizeof(allowed[0]); ++i)
        if (strcmp(cmd, allowed[i]) == 0)
            return 0;
    if ((strcmp(cmd, "task") == 0 || strcmp(cmd, "route") == 0) && argc == 1U) // 只是问一下
        return 0;
    ChassisTelemetry_Printf("BUSY the run is going, only S stops it\r\n");
    return 1;
}

static void CmdPose(const char *x_text, const char *y_text, const char *heading_text)
{
    float x, y, heading;
    if (!ParseNumber(x_text, &x) || !ParseNumber(y_text, &y) ||
        !ParseNumber(heading_text, &heading) || x < 0.0f || x > (float)MAP_WIDTH_MM || y < 0.0f ||
        y > (float)MAP_HEIGHT_MM || fabsf(heading) > 180.0f)
        ChassisTelemetry_Printf("ERR  usage: /pose <x 0..%d> <y 0..%d> <heading -180..180>\r\n",
                                MAP_WIDTH_MM, MAP_HEIGHT_MM);
    else if (!IsStopped() || Comp_SetPose(x, y, heading) != HAL_OK)
        ChassisTelemetry_Printf("BUSY chassis moving or not ready\r\n");
    else
        ChassisTelemetry_Printf("OK   pose (%s,%s,%s)\r\n", ChassisTelemetry_Num(x, 0),
                                ChassisTelemetry_Num(y, 0), ChassisTelemetry_Num(heading, 1));
}

static void CmdRoute(const char *first, const char *second)
{
    float x = 0.0f, y = 0.0f;
    uint8_t go = first && !second && strcmp(first, "go") == 0;
    HAL_StatusTypeDef result = HAL_OK;
    if (!first)
        Comp_RouteReport();
    else if (!go && (!second || !ParseWhole(first, &x) || !ParseWhole(second, &y) || x < 0.0f ||
                     x >= (float)MAP_WIDTH_MM || y < 0.0f || y >= (float)MAP_HEIGHT_MM))
        ChassisTelemetry_Printf("ERR  usage: /route [<x 0..%d> <y 0..%d> | go]\r\n",
                                MAP_WIDTH_MM - 1, MAP_HEIGHT_MM - 1);
    else if (!IsStopped())
        result = HAL_BUSY;
    else
        result = go ? Comp_RouteGo() : Comp_RoutePlan((int32_t)x, (int32_t)y);
    if (result == HAL_BUSY)
        ChassisTelemetry_Printf("BUSY /route needs the chassis ready and stopped, pose set\r\n");
    else if (result != HAL_OK)
        ChassisTelemetry_Printf("ERR  no route ready, plan one first\r\n");
}
#endif

/* 就地按空格切分并转成小写；参数比 CONSOLE_ARGS 还多时返回 CONSOLE_ARGS + 1，当成用法错误。 */
static unsigned SplitLine(char *text, char *argv[CONSOLE_ARGS])
{
    unsigned argc = 0U;
    while (*text)
    {
        while (*text == ' ' || *text == '\t')
            ++text;
        if (!*text)
            break;
        if (argc == CONSOLE_ARGS)
            return CONSOLE_ARGS + 1U;
        argv[argc++] = text;
        while (*text && *text != ' ' && *text != '\t')
        {
            if (*text >= 'A' && *text <= 'Z')
                *text = (char)(*text - 'A' + 'a');
            ++text;
        }
        if (*text)
            *text++ = '\0';
    }
    return argc;
}

void Console_Line(char *line)
{
    char *argv[CONSOLE_ARGS];
    unsigned argc = SplitLine(line, argv);
    const char *cmd = argc ? argv[0] : "help";

#if AGV_MODE == AGV_MODE_COMPETITION
    if (RunLocked(cmd, argc))
        return;
#endif
    if (strcmp(cmd, "help") == 0 && argc <= 1U)
        CmdHelp();
    else if (strcmp(cmd, "list") == 0 && argc <= 2U)
        CmdList(argc == 2U ? argv[1] : "");
    else if (strcmp(cmd, "set") == 0 && argc == 3U)
        CmdSet(argv[1], argv[2]);
    else if (strcmp(cmd, "reset") == 0 && argc == 1U)
        CmdReset();
    else if (strcmp(cmd, "plot") == 0 && (argc == 2U || argc == 3U))
        CmdPlot(argv[1], argc == 3U ? argv[2] : NULL);
    else if (strcmp(cmd, "log") == 0 && argc <= 2U)
        CmdLog(argc == 2U ? argv[1] : NULL);
    else if (strcmp(cmd, "mode") == 0 && argc == 1U)
        ChassisTelemetry_Printf("OK   mode %s\r\n", AGV_MODE_NAME);
#if AGV_MODE == AGV_MODE_COMPETITION
    else if (strcmp(cmd, "task") == 0 && argc <= 2U)
        CmdTask(argc == 2U ? argv[1] : NULL);
    else if (strcmp(cmd, "vision") == 0 && (argc == 2U || argc == 3U))
        CmdVision(argv[1], argc == 3U ? argv[2] : NULL);
    else if (strcmp(cmd, "obs") == 0 && argc == 1U)
        CmdObs();
    else if (strcmp(cmd, "scan") == 0 && argc == 1U)
        CmdScan();
    else if (strcmp(cmd, "start") == 0 && argc == 1U)
        CmdStart();
    else if (strcmp(cmd, "run") == 0 && argc == 1U)
        Comp_RunReport();
    else if (strcmp(cmd, "pose") == 0 && argc == 4U)
        CmdPose(argv[1], argv[2], argv[3]);
    else if (strcmp(cmd, "route") == 0 && argc <= 3U)
        CmdRoute(argc >= 2U ? argv[1] : NULL, argc == 3U ? argv[2] : NULL);
#elif AGV_MODE == AGV_MODE_SURVEY
    else if (strcmp(cmd, "route") == 0 && argc == 1U)
        Agv_RouteReport();
#elif AGV_MODE == AGV_MODE_ARM
    else if (strcmp(cmd, "arm") == 0)
        CmdArm(argc, argv);
#endif
    else
        ChassisTelemetry_Printf("ERR  bad console command, /help lists them\r\n");
}
