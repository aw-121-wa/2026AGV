#include "console.h"
#include "agv_config.h"
#include "chassis.h"
#include "chassis_telemetry.h"
#include <math.h>
#include <stddef.h>
#include <string.h>
#if AGV_MODE == AGV_MODE_COMPETITION
#include "competition.h"
#endif

#define CONSOLE_ARGS 4U // 最长的命令 /pose x y heading
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

static const Param params[] = {
    CHASSIS_PARAM(heading_kp, 0.0f, 20.0f, 3U, 0U),
    CHASSIS_PARAM(heading_ki, 0.0f, 20.0f, 3U, 0U),
    CHASSIS_PARAM(heading_i_max, 0.0f, 60.0f, 2U, 0U),
    CHASSIS_PARAM(heading_i_zone, 0.0f, 180.0f, 2U, 0U),
    CHASSIS_PARAM(turn_max_speed, 1.0f, 240.0f, 1U, 0U),
    CHASSIS_PARAM(distance_kp, 0.1f, 10.0f, 3U, 0U),
    CHASSIS_PARAM(distance_tol, 0.5f, 50.0f, 2U, 0U),
    CHASSIS_PARAM(heading_tol, 0.01f, 10.0f, 3U, 0U),
    CHASSIS_PARAM(settle_ms, 0.0f, 2000.0f, 0U, 0U),
    CHASSIS_PARAM(acc_linear, 100.0f, 5000.0f, 0U, 0U),
    CHASSIS_PARAM(acc_angular, 30.0f, 3000.0f, 0U, 0U),
    CHASSIS_PARAM(cmd_speed, 10.0f, 400.0f, 1U, 0U),
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
                          "  /plot <heading|speed|wheels> [1..100 Hz]  /plot off\r\n"
#if AGV_MODE == AGV_MODE_COMPETITION
                          "  /task <code>  /vision <mode 0..3> [color 0..6]  /obs [clear]\r\n"
                          "  /pose <x> <y> <heading>\r\n"
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

static void CmdObs(const char *action)
{
    if (!action)
    {
        MapPoint list[MAP_MAX_OBSTACLES];
        unsigned n = LidarObs_List(list, MAP_MAX_OBSTACLES);
        for (unsigned i = 0; i < n; ++i)
            ChassisTelemetry_Printf("  #%u (%ld,%ld)\r\n", i + 1U, (long)list[i].x,
                                    (long)list[i].y);
        ChassisTelemetry_Printf("OK   %u obstacles\r\n", n);
    }
    else if (strcmp(action, "clear") != 0)
        ChassisTelemetry_Printf("ERR  usage: /obs [clear]\r\n");
    else if (!IsStopped())
        ChassisTelemetry_Printf("BUSY /obs clear can only run while stopped\r\n");
    else
    {
        Comp_ClearObstacles();
        ChassisTelemetry_Printf("OK   obstacles cleared\r\n");
    }
}

static void CmdPose(const char *x_text, const char *y_text, const char *heading_text)
{
    float x, y, heading;
    if (!ParseNumber(x_text, &x) || !ParseNumber(y_text, &y) ||
        !ParseNumber(heading_text, &heading) || x < 0.0f || x > (float)MAP_WIDTH_MM || y < 0.0f ||
        y > (float)MAP_HEIGHT_MM || fabsf(heading) > 180.0f)
        ChassisTelemetry_Printf("ERR  usage: /pose <x 0..%d> <y 0..%d> <heading -180..180>\r\n",
                                MAP_WIDTH_MM, MAP_HEIGHT_MM);
    else if (Comp_SetPose(x, y, heading) != HAL_OK)
        ChassisTelemetry_Printf("BUSY chassis not ready, heading unknown\r\n");
    else
        ChassisTelemetry_Printf("OK   pose (%s,%s,%s)\r\n", ChassisTelemetry_Num(x, 0),
                                ChassisTelemetry_Num(y, 0), ChassisTelemetry_Num(heading, 1));
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
#if AGV_MODE == AGV_MODE_COMPETITION
    else if (strcmp(cmd, "task") == 0 && argc == 2U)
        Comp_InjectTask(argv[1], (unsigned)strlen(argv[1]));
    else if (strcmp(cmd, "vision") == 0 && (argc == 2U || argc == 3U))
        CmdVision(argv[1], argc == 3U ? argv[2] : NULL);
    else if (strcmp(cmd, "obs") == 0 && argc <= 2U)
        CmdObs(argc == 2U ? argv[1] : NULL);
    else if (strcmp(cmd, "pose") == 0 && argc == 4U)
        CmdPose(argv[1], argv[2], argv[3]);
#endif
    else
        ChassisTelemetry_Printf("ERR  bad console command, /help lists them\r\n");
}
