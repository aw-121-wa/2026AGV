#include "arm_app.h"
#if AGV_MODE == AGV_MODE_ARM
#include "arm.h"
#include "chassis_telemetry.h"
#include "ser_port.h"
#include "vision_link.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define ARM_VISION_BAUD       115200U
#define ARM_VISION_TIMEOUT_MS 500U // 这么久没收到上位机的帧判离线（同比赛模式）
#define ARM_MODE_PERIOD_MS    200U // MODE 帧 5 Hz（同比赛模式）
#define ARM_MATERIAL_FRESH_MS 500U // 状态行只列这么久以内看到的物料

static const char *const state_names[] = {"WAIT", "READY", "FREE", "FAULT"};
static const char *const color_names[] = {"any",   "red",   "yellow",   "blue",
                                          "green", "black", "lightblue"};

static uint8_t shown_state = 0xFFU, shown_detail, shown_follow, shown_seen, shown_reach;
static uint8_t shown_side = 0xFFU;
static uint8_t vision_online, mode_color = 0xFFU;
static uint32_t shown_moves, material_ms, mode_ms;

const char *ArmApp_ColorName(uint8_t color)
{
    return color <= COLOR_LIGHT_BLUE ? color_names[color] : "?";
}

uint8_t ArmApp_ParseColor(const char *text)
{
    for (uint8_t c = COLOR_RED; c <= COLOR_LIGHT_BLUE; ++c)
        if (strcmp(text, color_names[c]) == 0 || (text[0] == (char)('0' + c) && text[1] == '\0'))
            return c;
    return 0U;
}

HAL_StatusTypeDef ArmApp_Init(void)
{
    VisionLink_Init();
    if (Arm_Init() != HAL_OK)
        return HAL_ERROR;
    return SerPort_Init(SER_VISION, ARM_VISION_BAUD);
}

static void VisionTick(uint32_t now)
{
    uint8_t buf[128], frame[VL_FRAME_MAX];
    unsigned n;
    while ((n = SerPort_Read(SER_VISION, buf, sizeof(buf))) > 0U)
        for (unsigned i = 0; i < n; ++i)
            (void)VisionLink_Feed(buf[i], now);
    if (vision.material_ms != material_ms)
    {
        material_ms = vision.material_ms;
        Arm_SeeMaterials(vision.material, vision.material_count, material_ms);
    }
    uint8_t online = VisionLink_Online(now, ARM_VISION_TIMEOUT_MS);
    if (online != vision_online)
    {
        vision_online = online;
        ChassisTelemetry_Printf("ARM vision %s\r\n", online ? "ONLINE" : "OFFLINE");
    }
    if (now - mode_ms >= ARM_MODE_PERIOD_MS || arm.follow != mode_color) // 换颜色马上告诉上位机
    {
        mode_ms = now;
        mode_color = arm.follow;
        (void)SerPort_Write(SER_VISION, frame,
                            VisionLink_EncodeMode(VISION_MATERIAL, arm.follow, frame));
    }
}

static const char *FollowText(void)
{
    static char text[24];
    if (!arm.follow)
        return "off";
    (void)snprintf(text, sizeof(text), "%s:%s", color_names[arm.follow],
                   !arm.seen   ? "looking"
                   : arm.reach ? "tracking"
                               : "far");
    return text;
}

/* 等待/故障原因，例如 " | fault: J2 stall (send S)"；READY、FREE 是空串。 */
static const char *StateTail(void)
{
    static char text[96];
    static const char *const what[] = {"no feedback", "not enabled", "stall"};
    text[0] = '\0';
    if (arm.state != ARM_WAIT && arm.state != ARM_FAULT)
        return text;
    size_t n =
        (size_t)snprintf(text, sizeof(text), " | %s:", arm.state == ARM_WAIT ? "waiting" : "fault");
    for (unsigned k = 0; k < 3U; ++k)
        for (unsigned i = 0; i < 2U; ++i)
            if ((arm.detail >> (2U * k + i)) & 1U && n < sizeof(text))
                n += (size_t)snprintf(text + n, sizeof(text) - n, " J%u %s", i + 1U, what[k]);
    if (arm.state == ARM_FAULT && n < sizeof(text))
        (void)snprintf(text + n, sizeof(text) - n, " (send S)");
    return text;
}

void ArmApp_PrintStatus(void)
{
    const char *(*num)(float, unsigned) = ChassisTelemetry_Num;
    ChassisTelemetry_Printf(
        "          ARM %s q=%s,%s xy=%s,%s goal=%s,%s move=%u side=%u follow=%s%s\r\n",
        state_names[arm.state], num(arm.q[0], 2), num(arm.q[1], 2), num(arm.x, 1), num(arm.y, 1),
        num(arm.goal[0], 2), num(arm.goal[1], 2), (unsigned)arm.moving, (unsigned)arm.side,
        FollowText(), StateTail());
    unsigned count =
        vision.material_ms && HAL_GetTick() - vision.material_ms <= ARM_MATERIAL_FRESH_MS
            ? vision.material_count
            : 0U;
    ChassisTelemetry_Printf("          VIS %s n=%u", vision_online ? "ON" : "OFF", count);
    for (unsigned i = 0; i < count; ++i)
    {
        float x, y;
        if (!Arm_CameraToBody(vision.material[i].fwd_mm, vision.material[i].left_mm, &x, &y))
            x = y = NAN; // 打成 nan，调试台不画
        ChassisTelemetry_Printf(" %s:%s,%s", ArmApp_ColorName(vision.material[i].color), num(x, 1),
                                num(y, 1));
    }
    ChassisTelemetry_Printf("\r\n");
}

static void PrintEvents(void)
{
    if (arm.state != shown_state || arm.detail != shown_detail)
    {
        shown_state = arm.state;
        shown_detail = arm.detail;
        ChassisTelemetry_Printf("ARM state %s%s\r\n", state_names[arm.state], StateTail());
    }
    if (arm.side != shown_side) // 过死点那一拍就打，调试台的俯视图马上换边
    {
        shown_side = arm.side;
        ChassisTelemetry_Printf("ARM side %u\r\n", (unsigned)arm.side);
    }
    if (arm.moves != shown_moves)
    {
        float x, y; // 规划走完的地方；实测还要再过一两拍才追上
        if (!Arm_Forward(arm.goal[0], arm.goal[1], &x, &y))
            x = y = NAN;
        shown_moves = arm.moves;
        ChassisTelemetry_Printf("ARM done q=%s,%s xy=%s,%s\r\n",
                                ChassisTelemetry_Num(arm.goal[0], 2),
                                ChassisTelemetry_Num(arm.goal[1], 2), ChassisTelemetry_Num(x, 1),
                                ChassisTelemetry_Num(y, 1));
    }
    if (arm.follow != shown_follow || arm.seen != shown_seen || arm.reach != shown_reach)
    {
        shown_follow = arm.follow;
        shown_seen = arm.seen;
        shown_reach = arm.reach;
        ChassisTelemetry_Printf("ARM follow %s\r\n", FollowText());
    }
}

void ArmApp_Tick(void)
{
    VisionTick(HAL_GetTick());
    Arm_Tick();
    PrintEvents();
}
#endif
