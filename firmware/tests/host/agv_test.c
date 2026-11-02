#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "chassis.h"
#include "chassis_telemetry.h"
#include "agv_run.h"
#include "agv_config.h"
#include "map.h"
#include "user_math.h"

chassis_t chassis;
chassis_path_stats chassis_path_result;

/* 桩按起始航向 37° 把底盘系航点换算回场地坐标，用来独立验证覆盖与回库位置。 */
static const float kStartYaw = 37.0f;
static const float kAxis[3] = {355.0f, 1205.0f, 2055.0f};

static uint32_t now_ms;
static uint16_t physical_edges, test_zone;
static unsigned follow_calls, freeze, busy_left;

static chassis_waypoint s_tour[CHASSIS_PATH_CAPACITY];
static unsigned s_tour_count, s_tour_index;
static float s_from_x, s_from_y, s_from_heading;

/* 底盘系 -> 场地坐标；Agv 的 origin 固定取测试起始位姿 (0, 0, 37°)。 */
static void ToField(float x, float y, float *fx, float *fy)
{
    float a = kStartYaw * MATH_DEG_TO_RAD;
    float c = cosf(a), s = sinf(a);
    MapPoint home;
    (void)Map_StartPoint((uint8_t)test_zone, &home);
    *fx = (float)home.x + c * x + s * y;
    *fy = (float)home.y - s * x + c * y;
}

/* 独立的覆盖判据：一段直线要跨过通道中点才算走过该通道。 */
static void TraceSegment(float x0, float y0, float x1, float y1)
{
    for (unsigned row = 0; row < 3; ++row)
        for (unsigned col = 0; col < 2; ++col)
        {
            float mid = (kAxis[col] + kAxis[col + 1]) * 0.5f;
            if (fabsf(y0 - kAxis[row]) < 150.0f && fabsf(y1 - kAxis[row]) < 150.0f &&
                fminf(x0, x1) < mid && fmaxf(x0, x1) > mid)
                physical_edges |= (uint16_t)(1U << (row * 2U + col));
        }
    for (unsigned col = 0; col < 3; ++col)
        for (unsigned row = 0; row < 2; ++row)
        {
            float mid = (kAxis[row] + kAxis[row + 1]) * 0.5f;
            if (fabsf(x0 - kAxis[col]) < 150.0f && fabsf(x1 - kAxis[col]) < 150.0f &&
                fminf(y0, y1) < mid && fmaxf(y0, y1) > mid)
                physical_edges |= (uint16_t)(1U << (6U + col * 2U + row));
        }
}

/* 连续路径桩：一次整圈只调用一次，后续由 tick 推进。 */
HAL_StatusTypeDef Chassis_FollowPath(const chassis_waypoint *points, unsigned count,
                                     uint8_t stop_each)
{
    (void)stop_each;
    if (busy_left)
    {
        --busy_left;
        return HAL_BUSY;
    }
    if (chassis.state != CHASSIS_STATE_READY || chassis.mode != CHASSIS_MODE_VEL)
        return HAL_BUSY;
    if (!points || !count || count > CHASSIS_PATH_CAPACITY)
        return HAL_ERROR;
    memcpy(s_tour, points, count * sizeof(*points));
    s_tour_count = count;
    s_tour_index = 0;
    s_from_x = chassis.pos_x;
    s_from_y = chassis.pos_y;
    s_from_heading = chassis.yaw;
    chassis_path_result = (chassis_path_stats){0};
    chassis.mode = CHASSIS_MODE_PATH;
    ++follow_calls;
    return HAL_OK;
}

void Chassis_Stop(void)
{
    chassis.mode = CHASSIS_MODE_VEL;
}

void Chassis_SetMoveSpeedLimit(float mm_per_second)
{
    (void)mm_per_second;
}

uint32_t HAL_GetTick(void)
{
    return now_ms;
}

static unsigned route_count;
static telemetry_route_point route_point;

void ChassisTelemetry_SendRoute(unsigned count, telemetry_route_point point)
{
    route_count = count;
    route_point = point;
}

void ChassisTelemetry_Puts(const char *text)
{
    (void)text;
}

/* 每个节拍沿折线推进 120 mm，到点立刻切下一段；走完置 completed。 */
static void Advance(float step)
{
    while (s_tour_index < s_tour_count)
    {
        chassis_waypoint goal = s_tour[s_tour_index];
        float dx = goal.x - chassis.pos_x, dy = goal.y - chassis.pos_y;
        float length = hypotf(dx, dy);
        if (length > step)
        {
            float ratio = step / length;
            chassis.pos_x += dx * ratio;
            chassis.pos_y += dy * ratio;
            chassis.yaw =
                Math_WrapDeg(s_from_heading + Math_WrapDeg(goal.heading - s_from_heading) * ratio);
            chassis_path_result.distance_mm += step;
            step = 0.0f;
            break;
        }
        float fx0, fy0, fx1, fy1;
        ToField(s_from_x, s_from_y, &fx0, &fy0);
        ToField(goal.x, goal.y, &fx1, &fy1);
        TraceSegment(fx0, fy0, fx1, fy1);
        chassis_path_result.distance_mm += length;
        chassis.pos_x = goal.x;
        chassis.pos_y = goal.y;
        chassis.yaw = Math_WrapDeg(goal.heading);
        s_from_x = goal.x;
        s_from_y = goal.y;
        s_from_heading = goal.heading;
        ++s_tour_index;
        step -= length;
    }
    if (s_tour_index >= s_tour_count)
    {
        chassis.mode = CHASSIS_MODE_VEL;
        chassis_path_result.completed = 1;
    }
    chassis_path_result.elapsed_ms = now_ms;
}

static void tick(void)
{
    now_ms += 5U;
    if (!freeze && chassis.mode == CHASSIS_MODE_PATH)
        Advance(120.0f);
    Agv_Tick();
}

/* 复位模拟底盘到起始位姿，清空计数与覆盖；后续单点用例默认用 1 号出发区。 */
static void Reset(void)
{
    chassis = (chassis_t){0};
    chassis.state = CHASSIS_STATE_READY;
    chassis.yaw = kStartYaw;
    test_zone = 1U;
    physical_edges = 0;
    follow_calls = 0;
    route_count = 0;
    freeze = 0;
    busy_left = 0;
    now_ms = 0;
}

/* 把底盘里程计挪到指定场地坐标（起始位姿在原点），用于改变位姿检查的余量。 */
static void MoveToField(float fx, float fy)
{
    float a = kStartYaw * MATH_DEG_TO_RAD;
    float c = cosf(a), s = sinf(a);
    MapPoint home;
    (void)Map_StartPoint((uint8_t)test_zone, &home);
    float dx = fx - (float)home.x, dy = fy - (float)home.y;
    chassis.pos_x = c * dx - s * dy;
    chassis.pos_y = s * dx + c * dy;
}

static void Run(unsigned zone, unsigned limit)
{
    test_zone = (uint16_t)zone;
    Agv_Init(AGV_MODE_SURVEY, zone);
    for (unsigned n = 0; n < limit && agv.state != AGV_DONE && agv.state != AGV_FAULT; ++n)
        tick();
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    /* 整圈连续执行：8 圈巡检、12 条通道全覆盖、回到出发点。 */
    for (unsigned zone = 1; zone <= 2; ++zone)
    {
        Reset();
        Run(zone, 200000);
        if (agv.state != AGV_DONE)
            printf("zone %u state=%d error=%d leg=%u x=%f y=%f residual=%f\n", zone, (int)agv.state,
                   (int)agv.error, (unsigned)agv.leg, agv.x, agv.y, agv.home_residual_mm);
        assert(agv.state == AGV_DONE && agv.covered == 0x0fffU);
        assert(physical_edges == 0x0fffU);
        assert(fabsf(agv.x - 2250.0f) < 5.0f);
        assert(fabsf(agv.y - (zone == 1 ? 2250.0f : 150.0f)) < 5.0f);
        assert(agv.home_residual_mm < 5.0f);
        assert(follow_calls == 1U);
        /* 发给调试台的路线是场地坐标：底盘系航向 37° 也要换回来，和通道中心线对得上。 */
        assert(route_count == s_tour_count + 1U);
        MapPoint first = route_point(0U), last = route_point(route_count - 1U);
        assert(first.x == 2250 && first.y == (zone == 1 ? 2250 : 150));
        assert(last.x == first.x && last.y == first.y);
        for (unsigned i = 1; i < route_count; ++i)
        {
            MapPoint a = route_point(i - 1U), b = route_point(i);
            assert(a.x == b.x || a.y == b.y);
        }
    }
    /* 停车锁存：不再下发任何路径。 */
    Reset();
    Agv_Init(AGV_MODE_SURVEY, 1);
    Agv_Stop();
    for (unsigned n = 0; n < 1000; ++n)
        tick();
    assert(agv.state == AGV_STOPPED && follow_calls == 0U);
    /* 非巡检模式只置空闲，不产生运动。 */
    Reset();
    Agv_Init(AGV_MODE_COMPETITION, 1);
    for (unsigned n = 0; n < 1000; ++n)
        tick();
    assert(agv.state == AGV_IDLE && follow_calls == 0U);
    /* 出发区被占：READY 后第一次位姿检查就失败，不会启动路径。 */
    Reset();
    Agv_Init(AGV_MODE_SURVEY, 1);
    assert(Map_AddObstacle(2250, 2250, 25));
    for (unsigned n = 0; n < 2000 && agv.state != AGV_FAULT; ++n)
        tick();
    assert(agv.state == AGV_FAULT && agv.error == AGV_ERR_POSE && follow_calls == 0U);
    /* 底盘一直没 READY：超时进 AGV_ERR_READY。 */
    Reset();
    chassis.state = CHASSIS_STATE_IDLE;
    Agv_Init(AGV_MODE_SURVEY, 1);
    for (unsigned n = 0; n < 4000; ++n)
        tick();
    assert(agv.state == AGV_FAULT && agv.error == AGV_ERR_READY && follow_calls == 0U);
    /* 底盘故障不能被当成走完。 */
    Reset();
    Agv_Init(AGV_MODE_SURVEY, 1);
    for (unsigned n = 0; n < 1000 && agv.state == AGV_WAIT; ++n)
        tick();
    chassis.state = CHASSIS_STATE_FAULT;
    tick();
    assert(agv.state == AGV_FAULT && agv.error == AGV_ERR_CHASSIS && agv.covered == 0U);
    /* 通道被堵：规划失败，不能抄近路。 */
    Reset();
    Agv_Init(AGV_MODE_SURVEY, 1);
    assert(Map_AddObstacle(1605, 2055, 200));
    for (unsigned n = 0; n < 20000 && agv.state != AGV_FAULT; ++n)
        tick();
    assert(agv.state == AGV_FAULT && agv.covered != 0x0fffU && follow_calls == 0U);
    /* 地图在规划之后变化：已规划的路径作废。 */
    Reset();
    Agv_Init(AGV_MODE_SURVEY, 1);
    for (unsigned n = 0; n < 1000 && agv.state == AGV_WAIT; ++n)
        tick();
    assert(Map_AddObstacle(1200, 1200, 25));
    tick();
    assert(agv.state == AGV_FAULT && agv.error == AGV_ERR_MAP);
    /* 航向要持续超限才判故障，瞬时尖峰可容忍。
       让 FollowPath 一直 BUSY 把车停在入库前，保持在会做航向判据的分支里；
       同时把车挪到场地中部，留出 3° 的旋转余量。 */
    Reset();
    busy_left = 100000U;
    Agv_Init(AGV_MODE_SURVEY, 1);
    for (unsigned n = 0; n < 1000 && agv.state == AGV_WAIT; ++n)
        tick();
    MoveToField(1205.0f, 1205.0f);
    for (unsigned n = 0; n < 1000 && agv.state != AGV_DOCK_IN; ++n)
        tick();
    assert(agv.state == AGV_DOCK_IN);
    chassis.yaw = kStartYaw + 0.5f;
    tick();
    assert(agv.state != AGV_FAULT);
    chassis.yaw = kStartYaw + 2.9f;
    for (unsigned n = 0; n < 60; ++n)
        tick();
    assert(agv.state != AGV_FAULT);
    chassis.yaw = kStartYaw + 3.1f;
    for (unsigned n = 0; n < 30; ++n)
        tick();
    assert(agv.state != AGV_FAULT);
    chassis.yaw = kStartYaw;
    tick();
    chassis.yaw = kStartYaw + 3.1f;
    for (unsigned n = 0; n < 50 && agv.state != AGV_FAULT; ++n)
        tick();
    assert(agv.state == AGV_FAULT && agv.error == AGV_ERR_POSE);
    /* 规划途中车身擦出场地：物理间隙不足要判故障。 */
    Reset();
    Agv_Init(AGV_MODE_SURVEY, 1);
    for (unsigned n = 0; n < 2000 && agv.state != AGV_DRIVE; ++n)
        tick();
    assert(agv.state == AGV_DRIVE);
    MoveToField(1990.0f, 1990.0f);
    tick();
    assert(agv.state == AGV_FAULT && agv.error == AGV_ERR_POSE);
    /* 路径迟迟走不完：按整圈航程估算的总超时进 AGV_ERR_TIMEOUT。 */
    Reset();
    Agv_Init(AGV_MODE_SURVEY, 1);
    freeze = 1;
    for (unsigned n = 0; n < 200000 && agv.state != AGV_FAULT; ++n)
        tick();
    assert(agv.state == AGV_FAULT && agv.error == AGV_ERR_TIMEOUT);
    freeze = 0;
    /* FollowPath 短暂 BUSY 必须重试，而不是整趟作废。 */
    Reset();
    busy_left = 100;
    Run(1, 200000);
    assert(agv.state == AGV_DONE && follow_calls == 1U && busy_left == 0U);
    /* FollowPath 一直 BUSY：15 s 后报 AGV_ERR_READY。 */
    Reset();
    busy_left = 100000;
    Run(1, 20000);
    assert(agv.state == AGV_FAULT && agv.error == AGV_ERR_READY && follow_calls == 0U);
    puts("runner both docks, continuous tour coverage, stop latch, BUSY retry, timeouts OK");
    return 0;
}
