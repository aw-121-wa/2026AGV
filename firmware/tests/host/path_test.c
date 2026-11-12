#define main chassis_existing_test_main
#include "chassis_test.c"
#undef main

/* 拐角限速只管拐角本身：1000 mm 直线接 90° 弯，限速 300 mm/s 时直线中段要跑满，入弯前刹到
   拐角速度（上限由 CHASSIS_PATH_CORNER_FACTOR 决定，见 chassis.h），弯内明显低于直线速度，
   轨迹仍在容差内。理想用时约 6 s。 */
static void TestStraightBeforeCornerRunsAtLimit(void)
{
    Chassis_SetMoveSpeedLimit(300);
    float x0 = chassis.pos_x, y0 = chassis.pos_y, h = chassis.yaw;
    chassis_waypoint path[] = {{x0 + 1000, y0, h}, {x0 + 1000, y0 + 500, h}};
    assert(Chassis_FollowPath(path, 2, 0) == HAL_OK);
    uint32_t start = ms;
    float mid_speed = 0, corner_speed = 0, max_error = 0;
    for (unsigned i = 0; i < 10000 && chassis.mode == CHASSIS_MODE_PATH; ++i)
    {
        plant();
        float x = chassis.pos_x - x0, y = chassis.pos_y - y0;
        float v = hypotf(chassis.current.vx, chassis.current.vy);
        if (x > 490 && x < 510)
            mid_speed = v;
        if (hypotf(x - 1000, y) < 8)
            corner_speed = fmaxf(corner_speed, v);
        if (x > 900 || y > 0)
            max_error = fmaxf(max_error, fminf(fabsf(y), fabsf(x - 1000)));
    }
    printf("straight then corner: mid=%f corner=%f deviation=%f time=%u ms\n", mid_speed,
           corner_speed, max_error, (unsigned)(ms - start));
    assert(chassis_path_result.completed);
    assert(mid_speed > 295);
    assert(corner_speed < mid_speed * 0.5f);
    assert(max_error <= CHASSIS_PATH_DEVIATION_MM);
    assert(ms - start < 8000);
}

/* 两个拐角之间的直线也一样，后一个拐角不能压前面整段直线的速度。 */
static void TestStraightBetweenCornersRunsAtLimit(void)
{
    Chassis_SetMoveSpeedLimit(300);
    float x0 = chassis.pos_x, y0 = chassis.pos_y, h = chassis.yaw;
    chassis_waypoint path[] = {{x0 + 300, y0, h}, {x0 + 300, y0 + 1000, h}, {x0, y0 + 1000, h}};
    assert(Chassis_FollowPath(path, 3, 0) == HAL_OK);
    float mid_speed = 0;
    for (unsigned i = 0; i < 10000 && chassis.mode == CHASSIS_MODE_PATH; ++i)
    {
        plant();
        float y = chassis.pos_y - y0;
        if (y > 490 && y < 510)
            mid_speed = hypotf(chassis.current.vx, chassis.current.vy);
    }
    assert(chassis_path_result.completed);
    assert(mid_speed > 295);
}

/* 路线后面的一段正好压在当前拐角的内侧：偏线只能量正在走的那段，不能把后面那段认成当前段，
   否则真正的下一段被排除在外，机器人一离开就误报偏线（巡检回库段和第一段重合时就是这样）。 */
static void TestCrossTrackStaysOnCurrentLeg(void)
{
    Chassis_SetMoveSpeedLimit(300);
    float x = chassis.pos_x, y = chassis.pos_y, h = chassis.yaw;
    chassis_waypoint path[] = {{x + 200, y, h},       {x + 200, y + 200, h}, {x + 300, y + 200, h},
                               {x + 400, y + 200, h}, {x + 500, y + 200, h}, {x + 500, y + 100, h},
                               {x + 150, y + 50, h},  {x + 250, y - 50, h}};
    assert(Chassis_FollowPath(path, 8, 0) == HAL_OK);
    complete();
    printf("later leg over the corner: max_cross=%f\n", chassis_path_result.max_cross_mm);
    assert(chassis_path_result.completed);
    assert(chassis_path_result.max_cross_mm <= CHASSIS_PATH_DEVIATION_MM);
}

/* 走到端点原路折返，再从起点转去新方向：重合的几段都只能按走到哪算到哪。 */
static void TestCrossTrackOnOutAndBack(void)
{
    Chassis_SetMoveSpeedLimit(300);
    float x = chassis.pos_x, y = chassis.pos_y, h = chassis.yaw;
    chassis_waypoint path[] = {{x + 100, y, h}, {x + 300, y, h},      {x + 100, y, h},
                               {x, y, h},       {x, y + 300, h},      {x, y, h},
                               {x + 100, y, h}, {x + 100, y - 200, h}};
    assert(Chassis_FollowPath(path, 8, 0) == HAL_OK);
    complete();
    printf("out and back: max_cross=%f\n", chassis_path_result.max_cross_mm);
    assert(chassis_path_result.completed);
    assert(chassis_path_result.max_cross_mm <= CHASSIS_PATH_DEVIATION_MM);
}

/* 车偏到走过的段或后面的段上，偏线还是要按正在走的段算，不能被那些段吸收而显得没事。
   第三段 y=50 往回走，第一段在 y=0，第五段在 y=100，走到一半把位置推开 dy。 */
static float CrossAfterPush(float dy)
{
    Chassis_SetMoveSpeedLimit(300);
    float x = chassis.pos_x, y = chassis.pos_y, h = chassis.yaw;
    chassis_waypoint path[] = {{x + 300, y, h},
                               {x + 300, y + 50, h},
                               {x, y + 50, h},
                               {x, y + 100, h},
                               {x + 300, y + 100, h}};
    assert(Chassis_FollowPath(path, 5, 0) == HAL_OK);
    for (unsigned i = 0; i < 2000U && !(fabsf(chassis.pos_y - y - 50) < 0.5f &&
                                        fabsf(chassis.pos_x - x - 150) < 10);
         ++i)
        plant();
    assert(chassis.mode == CHASSIS_MODE_PATH);
    assert(chassis_path_result.max_cross_mm <= CHASSIS_PATH_DEVIATION_MM);
    chassis.pos_y += dy;
    complete();
    assert(chassis_path_result.completed);
    return chassis_path_result.max_cross_mm;
}

static void TestCrossTrackNotAbsorbedByOtherLegs(void)
{
    float onto_driven = CrossAfterPush(-50), onto_later = CrossAfterPush(50);
    printf("pushed onto other legs: driven=%f later=%f\n", onto_driven, onto_later);
    assert(onto_driven > 45);
    assert(onto_later > 45);
}

/* 最后一段后面没有下一段，上一条路径留在数组里的旧航点不能参与：第二条路径走到一半把车推到
   “终点 → 旧航点”这条不存在的线上，偏线还是要按真实的段算。 */
static void TestCrossTrackIgnoresStaleReference(void)
{
    Chassis_SetMoveSpeedLimit(300);
    float x = chassis.pos_x, y = chassis.pos_y, h = chassis.yaw;
    chassis_waypoint first[] = {{x + 100, y, h}, {x + 100, y + 100, h}, {x, y + 100, h}};
    assert(Chassis_FollowPath(first, 3, 0) == HAL_OK);
    complete();
    chassis_waypoint goal = {x - 100, y + 200, h};
    assert(Chassis_FollowPath(&goal, 1, 0) == HAL_OK);
    for (unsigned i = 0; i < 2000U && chassis.pos_x > x - 50; ++i)
        plant();
    chassis.pos_y += 25; // 正好落在 (x-100,y+200) 到 (x+100,y+100) 的连线上
    complete();
    printf("pushed onto a stale leg: max_cross=%f\n", chassis_path_result.max_cross_mm);
    assert(chassis_path_result.completed);
    assert(chassis_path_result.max_cross_mm > 15);
}

/* 指令模式边走边追加，累计远超参考折线容量：丢掉走过的点后，当前段下标要跟着平移。
   走楼梯形，每批是一个拐角加下一条边的中点，批次接缝落在直线上（跨批没有贝塞尔过渡）。 */
static void TestCrossTrackAfterReferenceDrop(void)
{
    Chassis_SetMoveSpeedLimit(300);
    float x = chassis.pos_x, y = chassis.pos_y, h = chassis.yaw;
    chassis_waypoint first = {x + 15, y, h};
    assert(Chassis_FollowPath(&first, 1, 0) == HAL_OK);
    x += 15;
    for (unsigned leg = 0; leg < REFERENCE_CAPACITY + 40U;)
    {
        plant();
        assert(chassis.mode == CHASSIS_MODE_PATH);
        if (s_path_count - s_path_index >= 20U)
            continue;
        chassis_waypoint corner =
            leg % 4U ? (chassis_waypoint){x, y + 15, h} : (chassis_waypoint){x + 15, y, h};
        chassis_waypoint more[2] = {corner, {x + 15, y + 15, h}};
        assert(Chassis_AppendPath(more, 2) == HAL_OK);
        x += 15;
        y += 15;
        leg += 2U;
    }
    assert(!s_reference_truncated);
    complete();
    printf("after reference drop: max_cross=%f\n", chassis_path_result.max_cross_mm);
    assert(chassis_path_result.completed);
    assert(chassis_path_result.max_cross_mm <= CHASSIS_PATH_DEVIATION_MM);
}

/* 一次排队太多，还没走到的航点也被挤出参考折线：退回量当前弦，不能拿挤剩下的折线算出假偏线。 */
static void TestCrossTrackWhenCurrentLegDropped(void)
{
    Chassis_SetMoveSpeedLimit(300);
    float x = chassis.pos_x, y = chassis.pos_y, h = chassis.yaw;
    static chassis_waypoint line[CHASSIS_PATH_CAPACITY];
    for (unsigned i = 0; i < CHASSIS_PATH_CAPACITY; ++i)
        line[i] = (chassis_waypoint){x + 10.0f * (float)(i + 1U), y, h};
    assert(Chassis_FollowPath(line, CHASSIS_PATH_CAPACITY, 0) == HAL_OK);
    chassis_waypoint more[20];
    for (unsigned i = 0; i < 20U; ++i)
        more[i] = (chassis_waypoint){x + 10.0f * (float)(CHASSIS_PATH_CAPACITY + i + 1U), y, h};
    assert(Chassis_AppendPath(more, 20) == HAL_OK);
    assert(s_reference_truncated);
    complete();
    printf("current leg dropped: max_cross=%f\n", chassis_path_result.max_cross_mm);
    assert(chassis_path_result.completed);
    assert(chassis_path_result.max_cross_mm < 1);

    /* 下一条路径恢复按参考折线量，拐角裁掉的部分又算进来。 */
    x = chassis.pos_x;
    y = chassis.pos_y;
    chassis_waypoint corner[] = {{x + 100, y, h}, {x + 100, y + 100, h}};
    assert(Chassis_FollowPath(corner, 2, 0) == HAL_OK);
    assert(!s_reference_truncated);
    complete();
    assert(chassis_path_result.max_cross_mm > 2);
}

/* 理想四轮闭环验证：连续过弯不能出现中途零速，轨迹仍在折线容差内。 */
int main(void)
{
    FDCAN_HandleTypeDef can = {0};
    assert(Chassis_Init(&can) == HAL_OK);
    tick(1);
    Chassis_SetMoveSpeedLimit(100);
    chassis_waypoint path[] = {{100, 0, 0}, {200, 0, 0}, {200, 200, 0}};
    assert(Chassis_FollowPath(path, 3, 0) == HAL_OK);
    float max_error = 0, min_speed = 1000;
    for (unsigned i = 0; i < 10000 && chassis.mode == CHASSIS_MODE_PATH; ++i)
    {
        plant();
        if (chassis.pos_x > 180 && chassis.pos_y < 20)
        {
            max_error = fmaxf(max_error, fminf(fabsf(chassis.pos_y), fabsf(chassis.pos_x - 200)));
            min_speed = fminf(min_speed, hypotf(chassis.current.vx, chassis.current.vy));
        }
    }
    printf("corner deviation=%f min_speed=%f\n", max_error, min_speed);
    assert(chassis_path_result.completed);
    assert(max_error <= CHASSIS_PATH_DEVIATION_MM);
    assert(min_speed > 1);
    assert(hypotf(chassis.pos_x - 200, chassis.pos_y - 200) <= 4);
    chassis_waypoint turn = {chassis.pos_x, chassis.pos_y, 90};
    assert(Chassis_FollowPath(&turn, 1, 0) == HAL_OK);
    complete();
    assert(fabsf(Math_WrapDeg(chassis.yaw - 90)) <= 0.1f);
    TestStraightBeforeCornerRunsAtLimit();
    TestStraightBetweenCornersRunsAtLimit();
    TestCrossTrackStaysOnCurrentLeg();
    TestCrossTrackOnOutAndBack();
    TestCrossTrackNotAbsorbedByOtherLegs();
    TestCrossTrackIgnoresStaleReference();
    TestCrossTrackAfterReferenceDrop();
    TestCrossTrackWhenCurrentLegDropped();
    puts("PASS: corner continuous speed, deviation, merged straight and pure turn, straights run "
         "at the set speed, cross track measured on the leg being driven");
    return 0;
}
