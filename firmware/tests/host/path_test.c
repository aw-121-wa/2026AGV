#define main chassis_existing_test_main
#include "chassis_test.c"
#undef main

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
    puts("PASS: corner continuous speed, deviation, merged straight and pure turn");
    return 0;
}
