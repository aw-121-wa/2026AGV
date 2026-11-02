/* 巡检闭环：真实的底盘控制 + 巡检状态机 + 地图和路线，只有电机和陀螺仪是理想模型。
   agv_test 用假底盘验证状态机，这里验证两者接在一起能把整圈跑完。 */
#define main original_chassis_tests
#include "chassis_test.c"
#undef main
#include "agv_run.h"

#define ALL_EDGES 0x0FFFU

static UART_HandleTypeDef uart;

static void RunTour(unsigned zone)
{
    Agv_Init(AGV_MODE_SURVEY, zone);
    assert(agv.state == AGV_WAIT);
    tx_log[0] = 0;
    float peak = 0;
    /* 整圈约 14 m、150 mm/s：理想用时约 100 s，留足余量。 */
    for (unsigned i = 0; i < 40000U && agv.state != AGV_DONE && agv.state != AGV_FAULT; ++i)
    {
        plant();
        Agv_Tick();
        uart.gState = 0;
        ChassisTelemetry_Poll();
        peak = fmaxf(peak, hypotf(chassis.current.vx, chassis.current.vy));
    }
    printf("zone %u: state=%d err=%d time=%.1fs dist=%.0fmm peak=%.0fmm/s max_cross=%.2fmm "
           "residual=%.2fmm covered=0x%03x\n",
           zone, agv.state, agv.error, (double)chassis_path_result.elapsed_ms * 0.001,
           (double)chassis_path_result.distance_mm, (double)peak,
           (double)chassis_path_result.max_cross_mm, (double)agv.home_residual_mm, agv.covered);
    assert(agv.state == AGV_DONE);
    assert(agv.error == AGV_ERR_NONE);
    assert(agv.covered == ALL_EDGES);
    assert(chassis_path_result.completed);
    assert(chassis_path_result.max_cross_mm <= CHASSIS_PATH_DEVIATION_MM);
    assert(agv.home_residual_mm <= AGV_POSITION_TOL_MM);
    assert(fabsf(peak - AGV_SURVEY_SPEED_MM_S) < 5);

    /* 调试台靠下面两行显示巡检结果（tools/agv_console.html），格式不能变。 */
    ms += 2000U;
    drain(&uart);
    assert(strstr(tx_log, "DONE path: time="));
    const char *line = strstr(tx_log, "          AGV state=6 err=0 leg=");
    assert(line);
    unsigned state, error, leg, covered;
    float x, y, residual;
    assert(sscanf(line, " AGV state=%u err=%u leg=%u covered=0x%x field=(%f,%f) residual=%f",
                  &state, &error, &leg, &covered, &x, &y, &residual) == 7);
    assert(strstr(line, "mm\r\n"));
    assert(covered == ALL_EDGES);
    assert(hypotf(x - AGV_START_X_MM, y - AGV_START_Y_MM(zone)) <= AGV_POSITION_TOL_MM);
}

int main(void)
{
    FDCAN_HandleTypeDef can = {0};
    assert(Chassis_Init(&can) == HAL_OK);
    tick(1);
    assert(ChassisCmd_Init(&uart) == HAL_OK);
    RunTour(1);
    RunTour(2);
    puts("PASS: full survey tour on the real chassis code, both start zones, status lines the "
         "console reads");
    return 0;
}
