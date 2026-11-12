/* 巡检闭环：真实的底盘控制 + 巡检状态机 + 地图和路线，只有电机和陀螺仪是理想模型。
   agv_test 用假底盘验证状态机，这里验证两者接在一起能把整圈跑完。 */
#define main original_chassis_tests
#include "chassis_test.c"
#undef main
#include "agv_run.h"

#define ALL_EDGES 0x0FFFU

static UART_HandleTypeDef uart;

static void Ask(const char *line)
{
    size_t n = strlen(line);
    memcpy(rx_buf, line, n);
    HAL_UARTEx_RxEventCallback(&uart, (uint16_t)n);
    ChassisCmd_Poll();
    /* 路线一拍一行：让时间走够，把它发完。 */
    for (unsigned i = 0; i < CHASSIS_PATH_CAPACITY + 8U; ++i)
    {
        ms += 5U;
        uart.gState = 0;
        ChassisTelemetry_Poll();
    }
    drain(&uart);
}

/* 上电不走：等调试台发 /start <出发区>。出发区不给、给错都不走。 */
static void RunTour(unsigned zone)
{
    Agv_Init(AGV_MODE_SURVEY);
    for (unsigned i = 0; i < 1000U; ++i)
    {
        plant();
        Agv_Tick();
    }
    assert(agv.state == AGV_IDLE && chassis.mode == CHASSIS_MODE_VEL);
    tx_log[0] = 0;
    Ask("/route\r\n"); // 开走之前没有路线可以报
    assert(strstr(tx_log, "ROUTE none\r\n"));
    tx_log[0] = 0;
    Ask("/start\r\n");
    Ask("/start 3\r\n");
    assert(strstr(tx_log, "ERR  usage: /start <1|2>") && agv.state == AGV_IDLE);
    tx_log[0] = 0;
    char start[16];
    snprintf(start, sizeof(start), "/start %u\r\n", zone);
    Ask(start);
    char ok[32];
    snprintf(ok, sizeof(ok), "OK   survey from zone %u\r\n", zone);
    assert(strstr(tx_log, ok) && agv.state == AGV_WAIT && agv.zone == zone);
    Ask(start); // 已经在跑
    assert(strstr(tx_log, "BUSY survey running, S stops it\r\n"));
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
    unsigned state, error, leg, covered, line_zone;
    float x, y, residual;
    assert(sscanf(line,
                  " AGV state=%u err=%u leg=%u covered=0x%x field=(%f,%f) residual=%fmm zone=%u",
                  &state, &error, &leg, &covered, &x, &y, &residual, &line_zone) == 8);
    assert(line_zone == zone && strstr(line, "\r\n") == strstr(line, "zone=") + 6);
    assert(covered == ALL_EDGES);
    assert(hypotf(x - AGV_START_X_MM, y - AGV_START_Y_MM(zone)) <= AGV_POSITION_TOL_MM);

    /* 调试台在地图上画的整圈路线：开走时发一次，从出发点开始，回到出发点，每段沿坐标轴。 */
    const char *route = strstr(tx_log, "ROUTE points ");
    assert(route && !strstr(route + 1, "ROUTE points "));
    unsigned total = 0U, got = 0U;
    assert(sscanf(route, "ROUTE points %u", &total) == 1);
    assert(total > 16U && total <= CHASSIS_PATH_CAPACITY + 1U);
    int first_x = 0, first_y = 0, px = 0, py = 0;
    const char *p = strchr(route, '\n') + 1;
    for (; *p && strncmp(p, "ROUTE end\r\n", 11U) != 0; p = strchr(p, '\n') + 1)
    {
        unsigned index;
        int rx, ry;
        if (sscanf(p, "ROUTE %u %d %d", &index, &rx, &ry) != 3)
            continue;
        assert(index == got);
        if (!got)
        {
            first_x = rx;
            first_y = ry;
        }
        else
            assert(rx == px || ry == py);
        px = rx;
        py = ry;
        ++got;
    }
    assert(*p && got == total);
    assert(first_x == AGV_START_X_MM && first_y == AGV_START_Y_MM(zone));
    assert(px == AGV_START_X_MM && py == AGV_START_Y_MM(zone));

    /* 中途连上的调试台再要一次。 */
    tx_log[0] = 0;
    Ask("/route\r\n");
    assert(strstr(tx_log, "ROUTE points ") && strstr(tx_log, "ROUTE end\r\n"));

    tx_log[0] = 0;
    Ask("/mode\r\n");
    assert(strstr(tx_log, "OK   mode SURVEY\r\n"));
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
