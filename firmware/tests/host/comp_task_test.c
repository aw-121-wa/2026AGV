/* 比赛模式的主机测试：真实的 competition.c、控制台、解析、视觉协议和雷达识别，
   只替换串口和文本输出。 */
#include "chassis.h"
#include "chassis_telemetry.h"
#include "comp_config.h"
#include "competition.h"
#include "console.h"
#include "ser_port.h"
#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

chassis_t chassis;
chassis_tuning chassis_tune = CHASSIS_TUNE_DEFAULTS;
const chassis_tuning chassis_tune_default = CHASSIS_TUNE_DEFAULTS;

const char *ChassisTelemetry_Num(float value, unsigned decimals)
{
    static char pool[24][24];
    static unsigned next;
    char *out = pool[next];
    next = (next + 1U) % 24U;
    snprintf(out, sizeof(pool[0]), "%.*f", (int)decimals, (double)value);
    return out;
}

void ChassisTelemetry_SetPlot(telemetry_plot group, unsigned hz)
{
    (void)group;
    (void)hz;
}

static uint32_t tick = 1000U;
uint32_t HAL_GetTick(void)
{
    return tick;
}

static char printed[8192];
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

static unsigned CountPrinted(const char *needle)
{
    unsigned count = 0;
    for (const char *p = strstr(printed, needle); p; p = strstr(p + 1, needle))
        ++count;
    return count;
}

static struct
{
    uint32_t baud;
    uint8_t data[512];
    unsigned head, tail;
    SerPortStats stats;
} port[SER_COUNT];

HAL_StatusTypeDef SerPort_Init(SerId id, uint32_t baud)
{
    port[id].baud = baud;
    return HAL_OK;
}

unsigned SerPort_Read(SerId id, uint8_t *dst, unsigned max)
{
    unsigned n = 0;
    while (n < max && port[id].tail < port[id].head)
    {
        dst[n++] = port[id].data[port[id].tail++];
        ++port[id].stats.rx_bytes;
    }
    return n;
}

static uint8_t written[SER_COUNT][512];
static unsigned written_len[SER_COUNT];

uint8_t SerPort_Write(SerId id, const uint8_t *src, unsigned len)
{
    if (written_len[id] + len <= sizeof(written[id]))
    {
        memcpy(written[id] + written_len[id], src, len);
        written_len[id] += len;
    }
    return 1;
}

void SerPort_Flush(SerId id)
{
    port[id].tail = port[id].head;
}

const SerPortStats *SerPort_Stats(SerId id)
{
    return &port[id].stats;
}

static void Inject(SerId id, const void *bytes, unsigned n)
{
    assert(port[id].head + n <= sizeof(port[id].data));
    memcpy(port[id].data + port[id].head, bytes, n);
    port[id].head += n;
}

static void Scan(const char *text)
{
    Inject(SER_SCANNER, text, (unsigned)strlen(text));
}

static void Tick(uint32_t ms)
{
    tick += ms;
    Comp_Tick();
}

static void Begin(void)
{
    memset(port, 0, sizeof(port));
    memset(written_len, 0, sizeof(written_len));
    memset(printed, 0, sizeof(printed));
    printed_len = 0;
    memset(&chassis, 0, sizeof(chassis));
    lidar_tune = lidar_tune_default;
    assert(Comp_Init(1) == HAL_OK);
}

/* 控制台行（不含开头的 '/'）。 */
static void Console(const char *line)
{
    char buffer[64];
    assert(strlen(line) < sizeof(buffer));
    strcpy(buffer, line);
    Console_Line(buffer);
}

static void ClearPrinted(void)
{
    memset(printed, 0, sizeof(printed));
    printed_len = 0;
}

/* 在写给上位机的字节流里找某种帧，返回 PAYLOAD 指针，没有返回 NULL。 */
static const uint8_t *FindVisionFrame(uint8_t type, uint8_t *payload_len)
{
    const uint8_t *b = written[SER_VISION];
    for (unsigned i = 0; i + 6U <= written_len[SER_VISION]; i += 6U + b[i + 2U])
    {
        assert(b[i] == VL_SOF0 && b[i + 1U] == VL_SOF1);
        if (b[i + 3U] == type)
        {
            *payload_len = b[i + 2U];
            return b + i + 5U;
        }
    }
    return NULL;
}

/* 雷达正前方 500 mm 处一小段弧，连续 6 圈，足够确认成障碍物。 */
static void SeeObstacle(void)
{
    LidarPose pose = {1200, 1200, 0};
    for (unsigned scan = 0; scan < 6U; ++scan)
        for (int a = -2; a <= 2; ++a)
        {
            RPLidarSample s = {(float)(a < 0 ? a + 360 : a), 500.0f, 47, a == -2};
            LidarObs_Feed(&s, &pose);
        }
}

static void TestScannerPortIsOpened(void)
{
    Begin();
    assert(port[SER_SCANNER].baud == COMP_SCANNER_BAUD);
}

static void TestCodeFromScannerIsAccepted(void)
{
    Begin();
    assert(!comp.task_valid);
    Scan("156+123+516+231\r\n");
    Tick(5);
    assert(comp.task_valid);
    assert(strcmp(comp.task.text, "156+123+516+231") == 0);
    assert(comp.task.storage_ring[1][0] == 2 && comp.task.storage_ring[1][1] == 1);
    assert(CountPrinted("156+123+516+231") == 1U);
}

/* 9600 波特下一拍只到几个字节，一行要跨很多拍才收齐。 */
static void TestCodeSplitAcrossTicks(void)
{
    Begin();
    Scan("156+123+");
    Tick(5);
    assert(!comp.task_valid);
    Scan("516+231\r");
    Tick(5);
    assert(comp.task_valid && strcmp(comp.task.text, "156+123+516+231") == 0);
}

/* 模块没有配置行尾符时，靠空闲时间判断一行结束。 */
static void TestCodeWithoutLineEnding(void)
{
    Begin();
    Scan("156+123+516+231");
    Tick(5);
    assert(!comp.task_valid);
    Tick(COMP_SCANNER_IDLE_MS);
    assert(comp.task_valid);
}

/* 连续读码模式下同一个码会反复收到，只报告一次，也不会把一行拆开重复处理。 */
static void TestRepeatedReadsReportOnce(void)
{
    Begin();
    for (int i = 0; i < 5; ++i)
    {
        Scan("156+123+516+231\r\n");
        Tick(5);
    }
    assert(comp.task_valid && comp.task_rejected == 0U);
    assert(CountPrinted("156+123+516+231") == 1U);
}

static void TestNewCodeReplacesOld(void)
{
    Begin();
    Scan("156+123+516+231\r\n");
    Tick(5);
    Scan("234+312+432+123\r\n");
    Tick(5);
    assert(comp.task_valid && strcmp(comp.task.text, "234+312+432+123") == 0);
    assert(comp.task.color[0][0] == COLOR_YELLOW);
}

/* 读错的内容要打印出来让现场能看到模块读到了什么，但不能覆盖已有的合法任务码。 */
static void TestBadTextKeepsValidCode(void)
{
    Begin();
    Scan("156+123+516+231\r\n");
    Tick(5);
    Scan("123+321\r\n");
    Tick(5);
    assert(comp.task_valid && strcmp(comp.task.text, "156+123+516+231") == 0);
    assert(comp.task_rejected == 1U);
    assert(CountPrinted("123+321") == 1U);
    /* 连续读到同样的错码：继续计数，但不刷屏。 */
    Scan("123+321\r\n");
    Tick(5);
    assert(comp.task_rejected == 2U);
    assert(CountPrinted("123+321") == 1U);
}

static void TestBadTextAloneGivesNoTask(void)
{
    Begin();
    Scan("Scanner V1.2\r\n");
    Tick(5);
    assert(!comp.task_valid && comp.task_rejected == 1U);
    assert(CountPrinted("Scanner V1.2") == 1U);
}

/* 相机是备用通道：TASK 帧的原文走同一套解析和校验。 */
static void TestCodeFromCamera(void)
{
    Begin();
    uint8_t frame[VL_FRAME_MAX];
    unsigned n = VisionLink_Encode(VL_HOST_TASK, (const uint8_t *)"123+321", 7, frame);
    Inject(SER_VISION, frame, n);
    Tick(5);
    assert(!comp.task_valid && comp.task_rejected == 1U);

    n = VisionLink_Encode(VL_HOST_TASK, (const uint8_t *)"156+123+516+231", 15, frame);
    Inject(SER_VISION, frame, n);
    Tick(5);
    assert(comp.task_valid && strcmp(comp.task.text, "156+123+516+231") == 0);
    assert(comp.task_rejected == 1U);
}

/* 周期状态行要显示当前任务码，现场不用翻历史就能确认。 */
static void TestReportShowsTask(void)
{
    Begin();
    Scan("156+123+516+231\r\n");
    Tick(5);
    assert(CountPrinted("156+123+516+231") == 1U);
    Tick(COMP_REPORT_PERIOD_MS);
    assert(CountPrinted("156+123+516+231") == 2U);
}

static void TestConsoleTask(void)
{
    Begin();
    Console("task 156+123+516+231");
    assert(comp.task_valid && strcmp(comp.task.text, "156+123+516+231") == 0);
    assert(CountPrinted("console") == 1U);
    Console("task 123+321"); // 错码不覆盖好码，走同一套校验
    assert(comp.task_rejected == 1U && strcmp(comp.task.text, "156+123+516+231") == 0);
    Console("task");
    assert(CountPrinted("ERR") == 1U);
}

static void TestConsoleVision(void)
{
    Begin();
    Console("vision 2 3");
    assert(comp.vision_mode == VISION_MATERIAL && comp.vision_color == COLOR_BLUE);
    uint8_t n = 0;
    const uint8_t *mode = FindVisionFrame(VL_MCU_MODE, &n); // 立刻下发，不等下一个周期
    assert(mode && n == 2U && mode[0] == VISION_MATERIAL && mode[1] == COLOR_BLUE);

    Console("vision 1");
    assert(comp.vision_mode == VISION_QR && comp.vision_color == COLOR_ANY);

    static const char *const bad[] = {"vision 4",   "vision 1 7", "vision x",
                                      "vision 1.5", "vision -1",  "vision 1 2 3"};
    for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i)
    {
        ClearPrinted();
        Console(bad[i]);
        assert(CountPrinted("ERR") == 1U);
        assert(comp.vision_mode == VISION_QR && comp.vision_color == COLOR_ANY);
    }
}

static void TestConsoleObstacles(void)
{
    Begin();
    Console("obs");
    assert(CountPrinted("0 obstacles") == 1U);

    SeeObstacle();
    Tick(COMP_LIDAR_BOOT_MS); // 雷达状态机走到运行状态后，比赛模式才统计新障碍物
    Tick(COMP_LIDAR_STOP_MS);
    Tick(5);
    assert(comp.obstacles == 1U);
    MapPoint list[16];
    assert(LidarObs_List(list, 16) == 1U);
    uint16_t r, c;
    assert(Map_WorldToGrid(list[0].x, list[0].y, &r, &c) && !Map_IsFree(r, c));
    char where[32];
    snprintf(where, sizeof(where), "(%ld,%ld)", (long)list[0].x, (long)list[0].y);
    ClearPrinted();
    Console("obs");
    assert(CountPrinted(where) == 1U && CountPrinted("1 obstacles") == 1U);

    /* 运动中不能清：重建地图要几毫秒，路径也可能正依赖它。 */
    chassis.mode = CHASSIS_MODE_PATH;
    ClearPrinted();
    Console("obs clear");
    assert(CountPrinted("BUSY") == 1U && LidarObs_List(list, 16) == 1U);
    chassis.mode = CHASSIS_MODE_VEL;

    ClearPrinted();
    Console("obs nonsense");
    assert(CountPrinted("ERR") == 1U && LidarObs_List(list, 16) == 1U);

    written_len[SER_VISION] = 0;
    Console("obs clear");
    assert(LidarObs_List(list, 16) == 0U && lidar_obs.confirmed == 0U && comp.obstacles == 0U);
    assert(Map_IsFree(r, c)); // 地图里也清掉了
    /* 上位机的障碍物表也要同步清空。 */
    Tick(5);
    uint8_t n = 0;
    const uint8_t *frame = FindVisionFrame(VL_MCU_OBSTACLES, &n);
    assert(frame && n == 1U && frame[0] == 0U);

    /* 清完还能重新识别，运行时参数不受影响。 */
    lidar_tune.obs_gap = 44.0f;
    SeeObstacle();
    assert(LidarObs_List(list, 16) == 1U);
    Console("obs clear");
    assert(lidar_tune.obs_gap == 44.0f);
}

static void TestConsolePose(void)
{
    Begin();
    chassis.pos_x = 50.0f;
    chassis.pos_y = -20.0f;
    chassis.yaw = 30.0f;

    /* 底盘没就绪时航向不可信，不能设。 */
    Console("pose 1200 600 90");
    assert(CountPrinted("BUSY") == 1U && !comp.anchored);

    chassis.state = CHASSIS_STATE_READY;
    Console("pose 1200 600 90");
    assert(comp.anchored);
    Tick(5);
    assert(fabsf(comp.x - 1200.0f) < 0.01f && fabsf(comp.y - 600.0f) < 0.01f);
    assert(fabsf(comp.heading_deg - 90.0f) < 0.01f);

    /* 里程计系航向 30° 对应场地航向 90°，两个系差 60°：
       里程计 +X 走 100 mm，场地里是 (100cos60°, 100sin60°)。 */
    chassis.pos_x += 100.0f;
    Tick(5);
    assert(fabsf(comp.x - 1250.0f) < 0.05f && fabsf(comp.y - 686.6f) < 0.05f);

    static const char *const bad[] = {"pose 3000 0 0", "pose -1 0 0", "pose 0 2401 0",
                                      "pose 0 0 181",  "pose 0 0 x",  "pose 0 0",
                                      "pose",          "pose 0 0 0 0"};
    for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i)
    {
        ClearPrinted();
        Console(bad[i]);
        assert(CountPrinted("ERR") == 1U);
        Tick(5);
        assert(fabsf(comp.x - 1250.0f) < 0.05f);
    }

    /* 设过位姿后，自动锚定不能把它盖掉。 */
    Tick(AGV_START_DELAY_MS + 5U);
    assert(fabsf(comp.x - 1250.0f) < 0.05f);
}

/* 自动锚定之后也能重设（把车搬到别处再测）。 */
static void TestConsolePoseAfterAutoAnchor(void)
{
    Begin();
    chassis.state = CHASSIS_STATE_READY;
    Tick(AGV_START_DELAY_MS + 5U);
    assert(comp.anchored && fabsf(comp.x - 2250.0f) < 0.01f && fabsf(comp.y - 2250.0f) < 0.01f);
    Console("pose 300 400 0");
    Tick(5);
    assert(fabsf(comp.x - 300.0f) < 0.01f && fabsf(comp.y - 400.0f) < 0.01f);
}

static void TestConsoleLidarParameters(void)
{
    Begin();
    Console("set lidar_yaw -90.5");
    assert(lidar_tune.lidar_yaw == -90.5f);
    Console("set obs_confirm 5.5"); // 整数参数取整
    assert(lidar_tune.obs_confirm == 6.0f);
    ClearPrinted();
    Console("set lidar_x 300");
    assert(CountPrinted("ERR") == 1U && lidar_tune.lidar_x == lidar_tune_default.lidar_x);

    /* 界面按这个格式解析，负数范围也要能读。 */
    ClearPrinted();
    Console("list lidar_yaw");
    assert(strcmp(printed, "lidar_yaw       = -90.50  [-180.00..180.00] default 0.00\r\n") == 0);

    ClearPrinted();
    Console("list obs_");
    assert(CountPrinted("obs_") == 4U);

    Console("reset");
    assert(lidar_tune.lidar_yaw == 0.0f && lidar_tune.obs_confirm == 4.0f);
}

/* 界面按这行取位姿、雷达、视觉和任务状态，格式变了界面就读不到，所以逐字固定。 */
static void TestReportFormatForUi(void)
{
    Begin();
    Tick(COMP_REPORT_PERIOD_MS);
    assert(strcmp(printed, "COMP pose=(0,0,0.0) lidar=STOP 0.0Hz pts=0 used=0 clu=0 obs=0 rs=0/0 | "
                           "vision=OFF mode=0 mat=0 ring=0 crc=0 | task=none rej=0 scan=0/0\r\n") == 0);
}

static void TestConsoleHelpListsBenchCommands(void)
{
    Begin();
    Console("help");
    assert(CountPrinted("/task") == 1U && CountPrinted("/vision") == 1U);
    assert(CountPrinted("/obs") == 1U && CountPrinted("/pose") == 1U);
}

int main(void)
{
    TestConsoleTask();
    TestConsoleVision();
    TestConsoleObstacles();
    TestConsolePose();
    TestConsolePoseAfterAutoAnchor();
    TestConsoleLidarParameters();
    TestConsoleHelpListsBenchCommands();
    TestReportFormatForUi();
    TestScannerPortIsOpened();
    TestCodeFromScannerIsAccepted();
    TestCodeSplitAcrossTicks();
    TestCodeWithoutLineEnding();
    TestRepeatedReadsReportOnce();
    TestNewCodeReplacesOld();
    TestBadTextKeepsValidCode();
    TestBadTextAloneGivesNoTask();
    TestCodeFromCamera();
    TestReportShowsTask();
    printf("PASS: console task, vision, obstacles, pose and lidar parameters; scanner port, "
           "accepted code, split across ticks, no line ending, repeated reads, replacement, "
           "bad text, camera channel, status line\n");
    return 0;
}
