/* 比赛模式任务码通道的主机测试：真实的 competition.c、解析和视觉协议，只替换串口和文本输出。 */
#include "chassis.h"
#include "chassis_telemetry.h"
#include "comp_config.h"
#include "competition.h"
#include "ser_port.h"
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

chassis_t chassis;

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

uint8_t SerPort_Write(SerId id, const uint8_t *src, unsigned len)
{
    (void)id;
    (void)src;
    (void)len;
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
    memset(printed, 0, sizeof(printed));
    printed_len = 0;
    assert(Comp_Init(1) == HAL_OK);
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

int main(void)
{
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
    printf("PASS: scanner port, accepted code, split across ticks, no line ending, repeated reads, "
           "replacement, bad text, camera channel, status line\n");
    return 0;
}
