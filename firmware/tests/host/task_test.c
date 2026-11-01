/* 任务码的主机测试：期望值都按比赛规则手算，核对解析、校验和扫码模块字节流的分行。 */
#include "task_code.h"
#include "vision_link.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static TaskCodeResult Parse(const char *s, TaskCode *t)
{
    return TaskCode_Parse(s, (unsigned)strlen(s), t);
}

static void AssertTriple(const uint8_t *got, uint8_t a, uint8_t b, uint8_t c)
{
    assert(got[0] == a && got[1] == b && got[2] == c);
}

/* 规则原文的例子：第一批 红 黑 浅蓝，两个区都放 1 2 3 号环；
   第二批 黑 红 浅蓝，粗加工区放 2 3 1 号环，暂存区码垛到同色物料上（黑 2、红 1、浅蓝 3）。 */
static void TestRuleExample(void)
{
    TaskCode t;
    memset(&t, 0xFF, sizeof(t)); // text 必须自己结束，不能靠内存里碰巧是 0
    assert(Parse("156+123+516+231", &t) == TASK_CODE_OK);
    assert(strcmp(t.text, "156+123+516+231") == 0);
    AssertTriple(t.color[0], COLOR_RED, COLOR_BLACK, COLOR_LIGHT_BLUE);
    AssertTriple(t.rough_ring[0], 1, 2, 3);
    AssertTriple(t.storage_ring[0], 1, 2, 3);
    AssertTriple(t.color[1], COLOR_BLACK, COLOR_RED, COLOR_LIGHT_BLUE);
    AssertTriple(t.rough_ring[1], 2, 3, 1);
    AssertTriple(t.storage_ring[1], 2, 1, 3);
}

/* 规则的颜色编号：红 1、黄 2、蓝 3、绿 4、黑 5、浅蓝 6。 */
static void TestColorNumbers(void)
{
    TaskCode t;
    assert(Parse("234+312+432+123", &t) == TASK_CODE_OK);
    AssertTriple(t.color[0], COLOR_YELLOW, COLOR_BLUE, COLOR_GREEN);
    AssertTriple(t.color[1], COLOR_GREEN, COLOR_BLUE, COLOR_YELLOW);
}

/* 第二批的暂存区环号不在码里：每个物料放到第一批同色物料所在的环上。 */
static void TestStackingRings(void)
{
    static const struct
    {
        const char *text;
        uint8_t want[3];
    } cases[] = {
        {"234+312+432+123", {2, 1, 3}}, // 绿 → 2，蓝 → 1，黄 → 3
        {"123+231+321+312", {1, 3, 2}}, // 蓝 → 1，黄 → 3，红 → 2
        {"345+213+345+132", {2, 1, 3}}, // 两批顺序相同：蓝 → 2，绿 → 1，黑 → 3
    };
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i)
    {
        TaskCode t;
        assert(Parse(cases[i].text, &t) == TASK_CODE_OK);
        AssertTriple(t.storage_ring[1], cases[i].want[0], cases[i].want[1], cases[i].want[2]);
    }
}

static void TestRejected(void)
{
    static const struct
    {
        const char *text;
        TaskCodeResult want;
    } cases[] = {
        {"", TASK_CODE_ERR_FORMAT},
        {"123+321", TASK_CODE_ERR_FORMAT},             // 往届的两组格式
        {"156+123+516", TASK_CODE_ERR_FORMAT},         // 少一组
        {"156123516231", TASK_CODE_ERR_FORMAT},        // 没有加号
        {"156-123-516-231", TASK_CODE_ERR_FORMAT},     // 分隔符不对
        {"15+123+516+231", TASK_CODE_ERR_FORMAT},      // 一组只有两位
        {"1156+123+516+231", TASK_CODE_ERR_FORMAT},    // 前面紧贴数字
        {"156+123+516+2311", TASK_CODE_ERR_FORMAT},    // 后面紧贴数字
        {"+156+123+516+231", TASK_CODE_ERR_FORMAT},    // 前面紧贴加号
        {"156+123+516+231+111", TASK_CODE_ERR_FORMAT}, // 多一组
        {"176+123+716+231", TASK_CODE_ERR_COLOR},      // 7 不是颜色
        {"056+123+056+231", TASK_CODE_ERR_COLOR},      // 0 不是颜色
        {"176+123+516+231", TASK_CODE_ERR_COLOR},      // 只有第一批出现 7
        {"156+123+716+231", TASK_CODE_ERR_COLOR},      // 只有第二批出现 7
        {"115+123+115+231", TASK_CODE_ERR_COLOR},      // 第一批颜色重复
        {"156+123+556+231", TASK_CODE_ERR_COLOR},      // 第二批颜色重复
        {"156+112+516+231", TASK_CODE_ERR_RING},       // 第一批环号重复
        {"156+124+516+231", TASK_CODE_ERR_RING},       // 没有 4 号环
        {"156+120+516+231", TASK_CODE_ERR_RING},       // 没有 0 号环
        {"156+123+516+234", TASK_CODE_ERR_RING},       // 第二批环号不对
        {"156+123+234+231", TASK_CODE_ERR_BATCH},      // 第二批三种颜色都不同
        {"156+123+512+231", TASK_CODE_ERR_BATCH},      // 第二批一种颜色不同，无法码垛
    };
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i)
    {
        TaskCode t;
        assert(Parse(cases[i].text, &t) == cases[i].want);
    }
}

/* 模块可能带前后缀或控制字节，只要码本身完整就接受，text 里只留任务码。 */
static void TestSurroundedByNoise(void)
{
    static const char *const cases[] = {"?156+123+516+231?", "code: 156+123+516+231",
                                        "156+123+516+231 "};
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i)
    {
        TaskCode t;
        assert(Parse(cases[i], &t) == TASK_CODE_OK);
        assert(strcmp(t.text, "156+123+516+231") == 0);
    }
}

/* 长度由调用方给：相机通道的文本可以不是任务码，也不能读过 len。 */
static void TestHonoursLength(void)
{
    TaskCode t;
    assert(TaskCode_Parse("156+123+516+231", 14U, &t) == TASK_CODE_ERR_FORMAT);
    assert(TaskCode_Parse("156+123+516+2319", 15U, &t) == TASK_CODE_OK);
    assert(strcmp(t.text, "156+123+516+231") == 0);
}

static unsigned FeedText(TaskLine *line, const char *s, uint32_t now)
{
    unsigned ended = 0;
    while (*s)
        ended += TaskLine_Feed(line, (uint8_t)*s++, now);
    return ended;
}

static void TestLineEndings(void)
{
    TaskLine line = {0};
    /* CRLF 只算一行，不产生空行。 */
    assert(FeedText(&line, "156+123+516+231", 1U) == 0U);
    assert(TaskLine_Feed(&line, '\r', 1U) == 1U);
    assert(line.len == 15U && strcmp(line.text, "156+123+516+231") == 0);
    line.len = 0;
    assert(TaskLine_Feed(&line, '\n', 1U) == 0U);

    assert(FeedText(&line, "ab", 2U) == 0U);
    assert(TaskLine_Feed(&line, '\n', 2U) == 1U);
    assert(line.len == 2U && strcmp(line.text, "ab") == 0);
    line.len = 0;

    /* 下一行从头开始，不残留上一行的内容。 */
    assert(FeedText(&line, "c\r", 3U) == 1U);
    assert(line.len == 1U && strcmp(line.text, "c") == 0);
}

/* 没有行尾符的模块：超过空闲时间才算一行，没数据时不报。 */
static void TestIdleEndsLine(void)
{
    TaskLine line = {0};
    assert(!TaskLine_Idle(&line, 1000U, 30U));
    assert(FeedText(&line, "156+123+516+231", 100U) == 0U);
    assert(!TaskLine_Idle(&line, 129U, 30U));
    assert(TaskLine_Idle(&line, 130U, 30U));
    assert(strcmp(line.text, "156+123+516+231") == 0);
    /* 新字节刷新空闲计时。 */
    assert(TaskLine_Feed(&line, '9', 125U) == 0U);
    assert(!TaskLine_Idle(&line, 130U, 30U));
}

/* 控制字节会被打印到串口助手，存成 '?'；任务码前后的控制字节不影响解析。 */
static void TestUnprintableBytes(void)
{
    static const uint8_t bytes[] = {0x02, 'A', 0xFF, 0x00, 0x7F};
    TaskLine line = {0};
    for (unsigned i = 0; i < sizeof(bytes); ++i)
        assert(TaskLine_Feed(&line, bytes[i], 1U) == 0U);
    assert(line.len == 5U && strcmp(line.text, "?A???") == 0);

    line.len = 0;
    assert(TaskLine_Feed(&line, 0x02, 1U) == 0U);
    assert(FeedText(&line, "156+123+516+231", 1U) == 0U);
    assert(TaskLine_Feed(&line, 0x03, 1U) == 0U);
    assert(TaskLine_Feed(&line, '\r', 1U) == 1U);
    TaskCode t;
    assert(TaskCode_Parse(line.text, line.len, &t) == TASK_CODE_OK);
}

/* 一直没有行尾符也不会写出缓冲；写满就交出去，下一个字节进入新的一行。 */
static void TestFullBufferEndsLine(void)
{
    TaskLine line = {0};
    for (unsigned i = 0; i + 1U < TASK_LINE_MAX; ++i)
        assert(TaskLine_Feed(&line, 'x', 1U) == 0U);
    assert(TaskLine_Feed(&line, 'y', 1U) == 1U);
    assert(line.len == TASK_LINE_MAX && line.text[TASK_LINE_MAX - 1U] == 'y' &&
           line.text[TASK_LINE_MAX] == '\0');
    line.len = 0;
    assert(TaskLine_Feed(&line, 'z', 1U) == 0U);
    assert(line.len == 1U && strcmp(line.text, "z") == 0);
}

int main(void)
{
    TestRuleExample();
    TestColorNumbers();
    TestStackingRings();
    TestRejected();
    TestSurroundedByNoise();
    TestHonoursLength();
    TestLineEndings();
    TestIdleEndsLine();
    TestUnprintableBytes();
    TestFullBufferEndsLine();
    printf("PASS: task code rule example, colour numbers, stacking rings, rejects, noise, length, "
           "line endings, idle, unprintable bytes, full buffer\n");
    return 0;
}
