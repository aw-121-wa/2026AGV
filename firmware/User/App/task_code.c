#include "task_code.h"
#include <stddef.h>
#include <string.h>

static uint8_t IsDigit(char c)
{
    return c >= '0' && c <= '9';
}

/* 找一段完整的 ddd+ddd+ddd+ddd；前后紧贴数字或加号说明是更长的串，不能截出一段来用。 */
static const char *FindCode(const char *text, unsigned len)
{
    for (unsigned i = 0; i + TASK_CODE_TEXT_LEN <= len; ++i)
    {
        const char *s = text + i;
        unsigned k = 0;
        while (k < TASK_CODE_TEXT_LEN && (k % 4U == 3U ? s[k] == '+' : IsDigit(s[k])))
            ++k;
        if (k < TASK_CODE_TEXT_LEN)
            continue;
        if (i > 0U && (IsDigit(s[-1]) || s[-1] == '+'))
            continue;
        if (i + TASK_CODE_TEXT_LEN < len &&
            (IsDigit(s[TASK_CODE_TEXT_LEN]) || s[TASK_CODE_TEXT_LEN] == '+'))
            continue;
        return s;
    }
    return NULL;
}

/* 一组三个数字的取值掩码；有数字不在 1~max 内或有重复时返回 0。 */
static uint8_t GroupMask(const char *group, unsigned max)
{
    uint8_t mask = 0;
    for (unsigned i = 0; i < 3U; ++i)
    {
        unsigned d = (unsigned)(group[i] - '0');
        if (d < 1U || d > max || (mask & (1U << d)))
            return 0;
        mask |= (uint8_t)(1U << d);
    }
    return mask;
}

TaskCodeResult TaskCode_Parse(const char *text, unsigned len, TaskCode *out)
{
    const char *s = FindCode(text, len);
    if (!s)
        return TASK_CODE_ERR_FORMAT;
    uint8_t first = GroupMask(s, 6U), second = GroupMask(s + 8, 6U);
    if (!first || !second)
        return TASK_CODE_ERR_COLOR;
    if (!GroupMask(s + 4, 3U) || !GroupMask(s + 12, 3U))
        return TASK_CODE_ERR_RING;
    if (first != second)
        return TASK_CODE_ERR_BATCH;

    memcpy(out->text, s, TASK_CODE_TEXT_LEN);
    out->text[TASK_CODE_TEXT_LEN] = '\0';
    for (unsigned b = 0; b < 2U; ++b)
        for (unsigned i = 0; i < 3U; ++i)
        {
            out->color[b][i] = (uint8_t)(s[8U * b + i] - '0');
            out->rough_ring[b][i] = (uint8_t)(s[8U * b + 4U + i] - '0');
        }
    for (unsigned i = 0; i < 3U; ++i)
    {
        out->storage_ring[0][i] = out->rough_ring[0][i];
        for (unsigned j = 0; j < 3U; ++j)
            if (out->color[1][i] == out->color[0][j])
                out->storage_ring[1][i] = out->rough_ring[0][j];
    }
    return TASK_CODE_OK;
}

uint8_t TaskLine_Feed(TaskLine *line, uint8_t byte, uint32_t now_ms)
{
    line->last_ms = now_ms;
    if (byte == '\r' || byte == '\n')
        return line->len > 0U;
    if (line->len < TASK_LINE_MAX)
    {
        line->text[line->len++] = (byte >= 0x20U && byte < 0x7FU) ? (char)byte : '?';
        line->text[line->len] = '\0';
    }
    return line->len >= TASK_LINE_MAX;
}

uint8_t TaskLine_Idle(const TaskLine *line, uint32_t now_ms, uint32_t idle_ms)
{
    return line->len > 0U && now_ms - line->last_ms >= idle_ms;
}
