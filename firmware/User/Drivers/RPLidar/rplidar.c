#include "rplidar.h"
#include <string.h>

const uint8_t RPLIDAR_CMD_STOP[2] = {0xA5, 0x25};
const uint8_t RPLIDAR_CMD_RESET[2] = {0xA5, 0x40};
const uint8_t RPLIDAR_CMD_SCAN[2] = {0xA5, 0x20};

static const uint8_t scan_descriptor[7] = {0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81};

void RPLidar_Reset(RPLidarParser *p)
{
    memset(p, 0, sizeof(*p));
}

/* 丢掉缓冲区首字节，其余左移，用于失步后逐字节重新对齐。 */
static void Shift(RPLidarParser *p)
{
    memmove(p->buf, p->buf + 1, --p->n);
}

static uint8_t SampleValid(const uint8_t *b)
{
    uint8_t s = b[0] & 1U, ns = (b[0] >> 1) & 1U;
    return s != ns && (b[1] & 1U);
}

uint8_t RPLidar_Feed(RPLidarParser *p, uint8_t byte, RPLidarSample *out)
{
    p->buf[p->n++] = byte;
    if (!p->streaming)
    {
        /* 滑动匹配 7 字节描述符。 */
        while (p->n && memcmp(p->buf, scan_descriptor, p->n) != 0)
            Shift(p);
        if (p->n == sizeof(scan_descriptor))
        {
            p->streaming = 1;
            p->n = 0;
        }
        return 0;
    }
    if (p->n < 5)
        return 0;
    if (!SampleValid(p->buf))
    {
        ++p->resync;
        Shift(p);
        return 0;
    }
    const uint8_t *b = p->buf;
    uint16_t angle_q6 = (uint16_t)((b[1] >> 1) | ((uint16_t)b[2] << 7));
    uint16_t dist_q2 = (uint16_t)(b[3] | ((uint16_t)b[4] << 8));
    out->angle_deg = angle_q6 / 64.0f;
    out->dist_mm = dist_q2 / 4.0f;
    out->quality = b[0] >> 2;
    out->start = b[0] & 1U;
    p->n = 0;
    if (out->angle_deg >= 360.0f) // 协议外的值视为失步
    {
        ++p->resync;
        return 0;
    }
    ++p->samples;
    if (out->start)
        ++p->scans;
    return 1;
}
