#include "vision_link.h"
#include <math.h>
#include <string.h>

VisionData vision;

static struct
{
    uint8_t buf[VL_FRAME_MAX];
    uint8_t n;
    uint8_t seq_rx, seq_seen;
    uint8_t seq_tx;
    uint32_t last_rx_ms;
} link;

void VisionLink_Init(void)
{
    memset(&vision, 0, sizeof(vision));
    memset(&link, 0, sizeof(link));
}

uint8_t VisionLink_Crc8(const uint8_t *data, unsigned len)
{
    uint8_t crc = 0;
    while (len--)
    {
        crc ^= *data++;
        for (unsigned i = 0; i < 8U; ++i)
            crc = (uint8_t)((crc & 0x80U) ? ((unsigned)crc << 1) ^ 0x07U : (unsigned)crc << 1);
    }
    return crc;
}

static int16_t GetI16(const uint8_t *p)
{
    return (int16_t)(uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static void PutI16(uint8_t *p, float v)
{
    long r = lroundf(v);
    if (r > 32767)
        r = 32767;
    if (r < -32768)
        r = -32768;
    uint16_t u = (uint16_t)(int16_t)r;
    p[0] = (uint8_t)u;
    p[1] = (uint8_t)(u >> 8);
}

/* 解析一帧完整、CRC 通过的数据；长度不符的帧计入 bad_frames 并丢弃。 */
static uint8_t Dispatch(uint8_t type, const uint8_t *p, uint8_t len, uint32_t now)
{
    switch (type)
    {
    case VL_HOST_HEARTBEAT:
        if (len != 2U)
            return 0;
        vision.host_state = p[0];
        vision.host_fps = p[1];
        vision.heartbeat_ms = now;
        return 1;
    case VL_HOST_TASK:
        if (len < 1U)
            return 0;
        memcpy(vision.task_text, p, len);
        vision.task_text[len] = '\0';
        vision.task_len = len;
        vision.task_ms = now;
        return 1;
    case VL_HOST_MATERIALS:
        if (len < 1U || p[0] > VL_MAX_ITEMS || len != 1U + 5U * p[0])
            return 0;
        vision.material_count = p[0];
        for (unsigned i = 0; i < p[0]; ++i)
        {
            const uint8_t *q = p + 1 + 5U * i;
            vision.material[i] =
                (VisionMaterial){q[0], GetI16(q + 1) * 0.1f, GetI16(q + 3) * 0.1f};
        }
        vision.material_ms = now;
        return 1;
    case VL_HOST_RING:
        if (len != 6U)
            return 0;
        vision.ring_found = p[0];
        vision.ring_aligned = p[1];
        vision.ring_fwd_mm = GetI16(p + 2) * 0.1f;
        vision.ring_left_mm = GetI16(p + 4) * 0.1f;
        vision.ring_ms = now;
        return 1;
    default:
        return 0;
    }
}

uint8_t VisionLink_Feed(uint8_t byte, uint32_t now_ms)
{
    uint8_t *b = link.buf;
    if (link.n == 0U && byte != VL_SOF0)
        return 0;
    if (link.n == 1U && byte != VL_SOF1)
    {
        link.n = (byte == VL_SOF0) ? 1U : 0U;
        return 0;
    }
    if (link.n == 2U && byte > VL_MAX_PAYLOAD)
    {
        ++vision.bad_frames;
        link.n = 0;
        return 0;
    }
    b[link.n++] = byte;
    if (link.n < 6U || link.n < 6U + b[2])
        return 0;
    /* 收满：AA 55 LEN TYPE SEQ PAYLOAD CRC */
    uint8_t len = b[2];
    link.n = 0;
    if (VisionLink_Crc8(b + 2, 3U + len) != b[5U + len])
    {
        ++vision.crc_errors;
        return 0;
    }
    if (!Dispatch(b[3], b + 5, len, now_ms))
    {
        ++vision.bad_frames;
        return 0;
    }
    if (link.seq_seen && (uint8_t)(b[4] - link.seq_rx) > 1U)
        vision.lost += (uint8_t)(b[4] - link.seq_rx - 1U);
    link.seq_rx = b[4];
    link.seq_seen = 1;
    link.last_rx_ms = now_ms;
    ++vision.frames;
    return 1;
}

uint8_t VisionLink_Online(uint32_t now_ms, uint32_t timeout_ms)
{
    return vision.frames && (now_ms - link.last_rx_ms) <= timeout_ms;
}

unsigned VisionLink_Encode(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *out)
{
    if (len > VL_MAX_PAYLOAD)
        return 0;
    out[0] = VL_SOF0;
    out[1] = VL_SOF1;
    out[2] = len;
    out[3] = type;
    out[4] = link.seq_tx++;
    if (len)
        memcpy(out + 5, payload, len);
    out[5U + len] = VisionLink_Crc8(out + 2, 3U + len);
    return 6U + len;
}

unsigned VisionLink_EncodePose(float x_mm, float y_mm, float yaw_deg, uint8_t valid,
                               uint8_t *out)
{
    uint8_t p[7];
    PutI16(p, x_mm);
    PutI16(p + 2, y_mm);
    PutI16(p + 4, yaw_deg * 100.0f);
    p[6] = valid;
    return VisionLink_Encode(VL_MCU_POSE, p, sizeof(p), out);
}

unsigned VisionLink_EncodeMode(VisionMode mode, uint8_t color, uint8_t *out)
{
    uint8_t p[2] = {(uint8_t)mode, color};
    return VisionLink_Encode(VL_MCU_MODE, p, sizeof(p), out);
}

unsigned VisionLink_EncodeObstacles(const int32_t (*xy)[2], unsigned n, uint8_t *out)
{
    uint8_t p[1 + 4 * 16];
    if (n > 16U)
        n = 16U;
    p[0] = (uint8_t)n;
    for (unsigned i = 0; i < n; ++i)
    {
        PutI16(p + 1 + 4 * i, (float)xy[i][0]);
        PutI16(p + 3 + 4 * i, (float)xy[i][1]);
    }
    return VisionLink_Encode(VL_MCU_OBSTACLES, p, (uint8_t)(1U + 4U * n), out);
}
