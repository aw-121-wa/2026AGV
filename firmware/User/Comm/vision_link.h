#ifndef VISION_LINK_H
#define VISION_LINK_H

#include <stdint.h>

/* 视觉上位机 ↔ STM32 二进制协议（比赛模式，USART2 115200 8N1）。
 *
 * 帧格式（多字节一律小端）：
 *   AA 55 | LEN | TYPE | SEQ | PAYLOAD[LEN] | CRC8
 *   LEN  = PAYLOAD 字节数（0..VL_MAX_PAYLOAD）
 *   SEQ  = 发送方自增序号，接收方只用来统计丢帧
 *   CRC8 = CRC-8（多项式 0x07，初值 0x00，不反转），覆盖 LEN TYPE SEQ PAYLOAD
 *
 * 上位机 → STM32
 *   0x01 HEARTBEAT  state u8 (0 正常 / 1 相机故障), fps u8                       ≥ 5 Hz
 *   0x02 TASK       n u8 (≤6), color[n] u8                    二维码任务码，按搬运顺序
 *   0x03 MATERIALS  n u8 (≤6), n × {color u8, fwd i16, left i16}
 *   0x04 RING       found u8, aligned u8, fwd i16, left i16
 *     fwd/left：目标相对“夹爪放置点”的偏差，车体坐标（前 / 左为正），单位 0.1 mm。
 *     STM32 让车向 (fwd, left) 平移即可对正。
 *
 * STM32 → 上位机
 *   0x81 POSE       x i16 mm, y i16 mm, yaw i16 (0.01°), valid u8              10 Hz
 *   0x82 MODE       mode u8 (0 关 / 1 二维码 / 2 物料 / 3 圆环), color u8 (0 = 任意)  5 Hz
 *   0x83 OBSTACLES  n u8 (≤16), n × {x i16 mm, y i16 mm}           雷达确认的障碍物，变化时发
 *
 * 颜色编号：1 红 2 黄 3 绿 4 浅蓝 5 蓝 6 黑。 */

#define VL_SOF0         0xAAU
#define VL_SOF1         0x55U
#define VL_MAX_PAYLOAD  64U
#define VL_FRAME_MAX    (VL_MAX_PAYLOAD + 6U)
#define VL_MAX_ITEMS    6U

enum
{
    VL_HOST_HEARTBEAT = 0x01,
    VL_HOST_TASK = 0x02,
    VL_HOST_MATERIALS = 0x03,
    VL_HOST_RING = 0x04,
    VL_MCU_POSE = 0x81,
    VL_MCU_MODE = 0x82,
    VL_MCU_OBSTACLES = 0x83,
};

typedef enum
{
    VISION_OFF = 0,
    VISION_QR = 1,
    VISION_MATERIAL = 2,
    VISION_RING = 3,
} VisionMode;

typedef enum
{
    COLOR_ANY = 0,
    COLOR_RED = 1,
    COLOR_YELLOW = 2,
    COLOR_GREEN = 3,
    COLOR_LIGHT_BLUE = 4,
    COLOR_BLUE = 5,
    COLOR_BLACK = 6,
} VisionColor;

typedef struct
{
    uint8_t color;
    float fwd_mm, left_mm;
} VisionMaterial;

/* 最近一次收到的上位机数据；*_ms 为收到时刻（HAL_GetTick），0 = 从未收到。 */
typedef struct
{
    uint32_t heartbeat_ms;
    uint8_t host_state, host_fps;

    uint8_t task[VL_MAX_ITEMS], task_len;
    uint32_t task_ms;

    VisionMaterial material[VL_MAX_ITEMS];
    uint8_t material_count;
    uint32_t material_ms;

    uint8_t ring_found, ring_aligned;
    float ring_fwd_mm, ring_left_mm;
    uint32_t ring_ms;

    uint32_t frames, crc_errors, bad_frames, lost;
} VisionData;

extern VisionData vision;

void VisionLink_Init(void);

/* 喂一个接收字节；now_ms 用来给数据打时间戳。返回 1 表示刚收完一帧有效帧。 */
uint8_t VisionLink_Feed(uint8_t byte, uint32_t now_ms);

/* 上位机是否在线（timeout_ms 内收到过心跳或任何有效帧）。 */
uint8_t VisionLink_Online(uint32_t now_ms, uint32_t timeout_ms);

/* 组帧：返回帧总长度，out 至少 VL_FRAME_MAX 字节。 */
unsigned VisionLink_Encode(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *out);
unsigned VisionLink_EncodePose(float x_mm, float y_mm, float yaw_deg, uint8_t valid,
                               uint8_t *out);
unsigned VisionLink_EncodeMode(VisionMode mode, uint8_t color, uint8_t *out);
unsigned VisionLink_EncodeObstacles(const int32_t (*xy)[2], unsigned n, uint8_t *out);

uint8_t VisionLink_Crc8(const uint8_t *data, unsigned len);

#endif
