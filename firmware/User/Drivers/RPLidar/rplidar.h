#ifndef RPLIDAR_H
#define RPLIDAR_H

#include <stdint.h>

/* SLAMTEC RPLIDAR（C1 / A 系列）标准扫描协议解析，纯软件，不碰硬件。
   请求：A5 25 = STOP，A5 40 = RESET，A5 20 = SCAN。
   SCAN 应答描述符：A5 5A 05 00 00 40 81，之后是连续的 5 字节采样：
     b0 = quality<<2 | !S<<1 | S       S = 新一圈起点
     b1 = angle_q6[6:0]<<1 | C(=1)     angle = angle_q6 / 64 度（顺时针）
     b2 = angle_q6[14:7]
     b3,b4 = distance_q2（小端）        distance = distance_q2 / 4 mm，0 = 无效 */

typedef struct
{
    float angle_deg; // 0..360，顺时针
    float dist_mm;   // 0 = 无效测量
    uint8_t quality;
    uint8_t start;   // 1 = 新一圈第一个点
} RPLidarSample;

typedef struct
{
    uint8_t buf[7];
    uint8_t n;
    uint8_t streaming; // 0 = 等应答描述符，1 = 采样流
    uint32_t samples, scans, resync;
} RPLidarParser;

extern const uint8_t RPLIDAR_CMD_STOP[2];
extern const uint8_t RPLIDAR_CMD_RESET[2];
extern const uint8_t RPLIDAR_CMD_SCAN[2];

/* 清空状态，回到“等 SCAN 应答描述符”。发 SCAN 之前调用。 */
void RPLidar_Reset(RPLidarParser *p);

/* 喂一个字节；凑满一个合法采样时写入 *out 并返回 1。 */
uint8_t RPLidar_Feed(RPLidarParser *p, uint8_t byte, RPLidarSample *out);

#endif
