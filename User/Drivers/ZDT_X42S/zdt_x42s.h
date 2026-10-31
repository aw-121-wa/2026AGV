#ifndef ZDT_X42S_H
#define ZDT_X42S_H

#include "fdcan.h"

/* ZDT_X42S 第二代闭环步进，仅支持 X 固件协议（手册 V1.0.3 第 4.2/5 章）。
   X 固件速度单位 0.1 RPM，位置单位 0.1°。 */

#define ZDT_MAX_MOTORS     4U
#define ZDT_MAX_RPM_X10    30000U // 3000.0 RPM
#define ZDT_BROADCAST_ADDR 0U

/* 电机地址，按实车接线修改；0 是广播地址，不能用作节点地址。 */
typedef enum
{
    ZDT_ADDR_LF = 1,
    ZDT_ADDR_RF = 2,
    ZDT_ADDR_LR = 3,
    ZDT_ADDR_RR = 4,
} zdt_address;

/* 同步标志：NOW 立即执行；CACHE 先缓存，收到 ZDT_SyncTrigger 后所有电机同时执行。 */
typedef enum
{
    ZDT_SYNC_NOW = 0,
    ZDT_SYNC_CACHE = 1,
} zdt_sync;

/* X 固件位置模式的运动模式字段。 */
typedef enum
{
    ZDT_POS_RELATIVE_TARGET = 0, // 相对上一输入目标位置
    ZDT_POS_ABSOLUTE = 1,        // 相对坐标零点
    ZDT_POS_RELATIVE_NOW = 2,    // 相对当前实时位置
} zdt_position_mode;

typedef struct
{
    uint8_t address;        // 电机地址 1-255，同时是 CAN 扩展帧 ID 的高字节
    uint8_t enabled;        // Ens_TF：已使能
    uint8_t in_position;    // Prf_TF 或 0x9F 到位上报
    uint8_t stalled;        // Cgi_TF：堵转
    uint8_t protected_flag; // Cgp_TF：堵转保护已触发
    uint8_t ack_code; // 最近一次命令应答码：0x02 正确、0xE2 参数/条件错、0xEE 格式错
    uint8_t ack_pending; // 1 表示命令已入队、尚未收到应答
    uint8_t speed_valid;
    uint8_t status_valid;
    float position_deg; // 实时位置，度
    float speed_rpm;    // 实时转速，RPM
    uint32_t position_ms;
    uint32_t speed_ms;
    uint32_t status_ms;
    uint32_t position_seq; // 每收到一帧位置加 1，跳过 0
    uint32_t rx_frames;    // 校验通过的帧数
    uint32_t rx_errors;    // 校验失败或字段长度不符的帧数
} zdt_motor;

// 在 FDCAN 外设初始化之后、启动调度器之前调用一次。
HAL_StatusTypeDef ZDT_Init(FDCAN_HandleTypeDef *hfdcan);
// 注册一台电机；地址非法、重复或表满返回 NULL。
zdt_motor *ZDT_AddMotor(uint8_t address);
zdt_motor *ZDT_FindMotor(uint8_t address);

HAL_StatusTypeDef ZDT_EnableMotor(zdt_motor *motor, uint8_t enable);
// 5.3.5 速度模式：rpm_x10 有符号，单位 0.1 RPM；acc 单位 RPM/s，0 表示直接启动。
HAL_StatusTypeDef ZDT_SetSpeed(zdt_motor *motor, int16_t rpm_x10, uint16_t acc, zdt_sync sync);
// 5.3.8 直通限速位置模式：angle_x10 有符号，单位 0.1°；rpm_x10 单位 0.1 RPM。
HAL_StatusTypeDef ZDT_MovePosition(zdt_motor *motor, int32_t angle_x10, uint16_t rpm_x10,
                                   zdt_position_mode mode, zdt_sync sync);
// 5.3.13 立即停止。
HAL_StatusTypeDef ZDT_Stop(zdt_motor *motor, zdt_sync sync);
// 5.3.14 广播触发所有已缓存命令同时执行。
HAL_StatusTypeDef ZDT_SyncTrigger(void);

// 读取命令，结果在 ZDT_ProcessData 中刷新。
HAL_StatusTypeDef ZDT_RequestPosition(zdt_motor *motor);
HAL_StatusTypeDef ZDT_RequestSpeed(zdt_motor *motor);
HAL_StatusTypeDef ZDT_RequestStatus(zdt_motor *motor);

// TX FIFO 剩余空位，调用方据此保证一组命令完整入队。
uint32_t ZDT_TxFreeLevel(void);
// 接收软件队列满导致丢弃的帧数。
uint32_t ZDT_RxDropped(void);
// 只在一个任务中周期调用，把中断收到的帧解析进 zdt_motor。
void ZDT_ProcessData(void);

#endif
