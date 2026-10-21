#ifndef ZDT_X42S_H
#define ZDT_X42S_H

#include "fdcan.h"

/* 底盘挂接的电机数量，按实际轴数修改。 */
#define ZDT_MAX_MOTORS 4U
/* 驱动器细分，位置指令按 200 * 细分 折算成一圈。 */
#define ZDT_MICROSTEPS 16U
/* 允许下发的最高转速，单位：RPM。超限会被驱动器判为参数错误。 */
#define ZDT_MAX_RPM 3000U
/* 速度模式默认加速度档位，0 表示不加速直接全速。 */
#define ZDT_DEFAULT_ACC 200U
/* 反馈位置换算：X 固件为 0.1 度/LSB；若换成 Emm 固件改为 360.0f / 65536.0f。 */
#define ZDT_POS_LSB_DEG 0.1f

/* 电机地址，按实车接线修改；0 是广播地址，不能用作节点地址。 */
typedef enum {
    ZDT_ADDR_LF = 1,
    ZDT_ADDR_RF = 2,
    ZDT_ADDR_LR = 3,
    ZDT_ADDR_RR = 4,
} zdt_address;

typedef struct {
    uint8_t address;        // 电机地址 1-255，同时是 CAN 扩展帧 ID 的高字节
    uint8_t enabled;        // 1 表示已使能锁轴
    uint8_t in_position;    // 1 表示已收到到位返回（0x9F）
    uint8_t stalled;        // 1 表示堵转
    uint8_t protected_flag; // 1 表示过流/过热/低压保护
    uint8_t ack_code;       // 最近一次命令应答码：0x02 正确、0xE2 参数错、0xEE 格式错
    uint8_t ack_pending;    // 1 表示已下发命令、尚未收到应答
    float position_deg;     // 实时位置，单位：度，符号已还原
    float speed_rpm;        // 实时转速，单位：RPM，符号已还原
    uint32_t rx_frames;     // 校验通过的帧数
    uint32_t rx_errors;     // 校验失败或字段长度不符的帧数
} zdt_motor;

// 在 FDCAN 外设初始化之后、启动调度器之前调用一次；返回 HAL_OK 表示总线与接收已就绪。
HAL_StatusTypeDef ZDT_Init(FDCAN_HandleTypeDef *hfdcan);
// 注册一台电机，返回其句柄；地址非法、重复或表满返回 NULL。
zdt_motor *ZDT_AddMotor(uint8_t address);
// 按地址查找已注册的电机，未注册返回 NULL。
zdt_motor *ZDT_FindMotor(uint8_t address);
// 使能/失能，enable 非 0 即使能；失能后电机松轴。
HAL_StatusTypeDef ZDT_EnableMotor(zdt_motor *motor, uint8_t enable);
// 速度模式：rpm 为负有符号转速，acc 为 0-255 加速度档位。
HAL_StatusTypeDef ZDT_SetSpeed(zdt_motor *motor, int16_t rpm, uint8_t acc);
// 位置模式：counts 为负有符号脉冲数，absolute 非 0 表示绝对位置。
HAL_StatusTypeDef ZDT_Move(zdt_motor *motor, int32_t counts, uint16_t rpm, uint8_t acc, uint8_t absolute);
// 立即停止，不减速直接断输出。
HAL_StatusTypeDef ZDT_Stop(zdt_motor *motor);
// 请求一次位置、转速、状态反馈，三条命令连发，结果在 ZDT_ProcessData 中刷新。
HAL_StatusTypeDef ZDT_RequestFeedback(zdt_motor *motor);
// 只在一个任务中周期调用，把中断收到的帧解析进 zdt_motor。
void ZDT_ProcessData(void);

// 脉冲数与角度互换。
float ZDT_CountsToDeg(int32_t counts);
int32_t ZDT_DegToCounts(float deg);

#endif
