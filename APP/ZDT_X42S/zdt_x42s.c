#include "zdt_x42s.h"
#include <string.h>

#define ZDT_RX_SIZE 16U      // 接收环形缓冲深度，中断写、任务读
#define ZDT_FRAME_SIZE 8U    // 经典 CAN 单帧最大字节数
#define ZDT_PAYLOAD_SIZE 16U // 单条命令的最大载荷（位置模式 12 字节）

/* 功能码 */
#define ZDT_CODE_ERROR 0x00U
#define ZDT_CODE_RPM 0x35U
#define ZDT_CODE_POSITION 0x36U
#define ZDT_CODE_STATUS 0x3AU
#define ZDT_CODE_ENABLE 0xF3U
#define ZDT_CODE_SPEED 0xF6U
#define ZDT_CODE_MOVE 0xFDU
#define ZDT_CODE_STOP 0xFEU

/* 辅助码 */
#define ZDT_PROTO_ENABLE 0xABU
#define ZDT_PROTO_STOP 0x98U

/* 应答码 */
#define ZDT_CHECKSUM 0x6BU   // 出厂默认固定校验字节
#define ZDT_ACK_OK 0x02U     // 命令正确；0xE2 参数或条件不满足、0xEE 格式错误，都记进 ack_code
#define ZDT_ACK_DONE 0x9FU   // 动作执行完成，由电机主动上报

/* 状态字节位定义 */
#define ZDT_STATUS_ENABLED 0x01U
#define ZDT_STATUS_IN_POS 0x02U
#define ZDT_STATUS_STALLED 0x04U
#define ZDT_STATUS_PROTECT 0x08U

/* 同步标志：0 立即执行，1 先缓存等同步命令 */
#define ZDT_SYNC_NOW 0x00U

typedef struct {
    uint32_t id;
    uint8_t length;
    uint8_t data[ZDT_FRAME_SIZE];
} zdt_frame;

static FDCAN_HandleTypeDef *zdt_can; // 单实例驱动，只初始化一次
static zdt_motor motors[ZDT_MAX_MOTORS];
static uint8_t motor_count;
static zdt_frame rx_buf[ZDT_RX_SIZE];
static volatile uint16_t rx_head; // 中断写入位置
static volatile uint16_t rx_tail; // 任务读取位置

HAL_StatusTypeDef ZDT_Init(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == NULL)
    {
        return HAL_ERROR;
    }
    if (zdt_can != NULL)
    {
        return HAL_BUSY; // 单实例驱动，只初始化一次
    }

    rx_head = 0;
    rx_tail = 0;
    motor_count = 0;
    memset(motors, 0, sizeof(motors));

    // 没有配置硬件过滤器时，由全局过滤器把不匹配的标准帧/扩展帧都放进 FIFO0，
    // 地址过滤留到任务里做，这样增删电机不用改 CubeMX。
    HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    // CubeMX 不会生成 HAL_FDCAN_Start()，必须在驱动里启动总线。
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
    {
        return HAL_ERROR;
    }

    // 只开 FIFO0 新报文中断，其余中断交给 HAL 轮询状态。
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK)
    {
        (void)HAL_FDCAN_Stop(hfdcan);
        return HAL_ERROR;
    }

    zdt_can = hfdcan;
    return HAL_OK;
}

zdt_motor *ZDT_FindMotor(uint8_t address)
{
    for (uint8_t i = 0; i < motor_count; ++i)
    {
        if (motors[i].address == address)
        {
            return &motors[i];
        }
    }
    return NULL;
}

zdt_motor *ZDT_AddMotor(uint8_t address)
{
    if (address == 0U)
    {
        return NULL; // 0 是广播地址，不能作为节点
    }
    if (ZDT_FindMotor(address) != NULL)
    {
        return NULL; // 地址重复
    }
    if (motor_count >= ZDT_MAX_MOTORS)
    {
        return NULL; // 表满
    }

    zdt_motor *motor = &motors[motor_count++];
    memset(motor, 0, sizeof(*motor));
    motor->address = address;
    return motor;
}

/* 经典帧下 DLC 编码与字节数一致，这里显式列表避免依赖这个巧合。 */
static const uint32_t zdt_dlc[ZDT_FRAME_SIZE + 1U] = {
    FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3,
    FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7,
    FDCAN_DLC_BYTES_8,
};

// 扩展帧 ID = (地址 << 8) | 包序号；载荷超过 8 字节时按 8 字节切片，包序号从 0 递增。
static HAL_StatusTypeDef ZDT_Transmit(uint8_t address, const uint8_t *payload, uint8_t length)
{
    if (zdt_can == NULL || payload == NULL || length == 0U)
    {
        return HAL_ERROR;
    }

    uint8_t packet = 0;
    uint8_t offset = 0;
    while (offset < length)
    {
        uint8_t chunk = (uint8_t)(length - offset);
        if (chunk > ZDT_FRAME_SIZE)
        {
            chunk = ZDT_FRAME_SIZE;
        }

        FDCAN_TxHeaderTypeDef header = {0};
        header.Identifier = ((uint32_t)address << 8) | packet;
        header.IdType = FDCAN_EXTENDED_ID; // 协议固定扩展帧，写错电机完全不响应
        header.TxFrameType = FDCAN_DATA_FRAME;
        header.DataLength = zdt_dlc[chunk];
        header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        header.BitRateSwitch = FDCAN_BRS_OFF;
        header.FDFormat = FDCAN_CLASSIC_CAN;
        header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        header.MessageMarker = 0;

        if (HAL_FDCAN_AddMessageToTxFifoQ(zdt_can, &header, (uint8_t *)&payload[offset]) != HAL_OK)
        {
            return HAL_ERROR; // FIFO 满，交给调用者下个周期重试，不在这里死等
        }

        offset = (uint8_t)(offset + chunk);
        ++packet;
    }
    return HAL_OK;
}

// 载荷 = 功能码 + 参数 + 校验码，校验码永远在最后一个字节。
static HAL_StatusTypeDef ZDT_SendCommand(uint8_t address, uint8_t code, const uint8_t *args, uint8_t args_length)
{
    uint8_t payload[ZDT_PAYLOAD_SIZE];

    if ((uint16_t)args_length + 2U > ZDT_PAYLOAD_SIZE)
    {
        return HAL_ERROR;
    }

    payload[0] = code;
    if (args_length > 0U)
    {
        memcpy(&payload[1], args, args_length);
    }
    payload[args_length + 1U] = ZDT_CHECKSUM; // 出厂默认固定 0x6B

    return ZDT_Transmit(address, payload, (uint8_t)(args_length + 2U));
}

HAL_StatusTypeDef ZDT_EnableMotor(zdt_motor *motor, uint8_t enable)
{
    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    // F3 AB <使能状态> <同步标志> 6B
    uint8_t args[3];
    args[0] = ZDT_PROTO_ENABLE;
    args[1] = (enable != 0U) ? 0x01U : 0x00U;
    args[2] = ZDT_SYNC_NOW;

    motor->ack_pending = 1U;
    return ZDT_SendCommand(motor->address, ZDT_CODE_ENABLE, args, sizeof(args));
}

HAL_StatusTypeDef ZDT_SetSpeed(zdt_motor *motor, int16_t rpm, uint8_t acc)
{
    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    // 方向和转速是两个字段：先把符号取出来，速度字段只放绝对值。
    uint16_t magnitude = (uint16_t)((rpm < 0) ? -rpm : rpm);
    if (magnitude > ZDT_MAX_RPM)
    {
        magnitude = ZDT_MAX_RPM; // 超限驱动器会回 0xE2，这里先兜住
    }

    // F6 <方向> <速度高> <速度低> <加速度> <同步标志> 6B
    uint8_t args[5];
    args[0] = (rpm < 0) ? 0x01U : 0x00U; // 0 正转，1 反转
    args[1] = (uint8_t)(magnitude >> 8);
    args[2] = (uint8_t)(magnitude & 0xFFU);
    args[3] = acc;                       // 0 表示不加速直接全速
    args[4] = ZDT_SYNC_NOW;

    motor->ack_pending = 1U;
    return ZDT_SendCommand(motor->address, ZDT_CODE_SPEED, args, sizeof(args));
}

HAL_StatusTypeDef ZDT_Move(zdt_motor *motor, int32_t counts, uint16_t rpm, uint8_t acc, uint8_t absolute)
{
    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    uint32_t magnitude = (counts < 0) ? (uint32_t)(-(int64_t)counts) : (uint32_t)counts;
    uint16_t speed = (rpm > ZDT_MAX_RPM) ? ZDT_MAX_RPM : rpm;

    // FD <方向> <速度高> <速度低> <加速度> <脉冲31:24 .. 7:0> <绝对/相对> <同步标志> 6B
    // 共 12 字节，必然拆成 8 + 4 两包。
    uint8_t args[10];
    args[0] = (counts < 0) ? 0x01U : 0x00U;
    args[1] = (uint8_t)(speed >> 8);
    args[2] = (uint8_t)(speed & 0xFFU);
    args[3] = acc;
    args[4] = (uint8_t)(magnitude >> 24);
    args[5] = (uint8_t)(magnitude >> 16);
    args[6] = (uint8_t)(magnitude >> 8);
    args[7] = (uint8_t)(magnitude & 0xFFU);
    args[8] = (absolute != 0U) ? 0x01U : 0x00U;
    args[9] = ZDT_SYNC_NOW;

    motor->ack_pending = 1U;
    motor->in_position = 0U; // 新目标已下发，到位标志先清掉
    return ZDT_SendCommand(motor->address, ZDT_CODE_MOVE, args, sizeof(args));
}

HAL_StatusTypeDef ZDT_Stop(zdt_motor *motor)
{
    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    // FE 98 <同步标志> 6B
    uint8_t args[2];
    args[0] = ZDT_PROTO_STOP;
    args[1] = ZDT_SYNC_NOW;

    motor->ack_pending = 1U;
    return ZDT_SendCommand(motor->address, ZDT_CODE_STOP, args, sizeof(args));
}

HAL_StatusTypeDef ZDT_RequestFeedback(zdt_motor *motor)
{
    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    // 三条读取命令都没有参数，载荷各 2 字节。
    if (ZDT_SendCommand(motor->address, ZDT_CODE_POSITION, NULL, 0U) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (ZDT_SendCommand(motor->address, ZDT_CODE_RPM, NULL, 0U) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return ZDT_SendCommand(motor->address, ZDT_CODE_STATUS, NULL, 0U);
}

static uint8_t ZDT_DlcToLength(uint32_t dlc)
{
    return (dlc <= ZDT_FRAME_SIZE) ? (uint8_t)dlc : (uint8_t)ZDT_FRAME_SIZE;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (zdt_can == NULL || hfdcan != zdt_can)
    {
        return;
    }
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
    {
        return;
    }

    FDCAN_RxHeaderTypeDef header;
    uint8_t data[ZDT_FRAME_SIZE];

    // 一次中断可能攒了多帧，循环把 FIFO 抽干。
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            break;
        }
        // 只认扩展数据帧，其余（标准帧/远程帧）直接丢弃。
        if (header.IdType != FDCAN_EXTENDED_ID || header.RxFrameType != FDCAN_DATA_FRAME)
        {
            continue;
        }

        uint8_t length = ZDT_DlcToLength(header.DataLength);
        if (length == 0U)
        {
            continue;
        }

        uint16_t next = (rx_head + 1U) % ZDT_RX_SIZE;
        if (next == rx_tail)
        {
            continue; // 满时丢弃新帧，仍继续接收
        }

        rx_buf[rx_head].id = header.Identifier;
        rx_buf[rx_head].length = length;
        memcpy(rx_buf[rx_head].data, data, length);
        rx_head = next;
    }
}

static uint8_t ZDT_PopFrame(zdt_frame *frame)
{
    if (rx_head == rx_tail)
    {
        return 0;
    }

    *frame = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1U) % ZDT_RX_SIZE;
    return 1;
}

static float ZDT_Signed(uint8_t sign, float magnitude)
{
    return (sign == 0U) ? magnitude : -magnitude;
}

static void ZDT_HandleFrame(const zdt_frame *frame)
{
    // 地址在 ID 的高字节，包序号在低字节。
    zdt_motor *motor = ZDT_FindMotor((uint8_t)((frame->id >> 8) & 0xFFU));
    if (motor == NULL)
    {
        return; // 不是本底盘注册的电机
    }

    uint8_t length = frame->length;
    const uint8_t *data = frame->data;
    if (length < 3U || data[length - 1U] != ZDT_CHECKSUM)
    {
        ++motor->rx_errors; // 长度不足或校验错
        return;
    }

    uint8_t code = data[0];
    ++motor->rx_frames;

    switch (code)
    {
    case ZDT_CODE_POSITION: // 36 <符号> <位置 4 字节> 6B
        if (length != 7U)
        {
            ++motor->rx_errors;
            return;
        }
        {
            uint32_t raw = ((uint32_t)data[2] << 24) | ((uint32_t)data[3] << 16) |
                           ((uint32_t)data[4] << 8) | (uint32_t)data[5];
            motor->position_deg = ZDT_Signed(data[1], (float)raw * ZDT_POS_LSB_DEG);
            motor->position_ms = HAL_GetTick();
            ++motor->position_seq;
            if (motor->position_seq == 0U) ++motor->position_seq;
        }
        return;

    case ZDT_CODE_RPM: // 35 <符号> <转速 2 字节> 6B
        if (length != 5U)
        {
            ++motor->rx_errors;
            return;
        }
        {
            uint16_t raw = (uint16_t)(((uint16_t)data[2] << 8) | data[3]);
            motor->speed_rpm = ZDT_Signed(data[1], (float)raw);
            motor->speed_ms = HAL_GetTick();
            motor->speed_valid = 1U;
        }
        return;

    case ZDT_CODE_STATUS: // 3A <状态字节> 6B
        if (length != 3U)
        {
            ++motor->rx_errors;
            return;
        }
        motor->enabled = ((data[1] & ZDT_STATUS_ENABLED) != 0U) ? 1U : 0U;
        motor->in_position = ((data[1] & ZDT_STATUS_IN_POS) != 0U) ? 1U : 0U;
        motor->stalled = ((data[1] & ZDT_STATUS_STALLED) != 0U) ? 1U : 0U;
        motor->protected_flag = ((data[1] & ZDT_STATUS_PROTECT) != 0U) ? 1U : 0U;
        motor->status_ms = HAL_GetTick();
        motor->status_valid = 1U;
        return;

    case ZDT_CODE_ERROR: // 00 <EE> 6B，命令格式错误
        ++motor->rx_errors;
        return;

    default:
        break;
    }

    // 剩下的 F3/F6/FD/FE 都是命令应答：<功能码> <应答码> 6B
    if (length != 3U)
    {
        ++motor->rx_errors;
        return;
    }

    if (data[1] == ZDT_ACK_DONE)
    {
        // 到位上报是电机主动发的，不是命令应答，不能清 ack_pending。
        if (code == ZDT_CODE_MOVE)
        {
            motor->in_position = 1U;
        }
        return;
    }

    motor->ack_code = data[1];
    motor->ack_pending = 0U;

    if (code == ZDT_CODE_ENABLE)
    {
        motor->enabled = (data[1] == ZDT_ACK_OK) ? 1U : 0U;
    }
}

void ZDT_ProcessData(void)
{
    zdt_frame frame;
    while (ZDT_PopFrame(&frame))
    {
        ZDT_HandleFrame(&frame);
    }
}

float ZDT_CountsToDeg(int32_t counts)
{
    return (float)counts * 360.0f / (200.0f * (float)ZDT_MICROSTEPS);
}

int32_t ZDT_DegToCounts(float deg)
{
    return (int32_t)(deg * (200.0f * (float)ZDT_MICROSTEPS) / 360.0f);
}
