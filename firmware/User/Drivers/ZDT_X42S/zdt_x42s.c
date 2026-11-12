#include "zdt_x42s.h"
#include <string.h>

#define ZDT_RX_SIZE     32U // 接收环形缓冲深度，中断写、任务读
#define ZDT_TX_STAGE    16U // 一拍最多攒的帧数，和 CubeMX 里的 TX FIFO 一样深
#define ZDT_FRAME_SIZE  8U  // 经典 CAN 单帧最大字节数
#define ZDT_PACKET_DATA 7U  // 每包 = 功能码 + 至多 7 字节命令数据
#define ZDT_MAX_ARGS    14U // 最长命令（位置模式）的参数字节数

/* 功能码 */
#define ZDT_CODE_ERROR    0x00U
#define ZDT_CODE_RPM      0x35U
#define ZDT_CODE_POSITION 0x36U
#define ZDT_CODE_STATUS   0x3AU
#define ZDT_CODE_CLEAR    0x0EU // 5.3.x 解除堵转保护
#define ZDT_CODE_ENABLE   0xF3U
#define ZDT_CODE_SPEED    0xF6U
#define ZDT_CODE_POS_X    0xFBU
#define ZDT_CODE_STOP     0xFEU
#define ZDT_CODE_SYNC     0xFFU

/* 辅助码 */
#define ZDT_PROTO_ENABLE 0xABU
#define ZDT_PROTO_STOP   0x98U
#define ZDT_PROTO_SYNC   0x66U
#define ZDT_PROTO_CLEAR  0x52U

#define ZDT_CHECKSUM 0x6BU // 出厂默认固定校验字节
#define ZDT_ACK_DONE 0x9FU // 到位，由电机主动上报
#define ZDT_ACK_OK   0x02U

/* 5.5.15 电机状态标志 */
#define ZDT_STATUS_ENABLED 0x01U // Ens_TF
#define ZDT_STATUS_IN_POS  0x02U // Prf_TF
#define ZDT_STATUS_STALLED 0x04U // Cgi_TF
#define ZDT_STATUS_PROTECT 0x08U // Cgp_TF

typedef struct
{
    uint32_t id;
    uint8_t length;
    uint8_t data[ZDT_FRAME_SIZE];
} zdt_frame;

static FDCAN_HandleTypeDef *zdt_can;
static zdt_motor motors[ZDT_MAX_MOTORS];
static uint8_t motor_count;
static zdt_frame rx_buf[ZDT_RX_SIZE];
static volatile uint16_t rx_head; // 中断写入位置
static volatile uint16_t rx_tail; // 任务读取位置
static volatile uint32_t rx_dropped;
static volatile uint32_t rx_lost;
static zdt_frame tx_stage[ZDT_TX_STAGE]; // 这一拍要发的帧，ZDT_Flush 分好组才交给硬件
static uint8_t tx_staged;
static uint32_t bus_errors;
static uint32_t bus_off_count;
static uint32_t bus_off_last_ms;

static const uint32_t zdt_dlc[ZDT_FRAME_SIZE + 1U] = {
    FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4,
    FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8,
};

/* 初始化 FDCAN：全部帧进 FIFO0、启动总线、开新报文和 FIFO 丢帧中断；单实例。 */
HAL_StatusTypeDef ZDT_Init(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == NULL)
        return HAL_ERROR;
    if (zdt_can != NULL)
        return HAL_BUSY;

    rx_head = rx_tail = 0U;
    rx_dropped = rx_lost = 0U;
    tx_staged = 0U;
    bus_errors = 0U;
    bus_off_count = 0U;
    bus_off_last_ms = 0U;
    motor_count = 0U;
    memset(motors, 0, sizeof(motors));

    // 不配硬件过滤器，地址过滤在任务里做，增删电机不用改 CubeMX。
    HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    // CubeMX 不会生成 HAL_FDCAN_Start()，必须在驱动里启动总线。
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
        return HAL_ERROR;
    if (HAL_FDCAN_ActivateNotification(
            hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST, 0U) != HAL_OK)
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
        if (motors[i].address == address)
            return &motors[i];
    return NULL;
}

zdt_motor *ZDT_AddMotor(uint8_t address)
{
    if (address == ZDT_BROADCAST_ADDR || ZDT_FindMotor(address) != NULL ||
        motor_count >= ZDT_MAX_MOTORS)
        return NULL;
    zdt_motor *motor = &motors[motor_count++];
    memset(motor, 0, sizeof(*motor));
    motor->address = address;
    return motor;
}

uint32_t ZDT_TxFreeLevel(void)
{
    if (zdt_can == NULL)
        return 0U;
    uint32_t free = HAL_FDCAN_GetTxFifoFreeLevel(zdt_can);
    if (free > ZDT_TX_STAGE)
        free = ZDT_TX_STAGE;
    return free > tx_staged ? free - tx_staged : 0U;
}

uint32_t ZDT_RxDropped(void)
{
    return rx_dropped;
}

uint32_t ZDT_BusOffCount(void)
{
    return bus_off_count;
}

uint32_t ZDT_RxLost(void)
{
    return rx_lost;
}

uint32_t ZDT_BusErrors(void)
{
    return bus_errors;
}

/* 每拍把 FDCAN 的错误计数（ECR.CEL，读一次清零）累加起来。
   总线关闭（Bus-Off）后 FDCAN 硬件会自己置 CCCR.INIT 并一直停在初始化态，
   不清 INIT 就永远不再收发：TX FIFO 堆满、所有电机命令都发不出去，只能断电重启。
   这里每 100 ms 最多检查/恢复一次：Stop 再 Start 清掉 INIT，硬件随后完成
   128×11 个隐性位的标准恢复序列。 */
void ZDT_Service(void)
{
    if (zdt_can == NULL)
        return;
    FDCAN_ErrorCountersTypeDef counters;
    if (HAL_FDCAN_GetErrorCounters(zdt_can, &counters) == HAL_OK)
        bus_errors += counters.ErrorLogging;
    uint32_t now = HAL_GetTick();
    if (now - bus_off_last_ms < 100U)
        return;
    FDCAN_ProtocolStatusTypeDef status;
    if (HAL_FDCAN_GetProtocolStatus(zdt_can, &status) != HAL_OK || status.BusOff == 0U)
        return;
    bus_off_last_ms = now;
    ++bus_off_count;
    (void)HAL_FDCAN_Stop(zdt_can);
    (void)HAL_FDCAN_Start(zdt_can);
}

/* 手册 4.2：扩展帧 ID = (地址 << 8) | 包序号；每包首字节都是功能码，后跟至多 7 字节
   命令数据，校验码只在最后一包。先确认放得下全部分包，避免只发出半条命令；帧先攒着，拍末
   ZDT_Flush 才交给硬件。 */
static HAL_StatusTypeDef ZDT_SendCommand(uint8_t address, uint8_t code, const uint8_t *args,
                                         uint8_t args_length)
{
    uint8_t body[ZDT_MAX_ARGS + 1U];
    if (zdt_can == NULL || args_length > ZDT_MAX_ARGS || (args_length && args == NULL))
        return HAL_ERROR;
    if (args_length)
        memcpy(body, args, args_length);
    body[args_length] = ZDT_CHECKSUM;
    uint8_t length = (uint8_t)(args_length + 1U);
    uint8_t packets = (uint8_t)((length + ZDT_PACKET_DATA - 1U) / ZDT_PACKET_DATA);
    if (ZDT_TxFreeLevel() < packets)
        return HAL_ERROR; // 交给调用者下个周期重试，不在这里死等

    for (uint8_t packet = 0, offset = 0; packet < packets; ++packet)
    {
        uint8_t chunk = (uint8_t)(length - offset);
        if (chunk > ZDT_PACKET_DATA)
            chunk = ZDT_PACKET_DATA;
        zdt_frame *frame = &tx_stage[tx_staged++];
        frame->id = ((uint32_t)address << 8) | packet;
        frame->length = (uint8_t)(chunk + 1U);
        frame->data[0] = code;
        memcpy(&frame->data[1], &body[offset], chunk);
        offset = (uint8_t)(offset + chunk);
    }
    return HAL_OK;
}

/* 把这一拍攒下的帧交给 TX FIFO（CubeMX 配的是 FIFO 模式，按交进去的顺序上总线）：地址大的电机先发，
   同一台的帧挨着、保持先后，广播最后。ZDT 的命令和应答用同一个 CAN ID，驱动器的应答还没发出去时
   又来一条发给它的帧，两边同时发同一个 ID 就撞成错误帧，应答常常就丢了。地址大的应答仲裁输给所有
   地址小的帧，只能等主控这一串发完：原来四条读取在前、四条速度命令在后，后轮的应答正好撞上发给
   自己的速度命令（台架上后轮回复少了 2%~4%，7 分钟总线错误两万多次，前轮一条没丢）。分好组以后，
   发给一台驱动器的帧都在它开始应答之前发完，它的应答后面只剩地址更小的帧，只会干等、不会撞。
   一组最多两帧：三帧时它对第一帧的应答在第二帧发的时候就准备好了，正好撞上第三帧，所以调用方一拍
   给一台驱动器最多攒两帧（ZDT_Staged）。 */
void ZDT_Flush(void)
{
    for (uint8_t i = 1; i < tx_staged; ++i)
    {
        zdt_frame frame = tx_stage[i];
        uint8_t j = i;
        while (j > 0U && (tx_stage[j - 1U].id >> 8) < (frame.id >> 8))
        {
            tx_stage[j] = tx_stage[j - 1U];
            --j;
        }
        tx_stage[j] = frame;
    }
    for (uint8_t i = 0; i < tx_staged; ++i)
    {
        FDCAN_TxHeaderTypeDef header = {0};
        header.Identifier = tx_stage[i].id;
        header.IdType = FDCAN_EXTENDED_ID;
        header.TxFrameType = FDCAN_DATA_FRAME;
        header.DataLength = zdt_dlc[tx_stage[i].length];
        header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        header.BitRateSwitch = FDCAN_BRS_OFF;
        header.FDFormat = FDCAN_CLASSIC_CAN;
        header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        header.MessageMarker = 0;
        // 攒的时候按硬件空位算过，这一拍里 FIFO 只会越来越空，放得下
        (void)HAL_FDCAN_AddMessageToTxFifoQ(zdt_can, &header, tx_stage[i].data);
    }
    tx_staged = 0U;
}

uint8_t ZDT_Staged(const zdt_motor *motor)
{
    uint8_t n = 0U;
    for (uint8_t i = 0; motor != NULL && i < tx_staged; ++i)
        if ((uint8_t)(tx_stage[i].id >> 8) == motor->address)
            ++n;
    return n;
}

static HAL_StatusTypeDef ZDT_SendMotorCommand(zdt_motor *motor, uint8_t code, const uint8_t *args,
                                              uint8_t args_length)
{
    if (motor == NULL)
        return HAL_ERROR;
    HAL_StatusTypeDef status = ZDT_SendCommand(motor->address, code, args, args_length);
    if (status == HAL_OK)
    {
        motor->ack_pending = 1U;
        ++motor->commands;
    }
    return status;
}

/* F3 AB <使能> <同步> 6B */
HAL_StatusTypeDef ZDT_EnableMotor(zdt_motor *motor, uint8_t enable)
{
    const uint8_t args[3] = {ZDT_PROTO_ENABLE, enable ? 1U : 0U, ZDT_SYNC_NOW};
    HAL_StatusTypeDef status = ZDT_SendMotorCommand(motor, ZDT_CODE_ENABLE, args, sizeof(args));
    if (status == HAL_OK)
        motor->enable_cmd = enable ? 1U : 0U;
    return status;
}

/* F6 <方向> <加速度 RPM/s, BE16> <速度 0.1RPM, BE16> <同步> 6B */
HAL_StatusTypeDef ZDT_SetSpeed(zdt_motor *motor, int16_t rpm_x10, uint16_t acc, zdt_sync sync)
{
    uint32_t magnitude = (uint32_t)(rpm_x10 < 0 ? -(int32_t)rpm_x10 : rpm_x10);
    if (magnitude > ZDT_MAX_RPM_X10)
        magnitude = ZDT_MAX_RPM_X10;
    const uint8_t args[6] = {rpm_x10 < 0 ? 1U : 0U,     (uint8_t)(acc >> 8), (uint8_t)acc,
                             (uint8_t)(magnitude >> 8), (uint8_t)magnitude,  (uint8_t)sync};
    return ZDT_SendMotorCommand(motor, ZDT_CODE_SPEED, args, sizeof(args));
}

/* FB <方向> <速度 0.1RPM, BE16> <角度 0.1°, BE32> <运动模式> <同步> 6B，共两包。 */
HAL_StatusTypeDef ZDT_MovePosition(zdt_motor *motor, int32_t angle_x10, uint16_t rpm_x10,
                                   zdt_position_mode mode, zdt_sync sync)
{
    uint32_t magnitude = angle_x10 < 0 ? (uint32_t)(-(int64_t)angle_x10) : (uint32_t)angle_x10;
    uint16_t speed = rpm_x10 > ZDT_MAX_RPM_X10 ? (uint16_t)ZDT_MAX_RPM_X10 : rpm_x10;
    const uint8_t args[9] = {angle_x10 < 0 ? 1U : 0U,
                             (uint8_t)(speed >> 8),
                             (uint8_t)speed,
                             (uint8_t)(magnitude >> 24),
                             (uint8_t)(magnitude >> 16),
                             (uint8_t)(magnitude >> 8),
                             (uint8_t)magnitude,
                             (uint8_t)mode,
                             (uint8_t)sync};
    HAL_StatusTypeDef status = ZDT_SendMotorCommand(motor, ZDT_CODE_POS_X, args, sizeof(args));
    if (status == HAL_OK)
        motor->in_position = 0U;
    return status;
}

/* FE 98 <同步> 6B */
HAL_StatusTypeDef ZDT_Stop(zdt_motor *motor, zdt_sync sync)
{
    const uint8_t args[2] = {ZDT_PROTO_STOP, (uint8_t)sync};
    return ZDT_SendMotorCommand(motor, ZDT_CODE_STOP, args, sizeof(args));
}

/* 0E 52 6B：解除堵转保护（Cgp_TF 置位后驱动器保持断电，必须发这条才能重新运行）。 */
HAL_StatusTypeDef ZDT_ClearProtection(zdt_motor *motor)
{
    const uint8_t args[1] = {ZDT_PROTO_CLEAR};
    return ZDT_SendMotorCommand(motor, ZDT_CODE_CLEAR, args, sizeof(args));
}

void ZDT_EmergencyStop(void)
{
    if (zdt_can == NULL)
        return;
    tx_staged = 0U;
    for (uint8_t i = 0; i < motor_count; ++i)
        (void)ZDT_Stop(&motors[i], ZDT_SYNC_NOW);
    ZDT_Flush();
}

/* 00 FF 66 6B：广播命令只有地址 1 应答。 */
HAL_StatusTypeDef ZDT_SyncTrigger(void)
{
    const uint8_t args[1] = {ZDT_PROTO_SYNC};
    return ZDT_SendCommand(ZDT_BROADCAST_ADDR, ZDT_CODE_SYNC, args, sizeof(args));
}

HAL_StatusTypeDef ZDT_RequestPosition(zdt_motor *motor)
{
    return motor ? ZDT_SendCommand(motor->address, ZDT_CODE_POSITION, NULL, 0U) : HAL_ERROR;
}

HAL_StatusTypeDef ZDT_RequestSpeed(zdt_motor *motor)
{
    return motor ? ZDT_SendCommand(motor->address, ZDT_CODE_RPM, NULL, 0U) : HAL_ERROR;
}

HAL_StatusTypeDef ZDT_RequestStatus(zdt_motor *motor)
{
    return motor ? ZDT_SendCommand(motor->address, ZDT_CODE_STATUS, NULL, 0U) : HAL_ERROR;
}

/* CAN 接收中断：记下硬件 FIFO 丢帧；抽空 FIFO0，只保留扩展数据帧，写入软件环形队列。 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (zdt_can == NULL || hfdcan != zdt_can)
        return;
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST)
        ++rx_lost;
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
        return;

    FDCAN_RxHeaderTypeDef header;
    uint8_t data[ZDT_FRAME_SIZE];
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
            break;
        if (header.IdType != FDCAN_EXTENDED_ID || header.RxFrameType != FDCAN_DATA_FRAME)
            continue;
        uint8_t length =
            header.DataLength <= ZDT_FRAME_SIZE ? (uint8_t)header.DataLength : ZDT_FRAME_SIZE;
        if (length == 0U)
            continue;
        uint16_t head = rx_head;
        uint16_t next = (uint16_t)((head + 1U) % ZDT_RX_SIZE);
        if (next == rx_tail)
        {
            ++rx_dropped; // 满时丢弃新帧并计数
            continue;
        }
        rx_buf[head].id = header.Identifier;
        rx_buf[head].length = length;
        memcpy(rx_buf[head].data, data, length);
        __DMB();
        rx_head = next;
    }
}

static uint8_t ZDT_PopFrame(zdt_frame *frame)
{
    uint16_t tail = rx_tail;
    if (rx_head == tail)
        return 0U;
    __DMB();
    *frame = rx_buf[tail];
    rx_tail = (uint16_t)((tail + 1U) % ZDT_RX_SIZE);
    return 1U;
}

static uint32_t ZDT_BigEndian(const uint8_t *data, uint8_t bytes)
{
    uint32_t value = 0U;
    for (uint8_t i = 0; i < bytes; ++i)
        value = (value << 8) | data[i];
    return value;
}

/* 解析一帧：按 ID 找到电机、校验长度与校验码，再按功能码更新。 */
static void ZDT_HandleFrame(const zdt_frame *frame)
{
    zdt_motor *motor = ZDT_FindMotor((uint8_t)((frame->id >> 8) & 0xFFU));
    if (motor == NULL)
        return;

    uint8_t length = frame->length;
    const uint8_t *data = frame->data;
    if (length < 3U || data[length - 1U] != ZDT_CHECKSUM)
    {
        ++motor->rx_errors;
        return;
    }
    uint8_t code = data[0];
    uint32_t now = HAL_GetTick();
    ++motor->rx_frames;

    switch (code)
    {
    case ZDT_CODE_POSITION: // 36 <符号> <位置 0.1°, BE32> 6B
        if (length != 7U)
            break;
        motor->position_deg = (float)ZDT_BigEndian(&data[2], 4U) * (data[1] ? -0.1f : 0.1f);
        motor->position_ms = now;
        if (++motor->position_seq == 0U)
            ++motor->position_seq;
        return;
    case ZDT_CODE_RPM: // 35 <符号> <转速 0.1RPM, BE16> 6B
        if (length != 5U)
            break;
        motor->speed_rpm = (float)ZDT_BigEndian(&data[2], 2U) * (data[1] ? -0.1f : 0.1f);
        motor->speed_ms = now;
        motor->speed_valid = 1U;
        ++motor->speed_seq;
        return;
    case ZDT_CODE_STATUS: // 3A <状态标志> 6B
        if (length != 3U)
            break;
        motor->enabled = (data[1] & ZDT_STATUS_ENABLED) ? 1U : 0U;
        motor->in_position = (data[1] & ZDT_STATUS_IN_POS) ? 1U : 0U;
        motor->stalled = (data[1] & ZDT_STATUS_STALLED) ? 1U : 0U;
        motor->protected_flag = (data[1] & ZDT_STATUS_PROTECT) ? 1U : 0U;
        motor->status_ms = now;
        motor->status_valid = 1U;
        ++motor->status_seq;
        return;
    case ZDT_CODE_ERROR: // 00 EE 6B，命令格式错误
        break;
    default:
        // 其余为命令应答：<功能码> <02/E2/EE/9F> 6B
        if (length != 3U)
            break;
        if (data[1] == ZDT_ACK_DONE)
        {
            // 到位上报是电机主动发的，不是命令应答，不清 ack_pending。
            if (code == ZDT_CODE_POS_X)
                motor->in_position = 1U;
            return;
        }
        motor->ack_code = data[1];
        motor->ack_pending = 0U;
        if (code != ZDT_CODE_SYNC) // 广播同步由地址 1 应答，不是发给它的命令
            ++motor->acks;
        if (code == ZDT_CODE_ENABLE)
            motor->enabled = data[1] == ZDT_ACK_OK && motor->enable_cmd;
        return;
    }
    ++motor->rx_errors;
}

void ZDT_ProcessData(void)
{
    zdt_frame frame;
    while (ZDT_PopFrame(&frame))
        ZDT_HandleFrame(&frame);
}
