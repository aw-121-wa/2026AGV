#include "chassis_cmd.h"
#include "critical.h"
#include "chassis.h"
#include "chassis_telemetry.h"
#include "agv_config.h"
#if AGV_MODE != AGV_MODE_COMMAND
#include "agv_run.h"
#endif

#define CMD_BUF_SIZE  32U
#define CMD_RING_SIZE 256U
#define CMD_END       256U
static UART_HandleTypeDef *cmd_uart;
static uint8_t rx_buf[CMD_BUF_SIZE];
static uint16_t ring[CMD_RING_SIZE];
static volatile uint16_t head, tail;
static volatile uint8_t overflow;
static char direction;
static uint32_t value;
static uint8_t digits, negative, sign_seen, invalid;
static uint8_t decimal, fractional;
static uint32_t divisor;
HAL_StatusTypeDef chassis_cmd_status;

/* 清空正在解析的这条命令的所有状态。 */
static void Reset(void)
{
    direction = 0;
    value = 0U;
    digits = negative = sign_seen = invalid = 0U;
    decimal = fractional = 0U;
    divisor = 1U;
}

/* 校验解析结果并派发：F/B/L/R 定距、A 绝对航向、T 相对转向、S 停车。 */
static void Execute(void)
{
    float x = (negative ? -(float)value : (float)value) / (float)divisor;
    if (invalid || !direction || (direction != 'S' && !digits) || (decimal && !fractional) ||
        (direction == 'S' && (digits || sign_seen)))
    {
        if (direction || invalid)
        {
            chassis_cmd_status = HAL_ERROR;
            ChassisTelemetry_Ack(direction, chassis_cmd_status);
        }
        Reset();
        return;
    }
#if AGV_MODE != AGV_MODE_COMMAND
    if (direction == 'S')
    {
        Agv_Stop();
        chassis_cmd_status = HAL_OK;
    }
    else
        chassis_cmd_status = HAL_BUSY;
    ChassisTelemetry_Ack(direction, chassis_cmd_status);
    Reset();
    (void)x;
    return;
#endif
    switch (direction)
    {
    case 'F':
        chassis_cmd_status = Chassis_MoveDistance(x, 0);
        break;
    case 'B':
        chassis_cmd_status = Chassis_MoveDistance(-x, 0);
        break;
    case 'L':
        chassis_cmd_status = Chassis_MoveDistance(0, x);
        break;
    case 'R':
        chassis_cmd_status = Chassis_MoveDistance(0, -x);
        break;
    case 'A':
        chassis_cmd_status = Chassis_TurnTo(x);
        break;
    case 'T':
        chassis_cmd_status = Chassis_TurnBy(x);
        break;
    case 'S':
        Chassis_Stop();
        chassis_cmd_status = HAL_OK;
        break;
    default:
        chassis_cmd_status = HAL_ERROR;
        break;
    }
    ChassisTelemetry_Ack(direction, chassis_cmd_status);
    Reset();
}

/* 命令字状态机：逐字符累积方向字母、正负号和数字，遇到结束符就执行。 */
static void Feed(uint16_t ch)
{
    if (ch == CMD_END || ch == '\r' || ch == '\n' || ch == ' ')
    {
        Execute();
        return;
    }
    if (ch >= 'a' && ch <= 'z')
        ch -= 'a' - 'A';
    if (!direction && !invalid &&
        (ch == 'F' || ch == 'B' || ch == 'L' || ch == 'R' || ch == 'A' || ch == 'T' || ch == 'S'))
    {
        direction = (char)ch;
    }
    else if (direction && !digits && !sign_seen && (ch == '-' || ch == '+'))
    {
        sign_seen = 1U;
        negative = ch == '-';
    }
    else if ((direction == 'A' || direction == 'T') && digits && !decimal && ch == '.')
    {
        decimal = 1U;
    }
    else if (direction && ch >= '0' && ch <= '9')
    {
        digits = 1U;
        if (value > 5000000U || (decimal && fractional >= 3U))
            invalid = 1U;
        else
            value = value * 10U + ch - '0';
        if (decimal && fractional < 3U)
        {
            ++fractional;
            divisor *= 10U;
        }
    }
    else
        invalid = 1U;
}

/* 把字符放进中断到任务的环形队列，满了置溢出标志。 */
static void Push(uint16_t ch)
{
    uint16_t next = (head + 1U) % CMD_RING_SIZE;
    if (next == tail)
    {
        overflow = 1U;
        return;
    }
    ring[head] = ch;
    __DMB();
    head = next;
}

/* 初始化命令串口，启动 IDLE 中断接收。 */
HAL_StatusTypeDef ChassisCmd_Init(UART_HandleTypeDef *huart)
{
    if (!huart)
        return HAL_ERROR;
    if (cmd_uart)
        return HAL_BUSY;
    Reset();
    head = tail = 0U;
    overflow = 0U;
    cmd_uart = huart;
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_IT(huart, rx_buf, CMD_BUF_SIZE);
    if (status != HAL_OK)
        cmd_uart = NULL;
    else
        ChassisTelemetry_Init(huart);
    return status;
}

/* 任务里调用：处理接收溢出、解析队列中的字符、必要时重启接收。 */
void ChassisCmd_Poll(void)
{
    if (overflow)
    {
        critical_state irq = Critical_Enter();
        tail = head;
        overflow = 0U;
        Critical_Exit(irq);
        Reset();
#if AGV_MODE != AGV_MODE_COMMAND
        Agv_Stop();
#else
        Chassis_Stop();
#endif
        chassis_cmd_status = HAL_ERROR;
        ChassisTelemetry_Ack('!', HAL_ERROR);
    }
    /* 限制单次处理量，接收溢出后停止解析。 */
    for (unsigned i = 0; i < CMD_RING_SIZE && tail != head && !overflow; ++i)
    {
        uint16_t ch = ring[tail];
        __DMB();
        tail = (tail + 1U) % CMD_RING_SIZE;
        Feed(ch);
    }
    if (cmd_uart && cmd_uart->RxState == HAL_UART_STATE_READY)
        (void)HAL_UARTEx_ReceiveToIdle_IT(cmd_uart, rx_buf, CMD_BUF_SIZE);
    ChassisTelemetry_Poll();
}

/* 串口接收中断：把本次收到的字节入队，空闲事件补一个结束符。 */
void ChassisCmd_RxEvent(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (!cmd_uart || huart != cmd_uart)
        return;
    if (Size > CMD_BUF_SIZE)
    {
        overflow = 1U;
        Size = CMD_BUF_SIZE;
    }
    for (uint16_t i = 0; i < Size; ++i)
        Push(rx_buf[i]);
    /* 缓冲区满不代表命令结束；空闲事件兼容不带换行的 F1000。 */
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_IDLE)
        Push(CMD_END);
    (void)HAL_UARTEx_ReceiveToIdle_IT(huart, rx_buf, CMD_BUF_SIZE);
}
