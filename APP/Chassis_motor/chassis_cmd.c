#include "chassis_cmd.h"
#include "chassis.h"

#define CMD_BUF_SIZE 32U
#define CMD_RING_SIZE 256U
#define CMD_END 256U
static UART_HandleTypeDef *cmd_uart;
static uint8_t rx_buf[CMD_BUF_SIZE];
static uint16_t ring[CMD_RING_SIZE];
static volatile uint16_t head, tail;
static volatile uint8_t overflow;
static char direction;
static uint32_t value;
static uint8_t digits, negative, sign_seen, invalid;
HAL_StatusTypeDef chassis_cmd_status;

static void Reset(void)
{
    direction = 0;
    value = 0U;
    digits = negative = sign_seen = invalid = 0U;
}

static void Execute(void)
{
    float x = negative ? -(float)value : (float)value;
    if (invalid || !direction || (direction != 'S' && !digits) ||
        (direction == 'S' && (digits || sign_seen))) {
        if (direction || invalid) chassis_cmd_status = HAL_ERROR;
        Reset();
        return;
    }
    switch (direction) {
    case 'F': chassis_cmd_status = Chassis_MoveDistance(x, 0); break;
    case 'B': chassis_cmd_status = Chassis_MoveDistance(-x, 0); break;
    case 'L': chassis_cmd_status = Chassis_MoveDistance(0, x); break;
    case 'R': chassis_cmd_status = Chassis_MoveDistance(0, -x); break;
    case 'A': chassis_cmd_status = Chassis_TurnTo(x); break;
    case 'T': chassis_cmd_status = Chassis_TurnBy(x); break;
    case 'S': Chassis_Stop(); chassis_cmd_status = HAL_OK; break;
    default: chassis_cmd_status = HAL_ERROR; break;
    }
    Reset();
}

static void Feed(uint16_t ch)
{
    if (ch == CMD_END || ch == '\r' || ch == '\n' || ch == ' ') {
        Execute();
        return;
    }
    if (ch >= 'a' && ch <= 'z') ch -= 'a' - 'A';
    if (!direction && !invalid && (ch == 'F' || ch == 'B' || ch == 'L' ||
        ch == 'R' || ch == 'A' || ch == 'T' || ch == 'S')) {
        direction = (char)ch;
    } else if (direction && !digits && !sign_seen && (ch == '-' || ch == '+')) {
        sign_seen = 1U;
        negative = ch == '-';
    } else if (direction && ch >= '0' && ch <= '9') {
        digits = 1U;
        if (value > 5000U) invalid = 1U;
        else value = value * 10U + ch - '0';
    } else invalid = 1U;
}

static void Push(uint16_t ch)
{
    uint16_t next = (head + 1U) % CMD_RING_SIZE;
    if (next == tail) { overflow = 1U; return; }
    ring[head] = ch;
    __DMB();
    head = next;
}

HAL_StatusTypeDef ChassisCmd_Init(UART_HandleTypeDef *huart)
{
    if (!huart) return HAL_ERROR;
    if (cmd_uart) return HAL_BUSY;
    Reset();
    head = tail = 0U;
    overflow = 0U;
    cmd_uart = huart;
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_IT(huart, rx_buf, CMD_BUF_SIZE);
    if (status != HAL_OK) cmd_uart = NULL;
    return status;
}

void ChassisCmd_Poll(void)
{
    if (overflow) {
        uint32_t irq = __get_PRIMASK();
        __disable_irq();
        tail = head;
        overflow = 0U;
        __set_PRIMASK(irq);
        Reset();
        Chassis_Stop();
        chassis_cmd_status = HAL_ERROR;
    }
    /* 限制单次处理量，接收溢出后停止解析。 */
    for (unsigned i = 0; i < CMD_RING_SIZE && tail != head && !overflow; ++i) {
        uint16_t ch = ring[tail];
        __DMB();
        tail = (tail + 1U) % CMD_RING_SIZE;
        Feed(ch);
    }
    if (cmd_uart && cmd_uart->RxState == HAL_UART_STATE_READY)
        (void)HAL_UARTEx_ReceiveToIdle_IT(cmd_uart, rx_buf, CMD_BUF_SIZE);
}

void ChassisCmd_RxEvent(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (!cmd_uart || huart != cmd_uart) return;
    if (Size > CMD_BUF_SIZE) { overflow = 1U; Size = CMD_BUF_SIZE; }
    for (uint16_t i = 0; i < Size; ++i) Push(rx_buf[i]);
    /* 缓冲区满不代表命令结束；空闲事件兼容不带换行的 F1000。 */
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_IDLE) Push(CMD_END);
    (void)HAL_UARTEx_ReceiveToIdle_IT(huart, rx_buf, CMD_BUF_SIZE);
}
