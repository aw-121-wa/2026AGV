#include "chassis_cmd.h"
#include "critical.h"
#include "chassis.h"
#include "chassis_telemetry.h"
#include "console.h"
#include "agv_config.h"
#include "user_math.h"
#include <math.h>
#if AGV_MODE != AGV_MODE_COMMAND
#include "agv_run.h"
#endif
#if AGV_MODE == AGV_MODE_COMPETITION
#include "competition.h"
#elif AGV_MODE == AGV_MODE_ARM
#include "arm.h"
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
static char bad_char; // 第一个无法识别的字符，用于错误提示
static uint32_t divisor;
HAL_StatusTypeDef chassis_cmd_status;
static chassis_waypoint pending[CHASSIS_PATH_CAPACITY], planned;
static unsigned pending_count;
static uint8_t line_error, discard_line;

/* 控制台行：行首是 '/' 的一整行交给 console.c，不走下面的单字母命令，里面的 S 也不算停车。
   和单字母命令一样，CR、LF 或串口空闲都算行尾。 */
#define CONSOLE_LINE_MAX 63U
static char console_text[CONSOLE_LINE_MAX + 1U];
static uint8_t console_len, console_active, console_too_long;

static void ConsoleFeed(uint16_t ch)
{
    if (ch != CMD_END && ch != '\r' && ch != '\n')
    {
        if (console_len < CONSOLE_LINE_MAX)
            console_text[console_len++] = (char)ch;
        else
            console_too_long = 1U;
        return;
    }
    if (console_too_long)
        ChassisTelemetry_Printf("ERR  console line too long, nothing executed\r\n");
    else
    {
        console_text[console_len] = '\0';
        Console_Line(console_text);
    }
    console_len = console_active = console_too_long = 0U;
}

static void StopRobot(void)
{
#if AGV_MODE == AGV_MODE_COMPETITION
    Comp_Stop();
#elif AGV_MODE == AGV_MODE_ARM
    Arm_Stop();
#endif
#if AGV_MODE != AGV_MODE_COMMAND
    Agv_Stop();
#else
    Chassis_Stop();
#endif
}

/* 巡检路线是固定的，比赛流程运行中规则禁止遥控，机械臂模式底盘一直停着：这些时候不接受手动运动。 */
static uint8_t ManualLocked(void)
{
#if AGV_MODE == AGV_MODE_SURVEY || AGV_MODE == AGV_MODE_ARM
    return 1U;
#elif AGV_MODE == AGV_MODE_COMPETITION
    return Comp_Running();
#else
    return 0U;
#endif
}

/* 整行校验通过才提交，避免错误行执行半条路径。 */
static void Commit(void)
{
    if (!pending_count && !line_error)
        return;
    if (chassis.mode == CHASSIS_MODE_VEL)
        Chassis_SetMoveSpeedLimit(chassis_tune.cmd_speed);
    chassis_cmd_status = line_error ? HAL_ERROR
                                    : (chassis.mode == CHASSIS_MODE_PATH
                                           ? Chassis_AppendPath(pending, pending_count)
                                           : Chassis_FollowPath(pending, pending_count, 0));
    if (chassis_cmd_status == HAL_OK)
        planned = pending[pending_count - 1];
    ChassisTelemetry_Ack('P', chassis_cmd_status);
    pending_count = 0;
    line_error = 0;
}

/* 清空正在解析的这条命令的所有状态。 */
static void Reset(void)
{
    direction = 0;
    value = 0U;
    digits = negative = sign_seen = invalid = 0U;
    decimal = fractional = 0U;
    divisor = 1U;
    bad_char = 0;
}

/* 不进路径的即时命令：H 帮助、I 诊断、D 状态行频率、W 单轮测试。 */
static uint8_t IsMeta(char c)
{
    return c == 'H' || c == 'I' || c == 'D' || c == 'W';
}

static void ExecuteMeta(void)
{
    switch (direction)
    {
    case 'H':
        ChassisTelemetry_Help();
        chassis_cmd_status = HAL_OK;
        break;
    case 'I':
        ChassisTelemetry_Info();
        chassis_cmd_status = HAL_OK;
        break;
    case 'D':
        chassis_cmd_status = value <= 10U ? HAL_OK : HAL_ERROR;
        if (chassis_cmd_status == HAL_OK)
        {
            ChassisTelemetry_SetRate(value);
            ChassisTelemetry_Printf("OK   status line %s\r\n", value ? "on" : "off");
        }
        else
            ChassisTelemetry_Ack('D', HAL_ERROR);
        break;
    case 'W':
        chassis_cmd_status = ManualLocked()              ? HAL_BUSY
                             : (!digits || value == 0U) ? Chassis_WheelTest(0x0FU)
                             : value <= 4U ? Chassis_WheelTest((uint8_t)(1U << (value - 1U)))
                                           : HAL_ERROR;
        ChassisTelemetry_Ack('W', chassis_cmd_status);
        break;
    default:
        break;
    }
}

/* 校验解析结果并派发：F/B/L/R 定距、A 绝对航向、T 相对转向，以及即时命令。S 在 Feed 里处理。 */
static void Execute(void)
{
    float x = (negative ? -(float)value : (float)value) / (float)divisor;
    uint8_t meta = IsMeta(direction);
    uint8_t need_digits = direction && direction != 'H' && direction != 'I' && direction != 'W';
    uint8_t no_digits = direction == 'H' || direction == 'I';
    if (invalid || !direction || (need_digits && !digits) || (decimal && !fractional) ||
        (no_digits && (digits || sign_seen)) || (meta && sign_seen))
    {
        if (direction || invalid)
        {
            line_error = 1;
            chassis_cmd_status = HAL_ERROR;
            ChassisTelemetry_Ack(direction ? direction : bad_char, chassis_cmd_status);
        }
        Reset();
        return;
    }
    if (meta)
    {
        ExecuteMeta();
        Reset();
        return;
    }
    if (ManualLocked())
    {
        chassis_cmd_status = HAL_BUSY;
        ChassisTelemetry_Ack(direction, chassis_cmd_status);
        Reset();
        return;
    }
    if (!pending_count && chassis.mode != CHASSIS_MODE_PATH)
        planned = (chassis_waypoint){chassis.pos_x, chassis.pos_y, chassis.yaw};
    chassis_waypoint point = pending_count ? pending[pending_count - 1] : planned;
    switch (direction)
    {
    case 'F':
        point.x += cosf(point.heading * MATH_DEG_TO_RAD) * x;
        point.y += sinf(point.heading * MATH_DEG_TO_RAD) * x;
        break;
    case 'B':
        point.x -= cosf(point.heading * MATH_DEG_TO_RAD) * x;
        point.y -= sinf(point.heading * MATH_DEG_TO_RAD) * x;
        break;
    case 'L':
        point.x -= sinf(point.heading * MATH_DEG_TO_RAD) * x;
        point.y += cosf(point.heading * MATH_DEG_TO_RAD) * x;
        break;
    case 'R':
        point.x += sinf(point.heading * MATH_DEG_TO_RAD) * x;
        point.y -= cosf(point.heading * MATH_DEG_TO_RAD) * x;
        break;
    case 'A':
        point.heading = x;
        if (pending_count)
            --pending_count;
        break;
    case 'T':
        point.heading += x;
        if (pending_count)
            --pending_count;
        break;
    default:
        chassis_cmd_status = HAL_ERROR;
        break;
    }
    if (!isfinite(x) ||
        ((direction == 'A' || direction == 'T') ? fabsf(x) > 180
                                                : fabsf(x) > CHASSIS_MOVE_MAX_MM || x == 0))
        line_error = 1;
    if (pending_count >= CHASSIS_PATH_CAPACITY)
        line_error = 1;
    else
        pending[pending_count++] = point;
    Reset();
}

/* 命令字状态机：逐字符累积方向字母、正负号和数字，遇到结束符就执行。 */
static void Feed(uint16_t ch)
{
    if (console_active)
    {
        ConsoleFeed(ch);
        return;
    }
    /* 这一行前面没有任何内容才算控制台行。 */
    if (ch == '/' && !direction && !invalid && !pending_count && !line_error && !discard_line)
    {
        console_active = 1U;
        return;
    }
    if (ch >= 'a' && ch <= 'z')
        ch -= 'a' - 'A';
    if (ch == '?')
        ch = 'H';
    if (ch == 'S')
    {
        StopRobot();
        pending_count = 0;
        line_error = 0;
        discard_line = 1;
        Reset();
        chassis_cmd_status = HAL_OK;
        ChassisTelemetry_Ack('S', HAL_OK);
        return;
    }
    if (ch == CMD_END || ch == '\r' || ch == '\n')
    {
        if (!discard_line)
        {
            Execute();
            Commit();
        }
        discard_line = 0;
        Reset();
        return;
    }
    if (discard_line)
        return;
    if (ch == ' ' || ch == '\t')
    {
        Execute();
        return;
    }
    if (!direction && !invalid &&
        (ch == 'F' || ch == 'B' || ch == 'L' || ch == 'R' || ch == 'A' || ch == 'T' ||
         IsMeta((char)ch)))
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
    {
        if (!invalid && !direction)
            bad_char = (char)ch;
        invalid = 1U;
    }
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
    /* 在新指令重置统计之前保存完成结果，UART 忙时也可捕获。 */
    ChassisTelemetry_Poll();
    if (overflow)
    {
        critical_state irq = Critical_Enter();
        tail = head;
        overflow = 0U;
        Critical_Exit(irq);
        Reset();
        console_len = console_active = console_too_long = 0U;
        StopRobot();
        pending_count = 0;
        line_error = 0;
        discard_line = 1;
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
    /* 头尾各发一次是有意的：开头那次在解析前抢存完成统计，结尾这次发本拍刚产生的回执。
       两次都靠 gState 判断串口忙闲，不会重复发送同一条。 */
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
