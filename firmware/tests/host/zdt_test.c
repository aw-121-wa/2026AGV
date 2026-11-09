/* ZDT_X42S X-firmware packing and parsing against manual V1.0.3 byte layouts. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#undef assert
#define assert(x)                                                                                  \
    do                                                                                             \
    {                                                                                              \
        if (!(x))                                                                                  \
        {                                                                                          \
            fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #x);                                   \
            exit(1);                                                                               \
        }                                                                                          \
    } while (0)
#include "zdt_x42s.h"

typedef struct
{
    uint32_t id;
    uint8_t len;
    uint8_t data[8];
} frame_t;
static frame_t tx[32];
static unsigned tx_count, tx_free = 16;
static frame_t rx[8];
static unsigned rx_count, rx_index;
uint32_t HAL_GetTick(void)
{
    return 123;
}
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *h, uint32_t a, uint32_t b,
                                               uint32_t c, uint32_t d)
{
    (void)h;
    (void)a;
    (void)b;
    (void)c;
    (void)d;
    return HAL_OK;
}
static unsigned starts, stops, bus_off;
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *h)
{
    (void)h;
    ++starts;
    bus_off = 0;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *h)
{
    (void)h;
    ++stops;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *h,
                                              FDCAN_ProtocolStatusTypeDef *p)
{
    (void)h;
    p->BusOff = bus_off;
    return HAL_OK;
}
static uint32_t error_logging; /* ECR.CEL：读一次清零 */
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(const FDCAN_HandleTypeDef *h, FDCAN_ErrorCountersTypeDef *c)
{
    (void)h;
    memset(c, 0, sizeof(*c));
    c->ErrorLogging = error_logging;
    error_logging = 0;
    return HAL_OK;
}
static uint32_t notifications;
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *h, uint32_t a, uint32_t b)
{
    (void)h;
    (void)b;
    notifications = a;
    return HAL_OK;
}
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *h)
{
    (void)h;
    return tx_free;
}
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *h, FDCAN_TxHeaderTypeDef *hd,
                                                uint8_t *d)
{
    (void)h;
    assert(tx_free && hd->IdType == FDCAN_EXTENDED_ID);
    --tx_free;
    tx[tx_count].id = hd->Identifier;
    tx[tx_count].len = (uint8_t)hd->DataLength;
    memcpy(tx[tx_count].data, d, hd->DataLength);
    ++tx_count;
    return HAL_OK;
}
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *h, uint32_t f)
{
    (void)h;
    (void)f;
    return rx_count - rx_index;
}
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *h, uint32_t f,
                                         FDCAN_RxHeaderTypeDef *hd, uint8_t *d)
{
    (void)h;
    (void)f;
    frame_t *r = &rx[rx_index++];
    hd->Identifier = r->id;
    hd->IdType = FDCAN_EXTENDED_ID;
    hd->RxFrameType = FDCAN_DATA_FRAME;
    hd->DataLength = r->len;
    memcpy(d, r->data, r->len);
    return HAL_OK;
}
#include "../../User/Drivers/ZDT_X42S/zdt_x42s.c"

static void expect(unsigned i, uint32_t id, const uint8_t *data, uint8_t len)
{
    assert(tx[i].id == id && tx[i].len == len && memcmp(tx[i].data, data, len) == 0);
}
static void receive(uint32_t id, const uint8_t *data, uint8_t len)
{
    rx_count = rx_index = 0;
    rx[0].id = id;
    rx[0].len = len;
    memcpy(rx[0].data, data, len);
    rx_count = 1;
    HAL_FDCAN_RxFifo0Callback(zdt_can, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    ZDT_ProcessData();
}
int main(void)
{
    FDCAN_HandleTypeDef can = {0};
    assert(ZDT_Init(&can) == HAL_OK);
    zdt_motor *m = ZDT_AddMotor(1);
    assert(m && !ZDT_AddMotor(0) && !ZDT_AddMotor(1));

    /* 5.3.5: F6 dir acc(BE16) speed(BE16, 0.1RPM) sync 6B */
    assert(ZDT_SetSpeed(m, -15005, 10, ZDT_SYNC_CACHE) == HAL_OK);
    assert(m->ack_pending);
    assert(ZDT_SetSpeed(m, 32000, 0, ZDT_SYNC_NOW) == HAL_OK); /* clamp to 3000.0 RPM */
    /* 5.3.14: broadcast 00 FF 66 6B */
    assert(ZDT_SyncTrigger() == HAL_OK);
    /* 帧先攒着，空位扣掉攒下的；拍末 Flush 才交给硬件。 */
    assert(tx_count == 0 && ZDT_TxFreeLevel() == 13);
    ZDT_Flush();
    expect(0, 0x0100, (const uint8_t[]){0xF6, 1, 0, 10, 0x3A, 0x9D, 1, 0x6B}, 8);
    expect(1, 0x0100, (const uint8_t[]){0xF6, 0, 0, 0, 0x75, 0x30, 0, 0x6B}, 8);
    expect(2, 0x0000, (const uint8_t[]){0xFF, 0x66, 0x6B}, 3);

    /* 5.3.8 + 4.2: FB split into two packets, each starting with the function code */
    tx_count = 0;
    assert(ZDT_MovePosition(m, -36000, 10000, ZDT_POS_RELATIVE_NOW, ZDT_SYNC_NOW) == HAL_OK);
    ZDT_Flush();
    expect(0, 0x0100, (const uint8_t[]){0xFB, 1, 0x27, 0x10, 0x00, 0x00, 0x8C, 0xA0}, 8);
    expect(1, 0x0101, (const uint8_t[]){0xFB, 2, 0, 0x6B}, 4);

    /* No partial multi-packet command when the FIFO lacks space. */
    tx_count = 0;
    tx_free = 1;
    assert(ZDT_MovePosition(m, 10, 10, ZDT_POS_ABSOLUTE, ZDT_SYNC_NOW) == HAL_ERROR);
    ZDT_Flush();
    assert(tx_count == 0);
    tx_free = 16;

    assert(ZDT_Stop(m, ZDT_SYNC_NOW) == HAL_OK);
    assert(ZDT_RequestPosition(m) == HAL_OK);
    ZDT_Flush();
    expect(0, 0x0100, (const uint8_t[]){0xFE, 0x98, 0, 0x6B}, 4);
    expect(1, 0x0100, (const uint8_t[]){0x36, 0x6B}, 2);

    /* 一拍攒下的帧按电机分组：地址大的先发，同一台的挨着、保持先后，广播最后。 */
    zdt_motor *m2 = ZDT_AddMotor(2), *m4 = ZDT_AddMotor(4);
    assert(m2 && m4);
    tx_count = 0;
    assert(ZDT_RequestPosition(m) == HAL_OK && ZDT_RequestPosition(m2) == HAL_OK &&
           ZDT_RequestPosition(m4) == HAL_OK);
    assert(ZDT_SetSpeed(m, 10, 0, ZDT_SYNC_CACHE) == HAL_OK &&
           ZDT_SetSpeed(m2, 10, 0, ZDT_SYNC_CACHE) == HAL_OK &&
           ZDT_SetSpeed(m4, 10, 0, ZDT_SYNC_CACHE) == HAL_OK && ZDT_SyncTrigger() == HAL_OK);
    /* 每台攒了几帧：读取加速度命令各两帧，广播不算哪一台的。 */
    assert(ZDT_Staged(m) == 2U && ZDT_Staged(m2) == 2U && ZDT_Staged(m4) == 2U && !ZDT_Staged(NULL));
    ZDT_Flush();
    assert(ZDT_Staged(m) == 0U);
    static const uint32_t order_id[7] = {0x0400, 0x0400, 0x0200, 0x0200, 0x0100, 0x0100, 0x0000};
    static const uint8_t order_code[7] = {0x36, 0xF6, 0x36, 0xF6, 0x36, 0xF6, 0xFF};
    assert(tx_count == 7);
    for (unsigned i = 0; i < 7; ++i)
        assert(tx[i].id == order_id[i] && tx[i].data[0] == order_code[i]);
    tx_free = 16;

    /* 5.5.11: X firmware speed reply is 0.1 RPM */
    receive(0x0100, (const uint8_t[]){0x35, 1, 0x05, 0xDC, 0x6B}, 5);
    assert(m->speed_valid && m->speed_rpm < -149.99f && m->speed_rpm > -150.01f);
    assert(m->speed_seq == 1 && m->status_seq == 0);
    /* 5.5.13: X firmware position reply is 0.1 degree */
    receive(0x0100, (const uint8_t[]){0x36, 0, 0, 0, 0x0E, 0x10, 0x6B}, 7);
    assert(m->position_seq == 1 && m->position_deg > 359.99f && m->position_deg < 360.01f);
    /* 5.5.15: Ens/Prf/Cgi/Cgp bits */
    receive(0x0100, (const uint8_t[]){0x3A, 0x83, 0x6B}, 3);
    assert(m->enabled && m->in_position && !m->stalled && !m->protected_flag);
    receive(0x0100, (const uint8_t[]){0x3A, 0x0C, 0x6B}, 3);
    assert(!m->enabled && m->stalled && m->protected_flag);
    assert(m->status_seq == 2 && m->speed_seq == 1); /* 每类回复各自计数，诊断里和发出去的读取比 */
    unsigned acks = m->acks;
    receive(0x0100, (const uint8_t[]){0xF6, 0x02, 0x6B}, 3);
    assert(!m->ack_pending && m->ack_code == 0x02 && m->acks == acks + 1);
    /* 广播同步的应答不算命令应答；发给它的命令（三次速度、位置、停止，没入队的那次不算）都记下了，
       读取不算命令。 */
    receive(0x0100, (const uint8_t[]){0xFF, 0x02, 0x6B}, 3);
    assert(m->acks == acks + 1 && m->commands == 5);
    unsigned errors = m->rx_errors;
    receive(0x0100, (const uint8_t[]){0x36, 0, 0, 0x6B}, 4);
    assert(m->rx_errors == errors + 1);

    /* Full software queue drops and counts frames. */
    rx_count = rx_index = 0;
    for (unsigned i = 0; i < 8; ++i)
    {
        rx[i].id = 0x0100;
        rx[i].len = 3;
        memcpy(rx[i].data, (uint8_t[]){0xF6, 2, 0x6B}, 3);
    }
    rx_count = 8;
    for (unsigned k = 0; k < 5; ++k)
    {
        rx_index = 0;
        HAL_FDCAN_RxFifo0Callback(zdt_can, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    }
    assert(ZDT_RxDropped() == 40 - (ZDT_RX_SIZE - 1));
    ZDT_ProcessData();
    /* 硬件 FIFO 丢帧单独计数；只有丢帧标志、没有新报文时不去读 FIFO。 */
    assert(notifications & FDCAN_IT_RX_FIFO0_MESSAGE_LOST);
    rx_count = rx_index = 0;
    HAL_FDCAN_RxFifo0Callback(zdt_can, FDCAN_IT_RX_FIFO0_MESSAGE_LOST);
    assert(ZDT_RxLost() == 1 && rx_index == 0);
    /* 总线错误：每拍把读一次就清零的错误计数累加起来。 */
    error_logging = 3;
    ZDT_Service();
    ZDT_Service();
    error_logging = 2;
    ZDT_Service();
    assert(ZDT_BusErrors() == 5);
    /* 0E 52 6B: release stall protection */
    tx_count = 0;
    assert(ZDT_ClearProtection(m) == HAL_OK);
    ZDT_Flush();
    expect(0, 0x0100, (const uint8_t[]){0x0E, 0x52, 0x6B}, 3);
    /* 使能命令的应答按命令要的写：松开（机械臂示教）的应答不能算成使能了。 */
    tx_count = 0;
    tx_free = 16;
    assert(ZDT_EnableMotor(m, 0U) == HAL_OK);
    ZDT_Flush();
    expect(0, 0x0100, (const uint8_t[]){0xF3, 0xAB, 0, 0, 0x6B}, 5);
    receive(0x0100, (const uint8_t[]){0xF3, 0x02, 0x6B}, 3);
    assert(!m->enabled);
    assert(ZDT_EnableMotor(m, 1U) == HAL_OK);
    ZDT_Flush();
    receive(0x0100, (const uint8_t[]){0xF3, 0x02, 0x6B}, 3);
    assert(m->enabled);
    /* 紧急停机（栈溢出钩子）：这一拍攒的帧丢掉，每台注册过的电机一条立即停止，马上交给硬件。 */
    tx_count = 0;
    tx_free = 16;
    assert(ZDT_SetSpeed(m, 100, 0, ZDT_SYNC_NOW) == HAL_OK && ZDT_RequestPosition(m2) == HAL_OK);
    ZDT_EmergencyStop();
    assert(tx_count == 3 && ZDT_Staged(m) == 0U && ZDT_Staged(m2) == 0U);
    static const uint32_t stop_id[3] = {0x0400, 0x0200, 0x0100};
    for (unsigned i = 0; i < 3; ++i)
        expect(i, stop_id[i], (const uint8_t[]){0xFE, 0x98, 0, 0x6B}, 4);
    tx_free = 16;
    /* Bus-Off: Stop + Start clears INIT, counted, throttled to one try per 100 ms. */
    unsigned s0 = starts;
    ZDT_Service();
    assert(starts == s0 && ZDT_BusOffCount() == 0);
    bus_off = 1;
    ZDT_Service();
    assert(starts == s0 + 1 && stops == 1 && ZDT_BusOffCount() == 1);
    bus_off = 1;
    ZDT_Service(); /* same tick: throttled */
    assert(starts == s0 + 1);
    puts("PASS: X speed/position/stop/sync packing, multi-packet split, FIFO guard, per-motor TX "
         "grouping, 0.1 RPM/0.1 deg parsing, status bits, reply counts, RX drop/lost count, bus "
         "errors, clear-protection, enable/disable acks, emergency stop, bus-off recovery");
    return 0;
}
