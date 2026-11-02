#include "agv_config.h"
#if AGV_MODE == AGV_MODE_COMPETITION
#include "ser_port.h"
#include <string.h>

#define RX_SIZE 2048U /* 460800 bps ≈ 46 B/ms，底盘任务 5 ms 取一次，留足 40 ms 余量 */
#define TX_SIZE 512U

typedef struct
{
    UART_HandleTypeDef h;
    uint8_t rx[RX_SIZE], tx[TX_SIZE];
    volatile uint16_t rx_head, rx_tail, tx_head, tx_tail;
    SerPortStats stats;
} Port;

static Port ports[SER_COUNT];
static USART_TypeDef *const instances[SER_COUNT] = {
    [SER_LIDAR] = USART1, [SER_VISION] = USART2, [SER_SCANNER] = USART3};
static const IRQn_Type irqs[SER_COUNT] = {
    [SER_LIDAR] = USART1_IRQn, [SER_VISION] = USART2_IRQn, [SER_SCANNER] = USART3_IRQn};

static void ConfigPins(SerId id)
{
    RCC_PeriphCLKInitTypeDef clk = {0};
    GPIO_InitTypeDef gpio = {0};
    GPIO_TypeDef *port = GPIOA;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_MEDIUM;
    if (id == SER_LIDAR)
    {
        clk.PeriphClockSelection = RCC_PERIPHCLK_USART1;
        clk.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
        (void)HAL_RCCEx_PeriphCLKConfig(&clk);
        __HAL_RCC_USART1_CLK_ENABLE();
        gpio.Pin = GPIO_PIN_9 | GPIO_PIN_10;
        gpio.Alternate = GPIO_AF7_USART1;
    }
    else if (id == SER_VISION)
    {
        clk.PeriphClockSelection = RCC_PERIPHCLK_USART2;
        clk.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
        (void)HAL_RCCEx_PeriphCLKConfig(&clk);
        __HAL_RCC_USART2_CLK_ENABLE();
        gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
        gpio.Alternate = GPIO_AF7_USART2;
    }
    else
    {
        clk.PeriphClockSelection = RCC_PERIPHCLK_USART3;
        clk.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
        (void)HAL_RCCEx_PeriphCLKConfig(&clk);
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
        gpio.Alternate = GPIO_AF7_USART3;
        port = GPIOB;
    }
    HAL_GPIO_Init(port, &gpio);
}

HAL_StatusTypeDef SerPort_Init(SerId id, uint32_t baud)
{
    if (id >= SER_COUNT)
        return HAL_ERROR;
    Port *p = &ports[id];
    memset(p, 0, sizeof(*p));
    ConfigPins(id);
    /* HAL_UART_Init 只用来算波特率、写 CR1~3；它调用的 HAL_UART_MspInit（usart.c）
       对这几路没有分支，什么也不做。之后收发全部由下面的中断函数直接操作寄存器。 */
    p->h.Instance = instances[id];
    p->h.Init.BaudRate = baud;
    p->h.Init.WordLength = UART_WORDLENGTH_8B;
    p->h.Init.StopBits = UART_STOPBITS_1;
    p->h.Init.Parity = UART_PARITY_NONE;
    p->h.Init.Mode = UART_MODE_TX_RX;
    p->h.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    p->h.Init.OverSampling = UART_OVERSAMPLING_16;
    p->h.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    p->h.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    p->h.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&p->h) != HAL_OK || HAL_UARTEx_DisableFifoMode(&p->h) != HAL_OK)
        return HAL_ERROR;
    USART_TypeDef *u = p->h.Instance;
    u->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
    (void)u->RDR;
    SET_BIT(u->CR1, USART_CR1_RXNEIE_RXFNEIE); /* RXNE + ORE 中断 */
    SET_BIT(u->CR3, USART_CR3_EIE);            /* FE / NE 中断 */
    HAL_NVIC_SetPriority(irqs[id], 6, 0);
    HAL_NVIC_EnableIRQ(irqs[id]);
    return HAL_OK;
}

static void Isr(Port *p)
{
    USART_TypeDef *u = p->h.Instance;
    uint32_t isr = u->ISR;
    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE))
    {
        u->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
        ++p->stats.hw_errors;
    }
    while (u->ISR & USART_ISR_RXNE_RXFNE)
    {
        uint8_t b = (uint8_t)u->RDR;
        uint16_t next = (uint16_t)((p->rx_head + 1U) % RX_SIZE);
        if (next == p->rx_tail)
            ++p->stats.rx_overflow;
        else
        {
            p->rx[p->rx_head] = b;
            p->rx_head = next;
            ++p->stats.rx_bytes;
        }
    }
    if ((u->CR1 & USART_CR1_TXEIE_TXFNFIE) && (u->ISR & USART_ISR_TXE_TXFNF))
    {
        if (p->tx_tail != p->tx_head)
        {
            u->TDR = p->tx[p->tx_tail];
            p->tx_tail = (uint16_t)((p->tx_tail + 1U) % TX_SIZE);
        }
        else
            CLEAR_BIT(u->CR1, USART_CR1_TXEIE_TXFNFIE);
    }
}

void USART1_IRQHandler(void)
{
    Isr(&ports[SER_LIDAR]);
}

void USART2_IRQHandler(void)
{
    Isr(&ports[SER_VISION]);
}

void USART3_IRQHandler(void)
{
    Isr(&ports[SER_SCANNER]);
}

unsigned SerPort_Read(SerId id, uint8_t *dst, unsigned max)
{
    Port *p = &ports[id];
    unsigned n = 0;
    uint16_t tail = p->rx_tail;
    while (n < max && tail != p->rx_head)
    {
        dst[n++] = p->rx[tail];
        tail = (uint16_t)((tail + 1U) % RX_SIZE);
    }
    p->rx_tail = tail;
    return n;
}

void SerPort_Flush(SerId id)
{
    ports[id].rx_tail = ports[id].rx_head;
}

void SerPort_EnableRx(SerId id, uint8_t on)
{
    USART_TypeDef *u = ports[id].h.Instance;
    if (!on)
    {
        CLEAR_BIT(u->CR1, USART_CR1_RXNEIE_RXFNEIE);
        CLEAR_BIT(u->CR3, USART_CR3_EIE);
        return;
    }
    u->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
    (void)u->RDR;
    SerPort_Flush(id);
    SET_BIT(u->CR1, USART_CR1_RXNEIE_RXFNEIE);
    SET_BIT(u->CR3, USART_CR3_EIE);
}

uint8_t SerPort_Write(SerId id, const uint8_t *src, unsigned len)
{
    Port *p = &ports[id];
    uint16_t head = p->tx_head, tail = p->tx_tail;
    unsigned free_bytes = (tail + TX_SIZE - head - 1U) % TX_SIZE;
    if (len > free_bytes)
    {
        p->stats.tx_dropped += len;
        return 0;
    }
    for (unsigned i = 0; i < len; ++i)
    {
        p->tx[head] = src[i];
        head = (uint16_t)((head + 1U) % TX_SIZE);
    }
    p->tx_head = head; /* 单生产者：先写数据再发布 head，ISR 只读 head */
    SET_BIT(p->h.Instance->CR1, USART_CR1_TXEIE_TXFNFIE);
    return 1;
}

const SerPortStats *SerPort_Stats(SerId id)
{
    return &ports[id].stats;
}
#endif
