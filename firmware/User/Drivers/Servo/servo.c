#include "agv_config.h"
#if AGV_MODE == AGV_MODE_COMPETITION
#include "servo.h"
#include "comp_config.h"
#include "main.h"

#define SERVO_PERIOD_US 20000U // 50 Hz

/* 寄存器直接配，不进 .ioc：CubeMX 重新生成不会覆盖。定时器按 1 us 计数。 */
void Servo_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &gpio);
    /* TIMPRE = 0：APB1 不分频或二分频时定时器跑 HCLK，再分频就是 PCLK1 的两倍。 */
    uint32_t clock = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) <= RCC_D2CFGR_D2PPRE1_DIV2
                         ? HAL_RCC_GetHCLKFreq()
                         : 2U * HAL_RCC_GetPCLK1Freq();
    TIM3->CR1 = 0U;
    TIM3->PSC = clock / 1000000U - 1U;
    TIM3->ARR = SERVO_PERIOD_US - 1U;
    TIM3->CCR1 = 0U; // 设角度之前不出脉冲，舵机不动
    TIM3->CCMR1 = TIM_OCMODE_PWM1 | TIM_CCMR1_OC1PE;
    TIM3->CCER = TIM_CCER_CC1E;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

void Servo_SetDeg(float deg)
{
    if (deg < 0.0f)
        deg = 0.0f;
    else if (deg > COMP_SERVO_RANGE_DEG)
        deg = COMP_SERVO_RANGE_DEG;
    TIM3->CCR1 =
        (uint32_t)(COMP_SERVO_MIN_US + (COMP_SERVO_MAX_US - COMP_SERVO_MIN_US) * deg / COMP_SERVO_RANGE_DEG + 0.5f);
}
#endif
