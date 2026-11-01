#ifndef CRITICAL_H
#define CRITICAL_H

#include "FreeRTOS.h"
#include "task.h"

/* FreeRTOS BASEPRI 临界区：只屏蔽优先级数值 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(5)
   的中断，更高优先级中断不受影响。任务、中断、调度器启动前都可调用，可嵌套。
   与任务共享数据的中断（FDCAN2 = 5，UART5/USART6/DMA = 6）都在屏蔽范围内。 */
typedef UBaseType_t critical_state;

static inline critical_state Critical_Enter(void)
{
    return taskENTER_CRITICAL_FROM_ISR();
}

static inline void Critical_Exit(critical_state state)
{
    taskEXIT_CRITICAL_FROM_ISR(state);
}

#endif
