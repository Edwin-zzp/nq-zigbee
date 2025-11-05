#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

void timer_init(void);      // 定时器初始化
void TIM2_IRQHandler(void); // 10ms 定时器中断处理
void TIM3_IRQHandler(void); // 1s 定时器中断处理

#endif // TIMER_H
