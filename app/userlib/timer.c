#include "timer.h"
#include "ch32v20x_tim.h" // 根据你的芯片型号选择对应的头文件
#include "device_hw.h"
#include "device_logic.h"

extern device_config_t g_dev_cfg; // 声明外部变量
extern device_runtime_t g_dev_rt; // 声明外部变量

void TIM3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void timer_init(void) {
  // 配置 1s 定时器（TIM3）
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // 开启定时器 TIM3 时钟

  TIM_TimeBaseInitTypeDef TIM_InitStructure;
  TIM_InitStructure.TIM_Prescaler =
      SystemCoreClock / 10000 - 1;          // 每 10ms 产生一次中断
  TIM_InitStructure.TIM_Period = 10000 - 1; // 计数 1000 次，达到 1s
  TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_InitStructure);

  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); // 使能更新中断
  TIM_Cmd(TIM3, ENABLE);                     // 启动定时器

  // 配置 10ms 定时器（TIM2）
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // 开启定时器 TIM2 时钟

  TIM_InitStructure.TIM_Prescaler =
      SystemCoreClock / 100000 - 1;        // 每 10ms 产生一次中断
  TIM_InitStructure.TIM_Period = 1000 - 1; // 计数 10 次，达到 10ms
  TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_InitStructure);

  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); // 使能更新中断
  TIM_Cmd(TIM2, ENABLE);                     // 启动定时器

  // 配置 NVIC 中断优先级，确保定时器中断能被正确响应
  NVIC_InitTypeDef NVIC_InitStructure;

  // 配置 TIM2 中断优先级
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 抢占优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;        // 子优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           // 使能 TIM2 中断
  NVIC_Init(&NVIC_InitStructure);

  // 配置 TIM3 中断优先级
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 抢占优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;        // 子优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           // 使能 TIM3 中断
  NVIC_Init(&NVIC_InitStructure);
}

void TIM3_IRQHandler(void) {
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    // printf("timer 1s interrupt");
    // 每秒中断，处理工作状态切换
    device_logic_tick_1s(&g_dev_cfg, &g_dev_rt); // 设备工作状态切换
  }
}

void TIM2_IRQHandler(void) {
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    // printf("timer 10ms interrupt");
    // 每 10ms 中断，处理 LED 闪烁和 PWM 控制
    device_logic_tick_ms(&g_dev_cfg, &g_dev_rt, 10); // 更新 LED 状态
    device_hw_apply(&g_dev_cfg, &g_dev_rt);          // 应用硬件状态
  }
}
