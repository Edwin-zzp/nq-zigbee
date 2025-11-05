// device_hw.c —— 设备逻辑层 ↔ CH32 实际硬件 的适配实现
#include "device_hw.h"

#if defined(CH32V20x) || defined(CH32V20X) || defined(CH32V203)
#include "ch32v20x_gpio.h"
#include "ch32v20x_rcc.h"
#include "ch32v20x_tim.h"
#elif defined(STM32F10X_MD) || defined(STM32F10X_HD)
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#endif

/*====================== 硬件资源映射区（按你板子改） ======================*/
/* LED1/2/3 引脚 —— 举例用 PB0/PB1/PB3，你按实际修改 */
#define LED1_PORT GPIOB
#define LED1_PIN GPIO_Pin_0
#define LED2_PORT GPIOB
#define LED2_PIN GPIO_Pin_1
#define LED3_PORT GPIOB
#define LED3_PIN GPIO_Pin_3

/* LED 逻辑电平：1=点亮，0=熄灭（如果你板子是低电平点亮，就改成 0/1 反过来） */
#define LED_ON_LEVEL 1
#define LED_OFF_LEVEL 0

/* 蜂鸣器：这里假设在 PA8，推挽输出，同样按实际改 */
#define BUZZ_PORT GPIOA
#define BUZZ_PIN GPIO_Pin_8
#define BUZZ_ON_LEVEL 1
#define BUZZ_OFF_LEVEL 0

/* 超声波 PWM 输出：举例 TIM1_CH3N → PB15，你按实际改 */
#define USONIC_TIM TIM1                    // 使用 TIM1
#define USONIC_TIM_RCC RCC_APB2Periph_TIM1 // TIM1 时钟
#define USONIC_GPIO_PORT GPIOB
#define USONIC_GPIO_PIN GPIO_Pin_15 // PB15
#define USONIC_GPIO_RCC RCC_APB2Periph_GPIOB

#define USONIC_TIM_CLK_HZ (SystemCoreClock) // 系统时钟
#define USONIC_DUTY_PERMILL 500             // 超声波占空比 50%

/* 超声波 PWM 时钟频率（定时器输入时钟），一般等于 SystemCoreClock 或其倍数/分频
 */
#define USONIC_TIM_CLK_HZ (SystemCoreClock)

/* 超声波占空比：简单用 50%，以后需要可做成可配 */
#define USONIC_DUTY_PERMILL 500 // 千分比（500 = 50%）

/*====================== 静态工具函数 ======================*/

static void hw_set_gpio_level(GPIO_TypeDef *port, uint16_t pin, int level) {
  if (level) {
    GPIO_SetBits(port, pin);
  } else {
    GPIO_ResetBits(port, pin);
  }
}

/* 配置 LED / 蜂鸣器 GPIO 输出 */
static void hw_init_gpio(void) {
  GPIO_InitTypeDef gpio;

  /* 打开 GPIO 时钟：根据实际端口组合打开 */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                             RCC_APB2Periph_GPIOC,
                         ENABLE);

  /* LED1/2/3 */
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  gpio.GPIO_Mode = GPIO_Mode_Out_PP;

  gpio.GPIO_Pin = LED1_PIN;
  GPIO_Init(LED1_PORT, &gpio);
  gpio.GPIO_Pin = LED2_PIN;
  GPIO_Init(LED2_PORT, &gpio);
  gpio.GPIO_Pin = LED3_PIN;
  GPIO_Init(LED3_PORT, &gpio);

  hw_set_gpio_level(LED1_PORT, LED1_PIN, LED_OFF_LEVEL);
  hw_set_gpio_level(LED2_PORT, LED2_PIN, LED_OFF_LEVEL);
  hw_set_gpio_level(LED3_PORT, LED3_PIN, LED_OFF_LEVEL);

  /* 蜂鸣器 */
  gpio.GPIO_Pin = BUZZ_PIN;
  gpio.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(BUZZ_PORT, &gpio);
  hw_set_gpio_level(BUZZ_PORT, BUZZ_PIN, BUZZ_OFF_LEVEL);
}

// 初始化超声波的 PWM 输出，TIM1_CH3N 控制 PB15
static void hw_init_usonic_pwm(void) {
  GPIO_InitTypeDef gpio;
  TIM_TimeBaseInitTypeDef tb;
  TIM_OCInitTypeDef oc;

  /* 打开 TIM1 时钟 */
  RCC_APB2PeriphClockCmd(USONIC_TIM_RCC, ENABLE);

  /* 打开 GPIOB 时钟 */
  RCC_APB2PeriphClockCmd(USONIC_GPIO_RCC, ENABLE);

  /* 配置 PB15 为复用推挽输出 */
  gpio.GPIO_Pin = USONIC_GPIO_PIN;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  gpio.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(USONIC_GPIO_PORT, &gpio);

  /* 配置 TIM1 基本参数 */
  TIM_TimeBaseStructInit(&tb);
  tb.TIM_ClockDivision = TIM_CKD_DIV1;
  tb.TIM_CounterMode = TIM_CounterMode_Up;
  tb.TIM_Prescaler = SystemCoreClock / 1000000 - 1;
  tb.TIM_Period = 1000 - 1; // 设置 PWM 频率
  tb.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(USONIC_TIM, &tb);

  /* 配置 TIM1 通道 3 为 PWM 输出模式 */
  TIM_OCStructInit(&oc);
  oc.TIM_OCMode = TIM_OCMode_PWM1;
  oc.TIM_OutputState = TIM_OutputState_Enable;
  oc.TIM_OCIdleState = TIM_OCIdleState_Set;
  oc.TIM_OutputNState = TIM_OutputNState_Enable;
  oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
  oc.TIM_Pulse = 500; // 占空比为 50%
  oc.TIM_OCPolarity = TIM_OCPolarity_Low;
  oc.TIM_OCNPolarity = TIM_OCNPolarity_Low;
  TIM_OC3Init(USONIC_TIM, &oc);
  TIM_OC3PreloadConfig(USONIC_TIM, TIM_OCPreload_Disable);
  TIM_CtrlPWMOutputs(USONIC_TIM, ENABLE);
  TIM_ARRPreloadConfig(USONIC_TIM, ENABLE);

  TIM_ARRPreloadConfig(USONIC_TIM, ENABLE);

  /* 启动 TIM1 */
  TIM_Cmd(USONIC_TIM, DISABLE); // 默认关闭
}

/* 按 cfg->us_freq_hz 重新配置 PWM 频率（占空比 50%） */
void hw_usonic_set_freq(uint16_t freq_hz) {
  if (freq_hz == 0) {
    TIM_Cmd(USONIC_TIM, DISABLE); // 关闭 PWM
    return;
  }

  /* 设置 PWM 的频率 */
  uint32_t tim_clk = USONIC_TIM_CLK_HZ;
  uint16_t prescaler = (uint16_t)((tim_clk / (freq_hz * 1000UL))); // 计算预分频
  if (prescaler == 0)
    prescaler = 1;

  uint32_t cnt_clk = tim_clk / prescaler;
  uint32_t arr = cnt_clk / freq_hz;
  if (arr == 0)
    arr = 1;
  if (arr > 0xFFFF)
    arr = 0xFFFF;

  TIM_PrescalerConfig(USONIC_TIM, prescaler - 1, TIM_PSCReloadMode_Immediate);
  TIM_SetAutoreload(USONIC_TIM, (uint16_t)(arr - 1));

  /* 设置占空比为 50% */
  uint16_t pulse = (uint16_t)((arr - 1) / 2U);
  TIM_SetCompare3(USONIC_TIM, pulse);

  /* 启动 PWM 输出 */
  TIM_Cmd(USONIC_TIM, ENABLE);
  printf("Freq: %d, Prescaler: %d, ARR: %d\n", freq_hz, prescaler, arr);
}

/*====================== 对外接口实现 ======================*/

void device_hw_init(void) {
  hw_init_gpio();
  hw_init_usonic_pwm();
}

/* 记录当前 PWM 状态，避免每次都重配 */
static uint16_t s_usonic_freq_cache = 0;
static uint8_t s_usonic_on_cache = 0;

void device_hw_apply(const device_config_t *cfg, const device_runtime_t *rt) {
  if (!cfg || !rt)
    return;

  /* --------- LED1/2/3 --------- */
  /* 根据 device_logic 的 led_on[] 来设置实际电平 */
  if (device_led_should_on(rt, 0))
    hw_set_gpio_level(LED1_PORT, LED1_PIN, LED_ON_LEVEL);
  else
    hw_set_gpio_level(LED1_PORT, LED1_PIN, LED_OFF_LEVEL);

  if (device_led_should_on(rt, 1))
    hw_set_gpio_level(LED2_PORT, LED2_PIN, LED_ON_LEVEL);
  else
    hw_set_gpio_level(LED2_PORT, LED2_PIN, LED_OFF_LEVEL);

  if (device_led_should_on(rt, 2))
    hw_set_gpio_level(LED3_PORT, LED3_PIN, LED_ON_LEVEL);
  else
    hw_set_gpio_level(LED3_PORT, LED3_PIN, LED_OFF_LEVEL);

  /* --------- 蜂鸣器 --------- */
  if (device_buzz_should_on(rt))
    hw_set_gpio_level(BUZZ_PORT, BUZZ_PIN, BUZZ_ON_LEVEL);
  else
    hw_set_gpio_level(BUZZ_PORT, BUZZ_PIN, BUZZ_OFF_LEVEL);

  /* --------- 超声波 PWM --------- */
  /* 更新超声波的 PWM 输出状态 */
  uint8_t want_on = device_usonic_should_on(rt) ? 1 : 0;
  uint16_t want_f = cfg->us_freq_hz;

  if (!want_on) {
    /* 超声波关闭 */
    if (s_usonic_on_cache) {
      TIM_Cmd(USONIC_TIM, DISABLE);
      s_usonic_on_cache = 0;
    }
  } else {
    /* 超声波开启，频率变化时重新配置 */
    if (!s_usonic_on_cache || want_f != s_usonic_freq_cache) {
      hw_usonic_set_freq(want_f);
      s_usonic_on_cache = 1;
      s_usonic_freq_cache = want_f;
    }
  }
}
