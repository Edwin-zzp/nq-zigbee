// =============================================================
// 文件：inc/board.h  —— 板级参数与编译开关
// =============================================================
#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>
#include <stddef.h>

// === 工程可配 ===
#define USART2_BAUDRATE      115200
#define DMA_RX_BUF_SIZE      512     // USART2 RX 的 DMA 循环缓冲
#define RX_RING_SIZE         1024    // 软件环形缓冲（ISR→主循环）

#define CH32V203
// === 芯片库头文件（按你的工程实际选择一个分支）===
#if defined(CH32V20x) || defined(CH32V20X) || defined(CH32V203)
  #include "ch32v20x.h"
  #include "ch32v20x_rcc.h"
  #include "ch32v20x_gpio.h"
  #include "ch32v20x_usart.h"
  #include "ch32v20x_dma.h"
#elif defined(STM32F10X_MD) || defined(STM32F10X_HD)
  #include "stm32f10x.h"
  #include "stm32f10x_rcc.h"
  #include "stm32f10x_gpio.h"
  #include "stm32f10x_usart.h"
  #include "stm32f10x_dma.h"
#else
  #error "请定义 CH32V20x 或 STM32F10X_* 的编译宏，并包含对应SPL头文件"
#endif

// === 兼容性小宏（DR/DATAR, SR/STATR, CNDTR/CNTR）===
#if defined(CH32V20x) || defined(CH32V20X) || defined(CH32V203)
  #define USARTx_DR(u)     ((u)->DATAR)
  #define USARTx_SR(u)     ((u)->STATR)
  #define DMA_CNT_REG(ch)  ((ch)->CNTR)
#else
  #define USARTx_DR(u)     ((u)->DR)
  #define USARTx_SR(u)     ((u)->SR)
  #define DMA_CNT_REG(ch)  ((ch)->CNDTR)
#endif

#endif // BOARD_H
