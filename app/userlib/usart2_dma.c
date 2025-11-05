// =============================================================
// 文件：src/usart2_dma.c —— 初始化与中断实现
// =============================================================
#include "usart2_dma.h"
#include <string.h>

#define DMA_REMAIN_CNT(ch) ((uint16_t)((ch)->CNTR))

// —— 硬件 DMA 缓冲 & 软件环形缓冲 ——
static uint8_t rx_dma_buf[DMA_RX_BUF_SIZE];
static uint8_t rx_ring_mem[RX_RING_SIZE];
static ringbuf_t rx_ring;

static volatile uint16_t dma_last_write = 0; // 上次 DMA 写指针（相对起点）

volatile uint32_t g_idle_irq_cnt = 0;
volatile uint32_t g_ring_overflow_cnt = 0;
volatile uint16_t g_max_chunk_len = 0;

void USART2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

// 读取 DMA 剩余计数，统一接口
static inline uint16_t DMA_GETCurrDataCounter(void) {
  return (uint16_t)DMA_REMAIN_CNT(DMA1_Channel6);
}

// —— 私有：启动 DMA RX 循环 ——
static void _dma_rx_start(void) {
  DMA_InitTypeDef dma;
  DMA_DeInit(DMA1_Channel6);
  dma.DMA_PeripheralBaseAddr =
      (uint32_t)&USART2->DATAR; // CH32 为 DATAR，STM32 SPL 也兼容
  dma.DMA_MemoryBaseAddr = (uint32_t)rx_dma_buf;
  dma.DMA_DIR = DMA_DIR_PeripheralSRC;
  dma.DMA_BufferSize = DMA_RX_BUF_SIZE;
  dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
  dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  dma.DMA_Mode = DMA_Mode_Circular;
  dma.DMA_Priority = DMA_Priority_Medium;
  dma.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel6, &dma);

  DMA_Cmd(DMA1_Channel6, ENABLE);
}

void usart2_dma_idle_init(void) {
  // 1) 时钟
#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
#else
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
#endif

  // 2) GPIO：PA2=TX (复用推挽), PA3=RX (浮空/上拉输入)
  GPIO_InitTypeDef gpio;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  // TX
  gpio.GPIO_Pin = GPIO_Pin_2;
  gpio.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &gpio);
  // RX
  gpio.GPIO_Pin = GPIO_Pin_3;
  gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &gpio);

  // 3) USART2 基本参数
  USART_InitTypeDef u;
  u.USART_BaudRate = USART2_BAUDRATE;
  u.USART_WordLength = USART_WordLength_8b;
  u.USART_StopBits = USART_StopBits_1;
  u.USART_Parity = USART_Parity_No;
  u.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  u.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &u);

  // 4) 使能 USART2 的 RX DMA 请求 & IDLE 中断
  USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
  USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);

  // 5) 软件环形缓冲
  ringbuf_init(&rx_ring, rx_ring_mem, RX_RING_SIZE);

  // 6) 启动 DMA RX 循环
  dma_last_write = 0;
  _dma_rx_start();

  // 7) 使能 USART2 与 NVIC
  USART_Cmd(USART2, ENABLE);

  NVIC_InitTypeDef n;
  n.NVIC_IRQChannel = USART2_IRQn;
  n.NVIC_IRQChannelPreemptionPriority = 0;
  n.NVIC_IRQChannelSubPriority = 0;
  n.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&n);
}

void usart2_tx_write_blocking(const uint8_t *p, uint16_t n) {
  for (uint16_t i = 0; i < n; i++) {
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {
    }
    USART_SendData(USART2, p[i]);
  }
  while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET) {
  }
}

uint16_t usart2_pull_chunk(uint8_t *out, uint16_t maxlen) {
  return ringbuf_read(&rx_ring, out, maxlen);
}

// —— USART2 IRQ：IDLE 分帧 ——
void USART2_IRQHandler(void) {
  if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET) {
    // 清 IDLE：按手册要求，读 SR/STATR 再读 DR/DATAR
    volatile uint16_t sr = USART2->STATR;
    (void)sr;
    volatile uint16_t dr = USART2->DATAR;
    (void)dr;

    // DMA 写入位置 = 缓冲大小 - 剩余计数
    uint16_t remain = DMA_GetCurrDataCounter(DMA1_Channel6);
    uint16_t cur = (uint16_t)(DMA_RX_BUF_SIZE - remain);
    uint16_t len = 0;

    if (cur >= dma_last_write) {
      len = cur - dma_last_write;
      if (len)
        ringbuf_write(&rx_ring, &rx_dma_buf[dma_last_write], len);
    } else {
      uint16_t l1 = DMA_RX_BUF_SIZE - dma_last_write;
      if (l1)
        ringbuf_write(&rx_ring, &rx_dma_buf[dma_last_write], l1);
      if (cur)
        ringbuf_write(&rx_ring, &rx_dma_buf[0], cur);
      len = l1 + cur;
    }
    dma_last_write = cur;

    g_idle_irq_cnt++;
    if (len > g_max_chunk_len)
      g_max_chunk_len = len;
    g_ring_overflow_cnt = rx_ring.overflow_cnt;
  }
}