// =============================================================
// 文件：inc/usart2_dma.h —— USART2 + DMA(循环) + IDLE 分帧
// =============================================================
#ifndef USART2_DMA_H
#define USART2_DMA_H
#include "board.h"
#include "ringbuf.h"

#ifdef __cplusplus
extern "C"{
#endif

void     usart2_dma_idle_init(void);                       // GPIO/USART/DMA/IDLE/NVIC 一次性初始化
void     usart2_tx_write_blocking(const uint8_t *p, uint16_t n);
uint16_t usart2_pull_chunk(uint8_t *out, uint16_t maxlen); // 从软件环取一块数据

// 若需在别处读取统计
extern volatile uint32_t g_idle_irq_cnt;
extern volatile uint32_t g_ring_overflow_cnt;
extern volatile uint16_t g_max_chunk_len;

#ifdef __cplusplus
}
#endif

#endif // USART2_DMA_H
