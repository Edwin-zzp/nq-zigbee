// =============================================================
// 文件：inc/ringbuf.h —— 简易环形缓冲（ISR 写，主循环读）
// =============================================================
#ifndef RINGBUF_H
#define RINGBUF_H
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"{
#endif

typedef struct {
    uint8_t  *buf;
    uint16_t  size;                 // 缓冲总大小（>0）
    volatile uint16_t head;         // 写指针（ISR 推进）
    volatile uint16_t tail;         // 读指针（主循环推进）
    volatile uint32_t overflow_cnt; // 写入不足时计数
} ringbuf_t;

void     ringbuf_init(ringbuf_t *rb, uint8_t *mem, uint16_t size);
uint16_t ringbuf_free (const ringbuf_t *rb);
uint16_t ringbuf_count(const ringbuf_t *rb);
uint16_t ringbuf_write(ringbuf_t *rb, const uint8_t *data, uint16_t len); // 尽力写
uint16_t ringbuf_read (ringbuf_t *rb, uint8_t *out, uint16_t len);        // 取出len或更少

#ifdef __cplusplus
}
#endif

#endif // RINGBUF_H