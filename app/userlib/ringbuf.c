// =============================================================
// 文件：src/ringbuf.c
// =============================================================
#include "ringbuf.h"

static inline uint16_t _min16(uint16_t a, uint16_t b){ return a<b?a:b; }

void ringbuf_init(ringbuf_t *rb, uint8_t *mem, uint16_t size){
    rb->buf = mem; rb->size = size; rb->head = rb->tail = 0; rb->overflow_cnt = 0;
}

uint16_t ringbuf_free(const ringbuf_t *rb){
    uint16_t h = rb->head, t = rb->tail;
    if (h >= t) return (rb->size - (h - t) - 1);
    else        return (t - h - 1);
}

uint16_t ringbuf_count(const ringbuf_t *rb){
    uint16_t h = rb->head, t = rb->tail;
    if (h >= t) return (h - t);
    else        return (rb->size - (t - h));
}

uint16_t ringbuf_write(ringbuf_t *rb, const uint8_t *data, uint16_t len){
    uint16_t free = ringbuf_free(rb);
    if (len > free){ rb->overflow_cnt++; len = free; }
    uint16_t h = rb->head;
    uint16_t n1 = _min16(len, rb->size - h);
    for (uint16_t i=0;i<n1;i++) rb->buf[h + i] = data[i];
    h = (h + n1) % rb->size;
    uint16_t n2 = len - n1;
    for (uint16_t i=0;i<n2;i++) rb->buf[h + i] = data[n1 + i];
    rb->head = (h + n2) % rb->size;
    return n1 + n2;
}

uint16_t ringbuf_read(ringbuf_t *rb, uint8_t *out, uint16_t len){
    uint16_t cnt = ringbuf_count(rb);
    if (len > cnt) len = cnt;
    uint16_t t = rb->tail;
    uint16_t n1 = _min16(len, rb->size - t);
    for (uint16_t i=0;i<n1;i++) out[i] = rb->buf[t + i];
    t = (t + n1) % rb->size;
    uint16_t n2 = len - n1;
    for (uint16_t i=0;i<n2;i++) out[n1 + i] = rb->buf[t + i];
    rb->tail = (t + n2) % rb->size;
    return n1 + n2;
}
