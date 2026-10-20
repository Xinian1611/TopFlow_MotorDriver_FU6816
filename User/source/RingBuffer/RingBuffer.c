#include "RingBuffer.h"

void rb_init(RingBuffer_t *rb, uint8_t xdata *buf, uint8_t size)
{
    rb->read  = 0;
    rb->write = 0;
    rb->mask  = size - 1;
    rb->buf   = buf;
}

uint8_t rb_write(RingBuffer_t *rb, uint8_t dat)
{
    if (rb_is_full(rb)) {
        return 0;
    }
    
    RB_ENTER_CRITICAL();
    rb->buf[rb->write] = dat;
    rb->write = (rb->write + 1) & rb->mask;
    RB_EXIT_CRITICAL();
    
    return 1;
}

uint8_t rb_read(RingBuffer_t *rb, uint8_t *dat)
{
    if (rb_is_empty(rb)) {
        return 0;
    }
    
    RB_ENTER_CRITICAL();
    *dat = rb->buf[rb->read];
    rb->read = (rb->read + 1) & rb->mask;
    RB_EXIT_CRITICAL();
    
    return 1;
}

uint8_t rb_count(RingBuffer_t *rb)
{
    return (rb->write - rb->read) & rb->mask;
}

uint8_t rb_is_empty(RingBuffer_t *rb)
{
    return (rb->read == rb->write);
}

uint8_t rb_is_full(RingBuffer_t *rb)
{
    return (((rb->write + 1) & rb->mask) == rb->read);
}
