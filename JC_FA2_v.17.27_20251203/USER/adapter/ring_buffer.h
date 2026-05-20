#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include "sys_def.h"
#define BUFFER_SIZE 40

// 定义缓冲区满时的处理策略
typedef enum {
    RB_PUT_DROP,    // 缓冲区满时丢弃新数据
    RB_PUT_OVERWRITE // 缓冲区满时覆盖最旧数据
} RingBufferPutPolicy;

// 定义环形缓冲区的结构
typedef struct {
    float buffer[BUFFER_SIZE];  // 存储数据的数组
    int head;                   // 指向最新数据的位置
    int tail;                   // 指向最旧数据的位置
    int count;                  // 当前存储的数据数量
    bool is_full;               // 缓冲区是否已满
} RingBuffer;

// 函数声明
void ring_buffer_init(RingBuffer *rb);
bool ring_buffer_put(RingBuffer *rb, float data, RingBufferPutPolicy policy);
bool ring_buffer_get(RingBuffer *rb, float *data);
bool ring_buffer_is_empty(const RingBuffer *rb);
bool ring_buffer_is_full(const RingBuffer *rb);
int ring_buffer_get_count(const RingBuffer *rb);
void ring_buffer_clear(RingBuffer *rb);
int ring_buffer_get_available(const RingBuffer *rb);
int ring_buffer_peek(const RingBuffer *rb, float *data, int max_count);

#endif
