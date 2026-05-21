#include "ring_buffer.h"

// 初始化环形缓冲区
void ring_buffer_init(RingBuffer *rb) {
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
    rb->is_full = false;
}

// 向环形缓冲区添加数据，带策略参数
bool ring_buffer_put(RingBuffer *rb, float data, RingBufferPutPolicy policy) {
    // 如果缓冲区已满且策略是丢弃，则返回失败
    if (rb->is_full && policy == RB_PUT_DROP) {
        return false; // 数据未被写入
    }
    
    // 存储数据（无论是否覆盖，先写入当前head位置）
    rb->buffer[rb->head] = data;
    
    // 更新头指针
    rb->head = (rb->head + 1) % BUFFER_SIZE;
    
    // 处理缓冲区状态
    if (rb->is_full) {
        // 缓冲区已满，若策略是覆盖，则需要移动尾指针（丢弃最旧数据）
        if (policy == RB_PUT_OVERWRITE) {
            rb->tail = (rb->tail + 1) % BUFFER_SIZE;
        }
        // 对于丢弃策略，这里不会执行，因为前面已经返回
    } else {
        // 缓冲区未满，增加计数
        rb->count++;
        // 检查是否已满
        if (rb->count == BUFFER_SIZE) {
            rb->is_full = true;
        }
    }
    
    return true; // 数据成功写入
}

// 从环形缓冲区获取数据
bool ring_buffer_get(RingBuffer *rb, float *data) {
    if (ring_buffer_is_empty(rb)) {
        return false; // 缓冲区为空
    }
    
    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % BUFFER_SIZE;
    rb->count--;
    rb->is_full = false;
    
    return true;
}

// 判断缓冲区是否为空
bool ring_buffer_is_empty(const RingBuffer *rb) {
    return (rb->count == 0 && !rb->is_full);
}

// 判断缓冲区是否已满
bool ring_buffer_is_full(const RingBuffer *rb) {
    return rb->is_full;
}

// 获取当前数据数量
int ring_buffer_get_count(const RingBuffer *rb) {
    return rb->count;
}

// 清空缓冲区
void ring_buffer_clear(RingBuffer *rb) {
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
    rb->is_full = false;
}

// 获取可用空间大小
int ring_buffer_get_available(const RingBuffer *rb) {
    return BUFFER_SIZE - rb->count;
}

// 预览数据不删除，返回实际预览的数据数量
int ring_buffer_peek(const RingBuffer *rb, float *data, int max_count) {
    if (ring_buffer_is_empty(rb) || data == NULL || max_count <= 0) {
        return 0;
    }
    
    int read_count = 0;
    int current_pos = rb->tail;
    
    // 最多读取max_count个或缓冲区中实际存在的数据
    int actual_read = (max_count < rb->count) ? max_count : rb->count;
    
    while (read_count < actual_read) {
        data[read_count] = rb->buffer[current_pos];
        current_pos = (current_pos + 1) % BUFFER_SIZE;
        read_count++;
    }
    
    return read_count;
}
