#ifndef __RING_BUFFER_H__
#define __RING_BUFFER_H__

#include <FU68xx_6.h>


typedef unsigned char  uint8_t;
typedef unsigned short uint16_t;
typedef signed short   int16_t;
typedef unsigned long  uint32_t;

// 原子保护：保存并关中断，退出时恢复原状态
#define RB_ENTER_CRITICAL()  do { bit __ea = EA; EA = 0;
#define RB_EXIT_CRITICAL()   EA = __ea; } while(0)

// 环形缓冲区结构体：管理变量放data区，访问最快
typedef struct {
    uint8_t  read;       // 读索引
    uint8_t  write;      // 写索引
    uint8_t  mask;       // 深度掩码（深度-1）
    uint8_t  xdata *buf; // 数据缓冲区指针（指向xdata）
} RingBuffer_t;

/**
 * @brief 初始化环形缓冲区
 * @param rb    缓冲区结构体指针
 * @param buf   数据存储数组（必须放在xdata）
 * @param size  缓冲区大小，必须为2的整数次幂（2/4/8/16/32/64/128/256）
 */
void rb_init(RingBuffer_t *rb, uint8_t xdata *buf, uint8_t size);

/**
 * @brief 写入一个字节
 * @return 1成功，0缓冲区满
 */
uint8_t rb_write(RingBuffer_t *rb, uint8_t dat);

/**
 * @brief 读取一个字节
 * @param dat 输出读取到的数据
 * @return 1成功，0缓冲区空
 */
uint8_t rb_read(RingBuffer_t *rb, uint8_t *dat);

/**
 * @brief 获取当前已存数据字节数
 */
uint8_t rb_count(RingBuffer_t *rb);

/**
 * @brief 判空
 */
uint8_t rb_is_empty(RingBuffer_t *rb);

/**
 * @brief 判满
 */
uint8_t rb_is_full(RingBuffer_t *rb);

#endif