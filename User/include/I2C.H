#ifndef __I2C_H__
#define __I2C_H__

#include <FU68xx_6.h>        /* SFR / SetBit / ClrBit / GPxx */
#include "RingBuffer.h"

typedef struct {
    uint32_t SCL_PORT;   // SCL 所在端口基地址，如 GPIOB
    uint16_t SCL_PIN;    // SCL 引脚，如 GPIO_PIN_6
    uint32_t SDA_PORT;   // SDA 所在端口基地址
    uint16_t SDA_PIN;    // SDA 引脚，如 GPIO_PIN_7
    void (*delay_us)(uint32_t us);  // 微秒延时函数
} SoftI2C_HandleTypeDef;

/* 引脚操作宏 */
#define I2C_SCL_H()         do { SetBit(P4_OE, P44); GP44 = 1; } while(0)
#define I2C_SCL_L()         do { SetBit(P4_OE, P44); GP44 = 0; } while(0)
#define I2C_SDA_H()         do { SetBit(P0_OE, P04); GP04 = 1; } while(0)
#define I2C_SDA_L()         do { SetBit(P0_OE, P04); GP04 = 0; } while(0)
#define I2C_SDA_READ()      ((P0 & P04) != 0)

#define I2C_DELAY()         delay_us(10)         /* 半位 10us ≈ 50kHz（同参考工程）*/
void delay_us(uint32_t us);

/* 函数声明（引脚固定，不带句柄） */
void SoftI2C_Init(void);
void SoftI2C_Start(void);
void SoftI2C_Stop(void);
void SoftI2C_SendByte(uint8_t byte);
uint8_t SoftI2C_ReceiveByte(void);
void SoftI2C_SendACK(void);
void SoftI2C_SendNoACK(void);
uint8_t SoftI2C_ReceiveACK(void);

/*************************************************************************************///External Function
//extern void I2C_Init_Master(void);
//extern void I2C_ID_Start(bool rw);
//extern void I2C_Addr_Write(unsigned char addr,unsigned char wdata);
//extern unsigned char I2C_Addr_Read(char addr);

#endif

