/**
 * @file    EEPROM.h
 * @brief   BL24C256A（24AA256 兼容）EEPROM 驱动 —— 软件 I2C
 * @note    引脚：SCL=P4.4 / SDA=P0.4 / WP=P0.7（软 I2C 实现在 I2C.c）
 *          命名规范：对外接口 EEPROM_<动作>（int16_t，0=OK，非0=EEPROM_ERR_*）
 *                    内部硬件层 EEPROM_Hw<动作>（static，只在本文件用）
 */
#ifndef EEPROM_H
#define EEPROM_H

#include <FU68xx_6.h>        /* SFR / SetBit / ClrBit / GPxx */
#include "RingBuffer.h"      /* uint8_t / uint16_t / int16_t / uint32_t */

/* ---- 器件 & 总线参数 ---- */
#define EEPROM_DEV_ADDR_W        0xA0    /* 写器件地址（A2A1A0=000）*/
#define EEPROM_DEV_ADDR_R        0xA1    /* 读器件地址 */
#define EEPROM_PAGE_SIZE         64      /* 页大小（字节）*/
#define EEPROM_TOTAL_SIZE        0x8000  /* 容量 32KB：0x0000~0x7FFF */

/* ---- 引脚位掩码 ---- */
#define EEPROM_WP_PIN            P07
#define EEPROM_SCL_PIN           P44
#define EEPROM_SDA_PIN           P04

/* ---- 自检参数 ---- */
#define EEPROM_SELFCHECK_ADDR    0x6FF8  /* 回环自检地址（避开配置区 0x0000~0x005F）*/
#define EEPROM_SELFCHECK_MAGIC   0xA5    /* 回环自检写入值 */

/* ---- 返回码（唯一一套）---- */
#define EEPROM_OK                0   /* 成功 */
#define EEPROM_ERR_DEV_ACK       1   /* 器件地址无应答（总线故障 / 器件不在位）*/
#define EEPROM_ERR_ADDRH         2   /* 高字节地址无应答 */
#define EEPROM_ERR_ADDRL         3   /* 低字节地址无应答 */
#define EEPROM_ERR_DATA          4   /* 数据字节无应答 */
#define EEPROM_ERR_TIMEOUT       5   /* 写周期 ACK 轮询超时 */
#define EEPROM_ERR_PARAM         6   /* 参数错误（越界 / 跨页 / 空指针）*/
#define EEPROM_ERR_LOOPBACK      7   /* 自检：写后读回不一致 */
#define EEPROM_ERR_RESTORE       8   /* 自检：测试区原值恢复失败 */

/* ---- 对外接口（上层只用这些）---- */
int16_t EEPROM_Init(void);
int16_t EEPROM_Check(void);
int16_t EEPROM_WriteProtect(uint8_t enable);
int16_t EEPROM_WriteByte(uint16_t addr, uint8_t dat);
int16_t EEPROM_ReadByte(uint16_t addr, uint8_t *pdat);
int16_t EEPROM_WriteBuffer(uint16_t addr, uint16_t len, const uint8_t *buf);
int16_t EEPROM_ReadBuffer(uint16_t addr, uint16_t len, uint8_t *buf);
int16_t EEPROM_SelfCheck(void);

#endif /* EEPROM_H */