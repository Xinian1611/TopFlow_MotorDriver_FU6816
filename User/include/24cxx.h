#ifndef __24CXX_H__
#define __24CXX_H__

#include <FU68xx_6.h>

/* 器件I2C地址 ----------------------------------------------------------------*/
#define AT24CXX_DEV_ADDR_W            0xA0
#define AT24CXX_DEV_ADDR_R            0xA1

/* 页大小（字节） -------------------------------------------------------------*/
#define AT24CXX_PAGE_SIZE             64
#define AT24CXX_MAX_ADDR              0x7FFF

/* I2C引脚定义 ---------------------------------------------------------------*/
#define AT24CXX_SCL_PIN               P06
#define AT24CXX_SDA_PIN               P05
#define AT24CXX_WP_PIN                GP04

/* I2C总线操作宏 -------------------------------------------------------------*/
#define AT24CXX_SCL_H()               do { GP06 = 1; } while(0)
#define AT24CXX_SCL_L()               do { GP06 = 0; } while(0)
#define AT24CXX_SDA_H()               do { ClrBit(P0_OE, P05); } while(0)   /* 输入模式，靠上拉拉高 */
#define AT24CXX_SDA_L()               do { SetBit(P0_OE, P05); GP05 = 0; } while(0) /* 输出低 */
#define AT24CXX_SDA_READ()            ((P0 & P05) != 0)

/* WP引脚操作 ---------------------------------------------------------------*/
#define AT24CXX_WP_SET()              do { GP04 = 1; } while(0)
#define AT24CXX_WP_CLR()              do { GP04 = 0; } while(0)

/* 擦除模式 ------------------------------------------------------------------*/
#define AT24CXX_ERASE_SECTOR          0   /* 扇区擦除（指定页数） */
#define AT24CXX_ERASE_CHIP            1   /* 整片擦除 */

/* 外部接口函数 -------------------------------------------------------------*/
extern void     AT24Cxx_Init(void);
extern void     AT24Cxx_WriteEnable(void);
extern void     AT24Cxx_WriteDisable(void);
extern uint8    AT24Cxx_ReadByte(uint16 addr);
extern uint16   AT24Cxx_ReadBytes(uint16 addr, uint8 *buf, uint16 len);
extern uint8    AT24Cxx_WriteByte(uint16 addr, uint8 dat);
extern uint8    AT24Cxx_WriteBytes(uint16 addr, const uint8 *buf, uint16 len);
extern uint8    AT24Cxx_WritePage(uint16 addr, const uint8 *buf, uint16 len);
extern uint8    AT24Cxx_PageFill(uint16 page_index, uint8 val);
extern void     AT24Cxx_Erase(uint16 page_start, uint16 page_cnt, uint8 mode);

#endif
