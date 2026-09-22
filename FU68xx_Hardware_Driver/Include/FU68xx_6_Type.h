/**
 * @file	 FU6866_Type.h
 * @version	 V1.0.0
 * @author	 FortiorTech Hardware Team
 * @date	 2021-07-15	22:29:47
 * @brief	 This file contains	...
 * 
 * @copyright Copyright(C) 2022, Fortior Technology	Co., Ltd. All rights reserved.
 */

#ifndef	__FU68xx_6_TYPE_H__
#define	__FU68xx_6_TYPE_H__

// Include external	header file.
#include <ctype.h> 
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************///Define Macro
#define _I                              volatile const                          /**< Defines 'read only' permissions */
#define _O                              volatile                                /**< Defines 'write only' permissions */
#define _IO                             volatile                                /**< Defines 'read&write' permissions */
/******************************************************************************///Define Type
typedef unsigned char                   uint8;
typedef unsigned short                  uint16;
typedef unsigned long                   uint32;
typedef long                            int32;
typedef short                           int16;
typedef char                            int8;

typedef enum{DISABLE = 0, ENABLE}       ebool;
/*******************************************************************************///Function Subject
#define SetReg(reg, val1, val2)         ((reg) = (reg) & (~(val1)) | (val2))    /**< reg中先对val1对应位写0, 后val2对应位写1, 其结果写回reg */
#define SetBit(reg, val)                ((reg) |=  (val))                       /**< reg中val对应位写1 */
#define ClrBit(reg, val)                ((reg) &= ~(val))                       /**< reg中val对应位写0 */
#define XorBit(reg, val)                ((reg) ^=  (val))                       /**< reg中val对应位取反 */
#define ReadBit(reg, val)               (((reg) & (val)) != 0)                  /**< 判断reg中val对应位是否为1 */


#ifdef __cplusplus
}
#endif

#endif	//__FU6866_TYPE_H__

/*** (C) Copyright 2022	Fortior	Technology Co.,	Ltd. ***/
