/**
 * @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      Flash.h
 * @author    Fortiortech  Appliction Team
 * @since     Create:2021-09-23
 * @date      Last modify:2022-09-25
 * @note      Last modify author is Leo.li
 * @brief     
 */
#include <FU68xx_6.h>


#ifndef __FLASH_H__
#define __FLASH_H__

typedef struct
{	
	uint8   FlashRead; 
	uint8   FlashWrite;
	uint16  FlashRead2Byte;
	uint16  FlashWrite2Byte;
	uint16  FlashCRCTest;
    
} MCUFlash;

extern MCUFlash xdata FlashTest;                                                       //flash数据测试
extern uint16 Get2ByteFromFlash(uint8 xdata *BlockStartAddr);                          //2字节的flash读取函数   
extern uint8 Write2Byte2Flash(uint8 xdata *BlockStartAddr,uint16 NewData2Flash);       //2字节的flash写入函数
extern void WriteOrReadFlashTest(uint8 xdata *BlockStartAddr,uint16 NewData2Flash);    //读写函数定义

extern  void Flash_Page_Erase(uint8 xdata *FlashAddress);                              //自擦除函数定义  
extern  void Flash_byte_Write(uint8 xdata *FlashAddress, uint8 FlashData);             //自写入函数定义

#endif

