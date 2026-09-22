/**
 * @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      Flash.c
 * @author    Fortiortech  Appliction Team
 * @since     Create:2021-09-23
 * @date      Last modify:2022-09-25
 * @note      Last modify author is Leo.li
 * @brief     Flash 擦写例程
 */

#include <Flash.h>

/**
 * @brief        页擦除，每次擦除2K空间,32K一共16页，
 *               每页地址固定：如第一页 0x0000--0x07FF
 *               最后一页也可以擦除，最后一个字节存在擦写保护
 * @note         Last modify author is Leo.li
 * @date         2022-09-24
 */
void Flash_Page_Erase(uint8 xdata * FlashAddress)
{
    uint8 Pre_Program = 0x25;             //预编程赋值
    uint8 Unlock_5a = 0x5a;               //解锁赋值
    uint8 Unlock_1f = 0x1f;               //解锁赋值
    uint8 Page_Erase = 0x23;              //扇区资擦除
	
    EA = 0;                               //Flash自擦除前先关总中断
    FLA_CR = Pre_Program;                 //预编程
    FLA_KEY = Unlock_5a;
    FLA_KEY = Unlock_1f;                  //flash预编程解锁
    *FlashAddress = 0xff;                 //写任意数据
    FLA_CR = 0x08;                        //开始预编程，完成后Flash再次上锁
    FLA_CR = Page_Erase;                  //使能自擦除
    FLA_KEY = Unlock_5a; 
    FLA_KEY = Unlock_1f;                  //flash预编程解锁
    *FlashAddress = 0xff;                 //写任意数据
    FLA_CR = 0x08;                        //开始预编程，完成后Flash再次上锁
	EA = 1;                               //Flash自烧写前先关总中断
}


/**
 * @brief        写入2个字节到FLASH，
 * @Input        uint8 xdata *BlockStartAddr：目标FLASH地址   
 * @Input        uint8 FlashData：被写入数据
 * @note         Last modify author is Leo.li
 * @date         2022-09-24
 */
void Flash_byte_Write(uint8 xdata * FlashAddress, uint8 FlashData)
{
    uint8 ProgramEnble = 0x01;            //预编程赋值
    uint8 Unlock_5a    = 0x5a;            //解锁赋值
    uint8 Unlock_1f    = 0x1f;            //解锁赋值
    uint8 Page_Erase   = 0x23;            //扇区资擦除	

    EA = 0;                               //Flash自烧写前先关总中断	
    FLA_CR       = ProgramEnble;          // 使能Flash编程
    FLA_KEY      = Unlock_5a;
    FLA_KEY      = Unlock_1f;             // flash预编程解锁
    *FlashAddress = FlashData;            // 写编程数据
    FLA_CR       = 0x08;                  // 开始预编程，完成后Flash再次上锁
	EA = 1;                               //Flash自烧写前先关总中断
}

/**
 * @brief        写入2个字节到FLASH，
 * @Input        uint8 xdata *BlockStartAddr：目标FLASH地址   
 * @Input        uint16 NewData2Flash：被写入数据
 * @ruturn       1:扇区未满,写入完成  0:扇区已满,写入失败
 * @note         部分地址可能没法写入，需要自动转移到下一个地址，程序中没做处理
 * @note         Last modify author is Leo.li
 * @date         2022-09-24
 */
uint8 Write2Byte2Flash(uint8 xdata * BlockStartAddr, uint16 NewData2Flash)
{
    uint8 xdata * FlashStartAddr = BlockStartAddr;
    uint16 tempofFlashData = 0;
    uint16 tempofNewFlashData = 0;
    uint8 i = 0;
    tempofNewFlashData = NewData2Flash;
    
	tempofFlashData = *(uint16 code *)(FlashStartAddr);
	
	if(tempofFlashData ==0)
	{
		tempofFlashData = *(uint16 code *)(FlashStartAddr);
        tempofFlashData = tempofNewFlashData >> 8;                          //待写入数据首字节
        Flash_byte_Write((FlashStartAddr + 2 * i), (uint8)tempofFlashData);
        tempofFlashData = tempofNewFlashData & 0x00ff;                     //待写入数据末字节
        Flash_byte_Write((FlashStartAddr + 1), (uint8)tempofFlashData);
	}

	tempofFlashData = *(uint16 code *)(FlashStartAddr);
	if(tempofFlashData == tempofNewFlashData)
	{
	   return 1;
	}
	else
	{
       return 0;
	}
}

/**
 * @brief        读取两个字节数据，
 * @Input        uint8 xdata *BlockStartAddr：目标FLASH首地址   
 * @ruturn       Flash数据
 * @note         Last modify author is Leo.li
 * @date         2022-09-24
 */
uint16 Get2ByteFromFlash(uint8 xdata * BlockStartAddr)
{
    uint8 xdata * FlashStartAddr = BlockStartAddr;
    uint16 tempofFlashData;
    
	tempofFlashData = *(uint16 code *)(FlashStartAddr);
}

/**
 * @brief        Flash 校验CRC
 * @brief        CRC_BEG: 扇区起始
 * @brief        CRC_CNT: 校验扇区数 
 * @note         Last modify author is Leo.li
 * @date         2022-09-24
 */
void Flash_CRC_APP(void)
{
	uint8 CRCL = 0;
    uint8 CRCH = 0;
	
    ClrBit(CRC_CR, AUTOINT); /* 停止计算 */
    SetBit(CRC_CR, CRCVAL);  /* 将 CRC 结果初始化为 0xFFFF */
    SetBit(CRC_CR, CRCDINI); /* 初始化有效 */
    CRC_BEG = 0;
    CRC_CNT = 127;
    SetBit(CRC_CR, AUTOINT); /* 开始计算 */
    
    while (ReadBit(CRC_CR, CRCDONE) == 0)
    {
        ;
    }
	
    SetBit(CRC_CR, CRCPNT); /* 取crc高位 */
    CRCH = CRC_DR;
	FlashTest.FlashCRCTest = CRCH;
	FlashTest.FlashCRCTest = FlashTest.FlashCRCTest <<8;
    ClrBit(CRC_CR, CRCPNT); /* 取crc低位 */
    CRCL = CRC_DR;
	FlashTest.FlashCRCTest = FlashTest.FlashCRCTest + CRCL;
}


/**
 * @brief        Flash 写入读取测试
 * @brief        uint8 xdata *BlockStartAddr：目标FLASH地址 
 * @brief        uint16 NewData2Flash：被写入数据
 * @note         Last modify author is Leo.li
 * @date         2022-09-25
 */
MCUFlash xdata FlashTest;                 //flash测试
void WriteOrReadFlashTest(uint8 xdata * BlockStartAddr, uint16 NewData2Flash)
{ 
	uint8 FlashWriteStatus = 0;
	
    Flash_Page_Erase(BlockStartAddr);      //擦除页 BlockStartAddr 所在页 注意：为2K Flash空间
	FlashWriteStatus = Write2Byte2Flash(BlockStartAddr,NewData2Flash);  //写16bit数值
	if(FlashWriteStatus)   //写入正确获取数值
	{
       FlashTest.FlashRead2Byte = Get2ByteFromFlash(BlockStartAddr);     
	}
	else
	{
		//写入失败，按实际情况处理		
	}
}
