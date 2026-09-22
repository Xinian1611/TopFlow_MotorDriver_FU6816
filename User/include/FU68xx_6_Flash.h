/**
 * @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      FU68xx_6_Flash.h
 * @author    Any Lin
 * @since     2022-09-30 
 * @date      2022-10-09 
 * @note      Last modify author is Any Lin, R&D
 * @version
 * <table>
 * <tr><th>Date <th>Version <th>Author <th>Description </tr>
 * </table>
 * @brief
 */

#ifndef __FU68XX_6_FLASH_H__
#define __FU68XX_6_FLASH_H__

/******************************************************************************/// Including Header Files
#include "FU68xx_6_MCU.h"
/******************************************************************************/// Define Macro
#define USERCODEADDRESS 0x6800
#define Flash_DATA_Length  32
#define Verify_Bit         Flash_DATA_Length -1        //Data数值最后位为校验位
#define Verify_Ture        0x55                        //校验正确数值
#define Verify_Error       0xFF                        //校验错误数值
/******************************************************************************/// Define Type
/**
 * @enum 编程命令, 以下列举了有效的芯片操作指令
 */
typedef enum
{
    FLASH_PROGARM  = FLAEN,                    /**< 编程 */
    FLASH_SEC_ERS  = FLAEN | FLAERS,           /**< 扇区擦除 */
    FLASH_SEC_PP   = FLAEN | FLAPRE,           /**< 扇区预擦除 */
    FLASH_PAGE_ERS = FLAEN | FLAERS | PAGE_EN, /**< 页擦除 */
    FLASH_PAGE_PP  = FLAEN | FLAPRE | PAGE_EN, /**< 页预擦除 */
} ETypeFlashCmd;

/**
 * @enum 编程结果
 */
typedef enum
{
	FLASH_UNKNOW,      /**< 初始化 */
    FLASH_SUCCESS,     /**< 编程成功 */
    FLASH_ERR_ADDR,    /**< 由于尝试编程非法地址导致的编程错误 */
    FLASH_ERR_CMD,     /**< 由于编程命令错误导致的编程错误 */
    FLASH_ERR_UNLOCK,  /**< 由于编程解锁过程中出现的错误 */
    FLASH_FORZEN,      /**< 编程锁被冻结 */
    FLASH_ERROR        /**< 编程失败 */
} ETypeFlashStatu;

typedef struct
{
    uint8   ReturnEraseStatus;
    uint8   Flag_FlashErase;	
	uint16  FlashWriteAddr;
	uint16  FlashReadAddr;
	uint16  FlashCRC;
	
	uint8   Flash_Write_Data[64];
	uint8   Flash_Read_Data[64];
	
	uint8   Flash_FaultSource_Updat;            //新触发错误状态
	uint8   FaultSource;

	uint16  FaultHardOVCurrentCount;
	uint16  FaultSoftOVCurrentCount;
	uint16  FaultUnderVoltageCount;
	uint16  FaultOverVoltageCount;
	uint16  FaultLossPhaseCount;
	uint16  FaultStallCount;
	uint16  FaultStartCount;
	uint16  FaultTSDCount;	
} MCUFlash;

extern MCUFlash xdata FlashTest; 
/******************************************************************************/// External Symbols
/******************************************************************************/// External Function
extern ETypeFlashStatu Flash_Ctrl(ETypeFlashCmd eCmd, unsigned short wAddr, unsigned char ucDat);
extern ETypeFlashStatu Flash_PageErase(unsigned short wAddr);
extern ETypeFlashStatu Flash_SectorErase(unsigned short wAddr);

extern void Flash_byte_Write(unsigned short FlashAddress, uint8 FlashData);
extern void Flash_CRC_APP(void);
extern uint8 Write_Sector_Flash(unsigned short FlashAddress, uint16 NewData2Flash);
extern uint16 GetWrite_Black_Addr(unsigned short FlashAddress, uint8 Write_Length);
extern uint8 Write_LengthByte(unsigned short FlashAddress, uint8 Length, uint8 *str);
extern void ReadFromFlash(unsigned short BlockStartAddr,uint8 Length, uint8 * str);

extern void Flash_DataInit(void);
extern void SaveFault_Flash(void);
extern void Flash_Erase(void);
#endif
