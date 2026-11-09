/**
 *                          @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      boot_flash.c
 * @brief     裁剪自厂商 FU68xx_6_Flash.c（第 15~166 行），只保留擦/写
 *            改动：include 由 MyProject.h 换成 FU68xx_6.h + boot_flash.h；
 *                  删除 APP 专用的 MCUFlash xdata FlashTest;
 */

/******************************************************************************/// Including Header Files
#include <FU68xx_6.h>
#include "boot_flash.h"

/******************************************************************************/// Define Macro
#define SEC_LEN_LOG  (8)
#define PAGE_LEN_LOG (3 + SEC_LEN_LOG)
#define FLASH_LEN    (32768)

#define __SEC_LAST   ((FLASH_LEN >> SEC_LEN_LOG) - 1)
#define __PAGE_LAST  ((FLASH_LEN >> PAGE_LEN_LOG) - 1)
/******************************************************************************/// Define Global Symbols
/******************************************************************************/// Function Subject
/**
 * @brief      Flash操作函数
 *
 * @param[in]  eCmd   操作命令
 * @param[in]  wAddr  编程地址
 * @param[in]  ucDat  编程内容
 *
 * @return     Flash操作结果, @see ETypeFlashStatu
 */
ETypeFlashStatu Flash_Ctrl(ETypeFlashCmd eCmd, unsigned short wAddr, unsigned char ucDat)
{
    unsigned char data   ucCmd    = (unsigned char)eCmd;
    unsigned short data  wAddrTmp = wAddr;
    unsigned char data   ucDatTmp = ucDat;
    unsigned char data   ucKey1   = 0x5a;
    unsigned char data   ucKey2   = 0x1f;
    bool                 bEA;
    unsigned char data   ucCfg;
    ETypeFlashStatu data eStatu = FLASH_UNKNOW;

    if ((ucCmd & FLAEN) == 0)
    {
        return FLASH_ERR_CMD;
    }
    else if ((ucCmd & (FLAERS | FLAPRE)) == (FLAERS | FLAPRE))
    {
        return FLASH_ERR_CMD;
    }
    else if ((ucCmd & (FLAERS | FLAPRE | PAGE_EN)) == PAGE_EN)
    {
        return FLASH_ERR_CMD;
    }

    if ((ucCmd & PAGE_EN) != 0 && (wAddr >> PAGE_LEN_LOG) >= __PAGE_LAST)
    {
        return FLASH_ERR_ADDR;
    }
    else if ((ucCmd & PAGE_EN) == 0 && (wAddr >> SEC_LEN_LOG) >= __SEC_LAST)
    {
        return FLASH_ERR_ADDR;
    }

    if (FLA_KEY == 0x06)
    {
        return FLASH_FORZEN;
    }

    bEA = EA;
    EA  = 0;
    ucCfg = CCFG1 & (WDT_EN | 0x08);
    CCFG1 &= ~(WDT_EN | 0x08);
    FLA_CR = ucCmd;
    FLA_KEY = ucKey1;
    FLA_KEY = ucKey2;

    if (FLA_KEY != 0x03)
    {
        eStatu = FLASH_ERR_UNLOCK;
        goto exit;
    }

    *(unsigned char xdata *)wAddrTmp = ucDatTmp;
    FLA_CR = 0x08;

    if ((FLA_CR & FLAERR) != 0)
    {
        eStatu = FLASH_ERROR;
        goto exit;
    }

    eStatu = FLASH_SUCCESS;
exit:
    EA    = bEA;
    CCFG1 |= ucCfg;
    return eStatu;
}


/**
 * @brief      Flash页擦除函数
 *
 * @param[in]  wAddr  编程地址
 *
 * @return     Flash操作结果, @see ETypeFlashStatu
 */
ETypeFlashStatu Flash_PageErase(unsigned short wAddr)
{
    ETypeFlashStatu eStatu;
    eStatu = Flash_Ctrl(FLASH_PAGE_PP, wAddr, 0xff);

    if (eStatu != FLASH_SUCCESS)
    {
        goto exit;
    }

    eStatu = Flash_Ctrl(FLASH_PAGE_ERS, wAddr, 0xff);
exit:
    return eStatu;
}

/**
 * @brief      Flash扇区擦除函数
 *
 * @param[in]  wAddr  编程地址
 *
 * @return     Flash操作结果, @see ETypeFlashStatu
 */
ETypeFlashStatu Flash_SectorErase(unsigned short wAddr)
{
    ETypeFlashStatu eStatu;
    eStatu = Flash_Ctrl(FLASH_SEC_PP, wAddr, 0xff);

    if (eStatu != FLASH_SUCCESS)
    {
        goto exit;
    }

    eStatu = Flash_Ctrl(FLASH_SEC_ERS, wAddr, 0xff);
exit:
    return eStatu;
}


/**
 * @brief        写入1个字节到FLASH，
 * @Input        unsigned short FlashAddress：目标FLASH地址
 * @Input        uint8 FlashData：被写入数据
 */
void Flash_byte_Write(unsigned short FlashAddress, unsigned char FlashData)
{
    unsigned char Write_status = 0;
    Write_status = (uint8)Flash_Ctrl(FLASH_PROGARM, FlashAddress, FlashData);

    if (Write_status)
    {
        ;
    }
}

/* ================= STM32 风格接口的实现 ================= */

void FLASH_Unlock(void)
{
    /* 厂商 Flash_Ctrl 内部已做 0x5A/0x1F 解锁，这里空实现，
       只为保持领导代码的结构不变 */
}

void FLASH_Lock(void)
{
    /* 厂商 Flash_Ctrl 内部已做 0x08 上锁，这里空实现 */
}

FLASH_Status FLASH_ProgramWord(unsigned long Address, unsigned long Data)
{
    unsigned char i;
    unsigned char *p = (unsigned char *)&Data;     /* 8051 小端，与 STM32 一致 */

    for (i = 0; i < 4; i++)
    {
        if (Flash_Ctrl(FLASH_PROGARM, (unsigned short)(Address + i), p[i]) != FLASH_SUCCESS)
        {
            return FLASH_ERROR_PG;
        }
    }
    return FLASH_COMPLETE;
}

FLASH_Status FLASH_ErasePage(unsigned long Page_Address)
{
    if (Flash_PageErase((unsigned short)Page_Address) == FLASH_SUCCESS)
    {
        return FLASH_COMPLETE;
    }
    return FLASH_ERROR_PG;
}

FLASH_Status FLASH_EraseSector(unsigned long Sector_Address)
{
    if (Flash_SectorErase((unsigned short)Sector_Address) == FLASH_SUCCESS)
    {
        return FLASH_COMPLETE;
    }
    return FLASH_ERROR_PG;
}
