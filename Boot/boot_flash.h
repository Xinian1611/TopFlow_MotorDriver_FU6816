/**
 * @file    boot_flash.h
 * @brief   boot 专用 Flash 驱动（裁剪自厂商 FU68xx_6_Flash.h，
 *          枚举、错误码、函数名与厂商完全一致）
 */
#ifndef BOOT_FLASH_H
#define BOOT_FLASH_H

#include <FU68xx_6.h>

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

extern ETypeFlashStatu Flash_Ctrl(ETypeFlashCmd eCmd, unsigned short wAddr, unsigned char ucDat);
extern ETypeFlashStatu Flash_PageErase(unsigned short wAddr);
extern ETypeFlashStatu Flash_SectorErase(unsigned short wAddr);
extern void Flash_byte_Write(unsigned short FlashAddress, unsigned char FlashData);

/* 这里用上面的厂商驱动实现 */

typedef unsigned char FLASH_Status;          /* 等价于 uint8_t */
#define FLASH_COMPLETE   1
#define FLASH_ERROR_PG   0

void         FLASH_Unlock(void);                                   /* 空实现 */
void         FLASH_Lock(void);                                     /* 空实现 */
FLASH_Status FLASH_ProgramWord(unsigned long Address, unsigned long Data);
FLASH_Status FLASH_ErasePage(unsigned long Page_Address);
FLASH_Status FLASH_EraseSector(unsigned long Sector_Address);      /* 本芯片补充：尾部不满一页时用 */

#endif