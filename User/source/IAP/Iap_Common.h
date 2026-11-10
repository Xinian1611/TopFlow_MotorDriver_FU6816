#ifndef IAP_COMMON_H
#define IAP_COMMON_H

/*=============================================================================
 * Flash 分区（Boot / APP 两边必须完全一致）
 *
 *   0x0000 ~ 0x0EFF   Bootloader 代码（上限 3840B，必须由链接器 Code Range 卡死）
 *   0x0F00 ~ 0x0FFF   参数页（1 个 256B 扇区）
 *                       +0x00 : 跳转标志 55 AA 12 34 D9 + XOR（5B）
 *                       +0x08 : 本机地址 55 AA + 4B(ID=addr<<4，大端) + XOR（7B）
 *   0x1000 ~ 0x17FF   预留（未使用；以后可放参数备份页/扩 boot）
 *   0x1800 ~ 0x7EFF   APP 程序（2KB 页对齐，最多 26,368B）
 *   0x7F00 ~ 0x7FFF   最后一个扇区，芯片禁止擦写，不要用
 *
 * 注意：
 *   1) APP 在 0x1800，复位/中断向量由 STARTUP_APP1800.A51 放在 0x1800/0x1803；
 *      Boot 的向量表在 0x0003~0x007B，内容为 LJMP 1800H+偏移。
 *   2) 参数页在 Boot 区内，APP 擦写参数页前必须确认 Boot 不用这一段
 *      （当前 Boot 已经越过 0x0F00，必须先压缩 boot，见 Boot 工程说明）。
 *   3) APP 代码区上限由 Keil 的 Code Range（0x1800~0x7EFF）保证，
 *      超了要在编译期报错，不要靠运行期发现。
 *===========================================================================*/

#define IAPData_FLASH_OFFSET   0x1700    /* 参数区起始（boot 与 app 共用） */
#define APP_FLASH_OFFSET       0x1800    /* APP 起始地址（2KB 对齐） */
#define IAPFlag_FLASH_OFFSET   0x1700    /* 跳转标志在参数区内的偏移 */
#define IAPID_FLASH_OFFSET     0x1708    /* 本机 CAN 地址在参数区内的偏移 */
#define STM_FLASH_BASE         ((uint32_t)0x0000)
#define STM_FLASH_PAGE_SIZE    ((uint32_t)0x0100)   /* 扇区 256B */

/* 参数页内偏移 */
#define ExcuteFlg_Offset   0
#define ExcuteFlgSize      5
#define StationID_Offset   8
#define StationIDSize      7

//IAP命令码定义--------------------------------------------------------------
#define CmdFail                            0
#define OnlineCheck                        1
#define EraseFlash                         3
#define SetBaudRate                        4
#define BlockWriteInfo                     5
#define WriteBlockFlash                    6
#define BlockReadInfo                      7
#define ReadBlockFlash                     8
#define ExcuteApp                          9
#define CmdSuccess                         10
//----------------------------------------------------------------------------
#endif
