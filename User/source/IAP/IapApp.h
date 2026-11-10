//###########################################################################
// FILE: IapApp.h   （由 CPDD_30_0001IAPSegment_V1.0.1.h 裁剪，名字不变）
//###########################################################################

#ifndef _IAP_APP_h_
#define _IAP_APP_h_

#include <MyProject.h>
#include "Iap_Common.h"		//IAP数据在Flash中存储的位置定义
#include <CAN.h>
#include "RingBuffer.h"

//#define IPADataType_EEPROM
#define IPADataType_FLASH

/* ---- IAP 帧 ID：ID = (Station_ID << 4) | 命令码 ----
 * 注意：ID 值恒 <= 0x7FF（Station_ID<=0x7F、cmd<=0x0F），但**帧类型是 29 位扩展帧**
 *      （与公司 F105/L431 方案一致），不能因为"值小"就当标准帧发。 */
#define IAP_CMD_WIDTH                      4
#define IAP_ID_BUILD(addr,cmd)             ((uint16_t)((((uint16_t)(addr)) << IAP_CMD_WIDTH) | ((cmd) & 0x0F)))
#define IAP_ID_ADDR(id)                    ((uint8_t)(((uint16_t)(id) >> IAP_CMD_WIDTH) & 0x7F))
#define IAP_ID_CMD(id)                     ((uint8_t)((uint16_t)(id) & 0x0F))

/* ---- 用 29 位 ID 区分 IAP 帧和业务帧 ----
 * 业务帧最低也是优先级 1（bit25），即 >= 0x02000000；
 * IAP 帧高位全 0（<=0x7FF）。所以"高 21 位为 0"的就是 IAP 帧。 */
#define IAP_ID_BASE                        0x00000000UL
#define IAP_ID_MASK                        0xFFFFF800UL

#define IAP_DEFAULT_STATION                1        /* 空片/参数无效时的默认地址 */
#define IAP_HS_TIMEOUT_MS                  3000     /* cmd1 后 3s 内才认 cmd9 */

//函数返回值定义-------------------------------------------------------------
#define CPDD_TRUE 				                         1
#define	CPDD_FALSE			                           0
#define CPDD_ERROR 				                         -1

/* ---- 版本号：实际版本 ---- */
#define IAP_APP_VER0                       0x01
#define IAP_APP_VER1                       0x00
#define IAP_APP_VER2                       0x00
#define IAP_APP_VER3                       0x00

struct IAP_IDENTIFY_BITS
{
	uint16_t  IAPCmd:4;              // 3:0   报文编号 
	uint16_t  DeviceNum:3;           // 6:4   设备编号 
	uint16_t  SystemType:1;          // 7     系统编号
	uint16_t  DeviceType:6;          // 13:8  设备类别
	uint16_t  NotUse:2;             // 31:14 高18位未使用
};

union IAP_IDENTIFY
{
   uint32_t                    all;
   struct IAP_IDENTIFY_BITS    bitf;
};

extern union IAP_IDENTIFY IAPIdentifyTx,IAPIdentifyRx;
extern uint16_t Station_ID;
extern uint16_t IAPCheckOverTime;
extern uint8_t  IAPRxFinishFlag;
// extern uint8_t  FlashData_Page[STM_FLASH_PAGE_SIZE];

extern void IAPData_Init(void);
extern void IAPRequestCheck(void);
extern int16_t IAPFlagErase(void);
extern int16_t IAPIDUpdate(void);
extern uint16_t ReadCANID(void);
extern void IAPRxStdFrame(uint16_t id, uint8_t *dat, uint8_t dlc);

extern void IAPTick1ms(void);

#endif // _IAP_APP_h_

