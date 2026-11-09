#include "can_bootloader.h"
#include "can_compat.h"

/* can_bootloader.c 里 extern 的收帧缓冲，必须在这里定义一次（只此一处）*/
CanRxMsg RxMessage[1];

void CAN_Init(void);
void CAN_detect(void);

void main(void)
{
	/* 1. 读参数页里的本机地址；无效时用默认 1 */
	Station_ID = (uint16_t)ReadCANID();
	if((Station_ID == 0) || (Station_ID > 0x7F))
		Station_ID = 1;

	/* 2. CAN + 看门狗（1s）*/
	CAN_Init();
#ifndef DEBUG_NO_WDT
	SetBit(CCFG1, WDT_EN);
	WDT_ARR = (uint8)((uint16)(65532 - (uint32)1000 * 32768 / 1000) >> 8);
	RST_SR = 0x80;
	SetBit(WDT_CR, WDTRF);
#endif

	/* 3. 标志有效（ReadExcuteFlg 返回 0）→ 直接跳 APP */
	if(0 == ReadExcuteFlg())
		CAN_BOOT_JumpToApplication(APP_START_ADDR);

	/* 4. 否则停在 boot 等上位机 */
	EA = 0;                /* 向量表 0x0003~0x007B 指向 APP 区，boot 全程不能开中断 */
	while(1)
	{
		SetBit(WDT_CR, WDTRF);
		CAN_detect();          /* 收一帧 → RxMessage[0] */
		CAN_BOOT_Command();    /* 解析执行，内部清 CAN1_CanRxMsgFlag */
	}
}
