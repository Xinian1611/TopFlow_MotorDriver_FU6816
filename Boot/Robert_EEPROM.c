
#include <string.h>
#include <FU68xx_6.h>
#include "can_bootloader.h"

uint8_t ExcuteFlg_Buff[5] = {0x55,0xAA,0x12,0x34,0xD9};
uint8_t CANID_Buff[7] = {0x55,0xAA,0xFF,0xFF,0xFF,0xFF,0xFF};

//异或校验 返回一个字节
uint8_t CheckXor(uint8_t *strData,int len)  
{  
    uint8_t checksum = 0;
    int i;
    for (i = 0;i < len;i++)
    {  
        checksum = checksum ^ strData[i];  
    }  
    return (uint8_t)checksum;  
}
// 读跳转标志
// retuen:
// 		1: 表示读取失败
// 		0: 表示读取成功
uint8_t ReadExcuteFlg(void)
{
	uint8_t readout[ExcuteFlgSize] = {0};
	uint8_t CRC_Result = 0;
	uint8_t result = 0;
	int i;

	for(i = 0;i < ExcuteFlgSize;i++)
	{
		readout[i] = *((uint8_t code *)(APP_EXE_FLAG_ADDR + ExcuteFlg_Offset + i));
	}
	for(i = 0;i < 4;i++)
	{
		if(readout[i] != ExcuteFlg_Buff[i])
			return 1;
	}
	CRC_Result = CheckXor(&readout[0],ExcuteFlgSize-1);
	if(CRC_Result == readout[ExcuteFlgSize-1])
	{
		result = 0;
	}
	else
		result = 1;
	
	return result;
}
// 读取CANID
// return :  校验通过时返回CANID 
// 					 校验未通过时返回0x10;
uint32_t ReadCANID(void)
{
	uint8_t readout[StationIDSize] = {0};
	uint8_t CRC_Result = 0;
	uint32_t result = 0;
	int i;

	for(i = 0;i < StationIDSize;i++)
	{
		readout[i] = *((uint8_t code *)(APP_EXE_FLAG_ADDR + StationID_Offset + i));
	}
	CRC_Result = CheckXor(&readout[0],StationIDSize-1);
	if((CRC_Result == readout[StationIDSize-1]) && (readout[0] == 0x55) && (readout[1] == 0xAA))
	{
		result = (uint32_t)((readout[2] << 24) | (readout[3] << 16) | (readout[4] << 8) | (readout[5] << 0));
	}
	else
		result = 0x10;
	return (result>>4);
}
// 写跳转标志
void  WriteExcuteFlg(void)
{
	uint8_t WriteBuff[16];          /* 下面固定写 0x10 字节，留够 */
	int i = 0;
	for(i = 0;i < (StationID_Offset+StationIDSize);i++)
	{
		WriteBuff[i] = *((uint8_t code *)(APP_EXE_FLAG_ADDR + ExcuteFlg_Offset + i));
	}
	
	memcpy(&WriteBuff[0],&ExcuteFlg_Buff[0],ExcuteFlgSize);
	
	CAN_BOOT_ErasePage(APP_EXE_FLAG_ADDR,APP_START_ADDR);
	SetBit(WDT_CR, WDTRF);//delayus(100);
	CAN_BOOT_ProgramDatatoFlash(APP_EXE_FLAG_ADDR,(uint8_t *)&WriteBuff,0x10);
	SetBit(WDT_CR, WDTRF);//delayus(100);
}
