/**
 *                          @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      FU68xx_6_Flash.c
 * @author    leo.li
 * @since     2022-10-01 22:24:50
 * @date      2022-10-09 17:46:47
 * @version
 * <table>
 * <tr><th>Date <th>Version <th>Author <th>Description </tr>
 * </table>
 * @brief
 */


/******************************************************************************/// Including Header Files
#include <MyProject.h>

MCUFlash xdata FlashTest;                 //flash测试

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

/**
 * @brief        Flash 校验CRC
 * @brief        CRC_BEG: 扇区起始
 * @brief        CRC_CNT: 校验扇区数
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
    FlashTest.FlashCRC = CRCH;
    FlashTest.FlashCRC = FlashTest.FlashCRC << 8;
    ClrBit(CRC_CR, CRCPNT); /* 取crc低位 */
    CRCL = CRC_DR;
    FlashTest.FlashCRC = FlashTest.FlashCRC + CRCL;
}


/**
* @brief        获取写入地址，必须保证写入的地址中数值全部为0
* @brief        unsigned short FlashAddress：目标FLASH首地址
* @brief        uint8 Write_Length：写入数值长度
*/
uint16 GetWrite_Black_Addr(unsigned short FlashAddress, uint8 Write_Length)
{
    uint16 TempFlashAddress = 0;
    uint16 TempFirstAddress = 0;             //用来保存保存首地址
    uint8  tempFlashData = 0xFF;
    uint8  Length = 0;
    TempFlashAddress = FlashAddress;
    TempFirstAddress = FlashAddress;
    
    while ((Length != Write_Length) && (TempFlashAddress < (TempFirstAddress + Write_Length))) //限定读取地址为当前页
    {
        tempFlashData = *(uint8 code *)(TempFlashAddress);  //读取目标地址中数值是否为0
        
        if (tempFlashData == 0)
        {
            Length ++;
            TempFlashAddress += 1;
        }
        else
        {
            TempFirstAddress += Write_Length;        //首地址跳转Write_Length字节
            TempFlashAddress  = TempFirstAddress;
            Length = 0;
        }
    }
    
    return TempFirstAddress;
}

/**
 * @brief        写入Length字节到FLASH，
 * @Input        unsigned short FlashAddress：写入目标FLASH首地址
 * @Input        uint8 *str：被写入数据
 * @ruturn       1:扇区未满,写入完成  0:扇区已满,写入失败
 * @note
 */
uint8 Write_LengthByte(unsigned short FlashAddress, uint8 Length, uint8 * str)
{
    uint8 tempFlashData = 0;
    uint8 i = 0;
    uint16 tempNewFlashAddress = FlashAddress;
    
    for (i = 0; i < Length; i++)
    {
        Flash_byte_Write((tempNewFlashAddress + i), str[i]);
        tempFlashData = *(uint8 code *)(tempNewFlashAddress + i);
        
        if (tempFlashData != str[i])
        {
            return 0;
        }
    }
    
    return 1;
}


/**
 * @brief        读取两个字节数据，
 * @Input        unsigned short BlockStartAddr：目标FLASH首地址
 * @Input        uint8 Length：数值长度
 * @Input        uint8 * str ：读回数值写入数组
 * @ruturn       Flash数据
 */
void ReadFromFlash(unsigned short BlockStartAddr, uint8 Length, uint8 * str)
{
    uint8 i = 0;
    
    for (i = 0; i < Length; i++)
    {
        str[i] = *(uint8 code *)(BlockStartAddr + i);
    }
}



/**
* @brief        获取掉电之前的错误数据，数值初始化
* @brief
* @brief
*/
void Flash_DataInit(void)
{
    uint16 ReadInfor_Addr = 0;                 //读取信息地址
    uint8 Check_code = 0;
	uint8 i = 0;
	
    memset(&FlashTest, 0, sizeof(MCUFlash));        // 变量清零
    /*确认写入对应长度地址内数值全为0 */
    FlashTest.FlashWriteAddr = GetWrite_Black_Addr(USERCODEADDRESS, Flash_DATA_Length);
    
    /* 获取上次的保存数值
    FlashTest.FlashWriteAddr：写入数值首地址 ，Flash_DATA_Length：读取字节数，读取值放入FlashTest.Flash_Read_Data数组中 */
    if (FlashTest.FlashWriteAddr > USERCODEADDRESS)
    {
        /*读取数值首地址*/
        FlashTest.FlashReadAddr  = FlashTest.FlashWriteAddr - Flash_DATA_Length;
        
        /*获取最新一次正确写入的数值*/
        while ((FlashTest.FlashReadAddr > USERCODEADDRESS) && (Check_code != 0x55))
        {
            /*读取数值校验位，确认当前数值在写入时为正确写入*/
            ReadInfor_Addr = FlashTest.FlashReadAddr + Verify_Bit;
            Check_code = *(uint8 code *)(ReadInfor_Addr + i);
            
            if (Check_code == 0x55)
            {
                ReadFromFlash(FlashTest.FlashReadAddr, Flash_DATA_Length, FlashTest.Flash_Read_Data);
            }
            else
            {
                FlashTest.FlashReadAddr  = FlashTest.FlashReadAddr - Flash_DATA_Length;
            }
        }
    }
    else
    {
        FlashTest.FlashReadAddr  = FlashTest.FlashWriteAddr;
    }
    
    /*错误信息从Flash读取到APP*/
    ReadInfor_Addr = FlashTest.FlashReadAddr + ((uint8)FaultHardOVCurrent - 1) * 2;
    FlashTest.FaultHardOVCurrentCount = *(uint16 code *)(ReadInfor_Addr);
    ReadInfor_Addr = FlashTest.FlashReadAddr + ((uint8)FaultSoftOVCurrent - 1) * 2;
    FlashTest.FaultSoftOVCurrentCount = *(uint16 code *)(ReadInfor_Addr);
    ReadInfor_Addr = FlashTest.FlashReadAddr + ((uint8)FaultOverVoltageDC - 1) * 2;
    FlashTest.FaultUnderVoltageCount = *(uint16 code *)(ReadInfor_Addr);
    ReadInfor_Addr = FlashTest.FlashReadAddr + ((uint8)FaultUnderVoltageDC - 1) * 2;
    FlashTest.FaultOverVoltageCount = *(uint16 code *)(ReadInfor_Addr);
    ReadInfor_Addr = FlashTest.FlashReadAddr + ((uint8)FaultPhaseLost - 1) * 2;
    FlashTest.FaultLossPhaseCount = *(uint16 code *)(ReadInfor_Addr);
    ReadInfor_Addr = FlashTest.FlashReadAddr + ((uint8)FaultStall - 1) * 2;
    FlashTest.FaultStartCount = *(uint16 code *)(ReadInfor_Addr);
    ReadInfor_Addr = FlashTest.FlashReadAddr + ((uint8)FaultNtcOTErr - 1) * 2;
    FlashTest.FaultStartCount = *(uint16 code *)(ReadInfor_Addr);
    ReadInfor_Addr = FlashTest.FlashReadAddr + ((uint8)FaultTSD - 1) * 2;
    FlashTest.FaultTSDCount = *(uint16 code *)(ReadInfor_Addr);
    
    /*Flash如果准备写满，擦写标志位置1.准备芯片温度在低于105°进行擦写*/
    if (FlashTest.FlashWriteAddr >= USERCODEADDRESS + 0x0FB0)                //擦写为2页（一共4K的数值）到最后一个写入64字节尝试擦写
    {
        for (i = 0; i < Flash_DATA_Length; i++)
        {
            FlashTest.Flash_Write_Data[i] = FlashTest.Flash_Read_Data[i];
        }
        
        FlashTest.Flag_FlashErase = 1;
    }
}

/**
* @brief        Flash的擦写
* @brief        FlashTest.Flag_FlashErase确认是否进行
* @brief
*/
void Flash_Erase(void)
{
    uint8 FlashWriteStatus = 0;
    
    if ((((FlashTest.Flag_FlashErase == 1) && (mcFocCtrl.MCU_TEMP < 105))|| (FlashTest.Flag_FlashErase == 2)) 
		&& (mcFocCtrl.mcDcbusFlt > Fault.Voltage.UnderVoltageVal))
    {
        FlashTest.Flag_FlashErase = 0;
        FlashTest.ReturnEraseStatus   = (uint8)Flash_PageErase(USERCODEADDRESS);            // 目标页写满，清除目标页
        FlashTest.FlashWriteAddr      = USERCODEADDRESS;                             // 重新给写入首地址
        
        while (!FlashWriteStatus)                                               // 初始写入或者写入异常、换到下个地址写入
        {
            /* FlashTest.FlashWriteAddr：指定地址 FlashTest.Flash_Write_Data:需要写入的数值
            FlashWriteStatus：返回数值 1：写入正确  0：写入错误 */
            
            /* 写入正确时最后一位写入校验位 */
            if (FlashWriteStatus == 0)   //当前数值写入错误
            {
                Flash_byte_Write((FlashTest.FlashWriteAddr + Verify_Bit), Verify_Error);
            }
            else if (FlashWriteStatus)    //当前数值写入正确
            {
                Flash_byte_Write((FlashTest.FlashWriteAddr + Verify_Bit), Verify_Ture);
            }
            
            //更新读写首地址
            FlashTest.FlashReadAddr = FlashTest.FlashWriteAddr;
            FlashTest.FlashWriteAddr = FlashTest.FlashWriteAddr + Flash_DATA_Length;
            
            if (FlashTest.FlashWriteAddr >= USERCODEADDRESS + 0x0800)
            {
                FlashWriteStatus = 1;     //全部页损坏，强行跳转
            }
        }
        
        FlashTest.ReturnEraseStatus   = (uint8)Flash_PageErase(USERCODEADDRESS + 0x0800);         //目标页写满，清除目标页
    }
}

/**
* @brief        保存错误
* @brief
* @brief
*/
void SaveFault_Flash(void)
{
    uint16 Array_Loca = 0;
    uint8 FlashWriteStatus = 0;
    
    if (FlashTest.Flash_FaultSource_Updat)
    {
        FlashTest.Flash_FaultSource_Updat = 0;
        
        switch (mcFaultSource)
        {
            case FaultHardOVCurrent:
                FlashTest.FaultHardOVCurrentCount++;
                Array_Loca = ((uint8)FaultHardOVCurrent - 1) * 2;
                FlashTest.Flash_Write_Data[Array_Loca] = FlashTest.FaultHardOVCurrentCount >> 8;
                FlashTest.Flash_Write_Data[Array_Loca + 1] = FlashTest.FaultHardOVCurrentCount;
                break;
                
            case FaultSoftOVCurrent:
                FlashTest.FaultSoftOVCurrentCount++;
                Array_Loca = ((uint8)FaultSoftOVCurrent - 1) * 2;
                FlashTest.Flash_Write_Data[Array_Loca] = FlashTest.FaultSoftOVCurrentCount >> 8;
                FlashTest.Flash_Write_Data[Array_Loca + 1] = FlashTest.FaultSoftOVCurrentCount;
                break;
                
            case FaultOverVoltageDC:
                FlashTest.FaultUnderVoltageCount++;
                Array_Loca = ((uint8)FaultOverVoltageDC - 1) * 2;
                FlashTest.Flash_Write_Data[Array_Loca] = FlashTest.FaultUnderVoltageCount >> 8;
                FlashTest.Flash_Write_Data[Array_Loca + 1] = FlashTest.FaultUnderVoltageCount;
                break;
                
            case FaultUnderVoltageDC:
                FlashTest.FaultOverVoltageCount++;
                Array_Loca = ((uint8)FaultUnderVoltageDC - 1) * 2;
                FlashTest.Flash_Write_Data[Array_Loca] = FlashTest.FaultOverVoltageCount >> 8;
                FlashTest.Flash_Write_Data[Array_Loca + 1] = FlashTest.FaultOverVoltageCount;
                break;
                
            case FaultPhaseLost:
                FlashTest.FaultLossPhaseCount++;
                Array_Loca = ((uint8)FaultPhaseLost - 1) * 2;
                FlashTest.Flash_Write_Data[Array_Loca] = FlashTest.FaultLossPhaseCount >> 8;
                FlashTest.Flash_Write_Data[Array_Loca + 1] = FlashTest.FaultLossPhaseCount;
                break;
                
            case FaultStall:
                FlashTest.FaultStallCount++;
                Array_Loca = ((uint8)FaultStall - 1) * 2;
                FlashTest.Flash_Write_Data[Array_Loca] = FlashTest.FaultStallCount >> 8;
                FlashTest.Flash_Write_Data[Array_Loca + 1] = FlashTest.FaultStallCount;
                break;
                
            case FaultNtcOTErr:
                FlashTest.FaultStartCount++;
                Array_Loca = ((uint8)FaultNtcOTErr - 1) * 2;
                FlashTest.Flash_Write_Data[Array_Loca]     = FlashTest.FaultStartCount >> 8;
                FlashTest.Flash_Write_Data[Array_Loca + 1] = FlashTest.FaultStartCount;
                break;
                
            case FaultTSD:
                FlashTest.FaultTSDCount++;
                Array_Loca = ((uint8)FaultTSD - 1) * 2;
                FlashTest.Flash_Write_Data[Array_Loca] = FlashTest.FaultTSDCount >> 8;
                FlashTest.Flash_Write_Data[Array_Loca + 1] = FlashTest.FaultTSDCount;
                break;
                
            default:
                break;
        }
        
        FlashTest.ReturnEraseStatus = 0;
        
        while (!FlashWriteStatus)   //写入异常，换到下个地址写入
        {
            /* FlashTest.FlashWriteAddr：指定地址 FlashTest.Flash_Write_Data:需要写入的数值
            FlashWriteStatus：返回数值 1：写入正确  0：写入错误 */
            FlashWriteStatus = Write_LengthByte(FlashTest.FlashWriteAddr, Flash_DATA_Length, FlashTest.Flash_Write_Data);
            
            /* 写入正确时最后一位写入校验位 */
            if (FlashWriteStatus == 0)   //当前数值写入错误
            {
                Flash_byte_Write((FlashTest.FlashWriteAddr + Verify_Bit), Verify_Error);
            }
            else   //当前数值写入正确
            {
                Flash_byte_Write((FlashTest.FlashWriteAddr + Verify_Bit), Verify_Ture);
            }
            
            //更新读写首地址
            FlashTest.FlashReadAddr = FlashTest.FlashWriteAddr;
            FlashTest.FlashWriteAddr = FlashTest.FlashWriteAddr + Flash_DATA_Length;
            
            /*写到相应的空间，不管之前写入是否准确，直接进行相关的擦除
              擦除后会当即写入Flash*/
            if (FlashTest.FlashWriteAddr >= USERCODEADDRESS + 0x0FD0) //&& (FlashWriteStatus == 0))
            {
                FlashTest.Flag_FlashErase = 1;    // 置擦除标志位
                FlashWriteStatus = 1;             // 没有空间，强行跳转
            }
        }
    }
}
