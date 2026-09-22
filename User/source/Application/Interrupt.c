/**
 * @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      Interrupt.c
 * @author    Fortiortech  Appliction Team
 * @date      2022-07-13
 * @brief     This file contains interrupt function used for Motor Control.
 */



#include <MyProject.h>

extern uint8 data g_1mTick;                   ///< 1ms滴答信号，每隔1ms在SYSTICK定时器被置1，需在大循环使用处清零
extern void HardwareInit(void);
extern void SoftwareInit(void);
uint16 xdata spidebug[4] = { 0 };             ///< SPI debug 输出通道缓存，SPI调试器会将该变量值进行输出


/**
 * @brief        低于预警中断与过温中断
 * @brief        开启低压检测中断后,MCU会对输入电压进行监测,当输入电压低于设定值，则会触发中断
 * @brief       
 * @date         2022-12-29
 */
void LVW_TSD_INT(void) interrupt 0  //LVW & TSD interrupt
{
    if (ReadBit(LVSR, LVWIF))
    {
        if (ReadBit(LVSR, LVWF))
        {
            #if (LVWProtectEn == 1)
            mcFaultSource = FaultUnderVoltageDC;
            #endif
            ClrBit(LVSR, LVWF);
        }
        
        ClrBit(LVSR, LVWIF);
    }
}
#if ((LIN_PORTSELECTION == TXDGP00_RXDGP01) &&(LIN_TRANSCEIVER == EXTERNAL_LIN_DOUBLELINE))//配置唤醒口为GP01，外部中断0
/**
 * @brief        外部中断0
 * @brief        一般用于响应IPM的FO过流信号，唤醒
 * @date         2022-07-14
 */
void EXTERN0_INT(void) interrupt 1                  //  外部中断0
{
    if (IF0)
    {
       if (SleepSet.SleepFlag == 1)
        {
            SleepSet.SleepFlag = 0;
            /*系统软件复位*/
            SetBit(RST_SR, SOFTR);
        }
        IF0 = 0;                             
    }
}
#endif
#if ((LIN_PORTSELECTION == TXDGP10_RXDGP11) ||(LIN_TRANSCEIVER == INNER_LIN_SINGLELINE))   //配置唤醒口为GP11，外部中断1
/** 
 * @brief        外部中断1，用于PWM信号唤醒
 * @date         2023-07-26
 */
void EXTERN1_INT(void) interrupt 2              //  外部中断1
{
    if (P1_IF)
    {
       if (SleepSet.SleepFlag == 1)
        {
            SleepSet.SleepFlag = 0;
            /*系统软件复位*/
            SetBit(RST_SR, SOFTR);
        }
        P1_IF = 0;                           
    }
}
#endif

/**
 * @brief        FOC中断(Drv中断),每个载波周期执行一次，用于处理响应较高的程序，中断优先级第二
 * @date         2022-07-14
 */
void DRV_ISR(void) interrupt 3
{
    if (ReadBit(DRV_SR, FGIF))
    {
        DRV_SR = (SYSTIE | FGIF | DCIM1 | SYSTIF) & (~FGIF);
    }
    
    if (ReadBit(DRV_SR, DCIF))    // 比较中断
    {
		#if (Open_Start_Mode == Squ_Start)
        if(mcState==mcSquStart)
        {
          SquStartProcess(); 
            SquToFOCInit();
        }  
		#endif
			
		   	#if (DBG_MODE ==  DBG_SPI_SW)            //软件调试模式
        spidebug[0] = FOC__THETA;
        spidebug[1] = mcFocCtrl.AlignAngle;
        spidebug[2] = mcFocCtrl.AlignAngle_Temp;
        #endif

        DRV_SR = (DRV_SR | SYSTIF) & (~DCIF);
    }
}



#if (TAILWIND_MODE == RSDMethod)
/**
 * @brief        Timer2中断服务函数
 * @note         本例程中用于RSD顺逆风检测
 * @date         2022-07-14
 */
void TIM2_INT(void) interrupt 4
{
    
    if (ReadBit(TIM2_CR1, T2IP))
    {
        RsdProcess();
        ClrBit(TIM2_CR1, T2IP);
    }
    
    if (ReadBit(TIM2_CR1, T2IF))// 溢出中断,用于判断静止,时间为349ms。
    {
        mcRsd.State         = STATIC;
        mcRsd.Period        = 65535;
        mcRsd.Speed         = 0;
        mcRsd.SpeedUpdate   = 1;
        ClrBit(TIM2_CR1, T2IF);
    }
    
    if (ReadBit(TIM2_CR1, T2IR))
    {
        ClrBit(TIM2_CR1, T2IR);
    }
    
}
#endif



/**
    @brief        Timer1中断服务函数
    @note         本例程中用于BEMF方式顺逆风检测
    @date         2022-07-14
*/
#if (TAILWIND_MODE == BEMFMethod)
void CMP012_INT(void) interrupt 7
{
    if (CMP_SR & 0x70)
    {
        BemfProcess();
        CMP_SR = CMP_SR & 0x8F;
    }
}
void TIM2_INT(void) interrupt 4
{
    if (ReadBit(TIM2_CR1, T2IP))
    {
        ClrBit(TIM2_CR1, T2IP);
    }
    
    if (ReadBit(TIM2_CR1, T2IF))// 溢出中断,用于判断静止,时间为349ms。
    {
        mcBemf.PeriodTime       = 65535;
        mcBemf.BEMFSpeed        = 0;
        mcBemf.SpeedUpdate      = 1;
        ClrBit(TIM2_CR1, T2IF);
    }
    
    if (ReadBit(TIM2_CR1, T2IR))
    {
        ClrBit(TIM2_CR1, T2IR);
    }
}

#endif


/** 
 * @brief     Can中断             
 * @date      2022-12-22
 */
void CAN_INT(void)  interrupt 8
{
    if (ReadBit(CAN_IFR, RXIF))                             //接收中断
    {
		CAN_Read();
        SetBit(CAN_CR1, BUFRLS);
    }
    
    if (ReadBit(CAN_IFR, TXIF))                             //发送成功中断
    {
        ClrBit(CAN_IFR, TXIF);
    }
    
    if (ReadBit(CAN_IFR, OVIF))                             //溢出中断
    {
        ClrBit(CAN_IFR, OVIF);
    }
    
    if (ReadBit(CAN_IFR, ARBIF))                           //仲裁中断
    {
        ClrBit(CAN_IFR, ARBIF);
    }
    
    if (ReadBit(CAN_IFR, ERRIF))                           //错误类型中断
    {
        ClrBit(CAN_IFR, ERRIF);
    }
}


/** 
 * @brief        定时器3中断服务函数
 * @note         本例程中用于PWM调速信号捕获
 * @date         2022-07-14
 */
void TIM3_INT(void) interrupt 9
{
    if (ReadBit(TIM3_CR1, T3IR))
    {
        ClrBit(TIM3_CR1, T3IR);
    }
    
    if (ReadBit(TIM3_CR1, T3IP))//周期中断
    {
        if (mcPwmInput.isUpdate == 0) // 空闲状态下  允许更新DR与ARR
        {
            mcPwmInput.TimerDR            = TIM3__DR;
            mcPwmInput.TimerARR           = TIM3__ARR;
            mcPwmInput.isUpdate           = 1;
            SleepSet.SleepDelayCout = 0;    //有PWM信号清零
        }
        
        ClrBit(TIM3_CR1, T3IP);
    }
    
    if (ReadBit(TIM3_CR1, T3IF))
    {
        if (mcPwmInput.isUpdate == 0)   // 空闲状态下  允许更新DR与ARR
        {
            if (ReadBit(P1, PIN1))      // PWM 100%输出
            {
                mcPwmInput.TimerDR       = 4000;
                mcPwmInput.TimerARR      = 4000;
            }
            else//PWM 为0%
            {
                mcPwmInput.TimerDR       = 0;
                mcPwmInput.TimerARR      = 4000;
            }
            
            mcPwmInput.isUpdate = 1;
        }
        
        ClrBit(TIM3_CR1, T3IF);
    }
}



/**
 * @brief        滴答定时器，默认用于产生1ms定时间隔
 * @date         2022-07-14
 */
void SYStick_INT(void) interrupt 10
{
    if (ReadBit(DRV_SR, SYSTIF))          // SYS TICK中断
    {
        g_1mTick = 1;
        DRV_SR = (DRV_SR | DCIF) & (~SYSTIF);
    }
}



/**
 * @brief        比较器硬件过流保护，该中断仅提供 故障码 赋值,用于状态机的切换。
 *               需要开启比较器CMP3  发生过流 自动清除MOE功能
 * @date         2022-07-14
 */

void CMP3_INT(void)  interrupt 12
{
    if (ReadBit(CMP_SR, CMP3IF))
    {
        if (mcState != mcPosiCheck)
        {
            mcFaultSource = FaultHardOVCurrent;     // 硬件过流保护
        }
        
        ClrBit(CMP_SR, CMP3IF);
    }
}
#if (SPEED_MODE == LINMODE)
void int14(void) interrupt 14
{
	if (ReadBit(LIN_CR, LINIE))
	{
        if (ReadBit(LIN_CSR, CLRERR)) // 错误中断
        {
            /*----- 获取错误源 -----*/
            LIN_Error_Handing();
            ClrBit(LIN_CSR, CLRERR);
            ClrBit(LIN_CR, LINRW);
						ClrBit(LIN_CSR, LINEN);
            SetBit(LIN_CSR, LINEN);	// 重启更新LIN波特率
					  LS.State = 0;
						LS.OverTimeCnt = 0;
						LS.BusLostCnt = 0;
        }

        else if (ReadBit(LIN_SR, LINREQ)) /* 没有错误才能处理数据 */
        {
            loc_linBusTimeoutCnt = 0;  /* 有LIN信号清零休眠计数 */
            LIN_list();                /* 读取ID号并处理信号 */
            LIN_SR = (uint8)(~LINREQ); /* 清除中断 */
					  LS.State = 1;
						LS.OverTimeCnt = 0;
						LS.BusLostCnt = 0;

        }

        if (ReadBit(LIN_SR, LINDONE)) // 数据收发完成
        {
            ReceiveSYS(LIN_ID);         /* 处理接收完成的数据 */
            LIN_SR = (uint8)(~LINDONE); /* 清除中断 */
						ClrBit(LIN_CSR, LINEN);
            SetBit(LIN_CSR, LINEN);			// 重启更新LIN波特率
					  LS.State = 0;
						LS.OverTimeCnt = 0;
						LS.BusLostCnt = 0;

				}

        if (ReadBit(LIN_CSR, LINWAKEUP)) // 唤醒信号
        {
            SetReg(LIN_CSR, LINWAKEUP | CLRERR, CLRERR);
						ClrBit(LIN_CSR, LINEN);
            SetBit(LIN_CSR, LINEN);			// 重启刷掉IDLE状态，else if 双重保险

        }
        else if (ReadBit(LIN_SR, LINIDLE)) // 总线空闲
        {
            SetReg(LIN_SR, LINDONE | LINIDLE | LINREQ, LINDONE | LINREQ);
        }
    }
}
#endif
