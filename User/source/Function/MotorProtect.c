/**
 * @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      MotorProtect.c
 * @author    Fortiortech  Appliction Team
 * @date      2022-07-13
 * @brief     This file contains  MotorProtect function used for Motor Control.
 */

#include <FU68xx_6.h>
#include <Myproject.h>

/* Private variables ---------------------------------------------------------*/
FaultStateType          data    mcFaultSource;      ///< 故障类型
FaultCurrentVarible     idata   mcCurVarible;
FaultVarible            xdata   Fault;              ///< 故障检测相关结构体变量
FaultRecoverTypedef     xdata   Restart;            ///< 故障恢复重启控制相关结构体变量





/**
 * @brief     TSD过温检测
 * @date      2022-07-14
 */
void Fault_TSDTemperature(void)
{
    if (mcFaultSource == FaultNoSource)
    {
        if (mcFocCtrl.MCU_TEMP >= TempProtValue) // 过温保护，MCU温度大于设定温度
        {
            if (Fault.Temperature.TSDDetecCnt < 10000)
            {
                Fault.Temperature.TSDDetecCnt++;
            }
            else
            {
                Fault.Temperature.TSDDetecCnt  = 0;
                mcFaultSource               = FaultTSD;
            }
        }
        else
        {
            Fault.Temperature.TSDDetecCnt = 0;
        }
    }
}

/**
    @brief     NTC过温检测
    @date      2022-07-14
*/
void Fault_NTCTemperature(void)
{
    if (mcFaultSource == FaultNoSource)
    {
        if (mcFocCtrl.NTCTempFlt <= OVER_Temperature) // 过温保护，NTC热敏电阻温度大于设定温度，NTC电阻值越小，温度越高
        {
            if (Fault.Temperature.NTCDetecCnt < TemperatureProtectTime)
            {
                Fault.Temperature.NTCDetecCnt++;
            }
            else
            {
                Fault.Temperature.NTCDetecCnt  = 0;
                mcFaultSource               = FaultNtcOTErr;
            }
        }
        else
        {
            Fault.Temperature.NTCDetecCnt = 0;
        }
    }
}



/**
 * @brief     过欠压检测
 * @date      2022-07-14
 */
void Fault_Voltage(void)
{
    if (Fault.Voltage.DectDealyCnt < 50)
    {
        Fault.Voltage.DectDealyCnt ++;
    }
    else if (mcFaultSource == FaultNoSource)
    {
        /* 过压检测 */
        if (mcFocCtrl.mcDcbusFlt >   Fault.Voltage.OverVoltageVal)//ADC采样实际电压大于设定电压
        {
            Fault.Voltage.OverVoltDetecCnt += 1;
            
            if (Fault.Voltage.OverVoltDetecCnt >= 100)
            {
                Fault.Voltage.OverVoltDetecCnt  = 0;
                mcFaultSource                    = FaultOverVoltageDC;
            }
        }
        else
        {
            if (Fault.Voltage.OverVoltDetecCnt > 0)
            {
                Fault.Voltage.OverVoltDetecCnt--;
            }
        }
        
        if (mcFocCtrl.mcDcbusFlt <  Fault.Voltage.UnderVoltageVal)//ADC采样实际电压小于设定电压
        {
            Fault.Voltage.UnderVoltDetecCnt += 1;
            
            if (Fault.Voltage.UnderVoltDetecCnt >= 100)
            {
                Fault.Voltage.UnderVoltDetecCnt  = 0;
                mcFaultSource                    = FaultUnderVoltageDC;
            }
        }
        else
        {
            if (Fault.Voltage.UnderVoltDetecCnt > 0)
            {
                Fault.Voltage.UnderVoltDetecCnt--;
            }
        }
    }
}






/**
 * @brief     堵转检测
 * @date      2022-07-14
 */
float SpeedFlt_EsValue_Ratio = 0;
void Fault_Stall(void)
{
    SpeedFlt_EsValue_Ratio = (float)mcFocCtrl.SpeedFlt/(float)mcFocCtrl.EsValue;//IO翻转耗时50us
    if (mcState == mcRun)
    {
        if (Fault.Stall.DectDealyCnt < 3000)    /* 正常运行3S后再检测，防止误判 */
        {
            Fault.Stall.DectDealyCnt++;
        }
        else
        {
            /* -----堵转判断方法1，在3s后判断反电动势太小 或  当反电动势太小，转速太大
            标定方法：测试最低速700RPM，反电动势值为13949，反电动势值过小，即小于最低转速时对应的0.7倍反电动势值，认为电机堵转
            ----- */
            if ((mcFocCtrl.EsValue < 0.7*EsThresholdValue ) && (mcFocCtrl.SpeedFlt >  0.7*EsThresholdSpeed ))
            {
                Fault.Stall.EsDectCnt++;
                
                if (Fault.Stall.EsDectCnt >= 3000)
                {
                    Fault.Stall.EsDectCnt = 0;
                    mcFaultSource         = FaultStall;
                    Fault.Stall.Type      = 14;
                }
            }
            else
            {
                if ( Fault.Stall.EsDectCnt > 0)
                {
                    Fault.Stall.EsDectCnt--;
                }
            }
            #if (0)
            {
                /* ****** 堵转判断方法4 ****** 
                标定方法：测试最低速700RPM记录转速值2293，记录反电动势值为13949，比值为2293/13949=0.19，最高转速2700RPM记录转速值8862，记录反电动势值44885，比值为8862/44885=0.197，正常运行时候，转速与反电动势的比值固定
                当发生堵转时，小于0.7倍比值或大于1.3倍比值认为堵转
                */
                if ((SpeedFlt_EsValue_Ratio < 0.7*SPEED_BEMF_RATIO ) || (SpeedFlt_EsValue_Ratio >  1.3*SPEED_BEMF_RATIO))
                {
                    Fault.Stall.SpeedAndEsDectCnt++;
                    
                    if (Fault.Stall.SpeedAndEsDectCnt >= 1000)
                    {
                        Fault.Stall.SpeedAndEsDectCnt = 0;
                        mcFaultSource         = FaultStall;
                        Fault.Stall.Type      = 41;
                    }
                }
                else
                {
                    if ( Fault.Stall.SpeedAndEsDectCnt > 0)
                    {
                        Fault.Stall.SpeedAndEsDectCnt--;
                    }
                }
            }
            #endif
            
             /* -----堵转判断方法 2，在3S后判断速度低于堵转最小值或者超过堵转最大值 ----- */
            if (mcFocCtrl.SpeedFlt < Fault.Stall.UnderSpeedVal || mcFocCtrl.SpeedFlt > Fault.Stall.OverSpeedVal)
            {
                Fault.Stall.SpeedMinCnt++;
                
                if (Fault.Stall.SpeedMinCnt >= 3000)
                {
                    Fault.Stall.SpeedMinCnt = 0;
                    mcFaultSource            = FaultStall;
                    Fault.Stall.Type         = 21;
                }
            }
            else
            {
                if (Fault.Stall.SpeedMinCnt > 0)
                {
                    Fault.Stall.SpeedMinCnt--;
                }
            }
         /* -----堵转判断方法3，在5S处于开环(电流环内环)拖动阶段，无法正常进入速度闭环 ----- */
        if (mcFocCtrl.CtrlMode == 0)
        {
            Fault.Stall.Mode0DectCnt++;
            
            if (Fault.Stall.Mode0DectCnt >= 5000)
            {
                Fault.Stall.Mode0DectCnt = 0;
                mcFaultSource            = FaultStall;
                Fault.Stall.Type         = 31;
            }
        }
				
        else
        {
            Fault.Stall.Mode0DectCnt = 0;
        }
      }
   }
	 else if(mcState == mcSquStart)
   {
	    if (Fault.Stall.StartDealyCnt < 5000)    /* 正常运行3S后再检测，防止误判 */
		{
			Fault.Stall.StartDealyCnt++;
			GP02 =~GP02;
		}
		
		if (Fault.Stall.StartDealyCnt >= 3000)
		{
			Fault.Stall.StartDealyCnt = 0;
			mcFaultSource             = FaultStall;
			Fault.Stall.Type          = 41;
			Fault.Stall.StallModeFlag=4;
		}
   }
   else
   {
	   Fault.Stall.StartDealyCnt =0;
   }
}





/* -------------------------------------------------------------------------------------------------
    Function Name  : Fault_PhaseLoss
    Description    : 缺相检测
    Date           : 2022-07-01
    Parameter      : None
------------------------------------------------------------------------------------------------- */
void Fault_PhaseLoss(void)
{
    if (mcState == mcRun)
    {
        if (Fault.PhaseLoss.DectDealyCnt < 2000)
        {
            Fault.PhaseLoss.DectDealyCnt++;
        }
        else
        {
            mcFocCtrl.Max_ia = FOC__IAMAX;
            mcFocCtrl.Max_ib = FOC__IBMAX;
            mcFocCtrl.Max_ic = FOC__ICMAX;
            #if (SWCurrentProtectEn)
            {
                if (mcFocCtrl.Max_ia > SW_OC_CurrentVal)//A相电流峰值大于设定值
                {
                    mcCurVarible.OverCurACnt++;
                }
                else
                {
                    mcCurVarible.OverCurACnt = 0;
                }
                
                if (mcFocCtrl.Max_ib > SW_OC_CurrentVal)//B相电流峰值大于设定值
                {
                    mcCurVarible.OverCurBCnt++;
                }
                else
                {
                    mcCurVarible.OverCurBCnt = 0;
                }
                
                if (mcFocCtrl.Max_ic > SW_OC_CurrentVal)//C相电流峰值大于设定值
                {
                    mcCurVarible.OverCurCCnt++;
                }
                else
                {
                    mcCurVarible.OverCurCCnt = 0;
                }
                
                if (mcCurVarible.OverCurACnt > 20 || mcCurVarible.OverCurBCnt > 20 || mcCurVarible.OverCurCCnt > 20)//持续20ms
                {
                    mcFaultSource = FaultSoftOVCurrent; //软件过流
                }
            }
            #endif
            Fault.PhaseLoss.Lphasecnt++;
            
            if (Fault.PhaseLoss.Lphasecnt > 100)
            {
                Fault.PhaseLoss.Lphasecnt = 0;
                
                if (((mcFocCtrl.Max_ia > (mcFocCtrl.Max_ib << 1)) || (mcFocCtrl.Max_ia > (mcFocCtrl.Max_ic << 1))) && (mcFocCtrl.Max_ia > PhaseLossCurrentValue))
                {
                    Fault.PhaseLoss.AOpencnt++;
                }
                else
                {
                    if (Fault.PhaseLoss.AOpencnt > 0)
                    {
                        Fault.PhaseLoss.AOpencnt --;
                    }
                }
                
                if (((mcFocCtrl.Max_ib > (mcFocCtrl.Max_ia << 1)) || (mcFocCtrl.Max_ib > (mcFocCtrl.Max_ic << 1))) && (mcFocCtrl.Max_ib > PhaseLossCurrentValue))
                {
                    Fault.PhaseLoss.BOpencnt++;
                }
                else
                {
                    if (Fault.PhaseLoss.BOpencnt > 0)
                    {
                        Fault.PhaseLoss.BOpencnt --;
                    }
                }
                
                if (((mcFocCtrl.Max_ic > (mcFocCtrl.Max_ia << 1)) || (mcFocCtrl.Max_ic > (mcFocCtrl.Max_ib << 1))) && (mcFocCtrl.Max_ic > PhaseLossCurrentValue))
                {
                    Fault.PhaseLoss.COpencnt++;
                }
                else
                {
                    if (Fault.PhaseLoss.COpencnt > 0)
                    {
                        Fault.PhaseLoss.COpencnt --;
                    }
                }
                
                mcFocCtrl.Max_ia = 0;
                mcFocCtrl.Max_ib = 0;
                mcFocCtrl.Max_ic = 0;
                SetBit(FOC_CR2, ICLR);
                
                if ((Fault.PhaseLoss.AOpencnt > 20) || (Fault.PhaseLoss.BOpencnt > 20) || (Fault.PhaseLoss.COpencnt > 20) || (Fault.PhaseLoss.ABCOpenCnt > 20))
                {
                    mcFaultSource = FaultPhaseLost;
                }
            }
        }
    }
}


/**
    @brief      偏置电压检测
*/
void Fault_GetCurrentOffset(void)
{
    if (mcCurOffset.OffsetFlag == 1)
    {
        #if (VHALF_OUT_EN == Enable)                                              //有加VHALF偏置，理论值为16383
        {
            #if (Shunt_Resistor_Mode == Single_Resistor)                   //单电阻模式
            {
                if ((mcCurOffset.Iw_busOffset < (16383 - GetCurrentOffsetValue)) || (mcCurOffset.Iw_busOffset > (16383 + GetCurrentOffsetValue)))
                {
                    mcFaultSource = FaultOffset;
                }
            }
            #elif (Shunt_Resistor_Mode == Double_Resistor)                 //双电阻模式
            {
                if ((mcCurOffset.IuOffset < (16383 - GetCurrentOffsetValue)) || (mcCurOffset.IuOffset > (16383 + GetCurrentOffsetValue))
                    || (mcCurOffset.IvOffset < (16383 - GetCurrentOffsetValue)) || (mcCurOffset.IvOffset > (16383 + GetCurrentOffsetValue)))
                {
                    mcFaultSource = FaultOffset;
                }
            }
            #elif (Shunt_Resistor_Mode == Three_Resistor)                 //三电阻模式
            {
                if ((mcCurOffset.IuOffset < (16383 - GetCurrentOffsetValue)) || (mcCurOffset.IuOffset > (16383 + GetCurrentOffsetValue))
                    || (mcCurOffset.IvOffset < (16383 - GetCurrentOffsetValue)) || (mcCurOffset.IvOffset > (16383 + GetCurrentOffsetValue))
                    || (mcCurOffset.Iw_busOffset < (16383 - GetCurrentOffsetValue)) || (mcCurOffset.Iw_busOffset > (16383 + GetCurrentOffsetValue)))
                {
                    mcFaultSource = FaultOffset;
                }
            }
            #endif
        }
        #else                                                              //没加VHALF偏置,理论值在0
        {
            #if (Shunt_Resistor_Mode == Single_Resistor)                   //单电阻模式
            {
                if (mcCurOffset.Iw_busOffset > GetCurrentOffsetValue)
                {
                    mcFaultSource = FaultOffset;
                }
        
            }
            #elif (Shunt_Resistor_Mode == Double_Resistor)                 //双电阻模式
            {
                if ((mcCurOffset.IuOffset > GetCurrentOffsetValue) || (mcCurOffset.IvOffset > GetCurrentOffsetValue))
                {
                    mcFaultSource = FaultOffset;
                }
            }
            #elif (Shunt_Resistor_Mode == Three_Resistor)                 //三电阻模式
            {
                if ((mcCurOffset.IuOffset > GetCurrentOffsetValue) || (mcCurOffset.IvOffset > GetCurrentOffsetValue) || (mcCurOffset.Iw_busOffset > GetCurrentOffsetValue))
                {
                    mcFaultSource = FaultOffset;
                }
            }
            #endif
        }
        #endif
    }
}


/* -------------------------------------------------------------------------------------------------
    Function Name  : Fault_Detection
    Description    : 故障检测与保护,扫描周期默认为1ms
                     所有故障发送只进行 故障码 赋值
                     禁止在状态机以外地方进行状态跳转
    Date           : 2022-07-01
    Parameter      : None
------------------------------------------------------------------------------------------------- */
void Fault_Detection(void)
{
    #if (OverTPProtectEn == 1)
    {
        Fault_TSDTemperature();
        Fault_NTCTemperature();
    }
    #endif
    #if (VoltageProtectEn == 1)
    {
        Fault_Voltage();
    }
    #endif
    #if (StallProtectEn == 1)
    {
        Fault_Stall();
    }
    #endif
    #if (PhaseLossProtectEn == 1)
    {
        Fault_PhaseLoss();
    }
    #endif
    #if (ProtectRecoveryEn == 1)
    {
        Fault_Recovery();
    }
    #endif	   
}



/* -------------------------------------------------------------------------------------------------
    Function Name  : Fault_Recovery
    Description    : 故障恢复，条件满足只清除故障码，状态跳转由状态机执行
    Date           : 2022-07-01
    Parameter      : None
------------------------------------------------------------------------------------------------- */
void Fault_Recovery(void)
{
      if (mcState == mcFault)
    {
        #if (OV_RecoveryTimes) /* DC电压保护恢复 */
    
        if (((mcFocCtrl.mcDcbusFlt > UNDER_VOLTAGE_RECOVER) && (mcFocCtrl.mcDcbusFlt < OVER_VOLTAGE_RECOVER))
            && (mcFaultSource == FaultUnderVoltageDC || mcFaultSource == FaultOverVoltageDC))
        {
            if (Restart.OV_Times <= OV_RecoveryTimes)
            {
                if (Restart.DC_DelayTcnt < OV_RecoveryDelayTime)
                {
                    Restart.DC_DelayTcnt ++;
                }
                else
                {
                    Restart.OV_Times++;
                    Restart.DC_DelayTcnt                = 0;
                    mcFaultSource                       = FaultNoSource;
                }
            }
        }
        else
        {
            Restart.DC_DelayTcnt = 0;
        }
        
        #endif
        #if (OT_RecoveryTimes)
        
        /* TSD过温保护恢复 */
        if (mcFaultSource == FaultTSD)
        {
            if(mcFocCtrl.MCU_TEMP <= TempRecoverValue)
            {
                if (Restart.OT_Times < OT_RecoveryTimes)
                {
                    if (Restart.OT_DelayTcnt < OT_RecoveryDelayTime)
                    {
                        Restart.OT_DelayTcnt ++;
                    }
                    else
                    {
                        Restart.OT_Times++;
                        Restart.OT_DelayTcnt                = 0;
                        mcFaultSource                       = FaultNoSource;
                    }
                }
            }
        }
        /* NTC过温保护恢复 */
        if (mcFaultSource == FaultNtcOTErr)
        {
            if (mcFocCtrl.NTCTempFlt >= UNDER_Temperature )
            {
                if (Restart.OT_Times < OT_RecoveryTimes)
                {
                    if (Restart.OT_DelayTcnt < OT_RecoveryDelayTime)
                    {
                        Restart.OT_DelayTcnt ++;
                    }
                    else
                    {
                        Restart.OT_Times++;
                        Restart.OT_DelayTcnt                = 0;
                        mcFaultSource                       = FaultNoSource;
                    }
                }
            }
        }
        
        #endif
        #if (LP_RecoveryTimes)
        
        /* 缺相保护恢复 */
        if (mcFaultSource == FaultPhaseLost )
        {
            if (Restart.LP_Times < LP_RecoveryTimes)
            {
                if (Restart.LP_DelayTcnt < LP_RecoveryDelayTime)
                {
                    Restart.LP_DelayTcnt ++;
                }
                else
                {
                    Restart.LP_Times ++;
                    Restart.LP_DelayTcnt       = 0;
                    mcFaultSource             = FaultNoSource;
                }
            }
        }
        
        #endif
        #if (Stall_RecoveryTimes)
        
        /* 堵转保护恢复 */
        if (mcFaultSource == FaultStall)
        {
            if (Restart.Stall_Times < Stall_RecoveryTimes)
            {
                if (Restart.Stall_DealyTcnt < Stall_RecoveryDelayTime)
                {
                    Restart.Stall_DealyTcnt ++;
                }
                else
                {
                    Restart.Stall_Times++;
                    Restart.Stall_DealyTcnt             = 0;
                    mcFaultSource                       = FaultNoSource;
                }
            }
        }
        
        #endif
        #if (Offset_RecoveryTimes)
        
        /* 堵转保护恢复 */
        if (mcFaultSource == FaultOffset)
        {
            if (Restart.Offset_Times < Offset_RecoveryDelayTime)
            {
                if (Restart.Offset_DealyTcnt < Offset_RecoveryDelayTime)
                {
                    Restart.Offset_DealyTcnt ++;
                }
                else
                {
                    Restart.Offset_Times++;
                    Restart.Offset_DealyTcnt             = 0;
                    mcFaultSource                       = FaultNoSource;
                }
            }
        }
        
        #endif
        #if (OC_RecoveryTimes)
        
        /* 软件过流恢复 */
        if (mcFaultSource == FaultSoftOVCurrent || mcFaultSource == FaultHardOVCurrent)
        {
            if (Restart.SWOC_Times < OC_RecoveryTimes)
            {
                if (Restart.SWOC_DelayTcnt < OC_RecoveryDelayTime)
                {
                    Restart.SWOC_DelayTcnt ++;
                }
                else
                {
                    Restart.SWOC_Times++;
                    Restart.SWOC_DelayTcnt             = 0;
                    mcFaultSource                      = FaultNoSource;
                }
            }
        }
        
        #endif
    }
}






