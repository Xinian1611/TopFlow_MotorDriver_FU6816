/**************************** (C) COPYRIGHT 2015 Fortiortech shenzhen *****************************
    File Name          : IRScan.c
    Author             : Billy Long Fortiortech  Market Dept
    Version            : V1.0
    Date               : 01/07/2015
    Description        : This file contains main function used for Motor Control.
***************************************************************************************************
    All Rights Reserved
**************************************************************************************************/


/* Includes -------------------------------------------------------------------------------------*/
#include <MyProject.h>
#include <FU68xx_4_MCU.h>


/* Private typedef ------------------------------------------------------------------------------*/
/* Private define -------------------------------------------------------------------------------*/
/* Private macro --------------------------------------------------------------------------------*/
/* Private variables ----------------------------------------------------------------------------*/
IRScan_TypeDef xdata IRScan;
IRControl_TypeDef xdata IRControl;
IRControl_AutoPowerDef AutoPowerState;
LineControlSpeedDEF xdata LineSpeedState;

uint8   NatureWinFlag = 0;

/* Private function prototypes ------------------------------------------------------------------*/
/* Private functions ----------------------------------------------------------------------------*/

/*  -------------------------------------------------------------------------------------------------
    Function Name : void IRInit(void)
    Description   :
    Input         : 无
    Output        : 无
    -------------------------------------------------------------------------------------------------*/
void IRInit(void)
{
    IRControl.LEDONOFFStatus = 1;
    IRScan.BitValue = 0;
    IRScan.BitCnt = 0;
    IRScan.Bit0Cnt = 0;
    IRScan.ByteValue = 0;
    IRScan.UserCode = 0;
    IRScan.DataCode = 0;
    IRScan.IRReceiveFlag = 0;
    IRScan.CurrentUserCode = 0;
    IRScan.OldDataCodeMixTimes1 = 0;
    IRScan.OldDataCodeMixTimes0 = 0;
    IRScan.CurrentDataCode = 0;
    IRScan.DataCodeLengthCover = 1;
    IRScan.UserCodeLengthCover = 1;
    IRScan.DataCodeLengthCover = (IRScan.DataCodeLengthCover << (DataCodeLength)) - 1;
    IRScan.UserCodeLengthCover = (IRScan.UserCodeLengthCover << (UserCodeLength)) - 1;
    IRControl.FlagONOFF = 0;
    IRControl.FlagFR = 0;
    IRControl.FlagSpeed = 0;
    IRControl.TargetSpeed = 0;
    IRControl.ONOFFStatus = 0;
    IRControl.FlagNatureWind = 0;
    IRControl.FlagLED = 0;
    IRControl.FlagAutoPower = 0;
    IRControl.FlagUserCodeSave = 0;
    IRControl.FlagUserCodeSaveOFF = 0;
    IRControl.FlagUserCodeRead = 1;                                     //上电读出一次UserCode
    IRControl.FlagLED1Protect = 0;                                      //LED欠压过压保护标记
    IRControl.FlagLED2Protect = 0;
    IRScan.TempDataCode[0] = 0;
    IRScan.TempDataCode[1] = 0;
    IRScan.TempDataCode[2] = 0;
    IRScan.TempDataCode[3] = 0;
    IRScan.DataCodeNum = 0;
    IRScan.BYTE0 = 0;
    IRScan.BYTE1 = 0;
    IRScan.BYTE2 = 0;
    IRScan.BYTE3 = 0;
    IRScan.PID = 0;
    IRScan.B = 0;
    IRControl.NatureFlage = 0;
    IRScan.OverFlowStatus = 0;
    mcFRState.OldTargetSpeed = 1;
    mcFRState.FR = CCW;
    mcFRState.FlagFR = 0;
    mcFRState.FRStatus = 0;
    //      Time.ReciveDelayFlage=0;
    //      Time.ReciveDelayCount=0;
    //      Time.PowerUpTimeCount=0;
    AutoPowerState.FlagAutoPower = 0;
    AutoPowerState.Timer10sec = 0;
    AutoPowerState.ShutDowntime = 0;
    AutoPowerState.CurrentTime = 0;
    #if (Motor_Speed_Control_Mode == SPEED_LOOP_CONTROL)
    {
        IRControl.SpeedLevel[0] = MOTOR_SPEED_LEVEL_0;
        IRControl.SpeedLevel[1] = MOTOR_SPEED_LEVEL_1;
        IRControl.SpeedLevel[2] = MOTOR_SPEED_LEVEL_2;
        IRControl.SpeedLevel[3] = MOTOR_SPEED_LEVEL_3;
        IRControl.SpeedLevel[4] = MOTOR_SPEED_LEVEL_4;
        IRControl.SpeedLevel[5] = MOTOR_SPEED_LEVEL_5;
        IRControl.SpeedLevel[6] = MOTOR_SPEED_LEVEL_6;
    }
    #elif (Motor_Speed_Control_Mode == POWER_LOOP_CONTROL)
    {
        IRControl.SpeedLevel[0] = MOTOR_POWER_LEVEL_0;
        IRControl.SpeedLevel[1] = MOTOR_POWER_LEVEL_1;
        IRControl.SpeedLevel[2] = MOTOR_POWER_LEVEL_2;
        IRControl.SpeedLevel[3] = MOTOR_POWER_LEVEL_3;
        IRControl.SpeedLevel[4] = MOTOR_POWER_LEVEL_4;
        IRControl.SpeedLevel[5] = MOTOR_POWER_LEVEL_5;
        IRControl.SpeedLevel[6] = MOTOR_POWER_LEVEL_6;
    }
    #endif
    LineSpeedState.TargetSpeed = 0;
    LineSpeedState.PreTargetSpeed = 0;
}


/*  -------------------------------------------------------------------------------------------------
    Function Name : void IRValue(void)
    Description   : 红外接收头解码
    Input         : 无
    Output        : 无
    -------------------------------------------------------------------------------------------------*/
//void IRValue(void)
//{
//    uint8 tempof1 = 0;
//    uint8 tempof2 = 0;
//
//    if (IRScan.BitCnt == 0)
//    {
//        IRScan.Bit0Cnt = TIM3__DR;
//
//        if ((IRScan.Bit0Cnt <= TempLeadCodeMax) && (IRScan.Bit0Cnt >= TempLeadCodeMin))
//        {
//            IRScan.BitCnt++;
//        }
//    }
//    else
//    {
//        IRScan.Bit0Cnt = TIM3__ARR - TIM3__DR;
//
//        if ((IRScan.Bit0Cnt >= TempBitValue1Min) && (IRScan.Bit0Cnt <= TempBitValue1Max))
//        {
//            IRScan.BitValue = 1;
//            tempof1 = 0x00;
//        }
//        else if ((IRScan.Bit0Cnt >= TempBitValue0Min) && (IRScan.Bit0Cnt <= TempBitValue0Max))
//        {
//            IRScan.BitValue = 0;
//            tempof1 = 0x00;
//        }
//        else
//        {
//            tempof1 = 0xFF;
//        }
//
//        if (tempof1 != 0xff)
//        {
//            if (IRScan.BitCnt <= UserCodeLength)
//            {
//                IRScan.ByteValue += (IRScan.BitValue << (UserCodeLength - IRScan.BitCnt));
//
//                if (IRScan.BitCnt == UserCodeLength)
//                {
//                    IRScan.UserCode = IRScan.ByteValue;
//                    IRScan.ByteValue = 0;
//                }
//
//                IRScan.BitCnt ++;
//            }
//            else
//            {
//                IRScan.ByteValue += (IRScan.BitValue << (DataCodeLength - (IRScan.BitCnt - UserCodeLength)));
//
//                if (IRScan.BitCnt == AllCodeLength)
//                {
//                    IRScan.DataCode = IRScan.ByteValue;
//                    IRScan.ByteValue = 0;
//                    IRScan.BitCnt = 0;
//
//                    //              tempof2 = (uint8)IRScan.DataCode;
//                    //              tempof1 = IRScan.DataCode>>8;
//                    //              tempof1 = 0xff -tempof1-1;
//                    //              tempof1 = 0xff -tempof1;
//
//                    if (IRScan.UserCode == UserCodeDef)
//                    {
//                        IRScan.IRReceiveFlag = 1;
//                    }
//                }
//                else
//                { IRScan.BitCnt ++; }
//            }
//        }
//        else
//        {
//            IRScan.ByteValue = 0;
//            IRScan.BitCnt = 0;
//        }
//    }
//}


/*  -------------------------------------------------------------------------------------------------
    Function Name : void SetSpeed(uint8 SpeedLevel)
    Description   : 速度设置
    Input         : SpeedLevel：速度挡位
    Output        : 无
    -------------------------------------------------------------------------------------------------*/
void SetSpeed(uint8 SpeedLevel)
{
    IRControl.TargetSpeed = SpeedLevel;
    
    if (!mcSpeedRamp.FlagONOFF)
    {
        mcSpeedRamp.FlagONOFF = 1;
    }
    
    IRControl.ONOFFStatus = 1;
    IRControl.FlagSpeed = 1;
    mcFRState.OldTargetSpeed = IRControl.TargetSpeed;
    NatureWinFlag = 0;          //正反转切换后根据遥控档位启动
    IRControl.NatureFlage = 0;
    // SetBuzzer(1,1);
}


/*  -------------------------------------------------------------------------------------------------
    Function Name : void SetAutoPower(uint16 Time)
    Description   : 定时设置
    Input         : Time：定时时长
    Output        : 无
    -------------------------------------------------------------------------------------------------*/
void SetAutoPower(uint16 Time)
{
    IRControl.FlagAutoPower = 1;
    AutoPowerState.ShutDowntime = Time;
}

uint32 idata TempByteValue;
uint16 buf1[32] = {0};
uint8 k = 0;
uint16 TIM4_DR_Temp = 0;
uint16 TIM4_ARR_Temp = 0;

///*-------------------------------------------------------------------------------------------------
//  Function Name : void RFValue(void)
//  Description   : RF接收头解码
//  Input         : 无
//  Output              :   无
//-------------------------------------------------------------------------------------------------*/
void IRValue(void)
{
    static uint8 UserSavedStatus = 0;
    static uint8 UserSavedOffStatus = 0;
    static idata uint32 TempByteValueEff;
    static idata uint32 TempBitValue;
    static idata uint8  effect_value = 0;
    uint8 tempof1 = 0;
    TIM4_ARR_Temp = TIM4__ARR;
    TIM4_DR_Temp = TIM4__DR;
    IRScan.Bit0Cnt = TIM4_ARR_Temp - TIM4_DR_Temp;
    
    //读出高电平
    //两组遥控信号之间的低电平信号认为是第二组信号的起始信号，低电平长度6ms
    //  if((TIM4_DR_Temp <= 15000)&&(TIM4_DR_Temp >= 12000)&&(IRScan.Bit0Cnt <= 2100)&&(IRScan.Bit0Cnt >= 1500))
    if ((TIM4_DR_Temp <= TempLeadCodeMax) && (TIM4_DR_Temp >= TempLeadCodeMin) && (IRScan.Bit0Cnt <= TempLeadCode0Max) && (IRScan.Bit0Cnt >= TempLeadCode0Min))
    {
        IRScan.BitCnt = 1;
        effect_value = 0;
    }
    
    if (IRScan.BitCnt == 1)
    {
        IRScan.BitCnt = 2;
        TempByteValue = 0;
        TempByteValueEff = 0;
    }
    else if (IRScan.BitCnt >= 2)
    {
        IRScan.Bit0Cnt = TIM4__DR;
        
        //        buf1[k] = IRScan.Bit0Cnt;
        //        if(++k == 32)
        //        {
        //            k = 0;
        //        }
        if ((IRScan.Bit0Cnt >= TempBitValue1Min) && (IRScan.Bit0Cnt <= TempBitValue1Max))
        {
            //当前数据为1
            TempBitValue = 1;
            tempof1 = 0x00;
        }
        else if ((IRScan.Bit0Cnt >= TempBitValue0Min) && (IRScan.Bit0Cnt <= TempBitValue0Max))
        {
            //当前数据为0
            TempBitValue = 0;
            tempof1 = 0x00;
        }
        else
        {
            tempof1 = 0xff;              //当前数据电平长度不符合要求
        }
        
        if (tempof1 != 0xff)
        {
            /****************************遥控解码部分**************/
            //存储有效数据至对应位
            TempByteValue += (TempBitValue << ((AllCodeLength - 1) - (IRScan.BitCnt - 2)));
            IRScan.BitCnt ++;
            
            //接收满24位数据时进行处理（数据长度实际为25位，最后一位在起始信号时读取）
            if ((IRScan.BitCnt - 2) == AllCodeLength)
            {
                IRScan.DataCode = (TempByteValue) & 0XFFFF;
                //存储当前遥控地址吗
                IRScan.UserCode = (TempByteValue >> 16) & 0XFFFF;
                TempByteValue = 0;
                
                if (IRScan.UserCode == 0x00ff)
                {
                    if (IRScan.DataCode == 0x8877)
                    {
                        if (KeySpeed == 0) //处于关机状态按下该按键则以一档速度运行并短“滴”一声
                        {
                            KeySpeed = 1;
                            SetBuzzer(1, 1);
                            SetSpeed_LEDPin;
                        }
                        else //处于开机状态按下该按键则关机并长“滴”一声
                        {
                            KeySpeed = 0;
                            SetBuzzer(5, 1);
                            ResetSpeed_LEDPin;
                            YaoTou_Flag = 0;
                            ResetYaoTou_LEDPin;
                        }
                    }
                    else if (IRScan.DataCode == 0x6897)//摇头
                    {
                        if (KeySpeed) //只有在主控电机运行时才开摇头
                        {
                            YaoTou_Flag ^= 1;
                            
                            if (YaoTou_Flag) //
                            {
                                SetBuzzer(1, 1);
                                SetYaoTou_LEDPin;
                            }
                            else
                            {
                                SetBuzzer(1, 2);
                                ResetYaoTou_LEDPin;
                            }
                        }
                    }
                    else if (IRScan.DataCode == 0x48b7)//加大风量并短“滴”一声
                    {
                        if (KeySpeed != 0)
                        {
                            if (KeySpeed < 3)
                            { KeySpeed++; }
                            
                            SetBuzzer(1, 1);
                            SetSpeed_LEDPin;
                        }
                    }
                    else if (IRScan.DataCode == 0xa857)//减小风量并短“滴”一声
                    {
                        if (KeySpeed != 0)
                        {
                            if (KeySpeed > 1)
                            { KeySpeed--; }
                            
                            SetSpeed_LEDPin;
                            SetBuzzer(1, 1);
                        }
                    }
                }
            }
        }
        else
        {
            TempByteValue = 0;
            IRScan.BitCnt = 0;
        }
    }
}

/*  -------------------------------------------------------------------------------------------------
    Function Name : void IRScanControl(void)
    Description   : 遥控信号处理
    Input         : 无
    Output        : 无
    -------------------------------------------------------------------------------------------------*/
void IRScanControl(void)
{
    if (IRScan.IRReceiveFlag)
    {
        //Time.ReciveDelayFlage=1;
        if (IRScan.DataCode == IRFRCCW)
        { SetBuzzer(2, 2); }
        else
        { SetBuzzer(2, 1); }
        
        switch (IRScan.DataCode)
        {
            case IRALLOFF:
            {
                IRControl.ONOFFStatus = 0;
                IRControl.TargetSpeed = 0;
                IRControl.FlagSpeed = 1;
                mcSpeedRamp.FlagONOFF = 0;
                IRControl.NatureFlage = 0;
                //在反转过程中关机时退出反转状态
                mcFRState.FlagFR = 0;
                mcFRState.FRStatus = 0;
                NatureWinFlag = 0;
                
                if (IRControl.LEDONOFFStatus)
                {
                    ResetLEDPin;
                    IRControl.LEDONOFFStatus = 0;
                }
                
                break;
            }
            
            case IRONOFF:
            {
                if (mcSpeedRamp.FlagONOFF == 1)
                {
                    IRControl.ONOFFStatus = 0;
                    IRControl.TargetSpeed = 0;
                    mcFRState.OldTargetSpeed = IRControl.TargetSpeed;
                    mcSpeedRamp.FlagONOFF = 0;
                    IRControl.NatureFlage = 0;
                    //在反转过程中关机时退出反转状态
                    mcFRState.FlagFR = 0;
                    mcFRState.FRStatus = 0;
                    IRControl.FlagSpeed = 1;
                    NatureWinFlag = 0;
                }
                
                break;
            }
            
            case IRFRCCW:
            {
                if (mcFRState.FR == CCW)
                {
                    if ((mcState == mcRun) || (mcState == mcStart))
                    { IRControl.FlagFR = 1; }
                    else if ((mcState == mcReady) || (mcState == mcFault))
                    {
                        mcFRState.FR = CW;
                        
                        if (IRControl.TargetSpeed == 0)
                        { SetSpeed(3); }
                        else
                        { SetSpeed(IRControl.TargetSpeed); }
                    }
                }
                else
                {
                    IRControl.FlagFR = 0;
                    
                    if (!NatureWinFlag)
                    {
                        if (IRControl.TargetSpeed == 0)
                        { SetSpeed(3); }
                        else
                        { SetSpeed(IRControl.TargetSpeed); }
                    }
                    else
                    {
                    }
                }
                
                break;
            }
            
            case IRFRCW:
            {
                if (mcFRState.FR == CW)
                {
                    if ((mcState == mcRun) || (mcState == mcStart))
                    { IRControl.FlagFR = 1; }
                    else if ((mcState == mcReady) || (mcState == mcFault))
                    {
                        mcFRState.FR = CCW;
                        
                        if (IRControl.TargetSpeed == 0)
                        { SetSpeed(3); }
                        else
                        { SetSpeed(IRControl.TargetSpeed); }
                    }
                }
                else
                {
                    IRControl.FlagFR = 0;
                    
                    if (!NatureWinFlag)
                    {
                        if (IRControl.TargetSpeed == 0)
                        { SetSpeed(3); }
                        else
                        { SetSpeed(IRControl.TargetSpeed); }
                    }
                    else
                    {
                    }
                }
                
                break;
            }
            
            case IRLED:
            {
                IRControl.FlagLED = 1;
                break;
            }
            
            case IRSpeed1:
            {
                SetSpeed(1);
                break;
            }
            
            case IRSpeed2:
            {
                SetSpeed(2);
                break;
            }
            
            case IRSpeed3:
            {
                SetSpeed(3);
                break;
            }
            
            case IRSpeed4:
            {
                SetSpeed(4);
                break;
            }
            
            case IRSpeed5:
            {
                SetSpeed(5);
                break;
            }
            
            case IRSpeed6:
            {
                SetSpeed(6);
                break;
            }
            
            case IRAUTOPOWER1H:
            {
                SetAutoPower(360);                                      //360*10s =3600s =1h
                break;
            }
            
            case IRAUTOPOWER2H:
            {
                SetAutoPower(720);
                break;
            }
            
            case IRAUTOPOWER4H:
            {
                SetAutoPower(1400);
                break;
            }
            
            case IRNatureWind:
            {
                if (!IRControl.ONOFFStatus)
                {
                    IRControl.ONOFFStatus = 1;
                    mcSpeedRamp.FlagONOFF = 1;
                }
                
                if (!NatureWinFlag)
                {
                    //                          mcFRState.OldTargetSpeed = 1;
                    //                          IRControl.TargetSpeed = mcFRState.OldTargetSpeed;
                    IRControl.TargetSpeed = NatureWinCode[0];
                    mcFRState.OldTargetSpeed = IRControl.TargetSpeed;
                    IRControl.FlagSpeed = 1;
                    IRControl.NatureFlage = 1;
                    //IRControl.FlagSpeed = 1;
                    NatureWinFlag = 1;
                }
                
                break;
            }
            
            default:
                break;
        }
        
        IRScan.IRReceiveFlag = 0;
    }
}
/*  -------------------------------------------------------------------------------------------------
    Function Name : void AutoPowerControl(void)
    Description   : 自动控制开关机
    Input         : 无
    Output        : 无
    -------------------------------------------------------------------------------------------------*/
void AutoPowerControl(void)
{
    if (IRControl.FlagAutoPower)
    {
        //        if (AutoPowerState.FlagAutoPower)
        //        {
        //            AutoPowerState.FlagAutoPower = 0;
        //        }
        //        else
        //        {
        AutoPowerState.FlagAutoPower = 1;
        //        }
        AutoPowerState.Timer10sec = 0;                                  //每次按遥控重新计时
        AutoPowerState.CurrentTime = 0;
        IRControl.FlagAutoPower = 0;
    }
    
    //自动关机
    if ((AutoPowerState.FlagAutoPower && IRControl.ONOFFStatus) || ( AutoPowerState.FlagAutoPower && IRControl.LEDONOFFStatus))
    {
        AutoPowerState.Timer10sec++;
        
        if (AutoPowerState.Timer10sec > 10000)
        {
            AutoPowerState.Timer10sec = 0;
            AutoPowerState.CurrentTime++;
            
            if (AutoPowerState.CurrentTime >= AutoPowerState.ShutDowntime)
            {
                //              if(IRControl.ONOFFStatus)
                //              {
                IRControl.ONOFFStatus = 0;
                mcFRState.OldTargetSpeed = 0;
                IRControl.TargetSpeed = 0;
                mcSpeedRamp.FlagONOFF = 0;
                //              }
                //              else
                //              {
                //                  IRControl.ONOFFStatus =1;
                //                  PIControlTime = SPEED_LOOP_TIME;
                //                  IRControl.TargetSpeed = MOTOR_POWER_LEVEL_1;
                //              }
                // IRControl.FlagONOFF = 1;
                //               BZScan.FlagBZ = 1;
                IRControl.FlagSpeed = 1;
                
                if (IRControl.LEDONOFFStatus)
                {
                    ResetLEDPin;
                    IRControl.LEDONOFFStatus = 0;
                }
                
                AutoPowerState.CurrentTime = 0;
                AutoPowerState.FlagAutoPower = 0;
            }
        }
    }
    else
    {
        AutoPowerState.FlagAutoPower = 0;
        AutoPowerState.Timer10sec = 0;
        AutoPowerState.ShutDowntime = 0;
        AutoPowerState.CurrentTime = 0;
    }
}
/*  -------------------------------------------------------------------------------------------------
    Function Name : uint8 GetUserCode(void)
    Description   : LED显示，IO口电平变化实现
    Input         : 无
    Output        : 无
    -------------------------------------------------------------------------------------------------*/
void LEDDisplay(void)
{
    // static uint8 LEDDiaplayStatus = 0;
    if (IRControl.FlagLED)
    {
        if (IRControl.LEDONOFFStatus == 0)
        {
            SetLEDPin;
            // IRControl.ONOFFStatus=1;
            IRControl.LEDONOFFStatus = 1;
        }
        else
        {
            // IRControl.ONOFFStatus=0;
            ResetLEDPin;
            IRControl.LEDONOFFStatus = 0;
        }
        
        IRControl.FlagLED = 0;
    }
}

void IRONOFF_Control(void)
{
    if ((mcSpeedRamp.TargetValue == 0) && (mcState != mcRun) && (mcState != mcStop))
    {
        //关机后堵转保护次数和缺相保护次数清零
        mcProtectTime.SecondStartTimes      = 0;
        mcProtectTime.StallTimes            = 0;
        mcProtectTime.LossPHTimes           = 0;
        mcProtectTime.CurrentPretectTimes   = 0;
    }
}

void NatureWind(void)
{
    static uint16 NatureWindCnt = 0;
    static uint8  NatureWindStep = 0;
    static uint8  FlageAddorDelace = 0;
    //    if ((!NatureWinFlag) || (IRControl.FlagFR) || (!IRControl.ONOFFStatus) || (mcState != mcRun))
    //    {
    //        NatureWindCnt = 0;
    //        NatureWindStep = 0;
    //        return;
    //    }
    //    else
    //    {
    NatureWindCnt++;
    //    }
    //    IRControl.ONOFFStatus =1;
    //      mcSpeedRamp.FlagONOFF = 1;
    
    if (NatureWindCnt > 5000)
    {
        NatureWindCnt = 0;
        
        if (FlageAddorDelace == 0)
        {
            if (NatureWindStep < 5)
            { NatureWindStep++; }
            else
            {
                NatureWindStep = 5;
                FlageAddorDelace = 1;
            }
            
            IRControl.TargetSpeed = NatureWinCode[NatureWindStep];
            IRControl.FlagSpeed = 1;
        }
        else if (FlageAddorDelace)
        {
            if (NatureWindStep > 0)
            { NatureWindStep--; }
            else
            {
                NatureWindStep = 0;
                FlageAddorDelace = 0;
            }
            
            IRControl.TargetSpeed = NatureWinCode[NatureWindStep];
            IRControl.FlagSpeed = 1;
        }
    }
}
