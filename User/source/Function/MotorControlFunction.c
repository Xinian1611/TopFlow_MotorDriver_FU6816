/**
 * @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      MotorControlFunction.c
 * @author    Fortiortech  Appliction Team
 * @since     Create:2021-11-07
 * @date      Last modify:2022-07-14
 * @note      Last modify author is Kris.huang
 * @brief
 */


#include <MyProject.h>

CurrentOffset      xdata mcCurOffset;  ///< 电流采样偏置电压采集缓存

/**
 * @brief      对FOC的相关寄存器进行配置,先清理寄存器，后配置，最后使能
 * @exception  初始化FOC，需要先关闭DRVIVER，对FOC寄存器配置完成后，使能FOC再打开DRVIVER,否则可能导致第一拍电流采样错误
 * @date       2022-07-14
 */
void FOC_Init(void)
{
    DRV_CMR         = 0x0ABF;  // UH/VH/WH UL/VL/WL 互补并使能
    /* 使能FOC */
    ClrBit(DRV_CR, DRVEN);                                // 关闭DRVIVER 计时器,防止第一拍采样出错
    ClrBit(DRV_CR, FOCEN);
    SetBit(DRV_CR, FOCEN);
    ClrBit(FOC_CR0, MERRS1);
    SetBit(FOC_CR0, MERRS0);
    #if (VOLTAGE_MODE == INTERNAL)       //必须在FOC使能后才能切换ADC通道
    {
        SetBit(FOC_CR0, UCSEL);         //UDC采样通道选择0：AD2 1:AD15
    }
    #endif
    FOC_EOMEKLPF    = _Q8(1.0);                            // FOC内部 速度滤波系数
    /* 配置FOC寄存器 */
    FOC_CR1         = 0;                                   // 清零 FOC_CR1
    FOC_CR2         = 0;                                   // 清零 FOC_CR2
    FOC_IDREF       = 0;                                   // 清零 Id
    FOC_IQREF       = 0;                                   // 清零 Iq
    FOC__THETA      = 0;                                   // 清零 角度
    FOC_RTHEACC     = 0;                                   // 清零 爬坡函数的初始加速度
    FOC__RTHESTEP   = 0;                                   // 清零 爬坡速度
    FOC_RTHECNT     = 0;                                   // 清零 爬坡次数
    FOC_THECOMP     = _Q15(3.0 / 180.0);                   // SMO 估算补偿角
    FOC_THECOR      = 0x04;                                // 误差角度补偿
    FOC_EFREQACC    = 0;
    FOC_EFREQMIN    = 0;
    FOC_EFREQHOLD   = 0;
    /* 电流环参数配置 */
    FOC_DMAX        = DOUTMAX;
    FOC_DMIN        = DOUTMIN;
    FOC_QMAX        = QOUTMAX;
    FOC_QMIN        = QOUTMIN;
    FOC_DKP             = DKP;
    FOC_DKI             = DKI;
    FOC_QKP             = QKP;
    FOC_QKI             = QKI;
    /* 位置估算参数配置 */
    FOC_EK1         = OBS_K1T;
    FOC_EK2         = OBS_K2T;
    FOC_EK3         = OBS_K3T;
    FOC_EK4         = OBS_K4T;
    /* -----AO/PLL/SMO ----- */
    #if (EstimateAlgorithm == AO)
    {
        ClrBit(FOC_CR2, ESEL);
        FOC_KSLIDE      = OBS_KSLIDE;
        FOC_EKLPFMIN    = OBS_EA_KS1;
        SetBit(FOC_CR0, ESCMS);
        SetBit(FOC_CR3, MFP_EN);
    }
    #elif (EstimateAlgorithm == SMO)
    {
        ClrBit(FOC_CR2, ESEL);
        FOC_KSLIDE      = OBS_KSLIDE;
        FOC_EKLPFMIN    = OBS_EA_KS;
    }
    #elif (EstimateAlgorithm == PLL)
    {
        SetBit(FOC_CR2, ESEL);
        FOC_KSLIDE      = OBSE_PLLKP_GAIN;
        FOC_EKLPFMIN    = OBSE_PLLKI_GAIN;
    }
    #else
    {
#error " EstimateAlgorithm  ERR"
    }
    #endif
    FOC_FBASE       = OBS_FBASE;                            // 由速度计算角度增量的系数
    FOC_OMEKLPF     = SPEED_KLPF;                           // 估算器内速度低通滤波系数
    FOC_TGLI        = PWM_TGLI_LOAD;                        // 死区配置
    SetBit(FOC_CR1, SVPWMEN);                               // SVPWM模式
    
    if (mcFocCtrl.FR == CW)
    {
        ClrBit(DRV_CR, DDIR);                               // 反转标志位
    }
    else
    {
        SetBit(DRV_CR, DDIR);                               // 反转标志位
    }
    
    /* 过调制 */
    #if (OverModulation == Enable)
    {
        SetBit(FOC_CR1, OVMDL);                             // 过调制
    }
    #endif //end OverModulation
    /* 单电阻采样； 需要最小采样窗,FOC_TRGDLY为0，七段式SVPWM方式 */
    #if (Shunt_Resistor_Mode == Single_Resistor)
    {
		#if (Single_Resistor_Mode == SVPWM_OPTIM)
		{
			SetReg(FOC_CR1, CSM0 | CSM1, CSM1);                 // 新单电阻采样配置
            FOC_TRGDLY = SVPWM_OPTIM_TRGDLY;                    // 采样时刻在中点，一般考虑开关噪声影响，会设置延迟；
		}
		#else
		{
            FOC_TRGDLY = SVPWM_COMMOM_TRGDLY;                   // 采样时刻在中点，一般考虑开关噪声影响，会设置延迟；
			SetReg(FOC_CR1, CSM0 | CSM1, 0);                    // 单电阻采样配置
		}
        #endif
        FOC_TSMIN  = PWM_TS_LOAD;                           // 最小采样窗口
        // 如：0x0c表示延迟12个clock，提前用反码形式，如0x84表示提前12个clock。
        ClrBit(FOC_CR2, F5SEG);                             // 7段式，单电阻仅支持7段式
    }
    /* 双电阻采样，可设置死区补偿值，在下降沿结束前开始采样Ia Ib */
    #elif (Shunt_Resistor_Mode == Double_Resistor)          // double resistor sample
    {
        SetReg(FOC_CR1, CSM0 | CSM1, CSM0);
        FOC_TSMIN       = PWM_DT_LOAD;                      // 死区补偿值
        FOC_TRGDLY      = 0x86;                             // ADC采样的时刻，采样时刻在计数器零点附近与单电阻不同
        // 如：0x83为下降沿结束前3个clock采样Ia 0x01为上升沿开始后第一个clock开始采样。根据实际情况调整。
        FOC_TBLO        = PWM_DLOWL_TIME;                   // 下桥臂最小脉冲，保证采样
        /*五段式或七段式选择*/
        #if (SVPMW_Mode == SVPWM_7_Segment)
        {
            ClrBit(FOC_CR2, F5SEG);                         // 7段式
        }
        #elif (SVPMW_Mode == SVPWM_5_Segment)
        {
            SetBit(FOC_CR2, F5SEG);                         // 5段式
        }
        #endif
        #if (DouRes_Sample_Mode == DouRes_1_Cycle)
        {
            ClrBit(FOC_CR2, DSS);                           // 7段式
        }
        #elif (DouRes_Sample_Mode == DouRes_2_Cycle)
        {
            SetBit(FOC_CR2, DSS);                           // 5段式
        }
        #endif //end DouRes_Sample_Mode
    }
    /*三电阻采样*/
    #elif (Shunt_Resistor_Mode == Three_Resistor)           // signel resistor sample
    {
        SetReg(FOC_CR1, CSM0 | CSM1, CSM0 | CSM1);          // 三电阻
        FOC_TSMIN  = PWM_DT_LOAD;                           // 死区补偿值
        FOC_TRGDLY = 0x06;                                  // ADC采样的时刻，采样时刻在计数器零点附近与单电阻不同。
        // 如：0x83为下降沿结束前3个clock采样Ia，0x01为上升沿开始后第一个clock开始采样。根据实际情况调整。
        FOC_TBLO = PWM_OVERMODULE_TIME;                     // 过调制电流采样处理的TB脉宽
        /* 五段式或七段式选择 */
        #if (SVPMW_Mode == SVPWM_7_Segment)
        {
            ClrBit(FOC_CR2, F5SEG);                         // 7段式
        }
        #elif (SVPMW_Mode == SVPWM_5_Segment)
        {
            SetBit(FOC_CR2, F5SEG);                         // 5段式
        }
        #endif // end SVPMW_Mode
        #if (DouRes_Sample_Mode == DouRes_1_Cycle)
        {
            ClrBit(FOC_CR2, DSS);                           // 7段式
        }
        #elif (DouRes_Sample_Mode == DouRes_2_Cycle)
        {
            SetBit(FOC_CR2, DSS);                           // 5段式
        }
        #endif // end DouRes_Sample_Mode
    }
    #endif  // end Shunt_Resistor_Mode
    /* 使能电流基准校正 */
    #if (CalibENDIS == Enable)
    {
        if (mcCurOffset.OffsetFlag == 1)
        {
            #if (Shunt_Resistor_Mode == Single_Resistor)    // 单电阻校正
            {
                /*set ibus current sample offset*/
                SetReg(FOC_CR2, CSOC0 | CSOC1, 0x00);
                FOC_CSO = mcCurOffset.Iw_busOffset;         // 写入Ibus的偏置
            }
            #elif (Shunt_Resistor_Mode == Double_Resistor)  // 双电阻校正
            {
                /*set ia, ib current sample offset*/
                SetReg(FOC_CR2, CSOC0 | CSOC1, CSOC0);
                FOC_CSO  = mcCurOffset.IuOffset;            // 写入IA的偏置
            
                SetReg(FOC_CR2, CSOC0 | CSOC1, CSOC1);
                FOC_CSO  = mcCurOffset.IvOffset;            // 写入IB的偏置
            }
            #elif (Shunt_Resistor_Mode == Three_Resistor)   // 三电阻校正
            {
                /*set ibus current sample offset*/
                SetReg(FOC_CR2, CSOC0 | CSOC1, CSOC0);
                FOC_CSO = mcCurOffset.IuOffset;             // 写入IA的偏置
            
                SetReg(FOC_CR2, CSOC0 | CSOC1, CSOC1);
                FOC_CSO = mcCurOffset.IvOffset;             // 写入IB的偏置
            
                SetReg(FOC_CR2, CSOC0 | CSOC1, 0x00);
                FOC_CSO = mcCurOffset.Iw_busOffset;         // 写入IC的偏置
            }
            #endif  //end Shunt_Resistor_Mode
        }
    }
    #endif  //end CalibENDIS
    ClrBit(FOC_CR2, UDD);
    ClrBit(FOC_CR2, UQD);
    /*  -------------------------------------------------------------------------------------------------
        DRV_CTL：PWM来源选择
        OCS = 0, DRV_COMR
        OCS = 1, FOC/SVPWM/SPWM
        -------------------------------------------------------------------------------------------------*/
    /*计数器比较值来源FOC*/
    SetBit(DRV_CR, DRVEN);  // 计数器使能      0-->Disable 1-->Enable
    SetBit(DRV_CR, OCS);
}


/**
 * @brief     预充电分三步，第一步是对U相进行预充电，第二步是对V两相进行预充电;第三步是对W三相进行预充电。
 * @brief     低压应用一般不需要预充电，在customer.h禁止预充电功能寄即可
 * @exception 在低功率高压应用中，过流值较小，充电电流过大可能导致触发硬件过流, 需要注意检查
 * @date      2022-07-14
 */
void Motor_Charge(void)
{
    if (McStaSet.SetFlag.ChargeSetFlag == 0)
    {
        McStaSet.SetFlag.ChargeSetFlag = 1;
        SetBit(DRV_CR, DRVEN);  // 计数器使能      0-->Disable 1-->Enable
        DRV_DR = CHARGE_DUTY * DRV_ARR;              //下桥臂10% duty
        DRV_CMR &= 0x0000;
        /*  -------------------------------------------------------------------------------------------------
            DRV_CTL：PWM来源选择
            OCS = 0, DRV_COMR
            OCS = 1, FOC/SVPWM/SPWM
            -------------------------------------------------------------------------------------------------*/
        ClrBit(DRV_CR, OCS);
        mcFocCtrl.ChargeStep = 0;
    }
    
    if ((mcFocCtrl.State_Count < CHARGE_TIME) && (mcFocCtrl.ChargeStep == 0))
    {
        mcFocCtrl.ChargeStep = 1;
        DRV_CMR |= 0x01;                          // U相下桥臂通
        MOE = 1;
    }
    
    if (( mcFocCtrl.State_Count <= (CHARGE_TIME << 1) / 3) && (mcFocCtrl.ChargeStep == 1))
    {
        mcFocCtrl.ChargeStep = 2;
        DRV_CMR |= 0x04;                         // V相下桥臂通
    }
    
    if ((mcFocCtrl.State_Count <= CHARGE_TIME / 3) && (mcFocCtrl.ChargeStep == 2))
    {
        mcFocCtrl.ChargeStep = 3;
        DRV_CMR |= 0x10;                          // W相下桥臂通
    }
    
    #if (IPMTEST ==Enable)
    {
        if (mcFocCtrl.ChargeStep == 3)
        {
            mcFocCtrl.ChargeStep = 4;
            DRV_CMR = 0x0ABF;
        }
    }
    #endif
}


/**
 * @brief     开启次功能启动时候会将电机强拉到 设定角度，之后再启动
 * @date      2022-07-14
 */
void Motor_Align(void)
{
    if (McStaSet.SetFlag.AlignSetFlag == 0)
    {      
        McStaSet.SetFlag.AlignSetFlag = 1;
        /* -----FOC初始化----- */
        FOC_Init();
        FOC_EKP     = OBSW_KP_GAIN_START;
        FOC_EKI     = OBSW_KI_GAIN_START;
        FOC__EOME   = 0;        
        SetBit(FOC_CR2, UDD);
        SetBit(FOC_CR2, UQD);
        if (mcFocCtrl.mcDcbusFlt < (UD_Align_Duty_Voltage - 650))
        {
            mcFocCtrl.ud_align_ref = UD_Align_Duty_Max + ((mcFocCtrl.mcDcbusFlt + 650 - UD_Align_Duty_Max_Voltage) * ((float)(UD_Align_Duty - UD_Align_Duty_Max)/(float)(UD_Align_Duty_Voltage - UD_Align_Duty_Max_Voltage)));
        }
        else
        {
            mcFocCtrl.ud_align_ref = UD_Align_Duty + ((mcFocCtrl.mcDcbusFlt + 650 - UD_Align_Duty_Voltage) * ((float)(UD_Align_Duty_Min - UD_Align_Duty)/(float)(UD_Align_Duty_Min_Voltage - UD_Align_Duty_Voltage)));
        }
        /* 配置预定位的电压、KP、KI */
        //FOC__UD     = mcFocCtrl.ud_align_ref;
        
        mcFocCtrl.ud_align = 0;
        FOC__UQ     = UQ_Align_Duty;
        FOC_DKP     = DQKP_Alignment;
        FOC_DKI     = DQKI_Alignment;
        FOC_QKP     = DQKP_Alignment;
        FOC_QKI     = DQKI_Alignment;

        /*配置预定位角度*/
        if (mcFocCtrl.FR == CW)
        {
            mcFocCtrl.AlignAngle           = Align_Theta_CW;
            mcFocCtrl.AlignAngle_Temp      = Align_Theta_CW;
        }
        else
        {
            mcFocCtrl.AlignAngle           = Align_Theta_CCW;
            mcFocCtrl.AlignAngle_Temp      = Align_Theta_CCW;
        }
        mcFocCtrl.ud_align_flag            = 0; 
        FOC__THETA  = mcFocCtrl.AlignAngle;

        /*********PLL或SMO**********/
//        #if (EstimateAlgorithm == SMO )
//        {
//            FOC__ETHETA   = FOC__THETA - 4096;
//        }
//        #else
//        {
//            FOC__ETHETA   = FOC__THETA;
//        }
//        #endif //end    EstimateAlgorithm
        //SetReg(FOC_CR1, EFAE | RFAE | ANGM, ANGM);
        /*使能输出*/
        //        DRV_CMR |= 0x3F;                         // U、V、W相输出
        MOE = 1;
        FOC__THETA  = mcFocCtrl.AlignAngle;
    }
}

/**
 * @brief     顺逆风处理函数
 * @date      2022-09-14
 */
void Motor_TailWind(void)
{
    if (McStaSet.SetFlag.TailWindSetFlag == 0) //初始化
    {
        SetBit(DRV_CR, DRVEN);  // 计数器使能      0-->Disable 1-->Enable
        McStaSet.SetFlag.TailWindSetFlag = 1;
        #if (TAILWIND_MODE == RSDMethod)
        RSDDetectInit();
        #elif (TAILWIND_MODE == BEMFMethod)
        BEMFDetectInit();
        #endif
    }
    
    #if (TAILWIND_MODE == RSDMethod)
    {
        if (mcFocCtrl.State_Count > 0)  // 顺逆风检测时间结束前判断转速
        {
            if (mcRsd.SpeedUpdate == 1)
            {
                if (mcRsd.Speed < S_Value(-100))  // 逆风
                {
                    mcFocCtrl.State_Count            = 2000;
                    mcFocCtrl.Start_Mode            = HEADWIND_START;      // 逆风启动
                    mcState                         = mcStart;             // 状态机切换到Start
                    ClrBit(CMP_CR2, CMP0EN);                               // 关闭比较器
                    ClrBit(TIM2_CR1, T2CEN);                               // 0，停止计数；1,使能计数
                }
//                else if (mcRsd.Speed > MOTOR_LOOP_RPM)  //正转
//                {
//                    if (mcRsd.HighSpdStart == 2) //启动完成，切状态机
//                    {
//                        mcFocCtrl.Start_Mode    = TAILWIND_START;
//                        mcState                 = mcStart;
//                    }
//                    
//                    if (mcRsd.HighSpdStart == 0)
//                    {
//                        mcRsd.HighSpdStart = 1; // 设置启动
//                    }
//                }
                else if ((mcRsd.Speed > MOTOR_LOOP_RPM)&&(mcRsd.Speed < mcFocCtrl.Ref))  //正转,顺风转速小于目标转速，允许启动
                {
                    if (mcRsd.HighSpdStart == 2) //启动完成，切状态机
                    {
                        mcFocCtrl.Start_Mode         = TAILWIND_START;
                        mcState                 = mcStart;
                    }
                    
                    if (mcRsd.HighSpdStart == 0)
                    {
                        mcRsd.HighSpdStart = 1; // 设置启动
                    }
                }
                else if(mcRsd.Speed >= mcFocCtrl.Ref)						//顺风转速大于目标转速，开始循环检测顺风。
				{
                    ClrBit(CMP_CR2, CMP0EN);                               // 关闭比较器
                    ClrBit(TIM2_CR1, T2CEN);                               // 0，停止计数；1,使能计数
                    mcState                 = mcReady;
				}
                else//其他，如静止
                {
                    ClrBit(CMP_CR2, CMP0EN);                               // 关闭比较器
                    ClrBit(TIM2_CR1, T2CEN);                               // 0，停止计数；1,使能计数
                    mcFocCtrl.Start_Mode                = STATIC_START;
                    #if (ALIGN_MOME != ALIGN_DSIABLE)   
                    if (mcFocCtrl.FR == CW)
                    {
                        mcFocCtrl.mcPosCheckAngle           = Align_Angle_CW;
                    }
                    else
                    {
                        mcFocCtrl.mcPosCheckAngle           = Align_Angle_CCW;
                    }
                    mcState                             = mcAlign;
                    mcFocCtrl.State_Count               = Align_Time;
                    #else
                    mcState                             = mcStart;
                    #endif
                }
            }
        }
        else  // 时间结束 还未检测到有效转速 则当做静止启动
        {
            ClrBit(CMP_CR2, CMP0EN);                               // 关闭比较器
            ClrBit(TIM2_CR1, T2CEN);                               // 0，停止计数；1,使能计数
            mcFocCtrl.Start_Mode                = STATIC_START;
            #if (ALIGN_MOME != ALIGN_DSIABLE)   
            if (mcFocCtrl.FR == CW)
            {
                mcFocCtrl.mcPosCheckAngle           = Align_Angle_CW;
            }
            else
            {
                mcFocCtrl.mcPosCheckAngle           = Align_Angle_CCW;
            }
            mcState                             = mcAlign;
            mcFocCtrl.State_Count               = Align_Time;
            #else
            mcState                             = mcStart;
            #endif
        }
    }
    #elif (TAILWIND_MODE == BEMFMethod)
    {
        if (mcFocCtrl.State_Count > 0)  // 顺逆风检测时间结束前判断转速
        {
            if (mcBemf.SpeedUpdate == 1)
            {
                if (mcBemf.BEMFSpeed < S_Value(-100))  // 逆风
                {
                    mcFocCtrl.State_Count            = 2000;
                    mcFocCtrl.Start_Mode            = HEADWIND_START;      // 逆风启动
                    mcState                         = mcStart;             // 状态机切换到Start
                    ClrBit(CMP_CR2, CMP0EN);                               // 关闭比较器
                    ClrBit(TIM2_CR1, T2CEN);                               // 0，停止计数；1,使能计数
                }
//                else if ((mcBemf.BEMFSpeed > MOTOR_LOOP_RPM))  //正转
//                {
//                    if (mcBemf.HighSpdStart == 2) //启动完成，切状态机
//                    {
//                        mcFocCtrl.Start_Mode         = TAILWIND_START;
//                        mcState                 = mcStart;
//                    }
//                    
//                    if (mcBemf.HighSpdStart == 0)
//                    {
//                        mcBemf.HighSpdStart = 1; // 设置启动
//                    }
//                }
                else if ((mcBemf.BEMFSpeed > MOTOR_LOOP_RPM)&&(mcBemf.BEMFSpeed < mcFocCtrl.Ref))  //正转,顺风转速小于目标转速，允许启动
                {
                    if (mcBemf.HighSpdStart == 2) //启动完成，切状态机
                    {
                        mcFocCtrl.Start_Mode         = TAILWIND_START;
                        mcState                 = mcStart;
                    }
                    
                    if (mcBemf.HighSpdStart == 0)
                    {
                        mcBemf.HighSpdStart = 1; // 设置启动
                    }
                }
                else if(mcBemf.BEMFSpeed >= mcFocCtrl.Ref)						//顺风转速大于d等于目标转速，开始循环检测顺风。
				{
                    ClrBit(CMP_CR2, CMP0EN);                               // 关闭比较器
                    ClrBit(TIM2_CR1, T2CEN);                               // 0，停止计数；1,使能计数
                    mcState                 = mcReady;
				}
                else  //其他，如静止
                {
                    mcFocCtrl.Start_Mode                = STATIC_START;
                    #if (ALIGN_MOME != ALIGN_DSIABLE)   
                    if (mcFocCtrl.FR == CW)
                    {
                        mcFocCtrl.mcPosCheckAngle           = Align_Angle_CW;
                    }
                    else
                    {
                        mcFocCtrl.mcPosCheckAngle           = Align_Angle_CCW;
                    }
                    mcState                             = mcAlign;
                    mcFocCtrl.State_Count               = Align_Time;
                    #else
                    mcState                             = mcStart;
                    #endif
                    ClrBit(CMP_CR2, CMP0EN);  // 关闭比较器
                    ClrBit(TIM1_CR0, T1BCEN); // 关闭定时器
                }
            }
        }
        else  // 时间结束 还未检测到有效转速 则当做静止启动
        {
            mcFocCtrl.Start_Mode                = STATIC_START;
            ClrBit(CMP_CR2, CMP0EN);  // 关闭比较器
            ClrBit(TIM1_CR0, T1BCEN); // 关闭定时器
            #if (ALIGN_MOME != ALIGN_DSIABLE)   
            if (mcFocCtrl.FR == CW)
            {
                mcFocCtrl.mcPosCheckAngle           = Align_Angle_CW;
            }
            else
            {
                mcFocCtrl.mcPosCheckAngle           = Align_Angle_CCW;
            }
            mcState                             = mcAlign;
            mcFocCtrl.State_Count               = Align_Time;
            #else
            mcState                             = mcStart;
            #endif
        }
    }
    #endif
}




/**
 * @brief     FOC计算方法顺逆风检测 的顺风启动配置函数
 * @date      2022-07-14
 */
void Motor_FocTailWind_Open(void)
{
    /* 启动方式选择 */
    FOC_EFREQACC        = 500;
    FOC_EFREQMIN        = MOTOR_OMEGA_RAMP_MIN;
    FOC_EFREQHOLD       = MOTOR_OMEGA_RAMP_END;
    //    SetBit(FOC_CR1, EFAE);
    //    ClrBit(FOC_CR1, RFAE);
    //    SetBit(FOC_CR1, ANGM);
    mcFocCtrl.State_Count   = 0;        // 取消ATO爬坡
    FOC_EKP                 = OBSW_KP_GAIN_RUN4;                       // 估算器里的PI的KP
    FOC_EKI                 = OBSW_KI_GAIN_RUN4;                       // 估算器里的PI的KI
}


/**
 * @brief     静止启动配置函数
 * @date      2022-07-14
 */
void Motor_Static_Open(void)
{
    FOC_Init();
    MOE = 1;
    /****启动初始角度赋值****/
    FOC__THETA          = mcFocCtrl.AlignAngle_Temp;              // 无初始位置检测，则用预定位角
    FOC__ETHETA         = mcFocCtrl.AlignAngle_Temp - 4096;
    /*启动电流、KP、KI、FOC_EKP、FOC_EKI*/
    FOC_IDREF           = ID_Start_CURRENT;                         // D轴启动电流
    mcFocCtrl.IqRef     = IQ_Start_CURRENT >> 3;                 // Q轴启动电流
    FOC__UD             = mcFocCtrl.ud_align;
    FOC_DKP             = DKP;
    FOC_DKI             = DKI;
    FOC_QKP             = QKP;
    FOC_QKI             = QKI;
    FOC_EKP             = OBSW_KP_GAIN_START;
    FOC_EKI             = OBSW_KI_GAIN_START;
    /*启动方式选择*/
    #if (Open_Start_Mode == Omega_Start)              // Omega 启动
    {
        FOC_EFREQACC        = MOTOR_OMEGA_RAMP_ACC;
        FOC_EFREQMIN        = MOTOR_OMEGA_RAMP_MIN;
        FOC_EFREQHOLD       = MOTOR_OMEGA_RAMP_END;
        SetReg(FOC_CR1, EFAE | RFAE | ANGM, EFAE | ANGM);
    }
    #elif (Open_Start_Mode == Open_Start)
    {
        FOC_RTHEACC         = MOTOR_OPEN_RAMP_ACC;      // 爬坡函数的初始加速度
        FOC__RTHESTEP       = MOTOR_OPEN_RAMP_MIN;      // 0.62 degree acce speed
        FOC_RTHECNT         = MOTOR_OPEN_RAMP_CNT;       // acce time
        SetReg(FOC_CR1, EFAE | RFAE | ANGM, RFAE);
    }
    #elif (Open_Start_Mode == Open_Omega_Start)
    {
        FOC_RTHEACC         = MOTOR_OPEN_RAMP_ACC;      // 爬坡函数的初始加速度
        FOC__RTHESTEP       = MOTOR_OPEN_RAMP_MIN;      // 0.62 degree acce speed
        FOC_RTHECNT         = MOTOR_OPEN_RAMP_CNT;       // acce time
        FOC_EFREQACC        = Motor_OMEGA_RAMP_ACC;
        FOC_EFREQMIN        = MOTOR_OMEGA_RAMP_MIN;
        FOC_EFREQHOLD       = MOTOR_OMEGA_RAMP_END;
        SetReg(FOC_CR1, EFAE | RFAE | ANGM, EFAE | RFAE | ANGM);
    }
    #endif //end Open_Start_Mode
    /*不同启动方式下，切换到MCRUN状态*/
    #if (Open_Start_Mode == Open_Start)         //OPEN状态启动时拖动多次
    {
        mcFocCtrl.State_Count   = 0;
        FOC_EKP                 = OBSW_KP_GAIN_RUN4;                            // 估算器里的PI的KP
        FOC_EKI                 = OBSW_KI_GAIN_RUN4;                            // 估算器里的PI的KI
    }
    #elif (Open_Start_Mode == Omega_Start)
    {
        /*********PLL或SMO**********/
        #if (EstimateAlgorithm == SMO  || EstimateAlgorithm == AO)
        {
            mcFocCtrl.State_Count = ATO_START_HOLDTIME + (ATO_RAMP_PERIOD << 2); //ATO 爬坡控制时间
        }
        #elif (EstimateAlgorithm == PLL)
        {
            mcFocCtrl.State_Count   = 0;
            FOC_EKP                 = OBSW_KP_GAIN_RUN4;                       // 估算器里的PI的KP
            FOC_EKI                 = OBSW_KI_GAIN_RUN4;                       // 估算器里的PI的KI
        }
        #endif //end    EstimateAlgorithm
    }
    #endif //end Open_Start_Mode
    FOC_IQREF = mcFocCtrl.IqRef;                          // Q轴启动电流
}


/*  -------------------------------------------------------------------------------------------------
    Function Name  : MC_Stop
    Description    : inital motor control parameter
    Date           : 2021-04-10
    Parameter      : None
    ------------------------------------------------------------------------------------------------- */
void MC_Stop(void)
{
    if ((mcFocCtrl.SpeedFlt < S_Value(3000.0)) || (mcFocCtrl.State_Count == 0))
    {
        #if (StopBrakeFlag == 0)
        {
            mcState = mcReady;
            MOE = 0;
            FOC_CR1 = 0x00;
            ClrBit(DRV_CR, FOCEN);
        }
        #else
        {
            if (mcFocCtrl.SpeedFlt < Stop_MOTOR_SPEED_RPM)
            {
                MOE = 0;
                MC_Break();          // 配置刹车代码
                mcState               = mcBrake;
                mcFocCtrl.State_Count = StopWaitTime;
            }
        }
        #endif
    }
}


/**
 * @brief     三下桥刹车
 * @date      2022-07-14
 */
void MC_Break(void)
{
    MOE         = 1;
    ClrBit(DRV_CR, FOCEN);  // 关闭FOC
    /*  软件设置PWM占空比输出以上桥为参考，配置为互补输出时下桥反向 */
    ClrBit(DRV_CR, OCS);    // OCS = 0, DRV_COMR;  OCS = 1, FOC/SVPWM/SPWM
    SetBit(DRV_CR, DRVEN);  // 计数器使能      0-->Disable 1-->Enable
    DRV_CMR = 0x0015;       //  关闭上桥输出，开启下桥输出
    DRV_DR  = (DRV_ARR) + 4 ; // ARR+4为全开
}


/**
 * @brief     控制变量上电初始化,包括保护参数的初始化、电机状态初始化
 * @brief     上电只运行一次
 * @date      2022-07-14
 */
void MotorcontrolInit(void)
{
    /*****电机状态机时序变量***********/
    McStaSet.SetMode                    = 0;
    /**********电机目标方向*************/
    #if (FR_MODE == CW)
    {
        mcFocCtrl.FR                    = CW;
    }
    #else
    {
        mcFocCtrl.FR                    = CCW;
    }
    #endif //end IRMODE
    /* -----电流偏置校准变量初始化----- */
    mcCurOffset.IuOffsetSum            = 16383;
    mcCurOffset.IvOffsetSum            = 16383;
    mcCurOffset.Iw_busOffsetSum        = 16383;
    mcPwmInput.TimerARR                = 65535;
    memset(&Fault, 0, sizeof(FaultVarible));         //保护变量清零
    memset(&Restart, 0, sizeof(FaultRecoverTypedef));//重启变量清零
}


/**
 * @brief     初始化电机参数，每次电机重启均会被调用
 * @warning   需要注意填写的变量是否可以在此时刻被初始化
 * @date      2022-07-14
 */
void VariablesPreInit(void)
{
    memset(&Fault, 0, sizeof(FaultVarible));                        //保护变量清零
    Fault.Voltage.OverVoltageVal        = OVER_VOLTAGE_PROTECT;
    Fault.Voltage.UnderVoltageVal       = UNDER_VOLTAGE_PROTECT;
    Fault.Stall.OverSpeedVal            = STALL_MAX_SPEED;
    Fault.Stall.UnderSpeedVal           = STALL_MIN_SPEED;
    McStaSet.SetMode                    = 0;                        ///< 整个配置模式使能位，每次电机重启清零                     
    mcFocCtrl.CtrlMode                  = 0;
    mcFocCtrl.ChargeStep                = 0;
    mcFocCtrl.Flg_ATORampEnd            = 0;
    mcFocCtrl.Start_Mode   = STATIC_START;                          //初始化为静止启动
    Flag_Single_Mode = 0;                                           //初始化为新单电阻     
    
}




/**
 * @brief     上电时，先对硬件电路的电流进行采集，写入对应的校准寄存器中。
              调试时，需观察mcCurOffset结构体中对应变量是否在范围内。采集结束后，OffsetFlag置1。
 * @date      2022-07-14
 */
void GetCurrentOffset(void)
{
    SetBit(ADC_CR, ADCBSY);             // 使能ADC
    
    while (ReadBit(ADC_CR, ADCBSY));
    
    #if (Shunt_Resistor_Mode == Single_Resistor)                   //单电阻模式
    {
        mcCurOffset.Iw_busOffsetSum += ((ADC4_DR & 0x7ff8));
        mcCurOffset.Iw_busOffset     = mcCurOffset.Iw_busOffsetSum >> 4;
        mcCurOffset.Iw_busOffsetSum -= mcCurOffset.Iw_busOffset;
    }
    #elif (Shunt_Resistor_Mode == Double_Resistor)                 //双电阻模式
    {
        mcCurOffset.IuOffsetSum     += ((ADC0_DR & 0x7ff8));
        mcCurOffset.IuOffset         = mcCurOffset.IuOffsetSum >> 4;
        mcCurOffset.IuOffsetSum     -= mcCurOffset.IuOffset;
        mcCurOffset.IvOffsetSum     += ((ADC1_DR & 0x7ff8));
        mcCurOffset.IvOffset         = mcCurOffset.IvOffsetSum >> 4;
        mcCurOffset.IvOffsetSum     -= mcCurOffset.IvOffset;
    }
    #elif (Shunt_Resistor_Mode == Three_Resistor)                 //三电阻模式
    {
        mcCurOffset.IuOffsetSum     += ((ADC0_DR & 0x7ff8));
        mcCurOffset.IuOffset         = mcCurOffset.IuOffsetSum >> 4;
        mcCurOffset.IuOffsetSum     -= mcCurOffset.IuOffset;
        mcCurOffset.IvOffsetSum     += ((ADC1_DR & 0x7ff8));
        mcCurOffset.IvOffset         = mcCurOffset.IvOffsetSum >> 4;
        mcCurOffset.IvOffsetSum     -= mcCurOffset.IvOffset;
        mcCurOffset.Iw_busOffsetSum += ((ADC4_DR & 0x7ff8));
        mcCurOffset.Iw_busOffset     = mcCurOffset.Iw_busOffsetSum >> 4;
        mcCurOffset.Iw_busOffsetSum -= mcCurOffset.Iw_busOffset;
    }
    #endif
    mcCurOffset.OffsetCount++;
    
    if (mcCurOffset.OffsetCount > Calib_Time)
    {
        mcCurOffset.OffsetFlag = 1;
		#if (OffsetProtectEn)
        {
            Fault_GetCurrentOffset();     //偏置电压保护
        }
        #endif
    }
}



/**
 * @brief     关闭输出，关闭FOC，电机切换到mcReady状态被调用一次
 * @date      2022-07-14
 */
void Motor_Ready(void)
{
    if (McStaSet.SetFlag.CalibFlag == 0)
    {
        McStaSet.SetFlag.CalibFlag = 1;
        MOE               = 0;
        ClrBit(DRV_CR, FOCEN);
        ClrBit(DRV_CR, DRVEN);
    }
}


/**
 * @brief     电机初始化，对电机相关变量、PI进行初始化设置,关闭FOC所需要使用到的ADC
 * @note      关闭FOC所需要使用到的ADC，FOC模块会自动调用相应ADC 无需外部使能
 * @date      2022-07-14
 */

void Motor_Init(void)
{
    VariablesPreInit();                           // 电机相关变量初始化
}

void SquStartInit(void)
{
    if (SquStart.StartState == 1)
    {
				memset(&SquStart, 0, sizeof(SquStartVar));
        /* -----FOC初始化----- */
        FOC_Init();
				#if (Shunt_Resistor_Mode == Single_Resistor)
						{
//								#if (Start_Single_Resistor_Mode == SVPWM_OPTIM)
//								{
//										SetReg(FOC_CR1, CSM0 | CSM1, CSM1);                 // 新单电阻采样配置
//										FOC_TRGDLY = SVPWM_OPTIM_TRGDLY;                    // 采样时刻在中点，一般考虑开关噪声影响，会设置延迟；
//								}
//								#else
//								{
										FOC_TRGDLY = SVPWM_COMMOM_TRGDLY;                   // 采样时刻在中点，一般考虑开关噪声影响，会设置延迟；
										SetReg(FOC_CR1, CSM0 | CSM1, 0);                    // 单电阻采样配置
//								}
//								#endif
								FOC_TSMIN  = PWM_TS_LOAD;                           // 最小采样窗口
								// 如：0x0c表示延迟12个clock，提前用反码形式，如0x84表示提前12个clock。
								ClrBit(FOC_CR2, F5SEG);                             // 7段式，单电阻仅支持7段式
						}
						#endif
        /* 配置预定位的电流、KP、KI */

        SquStart.PWMFreqency = PWM_FREQUENCY;
				SquStart.FreqencyBase = (MOTOR_SPEED_BASE / 60.0);
        SquStart.SwitchSpeed = FOCSwitchSpeed;
        SquStart.IgnoreCount = (uint16)(10 * SquStart.PWMFreqency);
        SquStart.MainCurrentSumMin = 16000;
        SquStart.ACurrent1SumMin   = 16000;
        SquStart.ACurrent2SumMin   = 16000;
//        SquStart.MotorSpeedCount1  = 10000;
        SquStart.MotorSpeedCount2  = StartCurrentTime;
				SquStart.MotorSpeedCount3  = 10000;
        SquStart.UqReady = 1;
        LPF1_K = 40 * 256;
        LPF1_YH = 0;
        LPF1_YL = 0;
        FOC__UQ = DeadTimeComCount;
        FOC__UD = _Q15(0.00);
        /*配置预定位角度*/
				FOC_IQREF = SquStartCurrent;
        SquStart.CurrentThreshold = StartCurrentAddThreshold;
        SquStart.ThetaOffset     = SquStartThetaOffset;
        SquStart.MinUq = MinUqSet;
        SquStart.StartSector     = SquStartSector;
        FOC__THETA               = SquStartTheta + SquStart.ThetaOffset;
        SquStart.StartCurrentThreshold = SquStartCurrent;
        SquStart.RotorTheta      = FOC__THETA;
				SquStart.ThetaRamp	= THETARAMP;
        /*********PLL或SMO**********/
        FOC__ETHETA   = FOC__THETA - 4096;
        /*使能输出*/
        SquStart.StartState = 3;
        MOE = 1;
    }
}

void SquToFOCInit(void)
{
    if (SquStart.StartState == 4)
    {
        SquStart.Temp = FOC__THETA;
        SquStart.Temp = SquStart.Temp + 150;
        SquStart.Temp1 = SquStart.Temp - (_Q15(91.0 / 180.0) + CW_THETA_BASE);
        
        if ((SquStart.Temp1 > 0) && (SquStart.Temp1 < (150 + 5)))
        {
						#if (Shunt_Resistor_Mode == Single_Resistor)
						{
								#if (Start_Single_Resistor_Mode == SVPWM_OPTIM)
								{
										SetReg(FOC_CR1, CSM0 | CSM1, CSM1);                 // 新单电阻采样配置
										FOC_TRGDLY = SVPWM_OPTIM_TRGDLY;                    // 采样时刻在中点，一般考虑开关噪声影响，会设置延迟；
								}
								#else
								{
										FOC_TRGDLY = SVPWM_COMMOM_TRGDLY;                   // 采样时刻在中点，一般考虑开关噪声影响，会设置延迟；
										SetReg(FOC_CR1, CSM0 | CSM1, 0);                    // 单电阻采样配置
								}
								#endif
								FOC_TSMIN  = PWM_TS_LOAD;                           // 最小采样窗口
								// 如：0x0c表示延迟12个clock，提前用反码形式，如0x84表示提前12个clock。
								ClrBit(FOC_CR2, F5SEG);                             // 7段式，单电阻仅支持7段式
						}
						#endif
            SquStart.StartState = 0;
            FOC__THETA = _Q15(91.0 / 180.0) + CW_THETA_BASE;
            FOC__ETHETA = FOC__THETA  - 4096;
            FOC_IDREF           = ID_Start_CURRENT;                         // D?????
            mcFocCtrl.IqRef         = IQ_Start_CURRENT;
            FOC_DKP             = DKP;
            FOC_DKI             = DKI;
            FOC_QKP             = QKP;
            FOC_QKI             = QKI;
            FOC_EKP    = OBSW_KP_GAIN_Switch;
            FOC_EKI    = OBSW_KI_GAIN_Switch;
            FOC_EFREQACC  = 300;
            FOC_EFREQMIN  = 0.8 * FOCSwitchSpeed / Pole_Pairs;
            FOC_EFREQHOLD = FOCSwitchSpeed / Pole_Pairs;
            SetReg(FOC_CR1, EFAE | RFAE | ANGM, EFAE | ANGM);
            mcFocCtrl.State_Count = 25 + (ATO_RAMP_PERIOD << 2);
            FOC_OMEKLPF        = SPEED_KLPF;
            FOC__EOME = SquStart.MotorSpeed / (uint8)(Pole_Pairs);
            FOC__UQ = SquStart.StartUq;
            FOC_IQREF = mcFocCtrl.IqRef;                          // Q?????
            ClrBit(FOC_CR2, UQD);
            ClrBit(FOC_CR2, UDD);
            mcState = mcRun;
        }
        else
        {
            FOC__THETA = SquStart.Temp;
        }
    }
}

