/**
 * @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      AddFunction.c
 * @author    Fortiortech  Appliction Team
 * @since     Create:2022-07-13
 * @date      Last modify:2022-07-14
 * @note      Last modify author is Kris.huang
 * @brief     This file contains main function used for Motor Control.
 */

#include <MyProject.h>

/* Public variables --------------------------------------------------------- */

bool data isCtrlPowOn = false;           ///< 开关机控制
PWMINPUTCAL xdata mcPwmInput;            ///< PWM捕获结构体变量
FOCCTRL xdata mcFocCtrl;                 ///< FOC电机控制相关结构体变量
MCRAMP xdata mcRefRamp;                  ///< 控制指令爬坡结构体相关变量
debugONOFFTypeDef xdata debug_ONOFFTest; ///< ONOFF启停测试小工具结构体变量
SLEEPMODE xdata SleepSet;                ///< 休眠结构体变量
uint16 xdata Power_Currt;
uint8 Flag_Single_Mode = 0;

/**
    @brief        对变量取16位的绝对值
    @param[in]    value
    @return       绝对值
    @date         2022-07-13
*/
uint16 Abs_F16(int16 value)
{
    if (value < 0)
    {
        return (-value);
    }
    else
    {
        return (value);
    }
}

/**
    @brief        对变量取32位的绝对值
    @param[in]    value
    @return       绝对值
    @date         2022-07-13
*/
uint32 Abs_F32(int32 value)
{
    if (value < 0)
    {
        return (-value);
    }
    else
    {
        return (value);
    }
}

/**
 * @brief        PWM调速信号计算，本例程提供Duty计算，如需频率信号可自行使用mcPwmInput.Period周期值计算
 * @date         2022-07-14
 */
void PWMDutyCal(void)
{
    static uint16 dutyTemp = 0;

    if (mcPwmInput.isUpdate) // 有新的duty更新
    {
        if ((Abs_F32(mcPwmInput.TimerDR - mcPwmInput.TimerDROld) < 0xFF)       // 误差在1个Byte之间再处理
            && (Abs_F32(mcPwmInput.TimerARROld - mcPwmInput.TimerARR) < 0xFF)) // 误差在1个Byte之间再处理
        {
            mcPwmInput.Compare = mcPwmInput.TimerDR; // 读取DR与ARR值
            mcPwmInput.Period = mcPwmInput.TimerARR;
            mcPwmInput.Duty = MDU_DIV0_GetQL16(mcPwmInput.Compare >> 1, 0x0000, mcPwmInput.Period);
/***速度随PWM增大而增大***/
#if (PWMDUTY_POLARITY == NegaPWMDUTY)
            {
                dutyTemp = 32768 - mcPwmInput.Duty;
            }
/***速度随PWM增大而减小***/
#else
            {
                dutyTemp = mcPwmInput.Duty;
            }
#endif

            if ((dutyTemp > ONPWMDuty) && (dutyTemp <= OFFPWMDutyHigh))
            {
                mcPwmInput.MotorOffFilter = MotorOffFilterTime;
                if (mcPwmInput.MotorOnFilter == 0) // 开机滤波
                {
                    mcPwmInput.MotorOnFilter = MotorOnFilterTime;
                    isCtrlPowOn = true; // 开机
                }
            }
            else if ((dutyTemp < OFFPWMDuty) || (dutyTemp > OFFPWMDutyHigh))
            {
                mcPwmInput.MotorOnFilter = MotorOnFilterTime;
                if (mcPwmInput.MotorOffFilter == 0) // 关机滤波
                {
                    mcPwmInput.MotorOffFilter = MotorOffFilterTime;
                    isCtrlPowOn = false; // 关机
                }
            }
            else
            {
                // 不做处理，保持前一个状态
            }

            // 转速曲线计算
            if (isCtrlPowOn == true)
            {
                if (dutyTemp <= MINPWMDuty)
                {
                    mcFocCtrl.Ref = MOTOR_SPEED_MIN_RPM;
                }
                else if (dutyTemp >= MAXPWMDuty)
                {
                    mcFocCtrl.Ref = MOTOR_SPEED_MAX_RPM;
                }
                else
                {
                    mcFocCtrl.Ref = MOTOR_SPEED_MIN_RPM + SPEED_K * (dutyTemp - MINPWMDuty);
                }
            }
            else
            {
                mcFocCtrl.Ref = 0;
            }
        }

        mcPwmInput.isUpdate = 0;
        mcPwmInput.TimerDROld = mcPwmInput.TimerDR;   // 将此次比较值赋值给上次比较值
        mcPwmInput.TimerARROld = mcPwmInput.TimerARR; // 将此次周期值赋值给上次周期值
    }
}

/**
 * @brief        VSP调速信号处理
 * @date         2022-07-14
 */
void VSPSample(void)
{
    static int16 VSP = 0;
    /*****VREF的采样获取值并滤波******/
    VSP = MDU_LPF0(ADC7_DR, VSP, 10); // 注意低通滤波器系数范围为0---127

    if ((VSP > ONPWMDuty) && (VSP <= OFFPWMDutyHigh)) // 在ONPWMDuty-OFFPWMDutyHigh之间，电机有转速运行
    {
        isCtrlPowOn = true; // 开机
    }
    else if ((VSP < OFFPWMDuty) || (VSP > OFFPWMDutyHigh))
    {
        isCtrlPowOn = false; // 关机
    }

    // 转速曲线计算
    if (isCtrlPowOn == true) //
    {
#if (MOTOR_CTRL_MODE == SPEED_LOOP_CONTROL)
        {
            if (VSP <= MINPWMDuty) // 最小转速运行
            {
                mcFocCtrl.Ref = MOTOR_SPEED_MIN_RPM;
            }
            else if (VSP < MAXPWMDuty) // 调速
            {
                mcFocCtrl.Ref = MOTOR_SPEED_MIN_RPM + SPEED_K * (VSP - MINPWMDuty);
            }
            else // 最大转速运行
            {
                mcFocCtrl.Ref = MOTOR_SPEED_MAX_RPM;
            }
        }
#endif
    }
    else
    {
        mcFocCtrl.Ref = 0;
    }
}

/**
 * @brief        启停测试工具，用于测试启动可靠性
 * @date         2022-07-14
 */
void ONOFF_Test(void)
{
    if (debug_ONOFFTest.State == 1) // 开机状态
    {
        debug_ONOFFTest.TimeCnt++;

        if (debug_ONOFFTest.TimeCnt > ONOFFTEST_ON_TIME)
        {
            debug_ONOFFTest.Times++;   // 启停次数+1
            debug_ONOFFTest.State = 0; // 切换到关机状态
            debug_ONOFFTest.TimeCnt = 0;
            mcFocCtrl.Ref = 0;   // 目标值也给0
            isCtrlPowOn = false; // 关机
        }
    }
    else // 关机状态
    {
        debug_ONOFFTest.TimeCnt++;

        if (debug_ONOFFTest.TimeCnt > ONOFFTEST_OFF_TIME)
        {
            debug_ONOFFTest.TimeCnt = 0;

            if ((mcState != mcFault) && (mcState != mcStop))
            {
                debug_ONOFFTest.State = 1; // 切换到开机状态
#if (MOTOR_CTRL_MODE == SPEED_LOOP_CONTROL)
                {
                    mcFocCtrl.Ref = ONOFFTEST_REF; // 固定转速赋值
                }
#elif (MOTOR_CTRL_MODE == POWER_LOOP_CONTROL)
                {
                    mcFocCtrl.Ref = ONOFFTEST_PowerREF; // 固定功率赋值
                }
#elif (MOTOR_CTRL_MODE == CURRENT_LOOP_CONTROL)
                {
                    mcFocCtrl.Ref = ONOFFTEST_CurrentREF; // 固定相电流赋值
                }
#endif
                isCtrlPowOn = true; // 开机

                //                if (mcFocCtrl.FR == CW)
                //                {
                //                    mcFocCtrl.FR         = CCW;
                //                }
                //                else
                //                {
                //                    mcFocCtrl.FR        = CW;
                //                }
            }
        }
    }
}

/**
 * @brief        调速信号处理包含：开关机控制、将调速信号处理成控制目标给定信号
 * @date         2022-07-14
 */
void TargetRef_Process(void)
{
#if (SPEED_MODE == PWMMODE)
    {
        PWMDutyCal();
#if (SLEEPEN == Enable)
        if ((mcPwmInput.Duty == 0) || (mcPwmInput.Duty == 32768)) // PWM信号输入悬空时进入休眠，悬空时PWM上拉到12V
        {
            sc_goToSleep(); // 休眠函数
        }
#endif
    }
#elif (SPEED_MODE == LINMODE)
    {
        EOP_RXLIN(); // EOP,LIN接收数据，调速
        EOP_TXLIN(); // EOP,LIN反馈数据,10ms更新一次

        if (loc_linBusTimeoutCnt < 4000) // 4S后进入休眠
        {
            loc_linBusTimeoutCnt++;
        }
        else
        {
            loc_linBusTimeoutFlag = 1; // LIN总线丢失
        }
/*****休眠********/
#if (SLEEPEN == Enable)
        if (loc_linBusTimeoutFlag == 1)
        {
            sc_goToSleep(); // 休眠函数
        }
#endif
    }
#elif (SPEED_MODE == SREFMODE)
    {
        VSPSample();
    }
#elif (SPEED_MODE == NONEMODE)
    {
        isCtrlPowOn = true; // 开机
#if (MOTOR_CTRL_MODE == SPEED_LOOP_CONTROL)
        {
            mcFocCtrl.Ref = ONOFFTEST_REF; // 固定转速赋值
        }
#elif (MOTOR_CTRL_MODE == POWER_LOOP_CONTROL)
        {
            mcFocCtrl.Ref = ONOFFTEST_PowerREF; // 固定功率赋值
        }
#elif (MOTOR_CTRL_MODE == CURRENT_LOOP_CONTROL)
        {
            mcFocCtrl.Ref = ONOFFTEST_CurrentREF; // 固定相电流赋值
        }
#endif
    }
#elif (SPEED_MODE == ONOFFTEST)
    {
        ONOFF_Test();
    }
#endif
}

/**
 * @brief        外部闭环控制函数，示例代码提供 电流环，速度环，功率环，UQ控制示例代码,可根据需要自行修改
 *               建议使用默认1ms周期运行
 * @date         2022-07-14
 */
void Speed_response(void)
{
    static int16 refRampOut = 0;

    if ((mcState == mcRun) || (mcState == mcStop))
    {
        switch (mcFocCtrl.CtrlMode)
        {
        case 0: // 开环(电流环内环)
        {
            if (mcFocCtrl.SpeedFlt > MOTOR_LOOP_RPM) // 转速大于一定值，切入转速闭环
            {
                mcFocCtrl.Mode0HoldCnt++;

                if (mcFocCtrl.Mode0HoldCnt > 10) // 切换后保持电流
                {
                    mcFocCtrl.CtrlMode = 1;
                    FOC_QKP = QKP; // 电流环PI
                    FOC_QKI = QKI;
                    FOC_DKP = DKP;
                    FOC_DKI = DKI;
                    PI_Init();  // 速度环PI初始化
                    PI2_Init(); // 限流环PI初始化
                    PI3_Init(); // 弱磁PI初始化
// 启动电流环与外环给定衔接
#if (MOTOR_CTRL_MODE == SPEED_LOOP_CONTROL)
                    {
                        if (mcFocCtrl.Start_Mode == TAILWIND_START)
                        {
#if (TAILWIND_MODE == RSDMethod)
                            mcRefRamp.OutValue_float = mcRsd.Speed;
#elif (TAILWIND_MODE == BEMFMethod)
                            mcRefRamp.OutValue_float = mcBemf.BEMFSpeed;
#endif
                        }
                        else
                        {
                            mcRefRamp.OutValue_float = mcFocCtrl.SpeedFlt;
                        }
                    }
#elif (MOTOR_CTRL_MODE == POWER_LOOP_CONTROL)
                    {
                        mcRefRamp.OutValue_float = mcFocCtrl.PowerFlt;
                    }
#elif (MOTOR_CTRL_MODE == UQ_LOOP_CONTROL)
                    {
                        mcRefRamp.OutValue_float = mcFocCtrl.UqFlt;
                        SetBit(FOC_CR2, UQD);
                    }
#endif
                    FOC_THECOMP = _Q15(3.0 / 180.0);
                    mcFocCtrl.LoopTime = LOOP_TIME;
                    mcRefRamp.IncValue = RAMP_INC;
                    mcRefRamp.DecValue = RAMP_DEC;
                    mcFocCtrl.IqRef = FOC_IQREF;
                    FOC_IDREF = ID_RUN_CURRENT;
                    PI1_UKH = mcFocCtrl.IqRef;
                }
            }
            else
            {
                mcFocCtrl.Angle_Com = _Q15(3.0 / 180.0);
                FOC_THECOMP = mcFocCtrl.Angle_Com;
                mcFocCtrl.Mode0HoldCnt = 0;
            }
        }
        break;

        case 1: // 闭环(速度闭环+电流内环)
        {
            mcFocCtrl.LoopTime++;

            if (mcFocCtrl.LoopTime >= LOOP_TIME)
            {
                mcFocCtrl.LoopTime = 0;
                refRampOut = Motor_Ramp(mcFocCtrl.Ref); // 控制命令爬坡函数，用于实现调速信号之间平滑过渡
#if (MOTOR_CTRL_MODE == CURRENT_LOOP_CONTROL)
                {
                    mcFocCtrl.IqRef = refRampOut;
                    FOC_IQREF = mcFocCtrl.IqRef;
                }
#elif (MOTOR_CTRL_MODE == SPEED_LOOP_CONTROL)
                {
#if (Weak_MagneticEn == Disable)
                    {
                        mcFocCtrl.IqSpeedRef = MDU_PI1(refRampOut - mcFocCtrl.SpeedFlt);
                        mcFocCtrl.IqADCCurrentRef = MDU_PI2(Motor_Max_ADCCurrentbus - mcFocCtrl.mcADCCurrentbus); // 限制最大电流
#if (ADCCurrentbusLitmitEn == Disable)
                        {
                            mcFocCtrl.IqRef = mcFocCtrl.IqSpeedRef;
                        }
#else
                        {
                            if (mcFocCtrl.IqSpeedRef >= mcFocCtrl.IqADCCurrentRef) // 二者取其小
                            {
                                mcFocCtrl.IqRef = mcFocCtrl.IqADCCurrentRef;
                            }
                            else
                            {
                                mcFocCtrl.IqRef = mcFocCtrl.IqSpeedRef;
                            }
                        }
#endif
                        FOC_IQREF = mcFocCtrl.IqRef;
                    }
#else
                    {
                        mcFocCtrl.IqSpeedRef = MDU_PI1(refRampOut - mcFocCtrl.SpeedFlt);
                        mcFocCtrl.IqADCCurrentRef = MDU_PI2(Motor_Max_ADCCurrentbus - mcFocCtrl.mcADCCurrentbus); // 限制最大电流
                        // 根据速度误差计算WeakRefRef
                        if (mcFocCtrl.IqSpeedRef >= mcFocCtrl.IqADCCurrentRef) // 二者取其小
                        {
                            mcFocCtrl.WeakRef = mcFocCtrl.IqADCCurrentRef;
                        }
                        else
                        {
                            mcFocCtrl.WeakRef = mcFocCtrl.IqSpeedRef;
                        }
						  mcFocCtrl.WeakRef = mcFocCtrl.IqSpeedRef;
                        // 计算(UD^2+UQ^2)开根号
                        mcFocCtrl.sqrtUdq = SqrtUDQ(FOC__UD, FOC__UQ);
                        // 根据计算Id Iq分配角度
                        mcFocCtrl.Angle = MDU_PI3(_Q15(0.80) - mcFocCtrl.sqrtUdq);
                        // 根据角度分配 Id Iq，实现自动弱磁
                        SinCal(mcFocCtrl.WeakRef, mcFocCtrl.Angle, &mcFocCtrl.IdRef, &mcFocCtrl.IqRef);
                        FOC_IQREF = mcFocCtrl.IqRef;
                        FOC_IDREF = mcFocCtrl.IdRef;
                    }
#endif
                }
#elif (MOTOR_CTRL_MODE == POWER_LOOP_CONTROL)
                {
                    mcFocCtrl.IqRef = MDU_PI1(refRampOut - mcFocCtrl.PowerFlt);
                    FOC_IQREF = mcFocCtrl.IqRef;
                }
#elif (MOTOR_CTRL_MODE == UQ_LOOP_CONTROL)
                {
                    mcFocCtrl.IqRef = MDU_PI1(refRampOut - mcFocCtrl.UqFlt);
                    FOC__UQ = mcFocCtrl.IqRef;
                }
#else
                {
                    /* ------------------自定义闭环START--------------------- */
                    /* ------------------自定义闭环 END--------------------- */
                }
#endif
            }
        }
        break;
        }

#if ((Shunt_Resistor_Mode == Single_Resistor) && (Single_Resistor_Mode == SVPWM_OPTIM))
        {
            if ((FOC__UQ > Single_Resistor_Mode_SwitchDuty1) && (Flag_Single_Mode == 0)) // 新单电阻采样由于UQ高占空比（%60以下）存在不可采样区，必须切换为常规单电阻采样，也可用转速作为切换条件
            {
                Flag_Single_Mode = 1;
                SetReg(FOC_CR1, CSM0 | CSM1, 0x00); // 单电阻采样配置
                FOC_TSMIN = PWM_TS_LOAD1;           // 最小采样窗口
                FOC_TRGDLY = SVPWM_COMMOM_TRGDLY;
            }
            else if ((FOC__UQ < Single_Resistor_Mode_SwitchDuty2) && (Flag_Single_Mode == 1))
            {
                Flag_Single_Mode = 0;
                SetReg(FOC_CR1, CSM0 | CSM1, CSM1); // 新单电阻采样配置
                FOC_TSMIN = PWM_TS_LOAD;            // 最小采样窗口
                FOC_TRGDLY = SVPWM_OPTIM_TRGDLY;
            }
        }
#endif
    }
}

/**
 * @brief        控制给定爬坡函数
 *               以浮点进行计算，解决整数爬坡由于精度的影响，导致爬坡结果阶梯变化
 *               函数控制周期默认为闭环控制周期，建议使用默认1ms周期运行
 * @param[in]    ref 给定目标值
 * @return       爬坡结果（int16）
 * @date         2022-07-14
 */
int16 Motor_Ramp(int16 ref)
{
    mcRefRamp.RefValue = ref; // 爬坡函数输入

    if (mcRefRamp.OutValue_float < mcRefRamp.RefValue)
    {
        if (mcRefRamp.OutValue_float + mcRefRamp.IncValue < mcRefRamp.RefValue)
        {
            mcRefRamp.OutValue_float += mcRefRamp.IncValue;
        }
        else
        {
            mcRefRamp.OutValue_float = mcRefRamp.RefValue;
        }
    }
    else
    {
        if (mcRefRamp.OutValue_float - mcRefRamp.DecValue > mcRefRamp.RefValue)
        {
            mcRefRamp.OutValue_float -= mcRefRamp.DecValue;
        }
        else
        {
            mcRefRamp.OutValue_float = mcRefRamp.RefValue;
        }
    }

    return (int16)mcRefRamp.OutValue_float; // 输出浮点数取整
}

/**
 * @brief        预定位控制
 * @date         2022-07-14
 */
void Motor_Align_Angle(void)
{
    if ((mcFocCtrl.ud_align < mcFocCtrl.ud_align_ref) && (mcFocCtrl.ud_align_flag == 0))
    {
        mcFocCtrl.ud_align = mcFocCtrl.ud_align + 6;

        if (mcFocCtrl.FR == CW)
        {
            mcFocCtrl.AlignAngle += 5;
            FOC__THETA = mcFocCtrl.AlignAngle;
        }
        else
        {
            mcFocCtrl.AlignAngle -= 5;
            FOC__THETA = mcFocCtrl.AlignAngle;
        }

        if (mcFocCtrl.ud_align >= mcFocCtrl.ud_align_ref)
        {
            mcFocCtrl.ud_align_flag = 1;
            PI2_KP = _Q12(0.05);
            PI2_KI = _Q15(0.005);
            PI2_EK1 = 0;
            PI2_EK = 0;
            PI2_UKH = mcFocCtrl.ud_align_ref;
            PI2_UKL = 0;
            PI2_UKMAX = UD_Align_Duty_Max;
            PI2_UKMIN = UD_Align_Duty_Min;
            mcFocCtrl.AlignAngle = Align_Theta_End;
        }

        mcFocCtrl.AlignAngle_Temp = mcFocCtrl.AlignAngle;
        mcFocCtrl.id_align_sample_pre = ID_Align_CURRENT - 70;
    }
    else
    {
        mcFocCtrl.id_align_sample = FOC__ID;
        mcFocCtrl.id_align_sample_error = mcFocCtrl.id_align_sample_pre - mcFocCtrl.id_align_sample;
        mcFocCtrl.id_align_sample_pre = mcFocCtrl.id_align_sample;
        PI2_EK = ID_Align_CURRENT - mcFocCtrl.id_align_sample; // 填入EK
        SMDU_RunBlock(2, PI);
        mcFocCtrl.ud_align = PI2_UKH;
        MUL0_MA = _Q8(8.0);

        if (mcFocCtrl.id_align_sample_error > 0)
        {
            MUL0_MB = mcFocCtrl.id_align_sample_error;
            SMDU_RunBlock(0, UMUL);
            mcFocCtrl.MDU_TEMP = MUL0_MCL;
            mcFocCtrl.MDU_TEMP = mcFocCtrl.MDU_TEMP << 8;

            if (mcFocCtrl.MDU_TEMP > 32767)
            {
                mcFocCtrl.align_theta_acc = (MUL0_MCH << 8) + (MUL0_MCL >> 8) + 1;
            }
            else
            {
                mcFocCtrl.align_theta_acc = (MUL0_MCH << 8) + (MUL0_MCL >> 8);
            }
        }
        else
        {
            MUL0_MB = -mcFocCtrl.id_align_sample_error;
            SMDU_RunBlock(0, UMUL);
            mcFocCtrl.MDU_TEMP = MUL0_MCL;
            mcFocCtrl.MDU_TEMP = mcFocCtrl.MDU_TEMP << 8;

            if (mcFocCtrl.MDU_TEMP > 32767)
            {
                mcFocCtrl.align_theta_acc = (MUL0_MCH << 8) + (MUL0_MCL >> 8) + 1;
            }
            else
            {
                mcFocCtrl.align_theta_acc = (MUL0_MCH << 8) + (MUL0_MCL >> 8);
            }

            mcFocCtrl.align_theta_acc = -mcFocCtrl.align_theta_acc;
        }

        mcFocCtrl.AlignAngle = mcFocCtrl.AlignAngle + mcFocCtrl.align_theta_acc;

        if (mcFocCtrl.AlignAngle_Temp > (mcFocCtrl.AlignAngle + 100))
        {
            mcFocCtrl.AlignAngle_Temp = mcFocCtrl.AlignAngle_Temp - 100;
        }
        else if (mcFocCtrl.AlignAngle_Temp < (mcFocCtrl.AlignAngle - 100))
        {
            mcFocCtrl.AlignAngle_Temp = mcFocCtrl.AlignAngle_Temp + 100;
        }
        else
        {
            mcFocCtrl.AlignAngle_Temp = mcFocCtrl.AlignAngle;
        }

        FOC__THETA = mcFocCtrl.AlignAngle_Temp;
    }
}

/**
 * @brief        启动ATO爬坡函数，用于静止启动时候对ATO进行爬坡，提高启动可靠性
 * @date         2022-07-14
 */
void ATORamp(void)
{
    if (mcState == mcRun)
    {
        if (mcFocCtrl.CtrlMode == 0)
        {
            if (mcFocCtrl.IqRef < IQ_Start_CURRENT) // 启动电流爬坡函数
            {
                mcFocCtrl.IqRef += 30;
            }
            else
            {
                mcFocCtrl.IqRef = IQ_Start_CURRENT;
            }

            FOC_IQREF = mcFocCtrl.IqRef;
        }
#if (Open_Start_Mode == Squ_Start)
        {
            if (mcFocCtrl.State_Count == (ATO_RAMP_PERIOD << 2))
            {
                FOC_EKP = OBSW_KP_GAIN_RUN1; // 估算器里的PI的KP
                FOC_EKI = OBSW_KI_GAIN_RUN1; // 估算器里的PI的KI
            }
            else if (mcFocCtrl.State_Count == ((ATO_RAMP_PERIOD << 1) + ATO_RAMP_PERIOD))
            {
                FOC_EKLPFMIN = OBS_EA_KS2;
                FOC_EKP = OBSW_KP_GAIN_RUN2; // 估算器里的PI的KP
                FOC_EKI = OBSW_KI_GAIN_RUN2; // 估算器里的PI的KI
            }
            else if (mcFocCtrl.State_Count == (ATO_RAMP_PERIOD << 1))
            {
                FOC_EKLPFMIN = OBS_EA_KS;
                FOC_EKP = OBSW_KP_GAIN_RUN3; // 估算器里的PI的KP
                FOC_EKI = OBSW_KI_GAIN_RUN3; // 估算器里的PI的KI
            }
            else if (mcFocCtrl.State_Count <= ATO_RAMP_PERIOD && mcFocCtrl.Flg_ATORampEnd == 0)
            {
                FOC_EKP = OBSW_KP_GAIN_RUN4;  // 估算器里的PI的KP
                FOC_EKI = OBSW_KI_GAIN_RUN4;  // 估算器里的PI的KI
                mcFocCtrl.Flg_ATORampEnd = 1; // ATO 爬坡结束
            }
        }
#else
        {
            if (mcFocCtrl.State_Count == (ATO_RAMP_PERIOD << 2))
            {
                FOC_EKP = OBSW_KP_GAIN_RUN1; // 估算器里的PI的KP
                FOC_EKI = OBSW_KI_GAIN_RUN1; // 估算器里的PI的KI
            }
            else if (mcFocCtrl.State_Count == ((ATO_RAMP_PERIOD << 1) + ATO_RAMP_PERIOD))
            {
                FOC_EKLPFMIN = OBS_EA_KS2;
                FOC_EKP = OBSW_KP_GAIN_RUN2; // 估算器里的PI的KP
                FOC_EKI = OBSW_KI_GAIN_RUN2; // 估算器里的PI的KI
            }
            else if (mcFocCtrl.State_Count == (ATO_RAMP_PERIOD << 1))
            {
                FOC_EKLPFMIN = OBS_EA_KS;
                FOC_EKP = OBSW_KP_GAIN_RUN3; // 估算器里的PI的KP
                FOC_EKI = OBSW_KI_GAIN_RUN3; // 估算器里的PI的KI
            }
            else if (mcFocCtrl.State_Count <= ATO_RAMP_PERIOD && mcFocCtrl.Flg_ATORampEnd == 0)
            {
                FOC_EKP = OBSW_KP_GAIN_RUN4;  // 估算器里的PI的KP
                FOC_EKI = OBSW_KI_GAIN_RUN4;  // 估算器里的PI的KI
                mcFocCtrl.Flg_ATORampEnd = 1; // ATO 爬坡结束
            }
        }
#endif
    }
}
/**
 * @brief        默认1ms周期服务函数，运行信号采样，调速信号处理，闭环控制，故障检测,ATO爬坡函数
 *               该函数运行于大循环中，由SYSTICK定时器间隔1ms触发运行。
 * @date         2022-07-14
 */
void TickCycle_1ms(void)
{
    SetBit(ADC_CR, ADCBSY); // 使能ADC的DCBUS采样

    if ((mcState != mcInit) && (mcState != mcReady))
    {
        /* -----速度滤波----- */
        mcFocCtrl.SpeedFlt = MDU_LPF0(FOC__EOME, mcFocCtrl.SpeedFlt, 50);   /* -----估算器估算的速度值滤波--- 注意低通滤波器系数范围为0---127 */
        mcFocCtrl.EsValue = MDU_LPF0(FOC__EMF << 2, mcFocCtrl.EsValue, 50); /* -----估算器估算的反电动势值滤波--- */

        if (mcState == mcRun)
        {
            mcFocCtrl.Power = FOC__POW << 1;
            /* -----功率滤波----- */
            mcFocCtrl.PowerFlt = LPFFunction(mcFocCtrl.Power, mcFocCtrl.PowerFlt, 30); /* -----功率值滤波----- */
        }
    }
    else
    {
        mcFocCtrl.SpeedFlt = 0;
        mcFocCtrl.PowerFlt = 0;
    }
    /* -----母线电流值采样----- */
    Power_Currt = (ADCCurrentbusPort);

    if (Power_Currt > mcCurOffset.Iw_busOffset)
    {
        Power_Currt = Power_Currt - mcCurOffset.Iw_busOffset;
    }
    else
    {
        Power_Currt = 0;
    }
    mcFocCtrl.mcADCCurrentbus = MDU_LPF0(Power_Currt, mcFocCtrl.mcADCCurrentbus, 32); /* -----母线电流值滤波----- */
#if (VOLTAGE_MODE == INTERNAL)
    {
        mcFocCtrl.mcDcbusFlt = MDU_LPF0(ADC15_DR, mcFocCtrl.mcDcbusFlt, 50); /* -----内部母线电压值滤波，固定为ADC15----- */
    }
#else
    {
        mcFocCtrl.mcDcbusFlt = MDU_LPF0(ADC2_DR, mcFocCtrl.mcDcbusFlt, 50); /* -----外部母线电压值滤波，固定为ADC2----- */
    }
#endif
    mcFocCtrl.NTCTempFlt = MDU_LPF0(ADC3_DR, mcFocCtrl.NTCTempFlt, 50); /* -----NTC电压值滤波----- */
    mcFocCtrl.UqFlt = MDU_LPF0(FOC__UQ, mcFocCtrl.UqFlt, 50);           /* -----Q轴电压值滤波----- */
    mcFocCtrl.UdFlt = MDU_LPF0(FOC__UD, mcFocCtrl.UdFlt, 50);           /* -----D轴电压值滤波----- */

    mcFocCtrl.MCU_TEMP = TSD_Gain(); // 获取芯片内部温度

    /* 获取调速信号，不同调速模式(PWMMODE,NONEMODE,SREFMODE,KEYSCANMODE)的目标值修改 */
    TargetRef_Process();
    /* 故障保护函数功能，如过欠压保护、启动保护、缺相、堵转等 */
    Fault_Detection();
    /* 启动ATO控制，环路响应，如速度环、转矩环、功率环等 */
    Speed_response();
    /* 电机启动ATO爬坡函数处理  */
    ATORamp();
    /* 电机状态机的时序处理 */
    if (mcFocCtrl.State_Count > 0)
    {
        mcFocCtrl.State_Count--;
    }
    /* PWM信号滤波 */
    if (mcPwmInput.MotorOnFilter > 0)
    {
        mcPwmInput.MotorOnFilter--;
    }
    if (mcPwmInput.MotorOffFilter > 0)
    {
        mcPwmInput.MotorOffFilter--;
    }
    /* LIN信号延时发送 */
    if (EOP.TxCnt)
        EOP.TxCnt--;

/* 预定位控制 */
#if (ALIGN_MOME != ALIGN_DSIABLE)
    if (mcState == mcAlign)
    {
        Motor_Align_Angle();
    }
#endif

    /* LIN状态检查 */
    if (LS.State == 1)
    {
        LS.OverTimeCnt++;
    }
    else
    {
        LS.BusLostCnt++;
    }
    if (LS.OverTimeCnt >= LIN_Message_OverTime)
    {
        ClrBit(LIN_CSR, LINEN);
        SetBit(LIN_CSR, LINEN); // 重启更新LIN波特率
        LS.State = 0;
        LS.OverTimeCnt = 0;
        LS.BusLostCnt = 0;
    }
    if (LS.BusLostCnt >= LIN_Bus_OverTime)
    {
        ClrBit(LIN_CSR, LINEN);
        SetBit(LIN_CSR, LINEN); // 重启更新LIN波特率
        LS.State = 0;
        LS.OverTimeCnt = 0;
        LS.BusLostCnt = 0;
    }
    /*-------------*/
}

uint16 SqrtUDQ(int16 sqrtUd, int16 sqrtUq)
{
    SCAT2_COS = sqrtUd;
    SCAT2_SIN = sqrtUq;
    SMDU_RunBlock(2, ATAN);
    return SCAT2_RES1;
}

void SinCal(int16 Ref, int16 Theta, int16 *Sin, int16 *Cos)
{
    SCAT3_COS = Ref;
    SCAT3_SIN = 0;
    SCAT3_THE = Theta;
    SMDU_RunBlock(3, SIN_COS);
    *Cos = SCAT3_RES1;
    *Sin = SCAT3_RES2;
}