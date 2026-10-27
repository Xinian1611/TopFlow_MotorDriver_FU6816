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

uint16 I_Num4;
uint16 I_Num11;
uint16 ticks_1ms;

#if (MOTOR_LOCKED_ROTOR)
static uint16 xdata s_stallTick = 0;     /* 到位堵转判断 10ms 分频 */
#endif

#if MOTOR_LIMIT_SWITCH
static uint16 xdata s_limitTick = 0;     /* 速度模式限位 10ms 分频 */
#endif

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
#if (MOTOR_TEXT_OR_DEBUG == 0)
    /* ===== 0：ONOFFTEST_REF 生效（固定给定测试） =====
     * 不依赖 CAN/PWM/LIN 调速源，直接接管给定；
     * 保留 CAN 急停，方便台架紧急停机。 */
    if (g_run_state.is_estop)
    {
        isCtrlPowOn   = false;
        mcFocCtrl.Ref = 0;
    }
    else
    {
        isCtrlPowOn = (ONOFFTEST_REF != 0);   // 给定 0 就不开机
#if (MOTOR_CTRL_MODE == SPEED_LOOP_CONTROL)
        mcFocCtrl.Ref = ONOFFTEST_REF;        // 固定转速
#elif (MOTOR_CTRL_MODE == POWER_LOOP_CONTROL)
        mcFocCtrl.Ref = ONOFFTEST_PowerREF;   // 固定功率
#elif (MOTOR_CTRL_MODE == CURRENT_LOOP_CONTROL)
        mcFocCtrl.Ref = ONOFFTEST_CurrentREF; // 固定相电流
#endif
    }
#else
    /* ===== 1：ONOFFTEST_REF 不生效，走原来的调速源 ===== */
#if (SPEED_MODE == CANMODE)
    {
        uint8_t target_fr;
        isCtrlPowOn = g_run_state.is_started;

#if (MOTOR_POSITION_CLOSE_LOOP != 0)
        /* 位置模式：到位/限位/堵转/故障时不允许再开机，防止主机持续 START 把它们重新拉起来 */
        if (g_run_state.work_mode == EXT_WORK_MODE_POSITION)
        {
            if ((g_pos_state == POS_IDLE) ||      /* 还没有有效位置目标 */
                (g_pos_state == POS_INPOS) ||
                (g_pos_state == POS_LIMIT) ||
                (g_pos_state == POS_STALL) ||
                (g_pos_state == POS_FAULT))
            {
                isCtrlPowOn     = false;
                mcFocCtrl.Ref   = 0;
                g_run_state.ref = 0.0f;
            }
        }
#endif

        if (g_run_state.is_estop)
        {
            mcFocCtrl.Ref = 0;
            isCtrlPowOn = false;
            g_run_state.is_started = false;
            g_run_state.reversal_braking = 0;  // 急停时退出换向
        }

        // 目标方向
        target_fr = (g_run_state.direction == CW) ? CW : CCW;

        // --- 换向减速处理 ---
        // 原理: 电机正转动时收到反转指令，先减速到0，再切换方向加速到目标值
        if (g_run_state.reversal_braking)
        {
            // 检查退出条件: 方向一致、停机、目标为0、或实际速度已降至接近0
            if (target_fr == mcFocCtrl.FR || !g_run_state.is_started || g_run_state.ref == 0)
            {
                g_run_state.reversal_braking = 0;  // 条件变化，取消换向
            }
            else if (Abs_F16(mcFocCtrl.SpeedFlt) < S_Value(50.0))  // 速度低于50RPM视为停止
            {
                g_run_state.reversal_braking = 0;  // 减速到位，准备切换方向

                // 应用旋转方向: 正数正转(CW), 负数反转(CCW)
                if (g_run_state.direction == CW)
                {
                    mcFocCtrl.FR = CW;       // 正转
                }
                else
                {
                    mcFocCtrl.FR = CCW;      // 反转
                }
            }
        }
        else
        {
            // 正常模式: 检测是否需要启动换向减速
            // 条件: 方向改变、电机正在运行、速度大于停止阈值
            if (target_fr != mcFocCtrl.FR &&
                g_run_state.is_started && !g_run_state.is_estop &&
                g_run_state.ref > 0)
            {
                if (Abs_F16(mcFocCtrl.SpeedFlt) >= S_Value(50.0))
                {
                    g_run_state.reversal_braking = 1;  // 启动换向减速
                    mcState = mcStop;
                }
                else
                {
                    mcFocCtrl.FR = target_fr;          // 低速直接换向，位置小步调节用
                }
            }

            // 应用目标值
            if (g_run_state.work_mode == EXT_WORK_MODE_SPEED)
            {
                mcFocCtrl.Ref = S_Value(g_run_state.ref);
            }
            else if (g_run_state.work_mode == EXT_WORK_MODE_CURRENT)
            {
                mcFocCtrl.Ref = I_Value(g_run_state.ref);
            }
#if (MOTOR_POSITION_CLOSE_LOOP != 0)
            else if (g_run_state.work_mode == EXT_WORK_MODE_POSITION)
            {
                mcFocCtrl.Ref = S_Value(g_run_state.ref);  // 位置环输出的是 rpm 幅值
            }
#endif
        }
    }
#endif
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
        isCtrlPowOn = (ONOFFTEST_REF != 0);       // ★ 给定 0 就不开机
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
#endif /* MOTOR_TEXT_OR_DEBUG */
}

/**
 * @brief        外部闭环控制函数，示例代码提供 电流环，速度环，功率环，UQ控制示例代码,可根据需要自行修改
 *               建议使用默认1ms周期运行
 * @date         2022-07-14
 */
void Speed_response(void)
{
    static int16 refRampOut = 0;
#if (Weak_MagneticEn != Disable)
    static int16 sqrtUdqFlt = 0;     // 电压幅值滤波值，仅弱磁使能时使用
#endif

    if (mcState == mcFault)
    {
        /*  故障态强制撤给定：否则 refRampOut 会保留 3999，
         *   下一轮 mcInit 时与实测量纲混在一起，容易误判"转速为 0" */
        mcFocCtrl.IqRef = 0;
        FOC_IQREF       = 0;
        mcRefRamp.OutValue_float = 0;
    }
    else if ((mcState == mcRun) || (mcState == mcStop))
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

/* ---- 切观测器闭环的最低转速门槛（S 值口径：S_Value(rpm) ）---- */
#define SWITCH_SPEED_S      S_Value(100)

/**
 * @brief  估算转速绝对值（S 值口径，和 mcFocCtrl.SpeedFlt 同一量纲）
 * @note   厂商工程里没有这个函数，是本工程新增的
 */
static int16 AbsSpeedS(void)
{
    int16 n;

    n = mcFocCtrl.SpeedFlt;             /* SpeedFlt 是 S 值，不是 rpm！*/
    return (int16)((n < 0) ? -n : n);
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

                /* ★★ 新增：估算器 PI 已升到 RUN4，此刻把角度从"强制欧米伽"交给估算器。
                *    只清 EFAE，ANGM 保持 1（估算模式）—— TopFlow 在本芯片上验证过的做法：
                *    "绝不是 EFAE=0/RFAE=0/ANGM=0 那种没有角度源的中间态，否则角度会跳变"。*/
                if (AbsSpeedS() >= SWITCH_SPEED_S)      /* 原来写的是 AbsSpeedRpm() >= 100 */
                {
                    FOC__ETHETA = FOC__THETA - 4096;   /* 交接前把估算角摆到当前强制角上
                                                        （原厂约定，见 Motor_Static_Open:584）*/
                    ClrBit(FOC_CR1, EFAE);             /* ★ 真正切到观测器角度 */
                }
                else
                {
                    mcFocCtrl.Flg_ATORampEnd = 0;      /* 这次不切，下个 1ms 再试 */
                }
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

                /*  估算器 PI 已升到 RUN4，此刻把角度从"强制欧米伽"交给估算器。
                *    只清 EFAE，ANGM 保持 1（估算模式）—— TopFlow 在本芯片上验证过的做法：
                *    "绝不是 EFAE=0/RFAE=0/ANGM=0 那种没有角度源的中间态，否则角度会跳变"。*/
                if (AbsSpeedS() >= SWITCH_SPEED_S)      /* 原来写的是 AbsSpeedRpm() >= 100 */
                {
                    FOC__ETHETA = FOC__THETA - 4096;   /* 交接前把估算角摆到当前强制角上
                                                        （原厂约定，见 Motor_Static_Open:584）*/
                    ClrBit(FOC_CR1, EFAE);             /* 真正切到观测器角度 */
                }
                else
                {
                    mcFocCtrl.Flg_ATORampEnd = 0;      /* 这次不切，下个 1ms 再试 */
                }
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
     /*  改动：去掉 (MOE==0)||(FOCEN==0) 这个门 —— 改成每 1ms 无条件触发一次"顺序采样"。
     *   依据《FU6816_66Q1 V1.9》25 章 ADC（PDF 342 / 正文 341）：
     *     顺序采样(结果进 ADCx_DR，软件写 ADCBSY 触发) 与 触发采样(FOC/Timer1 硬件自动，
     *     结果进 FOC 模块、不写 ADCx_DR) 是两条独立通路；且"触发采样优先级比顺序采样优先级高"，
     *     两者同时需要时先做触发采样、完成后自动恢复顺序采样
     *     → 所以插顺序采样不会破坏电流环，被让路的是我们自己。
     *   目的：让 ADC1_DR(抱闸检测) / ADC2_DR(NTC) 在电机运行时也能刷新
     *        （原来这两个寄存器在电机跑起来后是冻结的）。*/
    {
        uint16 adcGuard = 0;

        if (ReadBit(ADC_CR, ADCBSY) == 0)           /* 忙就不重复启动（该位是 rw1）*/
        {
            SetBit(ADC_CR, ADCBSY);
            while (ReadBit(ADC_CR, ADCBSY))
            {
                if (++adcGuard > 60000U) { break; } /* 超时保护，绝不无限等 */
            }
        }
    }
    // SetBit(ADC_CR, ADCBSY); // 使能ADC的DCBUS采样
    // while (ReadBit(ADC_CR, ADCBSY)); // ← 新增：等本次扫描转换完成

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
    // ticks_1ms += 1;
    // if(ticks_1ms == 99)
    // {
    //     I_Num11 = (Power_Currt & 0x7ff4) >> 3;
    //     I_Num4 = (ADC4_DR & 0x7ff4) >> 3;
    //     printf("ADC11 = %d, ADC4 = %d\r\n", I_Num11, I_Num4);
    //     ticks_1ms = 0;
    // }

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
    // mcFocCtrl.NTCTempFlt = MDU_LPF0(ADC2_DR, mcFocCtrl.NTCTempFlt, 50); /* -----NTC电压值滤波（ADC counts）----- */
    /* 改动：NTC 改用 50 点截尾平均（1ms 一点，50ms 出一批；去最大10/最小10，取中间30平均）。
     *   取 ADC2_DR 高 8 位（ADCALIGN=1 → 结果在 [14:3]），<<7 还原成 15 位量纲，
     *   与原来 MDU_LPF0(ADC2_DR,...) 的输出同量纲 → TSD.c 的 Motor_MD() 一个字都不用改。*/
    NTC_TrimPush((uint8)(ADC2_DR >> 7));
    if (NTC_TrimValue() != 0)
    {
        mcFocCtrl.NTCTempFlt = (int16)(NTC_TrimValue() << 7);
    }
    else
    {
        mcFocCtrl.NTCTempFlt = MDU_LPF0(ADC2_DR, mcFocCtrl.NTCTempFlt, 50);  /* 头 50ms 兜底 */
    }
    mcFocCtrl.UqFlt = MDU_LPF0(FOC__UQ, mcFocCtrl.UqFlt, 50);           /* -----Q轴电压值滤波----- */
    mcFocCtrl.UdFlt = MDU_LPF0(FOC__UD, mcFocCtrl.UdFlt, 50);           /* -----D轴电压值滤波----- */

    mcFocCtrl.MCU_TEMP = TSD_Gain(); // 获取芯片内部温度

#if (MOTOR_POSITION_CLOSE_LOOP != 0)
    /* 只有位置模式才运行位置环，避免位置模块在速度模式里清掉启停位 */
    if (g_run_state.work_mode == EXT_WORK_MODE_POSITION)
    {
        PositionCtrl_Update();
    }
#endif

    /* 获取调速信号，不同调速模式(PWMMODE,NONEMODE,SREFMODE,KEYSCANMODE)的目标值修改 */
    TargetRef_Process();
    /* 故障保护函数功能，如过欠压保护、启动保护、缺相、堵转等 */
    Fault_Detection();

#if (MOTOR_LOCKED_ROTOR)
    /* 到位堵转：10ms 一次，只置状态 byte20 并停机，不锁 FaultStall。
     * 主机一键流程看到 GetStall()=1 后会发 CmdStop，下次 CmdSpeed/CmdPosition 会清标志。 */
    if (++s_stallTick >= 10)
    {
        s_stallTick = 0;

        if (g_run_state.is_started &&
            (PositionProtect_StallCheck() == POS_PROTECT_STALL))
        {
            g_motor_stall = 1;
            g_run_state.ref = 0.0f;
            g_run_state.is_started = 0;

#if (MOTOR_FOLD_STATE)
            g_fold_state = (g_run_state.direction == MOTOR_FOLD_STOW_DIR) ? 3 : 4;
#endif
        }
    }
#endif

#if MOTOR_LIMIT_SWITCH
    /* 纯速度模式限位：10ms 一次，到软件 MAX/MIN 且还在往外给速度 → 停 */
    if (++s_limitTick >= 10)
    {
        s_limitTick = 0;

        if (PositionCtrl_SpeedLimitCheck() == POS_PROTECT_LIMIT)
        {
            g_run_state.ref        = 0.0f;   /* 撤速度给定 */
            g_run_state.is_started = 0;      /* 请求 mcStop */
        }
    }
#endif

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

    /*  新增：CAN 任务，必须放在 1ms 任务的最末尾（顺序与初版一致）
     *   CAN_MsgParse()            : 接收环形缓冲 → ext_slave_process_rx()，解析主机指令
     *   ext_slave_periodic_send() : 发送队列限流(每1ms弹1帧) + 心跳/状态/日志周期上报
     *                               + 多帧组装超时 + 运行时间统计 */
    CAN_MsgParse();
    ext_slave_periodic_send();

#if (DEBUG_UART_OR_CAN == 2)
    can_dbg_task();          /* 计时 */
    can_dbg_poll();          /* 发一帧 */
#endif
    AlarmMonitor_Process();       // 告警检测 → 故障码（与初版顺序一致
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
