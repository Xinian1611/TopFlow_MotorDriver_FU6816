/**
 * @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      Customer.h
 * @author    Fortiortech  Appliction Team
 * @note      Last modify author is Kris.huang
 * @since     2019-05-17
 * @date      2022-07-14
 * @brief     This file contains customer parameter used for Motor Control.  
 */
 

/* Define to prevent recursive inclusion -------------------------------------------------------- */
#ifndef __CUSTOMER_H_
#define __CUSTOMER_H_
 /**********************基础参数*****************************
 1. 芯片参数
 2. 电机参数
 3. 电流采样参数
 4. 母线电压采样参数
  **************************************************************/
  
/* ----------------------------------------------------------------------------------------------------------------------------
                                             1.芯片参数                                                 
---------------------------------------------------------------------------------------------------------------------------- */
/* PWM Parameter */
#define PWM_FREQUENCY                  (8.0)                                   ///< (kHz) 载波频率

/*deadtime Parameter*/
#define PWM_DEADTIME                   (1.0)                                    ///< (us) 死区时间

/* single resistor sample Parameter */
#define MIN_WIND_TIME                  (PWM_DEADTIME + 1.4)                     ///< (us) 单电阻最小采样窗口，建议值死区时间+0.9us
/* double resistor sample Parameter */
#define DLL_TIME                       (PWM_DEADTIME + 0.6)                     ///< (us) 双电阻最小脉宽设置, 建议值为死区时间值+0.2us以上

/* ----------------------------------------------------------------------------------------------------------------------------
                                             2.电机参数                                                 
---------------------------------------------------------------------------------------------------------------------------- */
/*电机参数值*/
#define Pole_Pairs                     (7.0)                                    ///<      电机 极对数
#define RS                             (0.019)    //(0.025*0.4)                                    ///< (Ω)  电机 相电阻
#define LD                             (0.000138*0.99)//(0.000100*0.4)								    ///< (H)  电机 相电感
#define LQ                             LD                                     ///< (H)  电机 相电感
#define MOTOR_SPEED_BASE               (4096.0)                                ///< (RPM) 速度基准
// 若选择AO自适应观测器 则无需填写Ke
#define KeVpp                           (7.6832)                                 ///< (V)      反电动势测量的峰峰值
#define KeT                             (100.40)                                 ///< (ms)     反电动势测量的周期
#define Ke                              (Pole_Pairs * KeVpp * KeT / 207.84)     ///< (V/KRPM) 反电动势常数

/* ----------------------------------------------------------------------------------------------------------------------------
                                             3.电流采样参数                                                   
---------------------------------------------------------------------------------------------------------------------------- */
/**
 * 硬件PCBA参数设置根据驱动芯片的类型选择，6866为 HIGH_LEVEL
 * @param (HIGH_LEVEL)   驱动高电平有效
 * @param (LOW_LEVEL)    驱动高电平有效
 * @param (UP_H_DOWN_L)  驱动高电平有效
 * @param (UP_L_DOWN_H)  驱动高电平有效
 */
#define PWM_LEVEL_MODE                  (HIGH_LEVEL)    ///< 驱动电平设置

/* 电机电流采样相关硬件参数 */
/**
 * @breaf  运放模式选择
 * @param (AMP_NOMAL)       外部放大
 * @param (AMP_PGA_DUAL)    内部PGA双端差分输入
 */
#define HW_AMP_MODE                     (AMP_PGA_DUAL)  ///< 运放模式选择       

#define HW_RSHUNT                       (0.001)         ///< (Ω)  采样电阻                               

/**
 * 放大倍数设置
 * @param (AMP2x)       内部PGA放大2倍
 * @param (AMP4x)       内部PGA放大4倍
 * @param (AMP8x)       内部PGA放大8倍
 * @param (AMP16x)      内部PGA放大16倍
 * @param (xxxxxx)      外部放大模式填写相应倍数
 */
#define HW_AMPGAIN                      (AMP8x)         ///< 放大倍数设置                       

/**
 * 参考电压设置
 * @param (VREF3_0)       参考电压设置为3.0V
 * @param (VREF4_0)       参考电压设置为4.0V
 * @param (VREF4_5)       参考电压设置为4.5V
 * @param (VREF5_0)       参考电压设置为5.0V
 */
#define HW_ADC_VREF                    (VREF4_5)        ///< (V)  ADC参考电压

/**
 * 电流采样模式选择
 * @param (Single_Resistor)       单电阻电流采样模式
 * @param (Double_Resistor)       双电阻电流采样模式
 * @param (Three_Resistor)        三电阻电流采样模式
 */
#define Shunt_Resistor_Mode            (Single_Resistor)    ///< 电流采样模式选择

#define SVPWM_COMMOM                   (0xA0)               ///< 常规单电阻采样   
#define SVPWM_OPTIM                    (0xB0)               ///< 优化后新单电阻采样  
#define Single_Resistor_Mode           (SVPWM_COMMOM)        ///< 电流采样模式选择
#define SVPWM_COMMOM_TRGDLY             (0x0B)              ///< 常规单电阻采样点，默认有效矢量的中点进行采样，延时采样=11*T=18*1/24MHZ=458ns，调整电流采样
#define SVPWM_OPTIM_TRGDLY              (0X12)              ///< 优化后新单电阻采样，默认有效矢量的中点进行采样，延时采样=18*T=18*1/24MHZ=750ns，调整电流采样
#define Single_Resistor_Mode_SwitchDuty1   _Q15(0.45)        ///< 切换常规单电阻采样的占空比
#define Single_Resistor_Mode_SwitchDuty2   _Q15(0.30)        ///< 切换新单电阻采样的占空比

/**
 * 基准电压VREF对外输出使能
 * @param (Disable)       禁止
 * @param (Enable)       使能
 */
#define VREF_OUT_EN                     (Enable)         ///< 基准电压VREF对外输出使能

/**
 * 偏置电压设置
 * @param (VHALF1_8)        VHALF电压设置为1/8VREF
 * @param (VHALF1_4)        VHALF电压设置为1/4VREF
 * @param (VHALF25_64)      VHALF电压设置为25/64VREF
 * @param (VHALF1_2)        VHALF电压设置为1/2VREF
 */
#define HW_VHALF_SEL                    (VHALF1_2)      ///< 偏置电压设置

/**
 * VHALF输出使能
 * @param (Disable)       禁止
 * @param (Enable)        使能
 */
#define VHALF_OUT_EN                    (Enable)        ///< VHALF输出使能

/* ----------------------------------------------------------------------------------------------------------------------------
                                             4.母线电压采样参数                                                  
---------------------------------------------------------------------------------------------------------------------------- */
#define RV1                             (1000.0)           ///< (kΩ) 母线电压分压电阻1
#define RV2                             (150.0)            ///< (kΩ) 母线电压分压电阻2

#define VOLTAGE_MODE                   (EXTERNAL)       // 母线电压选择使用分压通道，INTERNAL内部分压 内部分压2/13(默认)  EXTERNAL外部分压

/* ---RV分压比例系数  Ratio_12 ：母线分压系数为1:12，   Ratio_6_5 ：母线分压系数为1:6.5  -----    ((RV1 + RV2 + RV3) / RV3)  */
/**
 * RV分压比例系数
 * @param (Ratio_12)       母线分压系数为1:12    RV: 12
 * @param (Ratio_6_5)      母线分压系数为1:6.5   RV：6.5
 */
#define VOLTAGE_SCALE                  (Ratio_6_5) 

 /**********************电机运行参数*****************************
1.  调速开关模式    
2.  转向设置  
3.  预充电参数与IPMtest  
4.  预定位参数  
5.  顺逆风参数
6.  启动相关参数
7.  环路相关参数 
8.  限流功能参数
9.  弱磁功能参数
10. 其他功能参数
11. PWM调速功能参数
12. 启停功能参数
13. 休眠功能参数
  **************************************************************/
  
 /* ----------------------------------------------------------------------------------------------------------------------------
                                            1.  调速开关模式                                                  
---------------------------------------------------------------------------------------------------------------------------- */
/**
 * 闭环方式选择
 * @param (PWMMODE)       PWM调速
 * @param (SREFMODE)      模拟调速
 * @param (NONEMODE)      直接给定值，不调速
 * @param (ONOFFTEST)     启停测试工具
 * @param (LINMODE)       LIN通信调速
 */
#define SPEED_MODE                     (NONEMODE)           ///< 闭环方式选择      
 /* ----------------------------------------------------------------------------------------------------------------------------
                                            2.  转向设置                                               
---------------------------------------------------------------------------------------------------------------------------- */
/**
 * 转向设置
 * @param (CW)        顺时针
 * @param (CCW)       逆时针
 */
#define FR_MODE                        (CW)                 ///< 转向设置   
/* ----------------------------------------------------------------------------------------------------------------------------
                                            3.  预充电参数与IPMtest                                                  
---------------------------------------------------------------------------------------------------------------------------- */
/**
 * 预充电使能
 * @param (Disable)       禁止
 * @param (Enable)        使能
 */
#define CHARGE_EN                      (Enable)         ///< 预充电使能
#define CHARGE_DUTY                    (0.1)            ///< (%)  预充电下桥Duty
#define CHARGE_TIME                    (60)             ///< (ms) 预充电时间 

/**
 * IPM测试模式，用以检测MCU——MOS间电路是否正常。需要开启预充电使能,
 * @param (Disable)       禁止
 * @param (Enable)        使能
 */
 #define IPMTEST                       (Disable)

/* 正常运行时估算算法的参数设置值  */
#define OBS_KSLIDE                     _Q15(0.85)       ///< SMO算法里的滑膜增益值
#define E_BW                           (2500.0)          ///< PLL算法里的反电动势滤波值

/* ----------------------------------------------------------------------------------------------------------------------------
                                            4.  预定位参数                                                  
---------------------------------------------------------------------------------------------------------------------------- */
/**
 * 预定位模式选择
 * @param (ALIGN_DSIABLE)       禁止
 * @param (ALIGN_NOMAL)         正常预定位
 * @param (ALIGN_TEST)          测试模式，可用于手动测试电机极对数
 */
#define ALIGN_MOME                     (ALIGN_NOMAL)  ///< 预定位模式选择

#define Align_Angle_CW                 (-18.0)          ///< (°) 预定位角度
#define Align_Angle_CCW                (13.3)          ///< (°) 预定位角度
#define Align_Angle_End                (-2.35)          ///< (°) 预定位角度
#define Align_Time                     (20)              ///< (ms) 预定位时间 

/* 预定位的Kp、Ki */
#define DQKP_Alignment                 _Q12(0.5)        ///< 预定位的KP
#define DQKI_Alignment                 _Q15(0.2)        ///< 预定位的KI
#define ID_Align_CURRENT               I_Value(16.8)    ///< (A) Q轴运行电流 
#define UD_Align_Duty                  _Q15(0.10)       ///< (A) D轴定位电压
#define UD_Align_Duty_Voltage          UDC_Value(12.0)  ///< (A) 标准运行电压
#define UD_Align_Duty_Max              _Q15(0.127)      ///< (A) D轴定位电压最大值
#define UD_Align_Duty_Max_Voltage      UDC_Value(9.0)   ///< (A) 最小运行电压，对应D轴定位电压最大值
#define UD_Align_Duty_Min              _Q15(0.09)       ///< (A) D轴定位电压最小值
#define UD_Align_Duty_Min_Voltage      UDC_Value(18.0)  ///< (A) 最大运行电压，对应D轴定位电压最小值
#define UQ_Align_Duty                  _Q15(0.00)       ///< (A) Q轴定位电压

/* ----------------------------------------------------------------------------------------------------------------------------
                                            5.  顺逆风参数                                                  
---------------------------------------------------------------------------------------------------------------------------- */
/**
 * 顺逆风检测方式
 * @param (NoTailWind)      无逆风顺风判断
 * @param (RSDMethod)       RSD比较器方法
 * @param (BEMFMethod)      BEMF比较器方法
 */
#define TAILWIND_MODE                 (NoTailWind)     ///< 顺逆风检测方式
/**
 * 顺逆风引脚选择
 * @param (PIN_131415)     端口连接P1.3,P1.4,P1.5     
 * @param (PIN_141621)     端口连接P1.4,P1.6,P2.1 
 */
#define TAILWIND_PIN                   (PIN_131415)       ///< BEMFMethod可选择PIN_131415或PIN141621，RSDMethod只能选择PIN_131415
#define TAILWIND_TIME                  (1000)            ///< (ms) 顺逆风检测时间 
#define ATO_BW_WIND                    (200)            ///< 逆风判断观测器带宽的滤波值，经典值为8.0-100.0 
#define SPD_BW_WIND                    (20.0)           ///< 逆风判断速度带宽的滤波值，经典值为5.0-40.0 
#define MOTOR_SPEED_SMOMIN_RPM_WIND    (2000.0)         ///< (RPM) SMO顺风运行最小转速影响启动

/* ----------------------------------------------------------------------------------------------------------------------------
                                            6.  启动相关参数                                                  
---------------------------------------------------------------------------------------------------------------------------- */
/**
 * 开环启动模式选择
 * @param (Open_Start)          开环强拖启动
 * @param (Omega_Start)         Omega启动
 * @param (Open_Omega_Start)    Omega+强拖启动
 * @param (Squ_Start)           六步启动
 */
#define Open_Start_Mode                (Omega_Start)    ///< 开环启动模式选择

/* @brief 6步启动参数参数设置 */
#define SquStartThetaOffset        		    _Q15(10.0/180.0)		         		//单电阻填1-29度（度数越大，采样窗口越大，但角度偏离需要运行的角度），双电阻填0度
#define SquStartSector			        			(0)															//必须与下方角度一起修改，每加1下面角度加60度
#define SquStartTheta			        				_Q15(90.0/180.0)
#define SquStartCurrent			       				I_Value(15.0)			        			//启动电流
#define StartCurrentTime									(uint16)(15*PWM_FREQUENCY/2)		// ms	启动电流闭环时间
#define MinUqSet													_Q11(0.005)			    	        	//最小占空比，最大不超过0.125
#define DeadTimeComCount            	    _Q15(1.5*PWM_DEADTIME/(1000.0/PWM_FREQUENCY))
#define CW_THETA_BASE				   						_Q15(-20.0 / 180.0)
#define StartCurrentAddThreshold            I_Value(1.6)
#define RealFOCSwitchSpeed            	 (150.0)
#define FOCSwitchSpeed				      		 S_Value(RealFOCSwitchSpeed * Pole_Pairs)		    //切入转速  (以1对极为标准)
#define ATO_BW_Switch                    (1.0)
#define THETARAMP                        (300)
#define Start_Single_Resistor_Mode       (SVPWM_OPTIM)


/* 启动电流 */
#define ID_Start_CURRENT               I_Value(0.0)     ///< (A) D轴启动电流
#define IQ_Start_CURRENT               I_Value(10.0)     ///< (A) Q轴启动电流
/* 运行电流 */
#define ID_RUN_CURRENT                 I_Value(0.0)     ///< (A) D轴运行电流
#define IQ_RUN_CURRENT                 I_Value(0.0)     ///< (A) Q轴运行电流

/* 启动ATO参数 */
#define ATO_BW_START                   (0.0)            ///< 观测器带宽的滤波值，经典值为1.0-200.0
#define ATO_BW_RUN1                    (10.0)
#define ATO_BW_RUN2                    (20.0)
#define ATO_BW_RUN3                    (30.0)
#define ATO_BW_RUN4                    (50.0)

/* 启动ATO爬坡时间控制 */
#define ATO_START_HOLDTIME             (300)             ///< (ms)观测器带宽的滤波值,启动第一拍ATO持续时间
#define ATO_RAMP_PERIOD                (20)             ///< (ms)观测器带宽的滤波值,ATO爬坡递增 间隔时间

/* OMEGA启动参数 */
#define MOTOR_OMEGA_RAMP_ACC           (1)              ///< omega启动的增量（每个载波递增值） 
#define MOTOR_OMEGA_RAMP_MIN           S_Value(80.0)     ///< (RPM) omega启动的最小切换转速
#define MOTOR_OMEGA_RAMP_END           S_Value(130.0)     ///< (RPM) omega启动的限制转速

#define MOTOR_SPEED_SMOMIN_RPM         (50.0)           ///< (RPM) SMO运行最小转速影响启动

#define MOTOR_LOOP_RPM                 S_Value(180.0)    ///< (RPM) 启动后电流环切入外部环路转速阈值
#define SPD_BW                         (5.0)           ///< 速度带宽的滤波值，经典值为5.0-40.0
/* -----------电流环参数设置值-------------------------------------------------------------- */
#define DKP                            _Q12(2.50)        ///< 运行DQ轴电流环KP
#define DKI                            _Q15(0.21)       ///< 运行DQ轴电流环KI
#define QKP                            _Q12(2.50)        ///< 运行DQ轴电流环KP
#define QKI                            _Q15(0.21)       ///< 运行DQ轴电流环KI
/* D轴参数设置 */
#define DOUTMAX                        _Q15(0.9)        ///< D轴电压最大限幅值，单位：输出占空比
#define DOUTMIN                        _Q15(-0.9)       ///< D轴电压最小限幅值，单位：输出占空比
/* Q轴参数设置 */
#define QOUTMAX                        _Q15(0.99)       ///< Q轴电压最大限幅值，单位：输出占空比
#define QOUTMIN                        _Q15(-0.99)      ///< Q轴电压最小限幅值，单位：输出占空比

/* ----------------------------------------------------------------------------------------------------------------------------
                                            7.  环路相关参数                                                  
---------------------------------------------------------------------------------------------------------------------------- */
/**
 * 闭环方式选择
 * @param (CURRENT_LOOP_CONTROL)       电流环
 * @param (SPEED_LOOP_CONTROL)         速度环
 * @param (POWER_LOOP_CONTROL)         功率环
 * @param (UQ_LOOP_CONTROL)            UQ环
 */
#define MOTOR_CTRL_MODE                (SPEED_LOOP_CONTROL) ///< 闭环方式选择

#define LOOP_TIME                       (1)                 ///< (ms) 外环环调节周期,默认为 1m

#define MOTOR_SPEED_MIN_RPM            S_Value(100.0)        ///< (RPM) 运行最小转速
#define MOTOR_SPEED_MAX_RPM            S_Value(1900.0)       ///< (RPM) 运行最大转速
#define MOTOR_SPEED_STOP_RPM           S_Value(100.0)       ///< (RPM) 运行最小转速  

#define SKP                            _Q12(0.6)            ///< 外环KP     
#define SKI                            _Q15(0.004)           ///< 外环KI
#define SKD                            _Q12(0.0)            ///< 外环KD

#define SOUTMAX                        I_Value(75.0)        ///< (A) 外环电流最大限幅值
#define SOUTMIN                        I_Value(1.5)        ///< (A) 外环电流最小限幅值

/* -----环路爬坡增量----- */
#if (MOTOR_CTRL_MODE == CURRENT_LOOP_CONTROL) 
#define RAMP_INC                       I_Value(0.1)      ///< (A) 电流环增量
#define RAMP_DEC                       I_Value(0.1)      ///< (A) 电流环减量
#else 
#define RAMP_INC                       SAcc_Value(200)      ///< (RPS) 功率环/速度环每秒爬坡递增量
#define RAMP_DEC                       SAcc_Value(200)      ///< (RPS) 功率环/速度环每秒爬坡递减量
#endif

/* ----------------------------------------------------------------------------------------------------------------------------
                                            8.  限流功能参数                                                  
---------------------------------------------------------------------------------------------------------------------------- */
/**
 * 母线电流限流使能
 * @param (Disable)       禁止
 * @param (Enable)        使能
 */
#define ADCCurrentbusLitmitEn         (Disable)               // 限流功能使能
#define ADCCurrentbusPort              ADC11_DR              ///<(A) 母线电流采样端口，需使能相应ADC通道 
#define Motor_Max_ADCCurrentbus        I_Bus_Value(50.0)     ///< (A) 母线电流限流值
#define I_LimitiBus_K                  (1.0)               ///< (A) 系数K，调整限流值
#define LimitCurrent_KP                _Q12(2.0)             ///< 限流环KP     
#define LimitCurrent_KI                _Q15(0.02)            ///< 限流环KI

/* ----------------------------------------------------------------------------------------------------------------------------
                                            9.  弱磁功能参数                                                  
---------------------------------------------------------------------------------------------------------------------------- */
/**
 * 弱磁使能
 * @param (Disable)       禁止
 * @param (Enable)        使能
 */
#define Weak_MagneticEn                (Disable)             // 弱磁功能使能
#define AKP                            _Q12(1.0)             // 弱磁KP
#define AKI                            _Q15(0.01)            // 弱磁KI
#define AKD                            _Q12(0.00)            // 弱磁KD
#define AMAX                           (0.00)                // (°)角度
#define AMIN                           (-25.0)               // (°)角度
/* ----------------------------------------------------------------------------------------------------------------------------
                                            10.  其他功能参数                                                  
---------------------------------------------------------------------------------------------------------------------------- */
/**
 * 估算器模式选择
 * @param (SMO)       滑膜估算
 * @param (PLL)       锁相环
 * @param (AO)        自适应
 */
#define EstimateAlgorithm              (AO)                 ///< 估算器模式选择

/**
 * 过调制
 * @param (Disable)      禁止
 * @param (Enable)       使能
 */
#define OverModulation                 (Disable)             ///< 开启过调制UD,UQ会被放大1.15倍，但极限状态可能导致电流畸变                                                           

/**
 * 初始位置检测
 * @param (Disable)      禁止
 * @param (Enable)       使能
 */
#define PosCheckEnable                 (Disable)            ///< 初始位置检测  

/* ----------------------------------------------------------------------------------------------------------------------------
                                            11.  PWM调速功能参数                                                  
---------------------------------------------------------------------------------------------------------------------------- */
/**
 * PWM调速 PWM极性选择
 * @param (PosiPWMDUTY)       正逻辑
 * @param (NegaPWMDUTY)       反逻辑
 */
#define PWMDUTY_POLARITY               (PosiPWMDUTY)        ///< PWM调速 PWM极性选择 

/* -----PWM开关机滤波设置----- */
#define MotorOnFilterTime              (20)                 ///< (ms) 开机滤波值       
#define MotorOffFilterTime             (20)                 ///< (ms) 关机滤波值

/* 开关机Duty设置 */
#define OFFPWMDuty                     _Q15(0.07)           ///< 关机PWM占空比，小于该占空比关机
#define OFFPWMDutyHigh                 _Q15(0.9)            ///< 关机PWM占空比，大于该占空比关机
#define ONPWMDuty                      _Q15(0.08)           ///< 开机PWM占空比，大于该占空比时开机
#define MINPWMDuty                     _Q15(0.16)           ///< 速度曲线上最小PWM占空比
#define MAXPWMDuty                     _Q15(0.84)           ///< 速度曲线上最大PWM占空比

/* ----------------------------------------------------------------------------------------------------------------------------
                                            12.  启停功能参数                                                  
---------------------------------------------------------------------------------------------------------------------------- */

/* -----启停测试参数配置----- */
#define ONOFFTEST_REF                  S_Value(0)        ///< (RPM)速度环测试给定，注意闭环方式不同给定值需要更改
#define ONOFFTEST_CurrentREF           I_Value(10.0)        ///< (A)电流环测试给定，注意闭环方式不同给定值需要更改
#define ONOFFTEST_PowerREF             (2000)              ///< (W)功率环测试给定，注意闭环方式不同给定值需要更改
#define ONOFFTEST_ON_TIME              (6000)               ///< (ms) 启动运行时间
#define ONOFFTEST_OFF_TIME             (5000)               ///< (ms) 停止时间

/* -----停机刹车选择----- */
#define StopBrakeFlag                  (1)
#define StopWaitTime                   (4000)                ///< (ms) 刹车等待时间
#define Stop_MOTOR_SPEED_RPM           S_Value(3000)        ///< (RPM) 小于该转速刹车

/* ----------------------------------------------------------------------------------------------------------------------------
                                            13.   休眠功能参数                                                  
---------------------------------------------------------------------------------------------------------------------------- */
/**
 * 休眠设置
 * @param (Disable)       禁止
 * @param (Enable)        使能
 */
#define SLEEPEN                        (Disable)                  ///< 休眠设置
/**
 * LIN收发器选择
 * @param (EXTERNAL_LIN_DOUBLELINE)     外置LIN收发器，双线模式
 * @param (INNER_LIN_SINGLELINE)        内置LIN收发器，单线模式
 */
#define LIN_TRANSCEIVER             (INNER_LIN_SINGLELINE)       ///< 注意唤醒口，根据LIN收发器决定唤醒口

/**
 * LIN双线端口选择
 * @param (TXDGP00_RXDGP01)     
 * @param (TXDGP10_RXDGP11)     
 */    
#define LIN_PORTSELECTION             (TXDGP00_RXDGP01)          ///< 选双线模式有效
#endif

