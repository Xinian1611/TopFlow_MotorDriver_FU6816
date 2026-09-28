/**
 * @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      definition.h
 * @author    Kris.huang
 * @note      Last modify author is Kris.huang
 * @since     2022-07-14
 * @date      2022-07-14
 * @brief
 */


#ifndef __DEFINITION_H_
#define __DEFINITION_H_


/* Q format define ---------------------------------------------------------------------------------*/
#define _Q7(A)                          (int8) ((A) * 0x7f)         ///< Q7 format
#define _Q8(A)                          (int16)((A) * 0xff)         ///< Q8 format
#define _Q9(A)                          (int16)((A) * 0x1ff)        ///< Q9 format
#define _Q10(A)                         (int16)((A) * 0x3ff)        ///< Q10 format
#define _Q11(A)                         (int16)((A) * 0x07ff)       ///< Q11 format
#define _Q12(A)                         (int16)((A) * 0x0fff)       ///< Q12 format
#define _Q13(A)                         (int16)((A) * 0x1fff)       ///< Q13 format
#define _Q14(A)                         (int16)((A) * 0x3fff)       ///< Q14 format
#define _Q15(A)                         (int16)((A) * 0x7fff)       ///< Q15 format
#define _Q16                            (65535.0)                   ///< Q16 format value
#define _2PI                            (3.1415926 * 2)             ///< 2 PI value
//I_Value(1.0)= 145.63
#define I_ValueX(Curr_Value)            (Curr_Value * HW_RSHUNT * HW_AMPGAIN / (HW_ADC_REF))
#define I_Value(Curr_Value)             _Q15(I_ValueX(Curr_Value))
#define I_Bus_ValueX(Curr_Value)            (I_LimitiBus_K*Curr_Value * HW_RSHUNT * HW_AMPGAIN / (HW_ADC_REF))
#define I_Bus_Value(Curr_Value)             _Q15(I_Bus_ValueX(Curr_Value))
#define S_Value(SpeedValue)             _Q15((float)(SpeedValue) / MOTOR_SPEED_BASE)
#define SAcc_Value(SpeedValue)          ((float)32767.0*(SpeedValue*LOOP_TIME/1000.0) / MOTOR_SPEED_BASE)
#define A_Value(Angle)                  _Q15((float)(Angle) / 180.0)

#define UDC_Value(val)                  _Q15((val)/ HW_BOARD_VOLT_MAX)

#define TIM2_Fre                        (187500.0)      ///< 定时器频率


/* 使能定义 */
#define Disable                         (0)
#define Enable                          (1)

/* 转向定义 */
#define CW                              (0)             ///<正转
#define CCW                             (1)             ///<反转

/* ADC参考电压配置 */
#define VREF3_0                         (0xA0)          ///< 0XA0 远离实际值  减少直接填写数值导致错误，其他同理
#define VREF4_0                         (0xB0)
#define VREF4_5                         (0xC0)
#define VREF5_0                         (0xD0)

/* VHALF系数 */
#define VHALF1_8                        (0xA0)          ///< 0XA0 远离实际值  减少直接填写数值导致错误，其他同理
#define VHALF1_4                        (0xB0)
#define VHALF25_64                      (0xC0)
#define VHALF1_2                        (0xD0)

/* 运放模式选择 */
#define AMP_NOMAL                       (0xA0)          ///< 外部放大
#define AMP_PGA_DUAL                    (0xB0)          ///< 内部PGA 双端差分输入

/* 内部PGA放大倍数选择 */
#define AMP2x                           (2)
#define AMP4x                           (4)
#define AMP8x                           (8)
#define AMP16x                          (16)

/* 内部PGA放大倍数选择 */
#define INTERNAL                        (0xA0)
#define EXTERNAL                        (0xB0)

#define Ratio_12                        (0xA0)
#define Ratio_6_5                       (0xB0)

/* --驱动电平设置-- */
#define HIGH_LEVEL                      (0xA0)          ///< 驱动高电平有效
#define LOW_LEVEL                       (0xB0)          ///< 驱动低电平有效
#define UP_H_DOWN_L                     (0xC0)          ///< 上桥臂高电平有效，下桥臂低电平有效
#define UP_L_DOWN_H                     (0xD0)          ///< 上桥臂低电平有效，下桥臂高电平有效

/***正PWMduty or 负PWMduty Choose***/
#define PosiPWMDUTY                     (0xA0)          // 正PWMduty
#define NegaPWMDUTY                     (0xB0)          // 负PWMduty

#define SPEED_K                         ((float)(MOTOR_SPEED_MAX_RPM-MOTOR_SPEED_MIN_RPM)/(float)(MAXPWMDuty-MINPWMDuty))

/* SVPWM mode */
#define SVPWM_5_Segment                 (0xA0)          ///< 五段式SVPWM
#define SVPWM_7_Segment                 (0xB0)          ///< 七段式SVPWM

/* double resistor sample mode */
#define DouRes_1_Cycle                  (0xA0)          ///< 1 周期采样完 ia, ib
#define DouRes_2_Cycle                  (0xB0)          ///< 交替采用ia, ib, 2周期采样完成


/* 闭环模式 */
#define CURRENT_LOOP_CONTROL            (0xA0)          ///< 电流环
#define POWER_LOOP_CONTROL              (0xB0)          ///< 恒功率
#define SPEED_LOOP_CONTROL              (0xC0)          ///< 恒转速
#define UQ_LOOP_CONTROL                 (0xD0)          ///< UQ闭环
#define OTHERS_LOOP_CONTROL             (0xE0)          ///< 其他环路，自定义




#define ALIGN_DSIABLE                   (0xA0)          ///< 禁止 
#define ALIGN_NOMAL                     (0xB0)          ///< 正常预定位 
#define ALIGN_TEST                      (0xC0)          ///< 测试模式，可用于手动测试电机极对数 


/* 调速模式 */
#define CANMODE                         (0x90)          ///< CAN通信调速
#define NONEMODE                        (0xA0)          ///< 直接给定值，不调速
#define PWMMODE                         (0xB0)          ///< PWM调速
#define SREFMODE                        (0xC0)          ///< 模拟调速
#define ONOFFTEST                       (0xD0)          ///< 启停测试工具                                                        
#define OTHERS                          (0xE0)          ///< 其他调速方式   
#define LINMODE                         (0xF0)          ///< LIN通信调速

/* 估算器模式选择 */
#define SMO                             (0xA0)          ///< SMO ,滑膜估算
#define PLL                             (0xB0)          ///< PLL ,锁相环
#define AO                              (0xC0)          ///< AO，自适应估算器

/* 顺逆风判断设置 */
#define NoTailWind                      (0xA0)          ///< 无逆风顺风判断
#define RSDMethod                       (0xB0)          ///< RSD比较器方法
#define BEMFMethod                      (0xD0)          ///< BEMF比较器方法
/* 顺逆风引脚选择 */
#define PIN_131415                      (0xA0)          ///< P13,P14,P15
#define PIN_141621                      (0xB0)          ///< P14,P16,P21


/* 启动模式 */
#define STATIC_START                    (0xA0)          ///< 静止启动
#define TAILWIND_START                  (0xB0)          ///< 顺风启动
#define HEADWIND_START                  (0xC0)          ///< 逆风启动
#define TAILWIND_DECT                   (0xD0)          ///< 顺逆风检测中


/* 静止启动 强拖模式选择 */
#define Open_Start                      (0xA0)          ///< 开环强拖启动
#define Omega_Start                     (0xB0)          ///< Omega启动
#define Open_Omega_Start                (0xC0)          ///< 先开环启，后Omega启动
#define Squ_Start				                (0xD0)          ///< 免调试启动


/* 电流采样模式 */
#define Single_Resistor                 (0xA0)          ///< 单电阻电流采样模式
#define Double_Resistor                 (0xB0)          ///< 双电阻电流采样模式
#define Three_Resistor                  (0xC0)          ///< 三电阻电流采样模式


#define Long_Inject                     (0)             ///< 脉冲注入时间长于2ms,若时间长于4ms，则要修改定时器分频
#define Short_Inject                    (1)             ///< 脉冲注入时间低于2ms

/* LIN收发器选择*/       
#define EXTERNAL_LIN_DOUBLELINE          (0xA0)         ///< 外置LIN收发器，双线模式
#define INNER_LIN_SINGLELINE             (0xB0)         ///< 内置LIN收发器，单线模式

/* LIN双线端口选择*/       
#define TXDGP00_RXDGP01                 (0xA0)         ///< TXDGP00_RXDGP01
#define TXDGP10_RXDGP11                 (0xB0)         ///< TXDGP10_RXDGP11

#endif
