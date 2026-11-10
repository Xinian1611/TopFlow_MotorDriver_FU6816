/**
 * @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      Parameter.h
 * @author    Fortiortech  Appliction Team
 * @note      Last modify author is Kris.huang
 * @since     2021-04-11
 * @date      2022-07-14
 * @brief     This file contains all FOC debug  parameter used for Motor Control.
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __Parameter_H_
#define __Parameter_H_
#include <definition.h>
/* Define to prevent recursive inclusion -------------------------------------*/


/* Private define ------------------------------------------------------------*/
#define SystemPowerUpTime               (15000)                                                       // 上电等待时间，cpu计数时钟

/*芯片参数值------------------------------------------------------------------*/
/*CPU and PWM Parameter*/

/*CPU Parameter*/
#define MCU_CLOCK                       (24.0)                                                        ///< (MHz) 主频

#define PWM_CYCLE                       (1000.0 / PWM_FREQUENCY)                                      ///< 周期us
#define SAMP_FREQ                       (PWM_FREQUENCY * 1000)                                        ///< 采样频率(HZ)
#define TPWM_VALUE                      (1.0 / SAMP_FREQ)                                             ///< 载波周期(S)
#define PWM_VALUE_LOAD                  (uint16)(MCU_CLOCK * 1000 *2 / PWM_FREQUENCY)                 ///< PWM 定时器重载值 20190514修改




/*three resistor overmodule Parameter*/
#define OVERMOD_TIME                    (1.0)                                                         ///< 三电阻过调制时间(us)，建议值2.0
/*deadtime compensation*/
#define DT_TIME                         (0.0)                                                         ///< 死区补偿时间(us)，适用于双电阻和三电阻，建议值是1/2死区时间
/*min pulse*/
#define GLI_TIME                        (0.0)                                                         ///< 桥臂窄脉宽消除(us),建议值0.5

/*deadtime Parameter*/
#define PWM_LOAD_DEADTIME               (PWM_DEADTIME * MCU_CLOCK)                                    ///< 死区设置值
#define PWM_OVERMODULE_TIME             (OVERMOD_TIME * MCU_CLOCK / 2)                                ///< 过调制时间
#define PWM_DLOWL_TIME                  (DLL_TIME * MCU_CLOCK / 2)                                    ///<下桥臂最小时间
/*single resistor sample Parameter*/
#define PWM_TS_LOAD                     (uint16)(_Q16 / PWM_CYCLE * MIN_WIND_TIME / 16)               ///< 单电阻采样设置值
#define PWM_TS_LOAD1                    (uint16)(_Q16 / PWM_CYCLE * (MIN_WIND_TIME + 0.2) / 16)               ///< 单电阻采样设置值
#define PWM_DT_LOAD                     (uint16)(_Q16 / PWM_CYCLE * DT_TIME / 16)                     ///< 死区补偿值
#define PWM_TGLI_LOAD                   (uint16)(_Q16 / PWM_CYCLE * (GLI_TIME + PWM_DEADTIME) / 16)   ///< 最小脉冲

/*硬件板子参数设置值------------------------------------------------------------------*/
/*hardware current sample Parameter*/

#define Calib_Time                     (1000)           ///< 校正次数，固定1000次，单位:次
#if (HW_ADC_VREF==VREF5_0)
    #define HW_ADC_REF                     (5.0)                                    ///< (V)  ADC参考电压
#elif (HW_ADC_VREF==VREF4_5)
    #define HW_ADC_REF                     (4.5)                                    ///< (V)  ADC参考电压
#elif (HW_ADC_VREF==VREF4_0)
    #define HW_ADC_REF                     (4.0)                                    ///< (V)  ADC参考电压
#elif (HW_ADC_VREF==VREF3_0)
    #define HW_ADC_REF                     (3.0)                                    ///< (V)  ADC参考电压
#endif

#if(VOLTAGE_MODE ==INTERNAL)
    #if(VOLTAGE_SCALE == Ratio_6_5)
        #define RV                          (6.5)                               ///< 分压比
    #else
        #define RV                          (12.0)                               ///< 分压比
    #endif
#else
    #define RV                          ((RV1 + RV2 ) / RV2)                     ///< 分压比
#endif

/*电流基准的电路参数*/
#define HW_BOARD_CURR_MAX               (HW_ADC_REF / 2 / HW_AMPGAIN / HW_RSHUNT)                     ///< 最大采样电流
#define HW_BOARD_CURR_MIN               (-HW_BOARD_CURR_MAX)                                          ///< 最小采样电流
#define HW_BOARD_CURR_BASE              (HW_BOARD_CURR_MAX * 2)                                       ///< 电流基准

/*hardware voltage sample Parameter*/
/*母线电压采样分压电路参数*/
#define HW_BOARD_VOLT_MAX               (HW_ADC_REF * RV)                           ///< (V)  ADC可测得的最大母线电压
#define HW_BOARD_VOLTAGE_BASE           (HW_BOARD_VOLT_MAX / 1.732)                 ///< 电压基准

/*硬件过流保护DAC值*/
#define DAC_OverCurrentValue            _Q8(I_ValueX(HWOCValue)) + 0x7F             ///< + 0X7F为有偏置电压

#define Align_Theta_CW                  _Q15((float)Align_Angle_CW / 180.0)
#define Align_Theta_CCW                 _Q15((float)Align_Angle_CCW / 180.0)
#define Align_Theta_End                 _Q15((float)Align_Angle_End / 180.0)

/* -----过温保护值设置----- */
#define   Tempera_Value(NTC_Value) 		_Q15((5.0*NTC_Value/(10.0+NTC_Value))/HW_ADC_REF)									    ///< 10K上拉电阻时，NTC阻值对应Q15_AD值，单位：KΩ
/* obsever parameter set value */
#define BASE_FREQ                       ((MOTOR_SPEED_BASE / 60.0) * Pole_Pairs)                        ///< 基准频率

#define MAX_BEMF_VOLTAGE                ((MOTOR_SPEED_BASE*Ke)/(1000.0))
#define MAX_OMEG_RAD_SEC                ((float)(_2PI*BASE_FREQ))

#define ATT_COEF                       (0.85)                                  ///< 无需改动


//自适应估算器模式使能
#if (EstimateAlgorithm == AO)
    #define OBS_K1T                     _Q11((1.0*3.0/(125.0*1.0))*(LQ/TPWM_VALUE)*(HW_BOARD_CURR_BASE/HW_BOARD_VOLTAGE_BASE))
    #define OBS_K2T                     _Q11(0.8*RS*HW_BOARD_CURR_BASE/HW_BOARD_VOLTAGE_BASE)
    #define OBS_K3T                     _Q8(2.5)
    #define OBS_K4T                     _Q15(((LD-LQ)*TPWM_VALUE*MAX_OMEG_RAD_SEC)/(LD+RS*TPWM_VALUE))
#else
    #define OBS_K1T                     _Q15(LD/(LD+RS*TPWM_VALUE))
    #define OBS_K2T                     _Q13(1.4*(TPWM_VALUE/(LD+RS*TPWM_VALUE))*(HW_BOARD_VOLTAGE_BASE/HW_BOARD_CURR_BASE))
    #define OBS_K3T                     _Q15((TPWM_VALUE/(LD+RS*TPWM_VALUE))*(MAX_BEMF_VOLTAGE/HW_BOARD_CURR_BASE))
    #define OBS_K4T                     _Q15(((LD-LQ)*TPWM_VALUE*MAX_OMEG_RAD_SEC)/(LD+RS*TPWM_VALUE))
#endif


#define OBSW_KP_GAIN_START              _Q12(2*_2PI*ATT_COEF*ATO_BW_START/BASE_FREQ)
#define OBSW_KI_GAIN_START              _Q15(_2PI*ATO_BW_START*ATO_BW_START*TPWM_VALUE/BASE_FREQ)

#define OBSW_KP_GAIN_RUN1               _Q12(2*_2PI*ATT_COEF*ATO_BW_RUN1/BASE_FREQ)
#define OBSW_KI_GAIN_RUN1               _Q15(_2PI*ATO_BW_RUN1*ATO_BW_RUN1*TPWM_VALUE/BASE_FREQ)

#define OBSW_KP_GAIN_RUN2               _Q12(2*_2PI*ATT_COEF*ATO_BW_RUN2/BASE_FREQ)
#define OBSW_KI_GAIN_RUN2               _Q15(_2PI*ATO_BW_RUN2*ATO_BW_RUN2*TPWM_VALUE/BASE_FREQ)

#define OBSW_KP_GAIN_RUN3               _Q12(2*_2PI*ATT_COEF*ATO_BW_RUN3/BASE_FREQ)
#define OBSW_KI_GAIN_RUN3               _Q15(_2PI*ATO_BW_RUN3*ATO_BW_RUN3*TPWM_VALUE/BASE_FREQ)

#define OBSW_KP_GAIN_RUN4               _Q12(2*_2PI*ATT_COEF*ATO_BW_RUN4/BASE_FREQ)
#define OBSW_KI_GAIN_RUN4               _Q15(_2PI*ATO_BW_RUN3*ATO_BW_RUN4*TPWM_VALUE/BASE_FREQ)

#define OBSW_KP_GAIN_Switch             _Q12(2*_2PI*ATT_COEF*ATO_BW_Switch/BASE_FREQ)
#define OBSW_KI_GAIN_Switch             _Q15(_2PI*ATO_BW_Switch*ATO_BW_Switch*TPWM_VALUE/BASE_FREQ)

#define OBS_FBASE                       _Q15(BASE_FREQ*TPWM_VALUE)                                    ///< Fbase*Tpwm*32768
#define OBS_KLPF                        _Q15(_2PI*BASE_FREQ*TPWM_VALUE)                               ///< 2PI*Fbase*Tpwm
#define SPEED_KLPF                      _Q15(_2PI*SPD_BW*TPWM_VALUE)                                  ///< 2PI*SPD_BW*Tpwm
#define OBS_EA_KS1                      _Q15((0.5*MOTOR_SPEED_SMOMIN_RPM*_2PI*BASE_FREQ*TPWM_VALUE)/MOTOR_SPEED_BASE)     // SMO的最小速度
#define OBS_EA_KS2                      _Q15((1.0*MOTOR_SPEED_SMOMIN_RPM*_2PI*BASE_FREQ*TPWM_VALUE)/MOTOR_SPEED_BASE)     // SMO的最小速度
#define OBS_EA_KS                       _Q15((2.0*MOTOR_SPEED_SMOMIN_RPM*_2PI*BASE_FREQ*TPWM_VALUE)/MOTOR_SPEED_BASE)     // SMO的最小速度
#define OBS_EA_KS_WIND                       _Q15((2.0*MOTOR_SPEED_SMOMIN_RPM_WIND*_2PI*BASE_FREQ*TPWM_VALUE)/MOTOR_SPEED_BASE)     // SMO的最小速度

/*逆风判断时的估算算法设置值------------------------------------------------------------*/
#define SPEED_KLPF_WIND                 _Q15(_2PI*SPD_BW_WIND*TPWM_VALUE)               ///< 2PI*SPD_BW_Wind*Tpwm
#define OBSW_KP_GAIN_WIND               _Q12(2*_2PI*ATT_COEF*ATO_BW_WIND/BASE_FREQ)
#define OBSW_KI_GAIN_WIND               _Q15(_2PI*ATO_BW_WIND*ATO_BW_WIND*TPWM_VALUE/BASE_FREQ)

#define OBSE_PLLKP_GAIN                 _Q11(((2*ATT_COEF*_2PI*E_BW*LD - RS)*HW_BOARD_CURR_BASE)/HW_BOARD_VOLTAGE_BASE)
#define OBSE_PLLKI_GAIN                 _Q11((_2PI*E_BW*_2PI*E_BW*LD*TPWM_VALUE*HW_BOARD_CURR_BASE)/HW_BOARD_VOLTAGE_BASE)




/* -----Current Calib----- *
/* -----(Disable)---禁止-- */
/* -----(Enable)----使能-- */
#define CalibENDIS                      (Enable)

/* -----SVPWM mode----- *
/* -----(SVPWM_5_Segment)---五段式-- */
/* -----(SVPWM_7_Segment)---七段式-- */
#define SVPMW_Mode                      (SVPWM_7_Segment)

/* -----double resistor sample mode----- *
/* -----(DouRes_1_Cycle)---1 周期采样完 ia, ib-- */
/* -----(DouRes_2_Cycle)---交替采用ia, ib, 2周期采样完成-- */
#define DouRes_Sample_Mode              (DouRes_1_Cycle)


/* 电机启动参数值 */
//open 算法启动参数
#define MOTOR_OPEN_RAMP_ACC             (30.0)                                  ///< 强拖启动的增量(每载波周期加一次)
#define MOTOR_OPEN_RAMP_MIN             S_Value(0.0)                            ///< 强拖启动的初始速度
#define MOTOR_OPEN_RAMP_CNT             (100.0)                                 ///< 强拖启动的执行次数(MOTOR_OPEN_ACC_CNT*256)










#endif