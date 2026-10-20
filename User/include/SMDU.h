/**
 * @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      SMDU.h
 * @author    Kris.huang  
 * @note      Last modify author is Leo.li
 * @since     2022-07-14
 * @date      2022-10-09
 * @brief       
 */
 

#ifndef __S_M_D_U_H_
#define __S_M_D_U_H_

#include <FU68xx_6.h>

/**
 * @brief SMDU的模式类型
 *
 * @note 使用@ref SMDU_RunNoBlock时, 其中的mode参数可以直接使用本枚举的内容
 * @note 使用@ref SMDU_RunBlock时, 其中的mode参数可以直接使用本枚举的内容
 */
typedef enum
{
    S1MUL   = 0,  /**< 有符号乘法16x16, 计算结果左移1位, 最小计算周期: 11 */
    SMUL    = 1,  /**< 有符号乘法16x16, 最小计算周期: 11 */
    UMUL    = 2,  /**< 无符号乘法16x16, 最小计算周期: 11 */
    DIV     = 3,  /**< 无符号除法32/16, 最小计算周期: 33 */
    SIN_COS = 4,  /**< Sin/Cos, 最小计算周期: 32 */
    ATAN    = 5,  /**< ATan, 最小计算周期: 30 */
    LPF     = 6,  /**< 低通滤波, 最小计算周期: 17 */
    PI      = 7,  /**< PI/PID, 最小计算周期: 30 */
} ETypeSMDUMode;
/******************************************************************************///External Symbols
/******************************************************************************///External Function
/**
 * @brief 运行SMDU且不等待运行结束
 *
 * @param  stan (0-3) 要启动的计算单元编号
 * @param  mode (0-7) 指定计算单元的模式, 可使用@ref ETypeSMDUMode 作为计算模式的设置\n
 * @ref S1MUL   有符号乘法, 计算结果左移1位 \n
 * @ref SMUL    有符号乘法 \n
 * @ref UMUL    无符号乘法 \n
 * @ref DIV     32/16无符号除法 \n
 * @ref SIN_COS Sin/Cos \n
 * @ref ATAN    ATan \n
 * @ref LPF     低通滤波 \n
 * @ref PI      PI \n
 */
#define SMDU_RunNoBlock(stan, mode)   do                                                  \
                                      {                                                   \
                                          MDU_CR = MDUSTA0 << stan | (unsigned char)mode; \
                                      } while (0)

/**
 * @brief 运行SMDU且等待运行结束
 *
 * @param  stan (0-3) 要启动的计算单元编号
 * @param  mode (0-7) 指定计算单元的模式, 可使用@ref ETypeSMDUMode 作为计算模式的设置\n
 * @ref S1MUL   有符号乘法, 计算结果左移1位 \n
 * @ref SMUL    有符号乘法 \n
 * @ref UMUL    无符号乘法 \n
 * @ref DIV     32/16无符号除法 \n
 * @ref SIN_COS Sin/Cos \n
 * @ref ATAN    ATan \n
 * @ref LPF     低通滤波 \n
 * @ref PI      PI \n
 */
#define SMDU_RunBlock(stan, mode)   do                                       \
                                    {                                        \
                                        SMDU_RunNoBlock(stan, mode);         \
                                        while (MDU_CR & MDUBSY);             \
                                    } while (0);





/* SMDU控制 */
extern int16  HW_One_PI(int16 Xn1);
extern int16  HW_One_PI2(int16 Xn1);
extern int16  HW_One_PI3(int16 Xn1);
extern int16  LPFFunction(int16 Xn1, int16 Xn0, int8 K);
extern int16  Atan_Us_MDU(int16 Xn1, int16 Xn0);
extern int16  DivQ_L_MDU(uint16 Xn1, uint16 Xn0,uint16 Yn);

									extern int16 MDU_PI0(int16 Xn1);
extern int16 MDU_PI1(int16 Xn1);
extern int16 MDU_PI2(int16 Xn1);                                   
extern int16 MDU_PI3(int16 Xn1);
                                    
extern int16 MDU_LPF0(int16 Xn1, int16 Xn0, int8 K);
extern int16 MDU_LPF1(int16 Xn1, int16 Xn0, int8 K);
extern int16 MDU_LPF2(int16 Xn1, int16 Xn0, int8 K);
extern int16 MDU_LPF3(int16 Xn1, int16 Xn0, int8 K);

extern uint16 MDU_Atan0_GetAmp(int16 Usin, int16 Ucos);
extern uint16 MDU_Atan1_GetAmp(int16 Usin, int16 Ucos);
extern uint16 MDU_Atan2_GetAmp(int16 Usin, int16 Ucos);
extern uint16 MDU_Atan3_GetAmp(int16 Usin, int16 Ucos);

extern uint16 MDU_Atan0_GetTheta(int16 Usin, int16 Ucos);
extern uint16 MDU_Atan1_GetTheta(int16 Usin, int16 Ucos);
extern uint16 MDU_Atan2_GetTheta(int16 Usin, int16 Ucos);
extern uint16 MDU_Atan3_GetTheta(int16 Usin, int16 Ucos);
                                  
extern int16 MDU_MUL0_GetSH16(int16 Xn1, int16 Xn0);
extern int16 MDU_MUL1_GetSH16(int16 Xn1, int16 Xn0);
extern int16 MDU_MUL2_GetSH16(int16 Xn1, int16 Xn0);                    
extern int16 MDU_MUL3_GetSH16(int16 Xn1, int16 Xn0);
                                    
extern int16 MDU_MUL0_GetSL16(int16 Xn1, int16 Xn0);                                   
extern int16 MDU_MUL1_GetSL16(int16 Xn1, int16 Xn0);                                    
extern int16 MDU_MUL2_GetSL16(int16 Xn1, int16 Xn0);                                    
extern int16 MDU_MUL3_GetSL16(int16 Xn1, int16 Xn0);                                    
                                    
extern int32 MDU_MUL0_GetS32(uint16 Xn1, uint16 Xn0);                                   
extern int32 MDU_MUL1_GetS32(uint16 Xn1, uint16 Xn0);                                    
extern int32 MDU_MUL2_GetS32(uint16 Xn1, uint16 Xn0);
extern int32 MDU_MUL3_GetS32(uint16 Xn1, uint16 Xn0);

extern uint16 MDU_MUL0_GetUH16(uint16 Xn1, uint16 Xn0);
extern uint16 MDU_MUL1_GetUH16(uint16 Xn1, uint16 Xn0);
extern uint16 MDU_MUL2_GetUH16(uint16 Xn1, uint16 Xn0);
extern uint16 MDU_MUL3_GetUH16(uint16 Xn1, uint16 Xn0);

extern uint16 MDU_MUL0_GetUL16(uint16 Xn1, uint16 Xn0);
extern uint16 MDU_MUL1_GetUL16(uint16 Xn1, uint16 Xn0);
extern uint16 MDU_MUL2_GetUL16(uint16 Xn1, uint16 Xn0);
extern uint16 MDU_MUL3_GetUL16(uint16 Xn1, uint16 Xn0);

extern uint32 MDU_MUL0_GetU32(uint16 Xn1, uint16 Xn0);
extern uint32 MDU_MUL1_GetU32(uint16 Xn1, uint16 Xn0);
extern uint32 MDU_MUL2_GetU32(uint16 Xn1, uint16 Xn0);
extern uint32 MDU_MUL3_GetU32(uint16 Xn1, uint16 Xn0);

extern uint16 MDU_DIV0_GetQL16(uint16 daH, uint16 daL, uint16 Yn);
extern uint16 MDU_DIV1_GetQL16(uint16 daH, uint16 daL, uint16 Yn);
extern uint16 MDU_DIV2_GetQL16(uint16 daH, uint16 daL, uint16 Yn);
extern uint16 MDU_DIV3_GetQL16(uint16 daH, uint16 daL, uint16 Yn);

extern uint32 MDU_DIV0_GetQ32(uint16 daH, uint16 daL, uint16 Yn);
extern uint32 MDU_DIV1_GetQ32(uint16 daH, uint16 daL, uint16 Yn);
extern uint32 MDU_DIV2_GetQ32(uint16 daH, uint16 daL, uint16 Yn);
extern uint32 MDU_DIV3_GetQ32(uint16 daH, uint16 daL, uint16 Yn);
#endif

