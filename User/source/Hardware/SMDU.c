/**
 * @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      xxx.c
 * @author    Fortiortech  Appliction Team
 * @date      2022-07-13
 * @brief     This file contains TSD function used for Motor Control.
 */

#include <SMDU.h>
/********************************************************************************
    Internal Routine Prototypes
********************************************************************************/

/*  -------------------------------------------------------------------------------------------------
    Function Name  : HW_One_PI
    Description    : PI
    Date           : 2021-08-08
    Parameter      : Xn1: [输入]
    ------------------------------------------------------------------------------------------------- */
int16 HW_One_PI(int16 Xn1)
{
    PI1_EK =  Xn1;                                                                               //填入EK
    SMDU_RunBlock(1, PI);
    return PI1_UKH;
}
/*  -------------------------------------------------------------------------------------------------
    Function Name : int16 HW_One_PI2(int16 Xn1, int16 Yn0, int16 Xn2)
    Description   : PI控制
    Input         : Xn1--E(K)
    Output        : PI_UK--当前PI输出值,执行时间us
    -------------------------------------------------------------------------------------------------*/
int16 HW_One_PI2(int16 Xn1)
{
    PI2_EK =  Xn1;                                                                               //填入EK
    SMDU_RunBlock(2, PI);
    return PI2_UKH;
}
/*  -------------------------------------------------------------------------------------------------
    Function Name : int16 HW_One_PI3(int16 Xn1, int16 Yn0, int16 Xn2)
    Description   : PI控制
    Input         : Xn1--E(K)
    Output        : PI_UK--当前PI输出值,执行时间us
    -------------------------------------------------------------------------------------------------*/
int16 HW_One_PI3(int16 Xn1)
{
    PI3_EK =  Xn1;                                                                               //填入EK
    SMDU_RunBlock(3, PI);
    return PI3_UKH;
}

/*  -------------------------------------------------------------------------------------------------
    Function Name  : LPFFunction
    Description    : 低通滤波函数
    Date           : 2021-08-08
    Parameter      : Xn1: [输入]
**                   Xn0: [输入]
**                   K: [输入]
    ------------------------------------------------------------------------------------------------- */
int16 LPFFunction(int16 Xn1, int16 Xn0, int8 K)
{
    LPF0_K = K << 8;
    LPF0_X = Xn1;
    LPF0_YH = Xn0;
    SMDU_RunBlock(0, LPF);
    return LPF0_YH;
}


/*  -------------------------------------------------------------------------------------------------
    Function Name  : Atan_Us_MDU
    Description    : XX-XX-XX
    Date           : 2021-11-06
    Parameter      : Xn1: [输入/出]
**                 Xn0: [输入/出]
**                 K: [输入/出]
    ------------------------------------------------------------------------------------------------- */
int16 Atan_Us_MDU(int16 Xn1, int16 Xn0)
{
    SCAT1_COS = Xn1;
    SCAT1_SIN = Xn0;
    SCAT1_THE = 0;
    SMDU_RunBlock(1, ATAN);
    return SCAT1_RES1;
}

/*  -------------------------------------------------------------------------------------------------
    Function Name  : Atan_Us_MDU
    Description    : XX-XX-XX
    Date           : 2021-11-06
    Parameter      : Xn1: [输入/出]
**                       Xn0: [输入/出]
**                       Yn:  [输入/出]
    ------------------------------------------------------------------------------------------------- */
int16 DivQ_L_MDU(uint16 Xn1, uint16 Xn0, uint16 Yn)
{
    DIV0_DAH = Xn1;
    DIV0_DAL = Xn0;
    DIV0_DB  = Yn;
    SMDU_RunBlock(0, DIV);
    return DIV0_DQL;
}

/**
 * @brief        PI
 * @param[in]    Xn1，当前误差
 * @param[out]   PIx_UKH，PI计算结果高16位
 */
int16 MDU_PI0(int16 Xn1)
{
	PI0_EK = Xn1;                                                                               // Fill in EK
	SMDU_RunBlock(0, PI);
	return PI0_UKH;
}
int16 MDU_PI1(int16 Xn1)
{
	PI1_EK = Xn1;                                                                               // Fill in EK
	SMDU_RunBlock(1, PI);
	return PI1_UKH;
}
int16 MDU_PI2(int16 Xn1)
{
	PI2_EK = Xn1;                                                                               // Fill in EK
	SMDU_RunBlock(2, PI);
	return PI2_UKH;
}
int16 MDU_PI3(int16 Xn1)
{
	PI3_EK = Xn1;                                                                               // Fill in EK
	SMDU_RunBlock(3, PI);
	return PI3_UKH;
}

/**
 * @brief        低通滤波器
 * @param[in]    Xn1，滤波输入
 * @param[in]    Xn0，滤波输入
 * @Param[in]		 K，滤波系数
 * @param[out]   LPFx_YH，滤波结果高16位
 */
int16 MDU_LPF0(int16 Xn1, int16 Xn0, int8 K)
{
	LPF0_K = K << 8;
	LPF0_X = Xn1;
	LPF0_YH = Xn0;
	SMDU_RunBlock(0, LPF);
	return LPF0_YH;
}
int16 MDU_LPF1(int16 Xn1, int16 Xn0, int8 K)
{
	LPF1_K = K << 8;
	LPF1_X = Xn1;
	LPF1_YH = Xn0;
	SMDU_RunBlock(1, LPF);
	return LPF1_YH;
}
int16 MDU_LPF2(int16 Xn1, int16 Xn0, int8 K)
{
	LPF2_K = K << 8;
	LPF2_X = Xn1;
	LPF2_YH = Xn0;
	SMDU_RunBlock(2, LPF);
	return LPF2_YH;
}
int16 MDU_LPF3(int16 Xn1, int16 Xn0, int8 K)
{
	LPF3_K = K << 8;
	LPF3_X = Xn1;
	LPF3_YH = Xn0;
	SMDU_RunBlock(3, LPF);
	return LPF3_YH;
}

/**
 * @brief        幅值计算
                U = sqrt((Usinθ)^2 +  (Ucosθ)^2) 
                θ = atan(Usinθ/Ucosθ)
 * @param[in]    Ucos，Cos
 * @param[in]    Usin，Sin
 * @param[out]   SCAT1_RES1 幅值计算结果
 */
uint16 MDU_Atan0_GetAmp(int16 Usin, int16 Ucos)
{
	SCAT0_COS = Ucos;
	SCAT0_SIN = Usin;
	SMDU_RunBlock(0, ATAN);
	return SCAT0_RES1;
}
uint16 MDU_Atan1_GetAmp(int16 Usin, int16 Ucos)
{
	SCAT1_COS = Ucos;
	SCAT1_SIN = Usin;
	SMDU_RunBlock(1, ATAN);
	return SCAT1_RES1;
}
uint16 MDU_Atan2_GetAmp(int16 Usin, int16 Ucos)
{
	SCAT2_COS = Ucos;
	SCAT2_SIN = Usin;
	SMDU_RunBlock(2, ATAN);
	return SCAT2_RES1;
}
uint16 MDU_Atan3_GetAmp(int16 Usin, int16 Ucos)
{
	SCAT3_COS = Ucos;
	SCAT3_SIN = Usin;
	SMDU_RunBlock(3, ATAN);
	return SCAT3_RES1;
}

/**
 * @brief       角度计算
                U = sqrt((Usinθ)^2 +  (Ucosθ)^2) 
                θ = atan(Usinθ/Ucosθ)
 * @param[in]    Ucos，Cos
 * @param[in]    Usin，Sin
 * @param[out]   SCAT1_RES2 角度计算结果
 */
uint16 MDU_Atan0_GetTheta(int16 Usin, int16 Ucos)
{
	SCAT0_COS = Ucos;
	SCAT0_SIN = Usin;
	SMDU_RunBlock(0, ATAN);
	return SCAT0_RES2;
}
uint16 MDU_Atan1_GetTheta(int16 Usin, int16 Ucos)
{
	SCAT1_COS = Ucos;
	SCAT1_SIN = Usin;
	SMDU_RunBlock(1, ATAN);
	return SCAT1_RES2;
}
uint16 MDU_Atan2_GetTheta(int16 Usin, int16 Ucos)
{
	SCAT2_COS = Ucos;
	SCAT2_SIN = Usin;
	SMDU_RunBlock(2, ATAN);
	return SCAT2_RES2;
}
uint16 MDU_Atan3_GetTheta(int16 Usin, int16 Ucos)
{
	SCAT3_COS = Ucos;
	SCAT3_SIN = Usin;
	SMDU_RunBlock(3, ATAN);
	return SCAT3_RES2;
}

/** 
 * @brief     有符号乘法16x16
 * @param[in] Xn1 乘数0(16位)
 * @param[in] Xn0 乘数1(16位)
 * @return    积的高16位 
 * @date      2023-09-21
 */
int16 MDU_MUL0_GetSH16(int16 Xn1, int16 Xn0)
{
	MUL0_MA = Xn1;
	MUL0_MB = Xn0;
	SMDU_RunBlock(0, SMUL);
	return  (int16)MUL0_MCH;
}
int16 MDU_MUL1_GetSH16(int16 Xn1, int16 Xn0)
{
	MUL1_MA = Xn1;
	MUL1_MB = Xn0;
	SMDU_RunBlock(1, SMUL);
	return (int16)MUL1_MCH;
}
int16 MDU_MUL2_GetSH16(int16 Xn1, int16 Xn0)
{
	MUL2_MA = Xn1;
	MUL2_MB = Xn0;
	SMDU_RunBlock(2, SMUL);
	return (int16)MUL2_MCH;
}
int16 MDU_MUL3_GetSH16(int16 Xn1, int16 Xn0)
{
	MUL3_MA = Xn1;
	MUL3_MB = Xn0;
	SMDU_RunBlock(3, SMUL);
	return (int16)MUL3_MCH;
}

/** 
 * @brief     有符号乘法16x16
 * @param[in] Xn1 乘数0(16位)
 * @param[in] Xn0 乘数1(16位)
 * @return    积的低16位 
 * @date      2023-09-21
 */
int16 MDU_MUL0_GetSL16(int16 Xn1, int16 Xn0)
{
	MUL0_MA = Xn1;
	MUL0_MB = Xn0;
	SMDU_RunBlock(0, SMUL);
	return (int16)MUL0_MCL;
}
int16 MDU_MUL1_GetSL16(int16 Xn1, int16 Xn0)
{
	MUL1_MA = Xn1;
	MUL1_MB = Xn0;
	SMDU_RunBlock(1, SMUL);
	return (int16)MUL1_MCL;
}
int16 MDU_MUL2_GetSL16(int16 Xn1, int16 Xn0)
{
	MUL2_MA = Xn1;
	MUL2_MB = Xn0;
	SMDU_RunBlock(2, SMUL);
	return (int16)MUL2_MCL;
}
int16 MDU_MUL3_GetSL16(int16 Xn1, int16 Xn0)
{
	MUL3_MA = Xn1;
	MUL3_MB = Xn0;
	SMDU_RunBlock(3, SMUL);
	return (int16)MUL3_MCL;
}

/** 
 * @brief     有符号乘法16x16
 * @param[in] Xn1 乘数0(16位)
 * @param[in] Xn0 乘数1(16位)
 * @return    积32位 
 * @date      2023-09-21
 */
int32 MDU_MUL0_GetS32(uint16 Xn1, uint16 Xn0)
{
	MUL0_MA = Xn1;
	MUL0_MB = Xn0;
	SMDU_RunBlock(0, SMUL);

	return* (int32*) & MUL0_MCH;
}
int32 MDU_MUL1_GetS32(uint16 Xn1, uint16 Xn0)
{
	MUL1_MA = Xn1;
	MUL1_MB = Xn0;
	SMDU_RunBlock(1, SMUL);

	return* (int32*) & MUL1_MCH;
}
int32 MDU_MUL2_GetS32(uint16 Xn1, uint16 Xn0)
{
	MUL2_MA = Xn1;
	MUL2_MB = Xn0;
	SMDU_RunBlock(2, SMUL);

	return* (int32*) & MUL2_MCH;
}
int32 MDU_MUL3_GetS32(uint16 Xn1, uint16 Xn0)
{
	MUL3_MA = Xn1;
	MUL3_MB = Xn0;
	SMDU_RunBlock(3, SMUL);

	return* (int32*) & MUL3_MCH;
}

/** 
 * @brief     无符号乘法16x16
 * @param[in] Xn1 乘数0(16位)
 * @param[in] Xn0 乘数1(16位)
 * @return    积的高16位 
 * @date      2023-09-21
 */
uint16 MDU_MUL0_GetUH16(uint16 Xn1, uint16 Xn0)
{
	MUL0_MA = Xn1;
	MUL0_MB = Xn0;
	SMDU_RunBlock(0, UMUL);
	return (uint16)MUL0_MCH;
}
uint16 MDU_MUL1_GetUH16(uint16 Xn1, uint16 Xn0)
{
	MUL1_MA = Xn1;
	MUL1_MB = Xn0;
	SMDU_RunBlock(1, UMUL);
	return (uint16)MUL1_MCH;
}

uint16 MDU_MUL2_GetUH16(uint16 Xn1, uint16 Xn0)
{
	MUL2_MA = Xn1;
	MUL2_MB = Xn0;
	SMDU_RunBlock(2, UMUL);
	return (uint16)MUL2_MCH;
}

uint16 MDU_MUL3_GetUH16(uint16 Xn1, uint16 Xn0)
{
	MUL3_MA = Xn1;
	MUL3_MB = Xn0;
	SMDU_RunBlock(3, UMUL);
	return (uint16)MUL3_MCH;
}

/** 
 * @brief     无符号乘法16x16
 * @param[in] Xn1 乘数0(16位)
 * @param[in] Xn0 乘数1(16位)
 * @return    积的低16位 
 * @date      2023-09-21
 */
uint16 MDU_MUL0_GetUL16(uint16 Xn1, uint16 Xn0)
{
	MUL0_MA = Xn1;
	MUL0_MB = Xn0;
	SMDU_RunBlock(0, UMUL);
	return (uint16)MUL0_MCL;
}

uint16 MDU_MUL1_GetUL16(uint16 Xn1, uint16 Xn0)
{
	MUL1_MA = Xn1;
	MUL1_MB = Xn0;
	SMDU_RunBlock(1, UMUL);
	return (uint16)MUL1_MCL;
}
uint16 MDU_MUL2_GetUL16(uint16 Xn1, uint16 Xn0)
{
	MUL2_MA = Xn1;
	MUL2_MB = Xn0;
	SMDU_RunBlock(2, UMUL);
	return (uint16)MUL2_MCL;
}
uint16 MDU_MUL3_GetUL16(uint16 Xn1, uint16 Xn0)
{
	MUL3_MA = Xn1;
	MUL3_MB = Xn0;
	SMDU_RunBlock(3, UMUL);
	return (uint16)MUL3_MCL;
}

/** 
 * @brief     无符号乘法16x16
 * @param[in] Xn1 乘数0(16位)
 * @param[in] Xn0 乘数1(16位)
 * @return    积的32位 
 * @date      2023-09-21
 */
uint32 MDU_MUL0_GetU32(uint16 Xn1, uint16 Xn0)
{
	MUL0_MA = Xn1;
	MUL0_MB = Xn0;
	SMDU_RunBlock(0, UMUL);
	
	return *(uint32*)&MUL0_MCH;
}
uint32 MDU_MUL1_GetU32(uint16 Xn1, uint16 Xn0)
{
	MUL1_MA = Xn1;
	MUL1_MB = Xn0;
	SMDU_RunBlock(1, UMUL);

	return* (uint32*) & MUL1_MCH;
}
uint32 MDU_MUL2_GetU32(uint16 Xn1, uint16 Xn0)
{
	MUL2_MA = Xn1;
	MUL2_MB = Xn0;
	SMDU_RunBlock(2, UMUL);

	return* (uint32*) & MUL2_MCH;
}
uint32 MDU_MUL3_GetU32(uint16 Xn1, uint16 Xn0)
{
	MUL3_MA = Xn1;
	MUL3_MB = Xn0;
	SMDU_RunBlock(3, UMUL);

	return* (uint32*) & MUL3_MCH;
}

/**
 * @brief        32位/16位除法，求商，只取低位
 * @param[in]    daH 被除数高16位
 * @param[in]    daL 被除数低16位
 * @param[in]    Yn  除数
 * @param[out]   DIV0_DQL 商低16位
 */
uint16 MDU_DIV0_GetQL16(uint16 daH, uint16 daL, uint16 Yn)
{
	DIV0_DAH = daH;
	DIV0_DAL = daL;
	DIV0_DB = Yn;
	SMDU_RunBlock(0, DIV);
	return DIV0_DQL;
}
uint16 MDU_DIV1_GetQL16(uint16 daH, uint16 daL, uint16 Yn)
{
	DIV1_DAH = daH;
	DIV1_DAL = daL;
	DIV1_DB = Yn;
	SMDU_RunBlock(1, DIV);
	return DIV1_DQL;
}
uint16 MDU_DIV2_GetQL16(uint16 daH, uint16 daL, uint16 Yn)
{
	DIV2_DAH = daH;
	DIV2_DAL = daL;
	DIV2_DB = Yn;
	SMDU_RunBlock(2, DIV);
	return DIV2_DQL;
}
uint16 MDU_DIV3_GetQL16(uint16 daH, uint16 daL, uint16 Yn)
{
	DIV3_DAH = daH;
	DIV3_DAL = daL;
	DIV3_DB = Yn;
	SMDU_RunBlock(3, DIV);
	return DIV3_DQL;
}

/**
 * @brief        32位/16位除法，求商，取32位
 * @param[in]    daH 被除数高16位
 * @param[in]    daL 被除数低16位
 * @param[in]    Yn  除数
 * @param[out]   DIV0_DQL 商32位
 */
uint32 MDU_DIV0_GetQ32(uint16 daH, uint16 daL, uint16 Yn)
{
	DIV0_DAH = daH;
	DIV0_DAL = daL;
	DIV0_DB = Yn;
	SMDU_RunBlock(0, DIV);
	return *(uint32*)&DIV0_DQH;
}

uint32 MDU_DIV1_GetQ32(uint16 daH, uint16 daL, uint16 Yn)
{
	DIV1_DAH = daH;
	DIV1_DAL = daL;
	DIV1_DB = Yn;
	SMDU_RunBlock(1, DIV);
	return* (uint32*) & DIV1_DQH;
}
uint32 MDU_DIV2_GetQ32(uint16 daH, uint16 daL, uint16 Yn)
{
	DIV2_DAH = daH;
	DIV2_DAL = daL;
	DIV2_DB = Yn;
	SMDU_RunBlock(2, DIV);
	return* (uint32*) & DIV2_DQH;
}
uint32 MDU_DIV3_GetQ32(uint16 daH, uint16 daL, uint16 Yn)
{
	DIV3_DAH = daH;
	DIV3_DAL = daL;
	DIV3_DB = Yn;
	SMDU_RunBlock(3, DIV);
	return* (uint32*) & DIV3_DQH;
}