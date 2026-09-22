#ifndef __MOTORPOSCHECK_H_
#define __MOTORPOSCHECK_H_

typedef enum
{
    RPD_0				= 0,
    RPD_1				= 1,
    RPD_2       = 2,
    RPD_3		    = 3,
    RPD_4	      = 4,
    RPD_5				= 5,
    RPD_6			  = 6,
    RPD_7			  = 7,
    RPD_8				= 8,
    RPD_9				= 9,
    RPD_10       = 10,
    RPD_11		    = 11,
    RPD_12	      = 12,
    RPD_13				= 13,
    RPD_14			  = 14,
    RPD_15			  = 15,
}RPD_TypeDef;

typedef struct
{
    uint16 InsetIdStep1[6];// RPD检测处理
    uint16 InsetIdStep2[3];// RPD检测处理
    uint16 InsetIdStep3[2];// RPD检测处理
    
    uint16 InsetIdMax;
    uint8  injectmax1;  
    uint8  injectmax2;  
    uint8  injectmax3;
    
    int16  Angle;
    int16  ThetaGet;   // RPD的角度获取
    uint8  injecttimes;// RPD注入次数

    uint8  injectstep;


    uint8  injectCnt;
    uint8  injectCntTemp;
    
    uint16 InjectOffFocIntCnt;
    uint16 InjectOnFocIntCnt;

    uint16 InjectStep1VoltageProportion;
    uint16 InjectStep2VoltageProportion;

    int16 Theta;

    uint8 ForwardDirectionVectorFlag;
    uint8 OppositeDirectionVectorFlag;
    uint8 ZeroVectorFlag;
    uint8 NextShotFlag;
    
    uint8  injectstartflag;
    uint8  injectcntstartflag;
    
} RPD_Param_TypeDef;
extern void RPD (void);
extern void RPD_Inject(void);
extern void RPD_Detect(void);
extern void RPD_Init(void);
extern void Time2_RPD_Init(void);
extern void RPD_0_VUWinit(void);//VW
extern void RPD_1_WVinit(void);//WV
extern void RPD_2_UVWinit(void);//UV
extern void RPD_3_VUinit(void);//VU
extern void RPD_4_WUVinit(void);//WU
extern void RPD_5_UWinit(void);//UW
extern void RPD_ZeroVector(void);
extern void RPD_Angle_Inject(uint16 *InjectBuf);
extern void RPD_Get_ID(void);
extern RPD_Param_TypeDef RPDPara;
extern RPD_TypeDef RPD_Status;
#endif