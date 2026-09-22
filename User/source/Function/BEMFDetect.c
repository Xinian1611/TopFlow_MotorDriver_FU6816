/*  --------------------------- (C) COPYRIGHT 2022 Fortiortech ShenZhen -----------------------------
    File Name      : BEMFDetect.c
    Author         : Fortiortech  Appliction Team
    Version        : V1.1
    Date           : 2022-07-08
    Description    : This file contains Bemf detection function used for Motor Control.
    ----------------------------------------------------------------------------------------------------
                                       All Rights Reserved
    ------------------------------------------------------------------------------------------------- */

#include <MyProject.h>

uint8  code table_Bemf_CWNext[6]   = {3, 6, 2, 5, 1, 4};
uint8  code table_Bemf_CWPre[6]    = {5, 3, 1, 6, 4, 2};

BEMFDetect_TypeDef xdata mcBemf;
#if (TAILWIND_MODE == BEMFMethod)


static void  Cmp_Bemf_Init(void);
static uint8 FR_Detect(uint8 bemfStatus, uint8 setFr);
static void BEMFTailWindStart(void);




/**
    @brief        Bemf的顺逆风检测，比较器硬件初始化，注意需要核对IO是否对应
    @date         2022-07-09
*/
static void Cmp_Bemf_Init(void)
{
    /*  -------------------------------------------------------------------------------------------------
        CMP Input Pin Mode
        0: GPIO Mode,   P1.4--CMP0_IN+, P1.6--CMP1_IN+, P2.1--CMP2_IN+
                        P1.5--CMP0_IN-, P1.7--CMP1_IN-, P2.2--CMP2_IN-
        1: BEMF Mode,   比较器正端连接到内部星型连接电阻U、V、W的BMEF采样点，
                        比较器负端连接到内部星型连接电阻的虚拟中性点
                        比较器负端与P1.5/P1.7/P2.2断开，这三个GPIO可做其他用途
        -------------------------------------------------------------------------------------------------*/
    #if (TAILWIND_PIN == PIN_141621)
    SetBit(P1_AN, P14 | P16);   // CMP0/1 Pin设置为模拟模式  +
    SetBit(P2_AN, P21);         // CMP2 Pin设置为模拟模式  +
    ClrBit(P1_PU, P14);         // P14上拉关闭
    ClrBit(CMP_CR4, CMP0_FS);   //功能不转移
    #elif (TAILWIND_PIN == PIN_131415)
    SetBit(P1_AN, P14 | P15 | HBMOD);//配置为模拟模式
    ClrBit(P1_OE, P13);
    SetBit(CMP_CR4, CMP0_FS); //CMP1/2功能转移  仅CMP0_MOD=01时有效
    #endif
    /*  -------------------------------------------------------------------------------------------------
        CMP0_MOD：
        00：  无内置虚拟中心点电阻的BEMF模式
        01：  内置虚拟中心点电阻的BEMF模式
        10：  3差分比较器模式
        11：  2比较器模式
        -------------------------------------------------------------------------------------------------*/
    SetReg(CMP_CR2, CMP0MOD0 | CMP0MOD1, CMP0MOD0);
    /*  -------------------------------------------------------------------------------------------------
        比较器输出选择配置，与CMP0_MOD配合使用
        CMP0_SEL[1:0]=00，比较器0工作在3比较器轮询模式，正端在CMP0P、CMP1P、CMP2P之间自动轮流选择，
                      负端固定接内置BEMF电阻的中心点，其输出结果分别送至CMP0_OUT、CMP1_OUT、CMP2_OUT
        CMP0_SEL[1:0]=01，比较器0选择CMP0对应的端口组合，正端接CMP0P，负端接内置BEMF电阻的中心点，输出接CMP0_OUT
        CMP0_SEL[1:0]=10，比较器0选择CMP1对应的端口组合，正端接CMP1P，负端接内置BEMF电阻的中心点，输出接CMP1_OUT
        CMP0_SEL[1:0]=11，比较器0选择CMP2对应的端口组合，正端接CMP2P，负端接内置BEMF电阻的中心点，输出接CMP2_OUT
        -----------------------------------------------------------------------------*/
    SetReg(CMP_CR2, CMP0SEL0 | CMP0SEL1, 0x00);
    /*  -------------------------------------------------------------------------------------------------
    CMP0/1/2 迟滞电压选择:
    00: 无迟滞
    01: ±3mV
    10: + 6mV
    11: ±12mV
        -------------------------------------------------------------------------------------------------*/
    SetReg(CMP_CR1, CMP0HYS0 | CMP0HYS1, CMP0HYS0 | CMP0HYS1);
    /*  -------------------------------------------------------------------------------------------------
        CMP0的轮询时间设置
        -------------------------------------------------------------------------------------------------*/
    SetReg(CMP_CR2, CMP0CSEL1 | CMP0CSEL0, 0x00);
    /*  -------------------------------------------------------------------------------------------------
        比较器中断模式配置 CMP_CR0[CMP2IM1], CMP_CR0[CMP2IM0]
        00: 不产生中断  01: 上升沿产生中断  10: 下降沿产生中断  11: 上升/下降沿产生中断
        -------------------------------------------------------------------------------------------------*/
    SetReg(CMP_CR0, CMP2IM0 | CMP2IM1, CMP2IM0 | CMP2IM1);
    SetReg(CMP_CR0, CMP1IM0 | CMP1IM1, CMP1IM0 | CMP1IM1);
    SetReg(CMP_CR0, CMP0IM0 | CMP0IM1, CMP0IM0 | CMP0IM1);
    SetBit(CMP_CR2, CMP0EN);  //开三个比较器
}


/**
    @brief        Timer2 初始化函数
    @date         2022-07-09
*/
static void Time2_Bemf_Init(void)
{
    /*  -------------------------------------------------------------------------------------------------
        先停止计数，配置完寄存器后，最后启动计数
        -------------------------------------------------------------------------------------------------*/
    ClrBit(TIM2_CR1, T2CEN); // 0，停止计数；1,使能计数
    /*  -------------------------------------------------------------------------------------------------
        时钟分频设置(T2PSC)
        000:cpuclk(24MHz)         001:cpuclk/2^1(12MHz)   010:cpuclk/2^2(6MHz)    011:cpuclk/2^3(3MHz)
        100:cpuclk/2^4(1.5MHz)    101:cpuclk/2^5(750KHz)  110:cpuclk/2^6(375KHz)  111:cpuclk/2^7(187.5KHz)
        -------------------------------------------------------------------------------------------------*/
    SetReg(TIM2_CR0, T2PSC0 | T2PSC1 | T2PSC2, T2PSC0 | T2PSC1 | T2PSC2);
    /*  -------------------------------------------------------------------------------------------------
        /模式选择
        T2MODE1，T2MODE0
        00--输入Timer模式；01--输出模式
        10--输入Count模式；11--QEP或者ISD模式
        -------------------------------------------------------------------------------------------------*/
    SetReg(TIM2_CR0, T2MOD0 | T2MOD1, T2MOD0);
    /*  -------------------------------------------------------------------------------------------------
        清除中断标志位
        禁止PWM周期检测中断使能
        -------------------------------------------------------------------------------------------------*/
    ClrBit(TIM2_CR0, T2CES);                                // 清零脉冲计数器不使能
    SetBit(TIM2_CR1, T2IFE);                                // 溢出中断使能
    ClrBit(TIM2_CR1, T2IR | T2IF | T2IP);                   // 清零中断标志位
    /*  -------------------------------------------------------------------------------------------------
        配置周期值、比较值、计数值
        禁止PWM周期检测中断使能
        使能计数器上溢中断使能
        -------------------------------------------------------------------------------------------------*/
    TIM2__ARR  = 60000;                                    // TIM2 Period = 0.32s
    TIM2__DR   = TIM2__ARR;
    TIM2__CNTR = 0;
    /*-----------启动计数------------------------------------------------*/
    SetBit(TIM2_CR1, T2CEN);                                 //启动计数
}


/**
    @brief        基于Bemf的顺逆风检测初始化函数，电机状态切入 顺逆风检测状态时 运行一次
    @date         2022-07-09
*/
void BEMFDetectInit(void)
{
    //BEMF检测前关闭mos输出
    mcBemf.BEMFSpeed            = 0;
    mcBemf.BEMFSpeedBase        = 0;
    mcBemf.Status               = 0;
    mcBemf.FR                   = BEMF_FR_ERR;
    mcBemf.FRPre                = BEMF_FR_ERR;
    mcBemf.SpeedUpdate          = 0;
    mcBemf.BEMFSpeedBase        = BEMFSpeedCalBase;
    mcBemf.HighSpdStart          = 0;
    mcBemf.FR_SET               = mcFocCtrl.FR;
    /* -----使能比较器----- */
    Cmp_Bemf_Init();
    /* -----使能定时器1用于检测时间----- */
    Time2_Bemf_Init();
}



/**
    @brief        bemf顺序为 5,1,3,2,6,4
    @param        bemfStatus 当前bemf状态
    @return       BEMF_FR_CCW 反转
    @return       BEMF_FR_CW  正转
    @return       BEMF_FR_ERR 错误
    @date         2022-07-09
*/
static uint8 FR_Detect(uint8 bemfStatus, uint8 setFr)
{
    static uint8 temp_FR = 0;
    static uint8 temp_bemfStatusPre = 0;
    
    if (temp_bemfStatusPre == 0)
    {
        temp_bemfStatusPre = bemfStatus;
        temp_FR = BEMF_FR_ERR;
    }
    
    if (temp_bemfStatusPre != bemfStatus)
    {
        if (temp_bemfStatusPre == table_Bemf_CWPre[bemfStatus - 1])
        {
            if (setFr  == CW)                 ///< 转向设置
            {
                temp_FR = BEMF_FR_CW;
            }
            else
            {
                temp_FR = BEMF_FR_CCW;
            }
        }
        else if (temp_bemfStatusPre == table_Bemf_CWNext[bemfStatus - 1])
        {
            if (setFr  == CW)                 ///< 转向设置
            {
                temp_FR = BEMF_FR_CCW;
            }
            else
            {
                temp_FR = BEMF_FR_CW;
            }
        }
        else
        {
            temp_FR = BEMF_FR_ERR;
        }
        
        temp_bemfStatusPre = bemfStatus;
    }
    
    return temp_FR;
}


/**
    @brief        void BemfProcess(void)基于Bemf的顺逆风检测，运行于Bemf检测比较器中断
    @date         2022-07-09
*/
void BemfProcess(void)
{
    static uint8 temp_Cnt = 0;
    static uint8 temp_SumCnt = 0;
    uint16 temp_Sum = 0;
    int16 temp_speedCal;
    mcBemf.FRPre = mcBemf.FR;               // 获取上一次的转向
    mcBemf.FR = FR_Detect(CMP_SR & 0x07, mcBemf.FR_SET);  // 获取当前转向
    
    if (mcBemf.FR == BEMF_FR_CW)            // 当前为正转
    {
        if (mcBemf.FRPre == mcBemf.FR)
        {
            if (temp_Cnt > 5)  // 防溢出越界
            {
                temp_Cnt = 0;
            }
            
            mcBemf.SectorTime[temp_Cnt++] = TIM2__CNTR;
            
            if (mcBemf.FRCount < 100)
            {
                mcBemf.FRCount++;           //  连续两次均为正转则 正转计数+1
            }
            
            if (mcBemf.FRCount > 12)
            {
                temp_Sum = 0;
                
                for (temp_SumCnt = 0; temp_SumCnt < 6; temp_SumCnt++)
                {
                    temp_Sum += (mcBemf.SectorTime[temp_SumCnt] >> 3); // 防止溢出
                }
                
                mcBemf.PeriodTime = temp_Sum;
                mcBemf.Status = BEMF_FORWARD; // 连续12次均为正向转动,则bemf状态切换为 BEMF_FORWARD
            }
        }
        else
        {
            mcBemf.FRCount = 0;
            mcBemf.Status = BEMF_DETECTING;
        }
    }
    else if (mcBemf.FR == BEMF_FR_CCW)
    {
        if (mcBemf.FRPre == mcBemf.FR)
        {
            if (temp_Cnt > 5)   // 防溢出越界
            {
                temp_Cnt = 0;
            }
            
            mcBemf.SectorTime[temp_Cnt++] = TIM2__CNTR;
            
            if (mcBemf.FRCount > - 100)
            {
                mcBemf.FRCount--;//  连续两次均为反转则 反转计数+1
            }
            
            if (mcBemf.FRCount < -12)
            {
                temp_Sum = 0;
                
                for (temp_SumCnt = 0; temp_SumCnt < 6; temp_SumCnt++)
                {
                    temp_Sum += (mcBemf.SectorTime[temp_SumCnt] >> 3); // 防止溢出
                }
                
                mcBemf.PeriodTime = temp_Sum;
                mcBemf.Status =  BEMF_REVERSE;
            }
        }
        else
        {
            mcBemf.FRCount = 0;
            mcBemf.Status = BEMF_DETECTING;
        }
    }
    else
    {
        mcBemf.FRCount = 0; // 转向检测错误则清零转向计数
        mcBemf.Status = BEMF_DETECTING;
    }
    
    TIM2__CNTR = 0;
    
    if (mcBemf.Status == BEMF_FORWARD || mcBemf.Status == BEMF_REVERSE )  // 有效转向
    {
        //        if (mcBemf.SpeedUpdate  == 0)
        {
            if (mcBemf.PeriodTime < BEMFSpeedCalMinPeriod)
            {
                mcBemf.PeriodTime = BEMFSpeedCalMinPeriod ;    // 防止mcRsd.PeriodTime 太小导致计算出错
            }
            
            temp_speedCal = MDU_DIV0_GetQL16(mcBemf.BEMFSpeedBase >> 16, mcBemf.BEMFSpeedBase, mcBemf.PeriodTime);
            
            if (mcBemf.Status == BEMF_REVERSE)
            {
                mcBemf.BEMFSpeed = -temp_speedCal;
            }
            else
            {
                mcBemf.BEMFSpeed  = temp_speedCal;
            }
            
            mcBemf.SpeedUpdate  = 1;
        }
    }
    
    if (mcBemf.HighSpdStart == 1)     // 高速顺风启动
    {
        if ((CMP_SR & 0x07) == 0x01)  // 01状态 对应0启动
        {
            ClrBit(CMP_CR2, CMP0EN);  // 关闭比较器
            ClrBit(TIM2_CR1, T2CEN);  // 关闭定时器
            BEMFTailWindStart();
            mcBemf.HighSpdStart = 2;  // 启动完成
        }
    }
}

MotStateType Bemf_Start_Process(void)
{
    MotStateType returnStatus;
    returnStatus = mcTailWind;
    
    if (mcBemf.SpeedUpdate == 1)
    {
        if (mcBemf.BEMFSpeed < S_Value(-100))  // 逆风
        {
            mcFocCtrl.State_Count            = 3000;
            ClrBit(CMP_CR2, CMP0EN);  // 关闭比较器
            ClrBit(TIM2_CR1, T2CEN);  // 关闭定时器
            returnStatus                 = mcStart;
            mcFocCtrl.Start_Mode         = HEADWIND_START;
        }
        //        else if (mcBemf.BEMFSpeed < S_Value(-50))  //正转
        //        {
        //            mcBemf.SpeedUpdate = 0;
        //        }
        else if (mcBemf.BEMFSpeed > MOTOR_LOOP_RPM)  //正转
        {
            if (mcBemf.HighSpdStart == 2) //启动完成，切状态机
            {
                mcFocCtrl.Start_Mode         = TAILWIND_START;
                returnStatus                 = mcStart;
            }
            
            if (mcBemf.HighSpdStart == 0)
            {
                mcBemf.HighSpdStart = 1; // 设置启动
            }
        }
        else  //其他，如静止
        {
            mcFocCtrl.Start_Mode    = STATIC_START;
            returnStatus                 = mcStart;
            ClrBit(CMP_CR2, CMP0EN);  // 关闭比较器
            ClrBit(TIM1_CR0, T1BCEN); // 关闭定时器
            #if (PosCheckEnable == Enable)
            returnStatus                             = mcPosiCheck;
            McStaSet.SetFlag.PosiCheckSetFlag   = 0;
            mcFocCtrl.mcPosCheckAngle           = 0xffff;                     // 角度赋初值
            #elif (ALIGN_MOME != ALIGN_DSIABLE)
           if (mcFocCtrl.FR == CW)
            {
                mcFocCtrl.mcPosCheckAngle           = Align_Angle_CW;
            }
            else
            {
                mcFocCtrl.mcPosCheckAngle           = Align_Angle_CCW;
            }
            returnStatus                             = mcAlign;
            mcFocCtrl.State_Count               = Align_Time;
            #else
            returnStatus = mcStart;
            #endif
        }
    }
    
    return returnStatus;
}

static void BEMFTailWindStart(void)
{
    /* FOC初始化 */
    FOC_Init();
    /* 启动电流、KP、KI */
    FOC_IDREF           = ID_RUN_CURRENT;                      // D轴启动电流
    FOC_IQREF           = IQ_RUN_CURRENT;                      // Q轴启动电流
    mcFocCtrl.IqRef     = IQ_RUN_CURRENT;                      // Q轴启动电流
    FOC_DKP             = _Q12(5.0);
    FOC_DKI             = _Q15(0.5);
    FOC_QKP             = _Q12(5.0);
    FOC_QKI             = _Q15(0.5);
    FOC_EFREQACC        = MOTOR_OMEGA_RAMP_ACC;
    FOC_EFREQMIN        = MOTOR_OMEGA_RAMP_MIN;
    FOC_EFREQHOLD       = MOTOR_OMEGA_RAMP_END;
    SetBit(FOC_CR1, ANGM);                                     // 估算模式
    ClrBit(FOC_CR1, RFAE);                                     // 禁止强拉
    SetBit(FOC_CR1, EFAE);                                     // 估算器强制输出
    FOC_EKP           = OBSW_KP_GAIN_WIND;
    FOC_EKI           = OBSW_KI_GAIN_WIND;
    FOC_OMEKLPF        = SPEED_KLPF_WIND;
    FOC_EKLPFMIN    = OBS_EA_KS_WIND;
    FOC__EOME           = mcBemf.BEMFSpeed;
    mcFocCtrl.CtrlMode = 0;
    /* 使能输出 */
    DRV_CMR |= 0x3F;                         // U、V、W相输出
    MOE = 1;
    EA = 1;
}
#endif

