/**
 * @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      MotorControlFunction.c
 * @author    Fortiortech  Appliction Team
 * @since     Create:2022-08-05
 * @date      Last modify:2022-08-05
 * @note      Last modify author is Leo.li
 * @brief
 */
#include <MyProject.h>


/** 
 * @brief        外部中断睡眠引脚设置
 * @date         2022-09-13
 */
void SleepMode_Init(void)
{
    #if ((LIN_PORTSELECTION == TXDGP00_RXDGP01) &&(LIN_TRANSCEIVER == EXTERNAL_LIN_DOUBLELINE))//配置唤醒口为GP01，外部中断0
        /*外部中断 INT0 接口选择
        000: P0.0
        001: P0.1
        010: P0.2
        011: P0.3
        100: P1.1
        101: P0.5
        110: P0.6*/
        ClrBit(LVSR, EXT0CFG2);             
        ClrBit(LVSR, EXT0CFG1);
        SetBit(LVSR, EXT0CFG0);
        ClrBit(P0_OE, P01);                     // 配置为输入
        SetBit(P0_PU, P01);                     // RXD注意是否有硬件上拉，可以配置软件上拉
        IF0 = 0; 
        IT01 = 1;
        IT00 = 0;                               // 00: posedge mode interrupt，01: negedge mode interrupt，1x: edge-change mode interrupt
        SetBit(IP0, PX01);
        SetBit(IP0, PX00);                      // 外部中断0中断优先级别3，中断优先级最高
        EX0 = 1;                                // 使能外部中断0, P01位于外部中断输入硬件
        EA = 1;                                 // 使能全局中断
    #endif
	#if ((LIN_PORTSELECTION == TXDGP10_RXDGP11) ||(LIN_TRANSCEIVER == INNER_LIN_SINGLELINE))//配置唤醒口为GP11，外部中断1
        ClrBit(P1_OE, P11);                     // 配置为输入
        ClrBit(P1_IF, P11);                     // clear P11 interrupt flag
        ClrBit(P1_IE, P11);                     // config P11 as the source of EXTI1
        IT11 = 1;
        IT10 = 0;                               // 00: posedge mode interrupt，01: negedge mode interrupt，1x: edge-change mode interrupt
        SetBit(IP0, PX11);
        SetBit(IP0, PX10);                      // 外部中断1中断优先级别3，中断优先级最高
        EX1 = 1;                                // 使能外部中断1, P11位于外部中断输入硬件
        SetBit(P1_IE, P11);                     // 休眠前使能外部中断
        EA = 1;                                 // 使能全局中断
	#endif
}
/**
 * @brief        将控制器调到睡眠模式，休眠电流小于100uA，需注意每一个IO的配置，否则会有漏电
 * @date         2022-07-14
 */
void sc_goToSleep(void) {

        SleepSet.SleepDelayCout++;
        if ((SleepSet.SleepDelayCout >= 100)&&(SleepSet.SleepFlag ==0)) //最大65530，若要再大，需改数据类型，100ms进入休眠
        {
            MOE     = 0;
                                    
            /******睡眠模式初始化**********/
            /* 1.休眠前配置IO口为外部中断 */
            SleepMode_Init();      
            SetBit(P4_OE, P43);
            GP43 = 0;               /* 外置收发器休眠管脚使能配置，收发器休眠 */
            /* 2.配置为外部中断后允许唤醒 */
            SleepSet.SleepDelayCout = 0;
            SleepSet.SleepFlag = 1;
            
            /* 3.关闭相应外设，配置其余IO引脚状态 */
            ClrBit(DRV_CR, FOCEN);              //关闭FOC
            ClrBit(ADC_CR, ADCEN);              //关闭ADC
            ClrBit(CCFG1, WDT_EN);              //关闭WDT
            //关闭运放
            ClrBit(AMP_CR0, AMP0EN);         //AMP0 Enable
            ClrBit(AMP_CR0, AMP1EN);         //AMP1 Enable
            ClrBit(AMP_CR0, AMP2EN);         //AMP2 Enable
            //关闭比较器
            ClrBit(CMP_CR1, CMP3EN);    //CMP3 Enable
            CMP_CR3 = 0;                //CMP_CR3  关闭Debug信号配置
            //关闭VREF
            SetBit(VREF_VHALF_CR, 0x00);

            
//            /*- WLK175-PCB02-1产品板，没有引出的IO配置上拉，引出IO每个都需要有固定状态—-*/
//            P0_PU = P00 | P01 | P02 | P03 | P04 | P05 | P06| P07; // 需确认这些端口能接受上拉
//            
//            /*-  P10和P11为开漏硬件上需要根据实际加上拉,P11配置为外部中断引脚，不需要再配置 -*/
//            P1_OE =  P10 | P12 | P13 | P14 | P15 | P15 | P17;
//            GP10 =0; GP12 =0; GP13 = 0; GP14 =0;GP15 = 0; GP16 =0;GP17 =0;
//            SetBit(P1_PU, P11HV_EN);  // P11 高压输入模式，保持使能
//            SetBit(P1_PU, P10HV_EN);  // P10 高压输入模式，保持使能            

//            P2_PU = P20 | P21| P24;
//            P2_OE = P22 | P23 | P25| P26| P27;
//            GP22 = 0; GP23 = 0; GP25 = 0; GP26 = 0; GP27 = 0; 
//            
//            P3_PU = P34 | P35| P36| P37;                         // 需确认这些端口能接受上拉
//            P3_OE = P30 | P31| P32| P33;
//            GP30 = 0; GP31 = 0;GP32 = 0;GP33 = 0;
//            
//            P4_PU = P44 | P45|P46;                                // 需确认这些端口能接受上拉
//            P4_OE = P43;  
//            GP43 = 0;
                              
            #if (1)
            /*- FU6866Q1_V2.0Demo板配置：悬空引脚配置为上拉，反电动势配置为输出0，输入检测引脚配置为输出0—-*/
            /*- 没有引出的IO配置上拉，引出IO每个都需要有固定状态—-*/
            P2_OE = P20| P21 | P22 | P23 | P24 | P25 | P26 | P27;
            GP21 = 0;GP21 = 0; GP22 = 0; GP23 = 0; GP24 = 0;GP25 = 0; GP26 = 0;  GP27 = 0;
            
            P3_OE = P30 | P31 | P32 | P34 | P35;
            P3_PU = P33 | P34;            // 需确认这些端口能接受上拉
            GP30 = 0; GP31 = 0; GP32 = 0; GP34 = 0; GP35 = 0;       
            
             /*-  P01,P00口为LIN口，不再配置 -*/
            P0_OE = P02 | P03 | P04 | P05 | P06 | P07;
            GP02 = 1; GP03 = 1; GP04 = 1;   GP05 = 1 ;GP06 = 1;GP07 = 1;       
            
            /*-  P10和P11为开漏硬件上需要根据实际加上拉 -*/
            P1_OE = P10  | P12 | P13 | P14 | P15 | P16 | P17;
            GP10 =0; GP12 =0; GP13 = 0; GP14 =0; GP15 = 0; GP16 =0; GP17 = 0; //反电动势端口需要输出为0，1会漏电
            SetBit(P1_PU, P11HV_EN);  // P11 高压输入模式，保持使能
            SetBit(P1_PU, P10HV_EN);  // P10 高压输入模式，保持使能
            
            P4_OE = P44;
            P4_PU = P41 | P42 | P45 | P46;
            GP44 =0;                                                
            /*- Demo板配置：悬空引脚配置为上拉，反电动势配置为输出0，输入检测引脚配置为输出0—-*/  
            #endif  
            
            /* 4.休眠 */
            SetBit(CCFG1, VBB_DIS);          // 关闭HVIC供电，即VDRV引脚的供电，VDD5不会关闭
            SetBit(PCON, STOP);              // MCU睡眠使能

        }  
}