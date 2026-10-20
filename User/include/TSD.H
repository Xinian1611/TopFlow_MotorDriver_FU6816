#ifndef __TSD_H__
#define __TSD_H__


#include <FU68xx_6.h>

/*************************************************************************************///External Function
extern void TSD_Init(void);                
extern uint8 TSD_Gain(void);               

#define NTC_TEMP_ERR        (int16)9999

/* ====================  新增：驱动模块温度换算（NTC，P2.4 = AD2）====================
 *  分压网络：  +5V ──[ NTC ]──┬── P2.4(AD2)          （与旧 GD32 板电路相同）
 *                            └──[ 10k ]── GND
 *  换算见 TSD.c 的 Motor_MD()，函数体与旧工程一字不差。
 *  本板只有一路 NTC（旧工程有 ZXWD/ZDWD 两路），所以 Motor_MD() 不再带通道参数。
 * ============================================================================ */
#define MOTOR_MD_ERR        (-1000.0f)  /* NTC 读数不可信（开路/饱和）时的返回值 */

extern float Motor_MD(uint16 adc_filt_counts);
extern void  Motor_MD_Sample(void);

/* ====================  新增：NTC 原值的 50 点截尾平均 ====================
 *  TickCycle_1ms 里每 1ms 喂一个点（此时 ADC2_DR 刚被顺序采样刷新），
 *  满 50 点（=50ms）自动算一次：去掉最大的 10 个、最小的 10 个，取中间 30 个的平均。
 *  NTC_TrimValue() 返回 8 位尺度结果（对应 ADC2_DR[14:7]）；
 *  未满第一批时返回 0，调用方用原始值兜底。
 *   注意：文档写的是"中值滤波"，但描述是截尾平均(trimmed mean)，这里按描述实现。
 */
extern void   NTC_TrimPush(uint8 v);
extern uint16 NTC_TrimValue(void);

#endif