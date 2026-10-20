#ifndef __TIMER_H__
#define __TIMER_H__

#include <FU68xx_6.h>


#define TIM3_Fre                       (750000.0)                                // TIM0计数频率750KHz
//#define TIM4_Fre                       (3000.0)                                // TIM0计数频率750KHz
#define TIM4_Fre                       (6000.0)                                // TIM0计数频率750KHz

/*************************************************************************************///External Function
extern void Timer1_Init(void);
extern void Timer2_Init(void);
extern void Timer3_Init(void);
extern void Timer4_Init(void);
extern void TIM4_Init_RF(void);

#endif
