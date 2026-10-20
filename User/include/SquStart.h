/* --------------------------- (C) COPYRIGHT 2025 Fortiortech ShenZhen -----------------------------
    File Name      : SquStart.h
    Author         : Fortiortech  Appliction Team
    Version        : V1.0
    Date           : 2025-3-20
    Description    : This file contains motor parameter used for Start Process.
----------------------------------------------------------------------------------------------------
                                       All Rights Reserved
------------------------------------------------------------------------------------------------- */

/* Define to prevent recursive inclusion -------------------------------------------------------- */
#ifndef __SQUSTART_H_
#define __SQUSTART_H_

typedef struct
{		
		int16 MainCurrent1;
    int16 MainCurrent;              // 主电流
		int16 ACurrent1;             // 辅助电流1MainCurrent
		int16 ACurrent2;             // 辅助电流1
		int16 MainCurrentSum;
		int16 ACurrent1Sum;
		int16 ACurrent2Sum;	
		int16 MainCurrentSum1;
		int16 ACurrent1Sum1;
		int16 ACurrent2Sum1;	
		int16 MainCurrentSumMin;
		int16 ACurrent1SumMin;
		int16 ACurrent2SumMin;
		uint8 Pulse_Num;
		uint8 StartState;
		uint8 StartSector;
		uint8 PhaseCommutCount2;
		int16 ACurrent1Diff;
		int16 ACurrent2Diff;
		uint8 StartCWCount;
		uint16 ForcedStartCount;
		uint16 MotorSpeedCount;
		uint16 MotorSpeedCount1;
		uint16 MotorSpeedCount2;
		uint16 MotorSpeedCount3;
		int16 MotorSpeed;
		int16 Temp;
		int16 Temp1;
		int16 CurrentThreshold;
		uint8 UqReady;
		int16 StartUq;
		uint16 IgnoreCount;
		int16 SwitchSpeed;
		int16 StartCurrentThreshold;
		int16 ThetaOffset;
		uint8 MinUq;
		uint8 PWMFreqency;
		int16 RotorTheta;
		int16	ThetaRamp;
		uint16 FreqencyBase;
		uint8 FrocedTime;
}SquStartVar;

extern SquStartVar    			idata SquStart;
extern void   SquStartProcess(void);
extern void   CWPhaseCommutation(void);
extern void   CCWPhaseCommutation(void);
extern void   PhaseCommutationProcess1(void);
extern void   PhaseCommutationProcess2(void);
extern void   PhaseCommutationInit(void);
extern void 	SquTailWindProcess(void);
extern void 	TailWindPhaseCommutationProcess(void);
#endif