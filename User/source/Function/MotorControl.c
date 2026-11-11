/**
 * @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      MotorControl.c
 * @author    Fortiortech  Appliction Team
 * @since     Create:2021-04-10
 * @date      Last modify:2022-07-14
 * @note      Last modify author is Kris.huang
 * @brief
 */


#include <MyProject.h>
/* Private variables ----------------------------------------------------------------------------*/

MotStateType  data mcState;
MotStaM     McStaSet;


/**
 * @brief     电机控制状态机
 * @warning   电机的状态只能在电机状态控制中切换，禁止在其他地方切换电机状态
 * @date      2022-07-14
 */
void MC_Control(void)
{
    if (mcFaultSource != FaultNoSource) // 优先检查是否有错误，若有错误则跳转至错误状态
	{
		mcState = mcFault;
	}
    switch (mcState)
    {
        case mcReady:
            Motor_Ready();
            if ((mcCurOffset.OffsetFlag == 1) && (isCtrlPowOn == true))
            {
                mcState = mcInit;
                mcCurOffset.OffsetFlag  = 0;//每次启动时重新采集一次偏置电压，受温度的影响，高低温硬件偏置会有所变化
                mcCurOffset.OffsetCount = 0;
            }
            else
            {
            }
            
            break;
            
        case mcInit:
            if (isCtrlPowOn == false)
            {
                mcState = mcStop;
            }
            else if (mcCurOffset.OffsetFlag == 1)
            {
                Motor_Init();
                #if (CHARGE_EN == Enable)
                mcFocCtrl.State_Count   = CHARGE_TIME;
                mcState                 = mcCharge;             // 跳入mcCharge状态
                #else
                {
                    #if (TAILWIND_MODE == NoTailWind)
                    {
                        #if (PosCheckEnable == Enable)
                        mcState                             = mcPosiCheck;
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
                
                        mcState                             = mcAlign;
                        mcFocCtrl.State_Count               = Align_Time;
                        #else
                        mcState = mcStart;
                        #endif
                    }
                    #else
                    mcFocCtrl.State_Count   = TAILWIND_TIME;                                     //顺逆风判断时间
                    mcState                 = mcTailWind;
                    #endif
                }
                #endif
            }
            
            break;
#if (CHARGE_EN == Enable)           
        case mcCharge:
            if (isCtrlPowOn == false)
            {
                mcState = mcStop;
            }
            else
            {
                Motor_Charge();
                #if (IPMTEST ==Enable)
                {;}
                #else
                
                if (mcFocCtrl.State_Count == 0)
                {
                    MOE = 0;                              // 关闭输出
                    #if (TAILWIND_MODE == NoTailWind)
                    #if (PosCheckEnable == Enable)
                    mcState                             = mcPosiCheck;
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
                
                    mcState                             = mcAlign;
                    mcFocCtrl.State_Count               = Align_Time;
                    #else
                    mcState = mcStart;
                    #endif
                    #else
                    mcFocCtrl.State_Count   = TAILWIND_TIME;                                     //顺逆风判断时间
                    mcState                 = mcTailWind;
                    #endif
                }
                
                #endif
            }
            
            break;
#endif
#if (TAILWIND_MODE != NoTailWind)
            
        case mcTailWind:
            if (isCtrlPowOn == false)
            {
                mcState = mcStop;
            }
            else
            {
                Motor_TailWind();
            }
            
            break;
#endif
#if (PosCheckEnable == Enable)          
        case mcPosiCheck:
            if (isCtrlPowOn == false)
            {
                mcState = mcStop;
            }
            else
            {
                if (mcFocCtrl.State_Count == 0)
                {
                    mcFocCtrl.mcPosCheckAngle   = 0;
                    mcState                     = mcAlign;
                    mcFocCtrl.State_Count       = Align_Time;
                }
            }
            
            break;
#endif
#if (ALIGN_MOME != ALIGN_DSIABLE)           
        case mcAlign:
            if (isCtrlPowOn == false)
            {
                mcState = mcStop;
            }
            else
            {
                Motor_Align();
                #if (ALIGN_MOME == ALIGN_TEST)
                FOC__UD      = mcFocCtrl.ud_align;
                #else
                
                if (mcFocCtrl.State_Count == 0)// && (mcFocCtrl.UqFlt >= 40))
                {
                    mcState = mcStart;
                }
                else
                {
                    FOC__UD = mcFocCtrl.ud_align;
                }
                
                #endif
            }
            
            break;
#endif
            
        case mcStart:
            
            if (mcFocCtrl.Start_Mode ==  TAILWIND_START)     // 顺风启动
            {
                #if (TAILWIND_MODE == BEMFMethod)
                mcState                     = mcRun;
                #elif (TAILWIND_MODE == RSDMethod) 
                mcState                     = mcRun;
				#endif
                mcFocCtrl.State_Count   = 0;
               

            }
            else if (mcFocCtrl.Start_Mode == HEADWIND_START) // 逆风启动
            {
                if (mcFocCtrl.State_Count > 0) // 逆风启动刹车过程
                {
                    // 配置刹车代码
                    MC_Break();
                }
                else
                {
                    mcFocCtrl.State_Count   = 50;            // 顺逆风判断时间
                    mcState                 = mcInit;        // 刹车结束 切回初始化状态
                    mcFocCtrl.Start_Mode = STATIC_START;
                }
            }
            else if (mcFocCtrl.Start_Mode == STATIC_START)// 静止启动
            {
								#if (Open_Start_Mode == Squ_Start)
								{
										mcState                     = mcSquStart;
										SquStart.StartState 				= 1;
								}
								#else
								{
										Motor_Static_Open();
										mcState                     = mcRun;
								}
								#endif
            }
            
            break;
            
					#if (Open_Start_Mode == Squ_Start)
				  {
				  case mcSquStart:
					  if (mcFaultSource != FaultNoSource)
            {
                mcState                 = mcFault;
            }
            else if (isCtrlPowOn == false)
            {
                mcState                 = mcStop;
            }
            else
            {
							SquStartInit();
            }
						break;
				 }
				#endif
						
        case mcRun:
            if (isCtrlPowOn == false)
            {
                mcState                 = mcStop;
            }
            else
            {
            }
            
            break;
            
        case mcStop:
              MC_Stop();
        
            break;
            
        case mcBrake:
            if (mcFocCtrl.State_Count == 0)//等待mcStop刹车结束
            {
                MOE = 0;
                ClrBit(DRV_CR, FOCEN);
                mcState         = mcReady;
            }
            
            break;
            
        case mcFault:
            if (mcFaultSource == FaultNoSource)   //发生故障时进行保护关闭输出电机停机，直到故障码清除后，软件重新运行
            {
                mcState   = mcReady;
            }
            else
            {
                ClrBit(DRV_CR, FOCEN);  //FOC 关闭
                MOE     = 0;            // 六路桥臂主输出，关闭
            }
            
            break;
            
        default:
            mcState   = mcReady;
            break;
    }
}

