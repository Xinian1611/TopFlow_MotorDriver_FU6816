/**
 * @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      MyProject.h
 * @author    Fortiortech Appliction Team
 * @note      Last modify author is Kris.huang
 * @since     2017-12-27
 * @date      2022-07-14
 * @brief     This file contains all the common data types used for Motor Control.  
 *                      
 */


/* Define to prevent recursive inclusion --------------------------------------------------------*/
#ifndef __MYPROJECT_H_
#define __MYPROJECT_H_

/* Includes -------------------------------------------------------------------------------------*/
#include <FU68xx_6.h>

#include <definition.h>
#include <Customer.h>
#include <Parameter.h>
#include <Protect.h>
#include <Customer_Debug.h>

#include <AMP.h>
#include <GPIO.h>
#include <SPI.h>
#include <ADC.h>
#include <TSD.h>
#include <CMP.h>
#include <AMP.h>
#include <Driver.h>
#include <UART.h>
#include <Brake.h>      /*  新增：抱闸驱动 */
#include <CRC.h>
#include <GPIO.h>
#include <PIInit.h>
#include <TIMER.h>
#include <DMA.h>
#include <Lin.h>
#include <Can.h>
#include <CanSlave.h>       /*  新增：CAN 从机协议层（内含 CanProtocol.h → RingBuffer.h）*/
#include <FU68xx_6_Flash.h>
#include "WDT_INIT.h"
#include <MotorUserControl.h>   /* MotorRunState_t + MotorCtrl_* */
#include <DevConfig.h>          /* 88 字节配置 + EEPROM 持久化 */
#include <AlarmMonitor.h>       /* 12 路可配置告警 */
#include <MT6701.h>             /* MT6701 磁编码器（P0.5=SDA / P0.6=SCL，硬件I2C）*/
#include <PositionCtrl.h>       /* 位置环：g_pos_cmd_* / g_pos_state / 四个函数 */
#include <PositionProtect.h>    /* 位置环限位/堵转判断 */
#include <I2C.H>                /* 软件 I2C（P4.4=SCL / P0.4=SDA）*/
#include <EEPROM.h>             /* BL24C256A EEPROM（32KB，P0.7=WP）*/

#include <AddFunction.h>
#include <RSDDetect.h>
#include <MotorControlFunction.h>
#include <MotorControl.h>
#include <PosCheck.h>
#include <SMDU.h>
#include <BEMFDetect.h>
#include <SquStart.h>
#endif
