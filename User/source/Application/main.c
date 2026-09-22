/**
 * @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      main.c
 * @author    Fortiortech  Appliction Team
 * @since     Create:2021-05-14
 * @date      Last modify:2022-07-14
 * @note      Last modify author is Kris.huang
 * @brief     包含主函数，软硬件初始化函数，VRER配置函数,Debug小工具
 */


/********************************************************************************
    Header Definition
********************************************************************************/
#include <MyProject.h>
#include <SanityCheck.h>

uint8 data g_1mTick = 0;  ///< 1ms滴答信号，每隔1ms在SYSTICK定时器被置1，需在大循环使用处清零


/**
    @brief        Function Definition
*/
void HardwareInit(void);
void SoftwareInit(void);
void VREFConfigInit(void);
static void DebugSet(void);


/**
 * @brief        SPI Debug调试小工具
 * @date         2022-07-14
 */
static void DebugSet(void)
{
    #if (DBG_MODE ==  DBG_SPI_HW)      // 硬件调试模式
    SPI_Init();
    Set_DBG_DMA(&HARD_SPIDATA);
    #elif (DBG_MODE ==  DBG_SPI_SW)   // 软件调试模式
    SPI_Init();
    Set_DBG_DMA(spidebug);
    #elif (DBG_MODE ==  DBG_UART)
    UART2_Init();
    Conf_DMA(1, XDATA_UART1, (uint16)GetAddr_UARTDBG(), BUFFLEN);
	#elif (DBG_MODE ==  DBG_UART)
		_nop_;
    #endif
	
	    /*******************Observation signal Select*********************/
    SetReg(CMP_CR3, DBGSEL0 | DBGSEL1, GP01_DBG_Conf);
}

/**
 * @brief     硬件初始化，初始化需要使用的硬件设备配置，FOC必须配置的是运放电压、运放初始化、ADC初始化、Driver初始化
 *            其他的可根据实际需求加。
 * @date      2022-07-14
 */
void HardwareInit(void)
{
    // 为提高芯片的抗干扰能力，降低芯片功耗，请在具体项目时，将不需要用的GPIO默认都配置为输入上拉。
    //具体配置可在GPIO_Default_Init设置。
    //GPIO_Default_Init();
    /************************硬件外设初始化**************************/
    /* ADC参考电压电压配置 */
    VREFConfigInit();
    /*  硬件过流，比较器初始化，用于硬件过流比较保护  */
    CMP3_Init();
    /*  看门狗初始化 */
    WatchDogConfig(500);  
    /*  GPIO初始化 */    
    GPIO_Init();
    /* 先ADC初始化，后Driver初始化 */	
	  ADC_Init();
    Driver_Init();
    /* 运放AMP初始化 */	
    AMP_Init();
    /* TIM3初始化，用于信号PWM捕获 */	
    #if (SPEED_MODE == PWMMODE)
    Timer3_Init();
    #endif
    /* TSD温度传感器初始化 */
    TSD_Init();
    /*  比较器中断配置，与CMP3硬件配置间隔一段时间等待硬件稳定  */
    CMP3_Interrupt_Init();
    /* LIN初始化 */
    #if (SPEED_MODE == LINMODE)
    LIN_Init();
    #endif

	
	/*----- Can初始化 -----*/
	//CAN_Init();
    /*  SYSTICK定时器配置  */
    ClrBit(IP2, PSYSTICK1);    //1ms定时中断优先级别为1
    SetBit(IP2, PSYSTICK0);
    SetBit(DRV_SR, SYSTIE);
    EA = 1;
}



/**
 * @brief        部分变量初始化，上电运行一次
 * @date         2022-07-14
 */
void SoftwareInit(void)
{
    MotorcontrolInit();
	VariablesPreInit();                           // 电机相关变量初始化
    mcState         = mcReady;
    mcFaultSource   = FaultNoSource;
}


/**
 * @brief        参考电压，偏置电压配置
 * @date         2022-07-14
 */
void VREFConfigInit(void)
{
    /************************VREF&VHALF Config************************/
    #if (HW_ADC_VREF == VREF3_0)
    {
        SetBit(VREF_VHALF_CR, VRVSEL1);             //00-->4.5V   01-->VDD5
        ClrBit(VREF_VHALF_CR, VRVSEL0);             //10-->3.0V   11-->4.0V
    }
    #elif (HW_ADC_VREF == VREF4_0)
    {
        SetBit(VREF_VHALF_CR, VRVSEL1);             //00-->4.5V   01-->VDD5
        SetBit(VREF_VHALF_CR, VRVSEL0);             //10-->3.0V   11-->4.0V
    }
    #elif (HW_ADC_VREF == VREF4_5)
    {
        ClrBit(VREF_VHALF_CR, VRVSEL1);             //00-->4.5V   01-->VDD5
        ClrBit(VREF_VHALF_CR, VRVSEL0);             //10-->3.0V   11-->4.0V
    }
    #elif (HW_ADC_VREF == VREF5_0)
    {
        ClrBit(VREF_VHALF_CR, VRVSEL1);             //00-->4.5V   01-->VDD5
        SetBit(VREF_VHALF_CR, VRVSEL0);             //10-->3.0V   11-->4.0V
    }
    #endif
    // VHALF电压配置 00:1/8VREF  01:1/4VREF 10:25/64VREF 11:1/2VREF(default)
    #if (HW_VHALF_SEL == VHALF1_8)
    ClrBit(VREF_VHALF_CR, VHALFSEL1);
    ClrBit(VREF_VHALF_CR, VHALFSEL0);
    #elif (HW_VHALF_SEL == VHALF1_4)
    ClrBit(VREF_VHALF_CR, VHALFSEL1);
    SetBit(VREF_VHALF_CR, VHALFSEL0);
    #elif (HW_VHALF_SEL == VHALF25_64)
    {
        SetBit(VREF_VHALF_CR, VHALFSEL1);
        ClrBit(VREF_VHALF_CR, VHALFSEL0);
    }
    #elif (HW_VHALF_SEL == VHALF1_2)
    {
        SetBit(VREF_VHALF_CR, VHALFSEL1);
        SetBit(VREF_VHALF_CR, VHALFSEL0);
    }
    #else
    {
#error " VHALF MODE Err "
    }
    #endif
    #if (VREF_OUT_EN)
    SetBit(P3_AN, P35);                         //VREF Voltage -->P35 Output 是否输出到P35引脚，需同步配置输出
    SetBit(P3_OE, P35);                         //VREF Voltage -->P35 Output 是否输出到P35引脚
    #endif
    #if (VHALF_OUT_EN)
    SetBit(P3_AN, P32);
    #endif
    SetBit(VREF_VHALF_CR, VREFEN | VHALFEN);    //VREF_VHALF_CR = 0x11;
    /* ---------------------------------------------------------------------- */
}



/**
 * @brief        主函数，大循环运行偏置电流采集函数，电机状态机控制函数，以及环路响应函数
 * @date         2022-07-14
 */
void main(void)
{
    uint16 PowerUpCnt = 0;
    
    /* ----- 上电空指令延时 等待系统稳定 ----- */
    for (PowerUpCnt = 0; PowerUpCnt < SystemPowerUpTime; PowerUpCnt++);
    
    /* ----- 部分变量初始化 ----- */
    SoftwareInit();
	
    /* ----- 硬件初始化，配置MCU外设 ----- */
    HardwareInit();
    /* ----- debug配置(SPI调试)，量产程序可以删除 ----- */
    DebugSet();
    
    while (1)
    {
        /* -----获取电流采样偏置电压----- */
        if (!mcCurOffset.OffsetFlag)
        {
            GetCurrentOffset();
        }
        else
        {
            /* -----电机控制状态机----- */
            MC_Control();
        }
        
        /* -----1ms处理函数----- */
        if (g_1mTick)
        {
            GP12 = ~GP12;       //FICD,下载口复用为GPIO测试引脚 
            WatchDogRefresh();  //喂狗,若不喂狗则MCU复位
            TickCycle_1ms();
            g_1mTick = 0;
        }
    }
}
