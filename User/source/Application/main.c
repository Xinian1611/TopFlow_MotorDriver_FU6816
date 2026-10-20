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
#include "MT6701.h"

uint8 data g_1mTick = 0;  ///< 1ms滴答信号，每隔1ms在SYSTICK定时器被置1，需在大循环使用处清零
static uint16 data g_ms      = 0;      /*  新增：1ms 计数 */
static uint16 data g_dbgLast = 0;      
static uint16 data g_ntcLast = 0;      /*  新增：NTC 温度换算节拍（20ms） */

static uint16 xdata g_canDbgLast = 0;   /* 【调试】CAN 打印节拍（放 xdata，data 只剩 ~17 字节）*/
static uint16 xdata g_mtLast     = 0;   /* MT6701 读取 + CAN 观测帧节拍 */
static uint16 xdata g_hbLast     = 0;   /* 【调试】串口心跳节拍（1s 一行）*/
static uint16 xdata g_posDbgLast = 0;   /* 位置环打印节拍：200ms 一行 */

static uint16 xdata g_tglLast = 0;   /* 双线翻转节拍 */
static uint8  xdata g_tglLvl  = 0;   /* 当前电平 */


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
    // UART2_Init();
    // Conf_DMA(1, XDATA_UART1, (uint16)GetAddr_UARTDBG(), BUFFLEN);
    UART1_Init();                       /*  改到 UART1：P0.5 = TXD / P0.6 = RXD。
                                           原 UART2 在 P3.6/P3.7，新板这两脚贴着外部晶振，不能用 */
    /*  Conf_DMA 在本工程里找不到实现（只有 FU68xx_6_DMA.h:44 的声明，DMA.c 里只有 Set_DBG_DMA），
       一旦打开 DBG_UART 很可能直接链接失败；先注掉，将来要用 DMA 调试再自己补实现 */
    // Conf_DMA(1, XDATA_UART1, (uint16)GetAddr_UARTDBG(), BUFFLEN);
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
    /* 抱闸初始化（P4.3 推挽输出 → 上电即合闸，断电抱闸是安全默认）*/
    Brake_Init();
    /* 比较器中断配置，与CMP3硬件配置间隔一段时间等待硬件稳定  */
    CMP3_Interrupt_Init();
    /* LIN初始化 */
    #if (SPEED_MODE == LINMODE)
    LIN_Init();
    #endif

	
	/*----- Can初始化 -----*/
	CAN_Init();
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
    /* 新增：本机 CAN 从机地址（29位扩展ID里的 系统号/主类别/副类别/设备号）
     *   注意：头板实际设备类别待确认 —— 初版是“动力驱动引擎(11) / 拖钓电机(15)”，
     *   头板若是“上下驱动板”，可能需要改成别的副类别，见 §4.2 待确认项。*/
    ExtSlaveConfig_t ext_cfg = {
        EXT_SYS_MAIN,              // 系统编号: 主系统
        EXT_CAT_POWER_ENGINE,      // 设备主类别: 动力驱动引擎(11)
        EXT_POWER_TROLL_MOTOR,     // 设备副类别: 无刷电机(15)
        MOTOR_DEV_NUM              // 设备编号: 1=转向 2=升降 3=折叠
    };

    /* MotorCtrl_Init() 必须在 ext_slave_init() 之前
     *   （前者初始化 g_run_state，后者不再管 g_run_state）*/
    MotorCtrl_Init();             // 内含 DevConfig_LoadParam() 读 EEPROM

    MotorcontrolInit();
	VariablesPreInit();                           // 电机相关变量初始化
    mcState         = mcReady;
    mcFaultSource   = FaultNoSource;

    /* CAN 协议层初始化 */
    ext_slave_init(&ext_cfg);

    /* 告警模块初始化（依赖 g_motor_cfg，所以放在 MotorCtrl_Init() 之后）*/
    AlarmMonitor_Init();
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

    /* EEPROM（P0.4=SDA / P4.4=SCL / P0.7=WP）初始化 + 在位检查 */
    (void)EEPROM_Init();
#if (DEBUG_UART_OR_CAN != 0)                 /* 调试关闭时这些 printf 整块不编译 */
    printf("EEPROM check=%u (0=OK)\r\n", (uint16)EEPROM_Check());
    printf("EEPROM selfcheck=%u (0=OK)\r\n", (uint16)EEPROM_SelfCheck());

    /* 上电打印：当前系统时钟到底是谁 */
    printf("OSC_CFG=%bx  sysclk=%s\r\n",
                    (uint8)OSC_CFG,
                    ReadBit(OSC_CFG, CK_FLAG) ? "EXTERNAL" : "INTERNAL");
#endif

    /* MT6701：位置走 AD3 模拟量（P2.5），这条路不碰串口脚 */
    MT6701_AnaInit();            /* 断言 P2.5=AD3 通道（ADC.c 里也配了）*/

    /* ===== MT6701 的 I2C 与调试串口抢同一对脚（P0.5=TXD/SDA、P0.6=RXD/SCL）=====
     * MT6701_Init() 会 ClrBit(PH_SEL, UART1EN)——一调 I2C 串口就哑。
     *   MT6701_ZERO == 0：不校准，只走 AD3 模拟量位置；
     *   MT6701_ZERO == 1：上电把当前位置写为零点（标定用）；
     *                     标定完必须改回 0，否则每次上电都会写零点/烧 EEPROM。
     * 注意：MT6701_ZERO=1 时不能同时用 DEBUG_UART_OR_CAN=1。
     * （AD3 模拟量位置、CAN 观测帧不受影响）*/
#if (MT6701_ZERO)
#if (DEBUG_UART_OR_CAN == 1)
#error "MT6701_ZERO=1 conflicts with DEBUG_UART_OR_CAN=1 (P0.5/P0.6 shared by UART1 and I2C)"
#endif

    MT6701_Init();                 /* I2C 引脚/外设：P0.5=SDA P0.6=SCL，I2C_FS=1 */
    MT6701_Probe();                /* I2C 自检 -> g_mt6701_probe（0=从机应答）*/
    MT6701_InitSequence();         /* 当前位置写零点 + 烧 EEPROM（标定用）*/

#if (DEBUG_UART_OR_CAN == 2)
    MT6701_BootDump();             /* 标定结果发 CAN：0x1F0/0x1F1/0x1F3 */
#endif
#endif


#if 0
    /* ----- 【调试】CAN 上电自检（只跑一次：读 CAN 引脚电平 + 自发自收）----- */
    CAN_SelfTest();

    /* ----- 【调试】CAN 引脚/收发器连通性探针（只跑一次）----- */
    CAN_PinProbe();
#endif
    
    while (1)
    {
        /* 【新增】CAN"写配置"报文请求的落盘动作，放主循环执行：
         *   写 96 字节 EEPROM 约 50ms，放在 1ms 任务里会把节拍卡停摆；
         *   主循环本来就是"空转等 1ms"，在这里阻塞不影响电机保护 */
        ext_slave_cfg_save_pending();

#if (DEBUG_UART_OR_CAN != 0)
        /* 电压/温度报警事件：1ms任务只置位，这里打印，避免 printf 阻塞 1ms 节拍 */
        if (g_alarm_evt != ALARM_EVT_NONE)
        {
            switch (g_alarm_evt)
            {
            case ALARM_EVT_VOLT_HIGH:
                printf("ALARM: OVP HIGH %u.%u%uV (limit %u.%u%uV)\r\n",
                       (uint16_t)(g_alarm_evt_value / 100),
                       (uint16_t)((g_alarm_evt_value / 10) % 10),
                       (uint16_t)(g_alarm_evt_value % 10),
                       (uint16_t)(g_alarm_evt_limit / 100),
                       (uint16_t)((g_alarm_evt_limit / 10) % 10),
                       (uint16_t)(g_alarm_evt_limit % 10));
                break;

            case ALARM_EVT_VOLT_LOW:
                printf("ALARM: OVP LOW %u.%u%uV (limit %u.%u%uV)\r\n",
                       (uint16_t)(g_alarm_evt_value / 100),
                       (uint16_t)((g_alarm_evt_value / 10) % 10),
                       (uint16_t)(g_alarm_evt_value % 10),
                       (uint16_t)(g_alarm_evt_limit / 100),
                       (uint16_t)((g_alarm_evt_limit / 10) % 10),
                       (uint16_t)(g_alarm_evt_limit % 10));
                break;

            case ALARM_EVT_TEMP_HIGH:
                printf("ALARM: NTC HIGH %u.%uC (limit %u.%uC)\r\n",
                       (uint16_t)(g_alarm_evt_value / 10),
                       (uint16_t)(g_alarm_evt_value % 10),
                       (uint16_t)(g_alarm_evt_limit / 10),
                       (uint16_t)(g_alarm_evt_limit % 10));
                break;

            default:
                break;
            }
            g_alarm_evt = ALARM_EVT_NONE;
        }
#endif

#if (DEBUG_UART_OR_CAN != 0)                 /* 原先是无条件 printf，模式0 下必须去掉 */
        /* 【调试】心跳：每秒一行。用来区分"板子在复位"（计数从头开始）和"主循环卡死"（计数停住）*/
        if ((uint16)(g_ms - g_hbLast) >= 1000)
        {
            g_hbLast = g_ms;
            printf("alive ms=%u\r\n", (uint16)g_ms);
        }
#endif

#if (DEBUG_UART_OR_CAN != 0)
        // if ((uint16)(g_ms - g_dbgLast) >= 100)          /*  100ms 一行，别更密 */
        // {
        //     g_dbgLast = g_ms;
        //     printf("st=%d cm=%d th=%d eth=%d cnt=%d spd=%d ref=%d iqref=%d iq=%d id=%d IA=%d IB=%d IC=%d MOE=%d h=%d CR1=%x\r\n",
        //     (uint16)mcState,               /* st    : 电机状态机 */
        //     (uint16)mcFocCtrl.CtrlMode,    /* cm    : 0=开环电流环 1=速度环 */
        //     (int16)FOC__THETA,             /* th    : 当前角度 Q15 */
        //     (int16)FOC__ETHETA,            /* eth   : 估算角度 Q15 */
        //     (uint16)g_ms,                  /* cnt   : 1ms 计数（运行节拍，见下方说明） */
        //     (int16)mcFocCtrl.SpeedFlt,     /* spd   : 滤波后转速 Q15 */
        //     (int16)mcFocCtrl.Ref,          /* ref   : 速度给定 Q15 */
        //     (int16)mcFocCtrl.IqRef,             /* iqref : 电流环 Iq 给定 */
        //     (int16)FOC__IQ,                /* iq    : 反馈 Iq */
        //     (int16)FOC__ID,                /* id    : 反馈 Id */
        //     (int16)FOC__IA,                /* IA    : A 相电流 */
        //     (int16)FOC__IB,                /* IB    : B 相电流 */
        //     (int16)FOC__IC,                /* IC    : C 相电流 */
        //     (uint16)MOE,                   /* MOE   : 主输出使能 */
        //     (uint16)g_runBlocked,          /* h     : 1=启动闸门还关着 */
        //     (uint16)FOC_CR1);              /* CR1   : FOC 控制寄存器1（bit6=EFAE 强制角度） */
        // }
#endif
#if (DEBUG_UART_OR_CAN != 0)
        // /* 位置环调试：200ms 一行；printf 阻塞，只能放主循环 */
        // if ((uint16)(g_ms - g_posDbgLast) >= 200)
        // {
        //     int16 spdCmd;
        //     int16 spdAct;

        //     g_posDbgLast = g_ms;

        //     /* 位置环输出：g_run_state.ref 是幅值，方向在 direction 里 */
        //     spdCmd = (int16)((g_run_state.direction == CCW) ? -g_run_state.ref : g_run_state.ref);

        //     /* SpeedFlt 是 Q15，换回 rpm */
        //     spdAct = (int16)((float)mcFocCtrl.SpeedFlt / 32767.0f * MOTOR_SPEED_BASE);

        //     printf("POS tgt=%u fb=%u ang10=%u spd=%d refq=%d act=%d st=%u run=%u wm=%u ctrl=%u es=%u\r\n",
        //             (uint16)g_pos_cmd_raw,
        //             (uint16)g_pos_fb_raw,
        //             (uint16)MT6701_GetAngle10(),
        //             spdCmd,
        //             (int16)mcFocCtrl.Ref,
        //             spdAct,
        //             (uint16)g_pos_state,
        //             (uint16)g_pos_cmd_start,
        //             (uint16)g_run_state.work_mode,     /* 3=POSITION，其它值就不会调 Update */
        //             (uint16)g_dbg_lastCtrl,            /* 最近一次真正进入解析的 ctrl 字节 */
        //             (uint16)g_run_state.is_estop);
        // }
#endif
        /* ----- 【调试】CAN 状态，每 500ms 一组（受 DEBUG_UART_OR_CAN 控制：
         *        串口调试期间关掉，否则刷屏把别的输出冲掉）----- */
#if (DEBUG_UART_OR_CAN != 0)
        if ((uint16)(g_ms - g_canDbgLast) >= 500)
        {
            g_canDbgLast = g_ms;
            // CAN_DebugPrint();           /* CAN.c：寄存器/TEC/REC + 收发计数 + 最近一帧 */
            // ext_slave_debug_print();    /* CanSlave.c：协议层计数 + 发送队列 */
        }
#endif
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
            Brake_Update();     //  新增：抱闸检测/状态机（内部按 10ms 分频）

            /* MT6701 位置：每 1ms 采一点，攒够 MT6701_ANA_POINTS 点出一批并上报 */
            if (MT6701_AnaPush((uint16)(ADC3_DR & 0x7FFF)))
            {
#if (DEBUG_UART_OR_CAN == 2)
                MT6701_SendObs();   /* 0x1F2 观测帧：只有 CAN 调试模式才发 */
#endif
            }

            /* 新增：NTC 温度换算，20ms 一次*/
            if ((uint16)(g_ms - g_ntcLast) >= 20)
            {
                g_ntcLast = g_ms;
                Motor_MD_Sample();
            }

            g_1mTick = 0;
            g_ms++;             //  新增：1ms 计数（没有它 g_ms 永远是 0，打印就不会触发）
            // GP00 = ~GP00;               /* 或者放到 1ms 里翻转 */
        }
    }
}
