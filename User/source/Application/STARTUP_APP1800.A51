;=============================================================================
; 文件：STARTUP_APP1800.A51
; 作用：APP 运行在 0x1800 时的启动文件（复位入口 + 中断向量表）
;
; 背景（为什么要这个文件）：
;   8051 的复位/中断向量是硬件固定在 0x0000/0x0003/0x000B ... 的，不能改。
;   APP 搬到 0x1800 以后，0x0000~0x17FF 属于 Bootloader：
;     - 0x0000        -> Bootloader 的复位入口（LJMP 到 boot 代码）
;     - 0x0003~0x007B -> Bootloader 的向量表，内容是 LJMP 1800H+偏移
;   所以 APP 必须：
;     1) 把复位入口放在 0x1800（本文件 ?C_STARTUP，且必须 PUBLIC，
;        因为 main.obj 里对 ?C_STARTUP 有外部引用，不 PUBLIC 就报 L127）；
;     2) 把中断向量表放在 0x1803~0x187B，每个向量 LJMP 到本工程的 C ISR。
;   C51 编译器默认会把向量表生成在 0x0003，所以工程里必须加编译开关
;   NOINTVECTOR（Options for Target -> C51 -> Misc controls），由本文件负责向量表。
;
; 中断映射（与当前工程实测一致，来自 map + hex 反查）：
;   INT0  0x1803  LVW_TSD_INT
;   INT1  0x180B  ISR_DEFAULT（未使用）
;   INT2  0x1813  EXTERN1_INT
;   INT3  0x181B  DRV_ISR        (FOC/PWM 主中断)
;   INT4  0x1823  ISR_DEFAULT
;   INT5  0x182B  ISR_DEFAULT
;   INT6  0x1833  ISR_DEFAULT
;   INT7  0x183B  ISR_DEFAULT    (CMP012_INT 未参与编译)
;   INT8  0x1843  CAN_INT
;   INT9  0x184B  TIM3_INT
;   INT10 0x1853  SYSTICK_INT    (1ms 调度)
;   INT11 0x185B  ISR_DEFAULT
;   INT12 0x1863  CMP3_INT
;   INT13 0x186B  ISR_DEFAULT
;   INT14 0x1873  ISR_DEFAULT
;   INT15 0x187B  ISR_DEFAULT
;
; 【维护提醒】
;   以后新增/删除 interrupt 函数，必须同步改向量表：
;   - 新增：把对应槽位的 ISR_DEFAULT 换成 LJMP 你的ISR，并在 EXTRN 里补名字；
;   - 删除：改回 LJMP ISR_DEFAULT，并去掉 EXTRN 里的名字；
;   否则该中断会被静默丢弃（进 ISR_DEFAULT 直接 RETI），现象是"时好时坏"。
;
; 启动流程（清 IDATA / 清 XDATA / 设 SP / 跳 ?C_START）与原
;   FU68xx_Hardware_Driver\Source\STARTUP_FU6816.A51 完全一致，未做任何改动。
;=============================================================================
$NOMOD51

; ---- 内存清零长度（与原厂商启动文件保持一致，不要随意改）-------------------
IDATALEN        EQU     256             ; 内部 RAM(idata) 清零字节数
XDATASTART      EQU     0               ; 外部 RAM(xdata) 清零起始
XDATALEN        EQU     4096            ; 外部 RAM 清零字节数（与原文件一致）

IBPSTACK        EQU     0
IBPSTACKTOP     EQU     IDATALEN
XBPSTACK        EQU     0
XBPSTACKTOP     EQU     XDATALEN

; ---- 8051 特殊功能寄存器 ---------------------------------------------------
ACC     DATA    0E0H
B       DATA    0F0H
SP      DATA    81H
DPL     DATA    82H
DPH     DATA    83H

                NAME    ?C_STARTUP
?C_C51STARTUP   SEGMENT CODE
?STACK          SEGMENT IDATA

                RSEG    ?STACK
                DS      1               ; 保留 1 字节，SP 初值 = ?STACK-1

                EXTRN CODE (?C_START)

; 这一行必须有：?C_STARTUP 是链接器要用的全局符号（main.obj 有引用）
                PUBLIC  ?C_STARTUP

; C 语言中断服务函数（名字必须与 C 源文件里的函数名一致，C51 默认输出大写）
                EXTRN CODE (LVW_TSD_INT, EXTERN1_INT, DRV_ISR, CAN_INT)
                EXTRN CODE (TIM3_INT, SYSTICK_INT, CMP3_INT)

;=============================================================================
; 复位入口 + 中断向量表（整体重定位到 0x1800 起）
;   0x1800       : LJMP STARTUP1
;   0x1803 + 8*n : LJMP <INT n 的服务函数>
;   每条必须是 LJMP(0x02)：boot 跳转前会检查 0x1800 的首字节是不是 0x02
;=============================================================================
                CSEG    AT      1800H
?C_STARTUP:     LJMP    STARTUP1

                CSEG    AT      1803H   ; INT0  LVW / TSD 中断
                LJMP    LVW_TSD_INT

                CSEG    AT      180BH   ; INT1  未使用
                LJMP    ISR_DEFAULT

                CSEG    AT      1813H   ; INT2  外部中断1
                LJMP    EXTERN1_INT

                CSEG    AT      181BH   ; INT3  DRV（FOC/PWM 主中断）
                LJMP    DRV_ISR

                CSEG    AT      1823H   ; INT4  TIM2  未使用
                LJMP    ISR_DEFAULT

                CSEG    AT      182BH   ; INT5  未使用
                LJMP    ISR_DEFAULT

                CSEG    AT      1833H   ; INT6  未使用
                LJMP    ISR_DEFAULT

                CSEG    AT      183BH   ; INT7  CMP012 未参与编译
                LJMP    ISR_DEFAULT

                CSEG    AT      1843H   ; INT8  CAN
                LJMP    CAN_INT

                CSEG    AT      184BH   ; INT9  TIM3
                LJMP    TIM3_INT

                CSEG    AT      1853H   ; INT10 SysTick（1ms 调度）
                LJMP    SYSTICK_INT

                CSEG    AT      185BH   ; INT11 未使用
                LJMP    ISR_DEFAULT

                CSEG    AT      1863H   ; INT12 CMP3
                LJMP    CMP3_INT

                CSEG    AT      186BH   ; INT13 未使用
                LJMP    ISR_DEFAULT

                CSEG    AT      1873H   ; INT14 未使用
                LJMP    ISR_DEFAULT

                CSEG    AT      187BH   ; INT15 未使用
                LJMP    ISR_DEFAULT

;=============================================================================
; 未使用中断的兜底处理：直接 RETI（防止误触发时跑飞）
;=============================================================================
                RSEG    ?C_C51STARTUP
ISR_DEFAULT:    RETI

;=============================================================================
; 标准启动流程：清 IDATA -> 清 XDATA -> 设栈顶 -> 跳 C 语言入口
;=============================================================================
STARTUP1:

IF IDATALEN <> 0
                MOV     R0,#IDATALEN - 1
                CLR     A
IDATALOOP:      MOV     @R0,A
                DJNZ    R0,IDATALOOP
ENDIF

IF XDATALEN <> 0
                MOV     DPTR,#XDATASTART
                MOV     R7,#LOW(XDATALEN)
  IF (LOW (XDATALEN)) <> 0
                MOV     R6,#HIGH(XDATALEN) +1
  ELSE
                MOV     R6,#HIGH(XDATALEN)
  ENDIF
                CLR     A
XDATALOOP:      MOVX    @DPTR,A
                INC     DPTR
                DJNZ    R7,XDATALOOP
                DJNZ    R6,XDATALOOP
ENDIF

IF IBPSTACK <> 0
                EXTRN DATA (?C_IBP)
                MOV     ?C_IBP,#LOW IBPSTACKTOP
ENDIF

IF XBPSTACK <> 0
                EXTRN DATA (?C_XBP)
                MOV     ?C_XBP,#HIGH XBPSTACKTOP
                MOV     ?C_XBP+1,#LOW XBPSTACKTOP
ENDIF

                MOV     SP,#?STACK-1    ; 设栈顶（boot 是 LCALL 跳进来，这里会把返回地址丢掉，正常）
                LJMP    ?C_START

                END
