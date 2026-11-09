$NOMOD51

IDATALEN        EQU     256

XDATASTART      EQU     0
XDATALEN        EQU     3840

IBPSTACK        EQU     0
IBPSTACKTOP     EQU     IDATALEN

XBPSTACK        EQU     0
XBPSTACKTOP     EQU     XDATALEN
 
PADDR       EQU    1800H

ACC     DATA    0E0H
B       DATA    0F0H
SP      DATA    81H
DPL     DATA    82H
DPH     DATA    83H

                NAME    ?C_STARTUP
?C_C51STARTUP   SEGMENT   CODE
?STACK          SEGMENT   IDATA

                RSEG    ?STACK
                DS      1

                EXTRN CODE (?C_START)
                PUBLIC  ?C_STARTUP
 CSEG AT  0003H
     LJMP PADDR + 00003H
 CSEG AT  000BH
     LJMP PADDR + 0000BH
 CSEG AT  0013H
     LJMP PADDR + 00013H
 CSEG AT  001BH
     LJMP PADDR + 0001BH
 CSEG AT  0023H
     LJMP PADDR + 00023H
 CSEG AT  002BH
     LJMP PADDR + 0002BH
 CSEG AT  0033H
     LJMP PADDR + 00033H
 CSEG AT  003BH
     LJMP PADDR + 0003BH
 CSEG AT  0043H
     LJMP PADDR + 00043H
 CSEG AT  004BH
     LJMP PADDR + 0004BH
 CSEG AT  0053H
     LJMP PADDR + 00053H
 CSEG AT  005BH
     LJMP PADDR + 0005BH
 CSEG AT  0063H
     LJMP PADDR + 00063H
 CSEG AT  006BH
     LJMP PADDR + 0006BH
 CSEG AT  0073H
     LJMP PADDR + 00073H
 CSEG AT  007BH
     LJMP PADDR + 0007BH
    
                CSEG    AT      0
?C_STARTUP:     LJMP    STARTUP1

                RSEG    ?C_C51STARTUP
STARTUP1:
; ���� idata
IF IDATALEN <> 0
                MOV     R0,#IDATALEN - 1
                CLR     A
IDATALOOP:      MOV     @R0,A
                DJNZ    R0,IDATALOOP
ENDIF
; ���� Xdata ע��xdata�ĺ�32λ��оƬid�������оƬid�в��� ���ﲻ����������32λ
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

                MOV     SP,#?STACK-1      ; Keil 标准启动都有，厂商那份漏了
                LJMP    ?C_START

                END
