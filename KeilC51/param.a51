; param.a51 - IAP parameter page at 0x1700
; station 1 = drive motor (change the 6th data byte + XOR for other station)
		NAME    IAP_PARAM
        CSEG    AT      1700H
        DB      55H,0AAH,12H,34H,0D9H,0FFH,0FFH,0FFH
        DB      55H,0AAH,00H,00H,00H,10H,0EFH,0FFH
        END