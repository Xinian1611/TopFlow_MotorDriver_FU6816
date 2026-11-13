; param.a51 - IAP parameter page at 0x1700
; station 4 = drive board (change the 6th data byte + XOR for other station)
		NAME    IAP_PARAM
        CSEG    AT      1700H
        DB      55H,0AAH,12H,34H,0D9H,0FFH,0FFH,0FFH
        DB      55H,0AAH,00H,00H,00H,40H,0BFH,0FFH
        END