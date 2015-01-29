.set SUBISA, 0b00010

;-------------------------------------------------------------------------------
; table defining availability of instructions for various ISA subsets
;-------------------------------------------------------------------------------

; mnemonic           |  core  |
; ------------------ | ------ |
.set ISA_MSK_NOP     , 0b11111
.set ISA_MSK         , 0b00000
.set ISA_MSK_MOVW    , 0b00011
.set ISA_MSK_MULS    , 0b00011
.set ISA_MSK_MULSU   , 0b00011
.set ISA_MSK_FMUL    , 0b00011
.set ISA_MSK_FMULS   , 0b00011
.set ISA_MSK_FMULSU  , 0b00011
.set ISA_MSK_CPC     , 0b11111
.set ISA_MSK_SBC     , 0b11111
.set ISA_MSK_ADD     , 0b11111
.set ISA_MSK_CPSE    , 0b11111
.set ISA_MSK_CP      , 0b11111
.set ISA_MSK_SUB     , 0b11111
.set ISA_MSK_ADC     , 0b11111
.set ISA_MSK_AND     , 0b11111
.set ISA_MSK_EOR     , 0b11111
.set ISA_MSK_OR      , 0b11111
.set ISA_MSK_MOV     , 0b11111
.set ISA_MSK_CPI     , 0b11111
.set ISA_MSK_SBCI    , 0b11111
.set ISA_MSK_SUBI    , 0b11111
.set ISA_MSK_ORI     , 0b11111
.set ISA_MSK_ANDI    , 0b11111
.set ISA_MSK_LDS_16  , 0b10000
.set ISA_MSK_STS_16  , 0b10000
.set ISA_MSK_LD_Z    , 0b11111
.set ISA_MSK_LDD_Z   , 0b00111
.set ISA_MSK_LD_Y    , 0b10111
.set ISA_MSK_LDD_Y   , 0b00111
.set ISA_MSK_ST_Z    , 0b11111
.set ISA_MSK_STD_Z   , 0b00111
.set ISA_MSK_ST_Y    , 0b10111
.set ISA_MSK_STD_Y   , 0b00111
.set ISA_MSK_LDS_32  , 0b00111
.set ISA_MSK_LD_ZP   , 0b10111
.set ISA_MSK_LD_ZM   , 0b10111
.set ISA_MSK         , 0b00000
.set ISA_MSK_LPM_RZ  , 0b00011
.set ISA_MSK_LPM_RZP , 0b00011
.set ISA_MSK_ELPM_RZ , 0b00011
.set ISA_MSK_ELPM_RZP, 0b00011
.set ISA_MSK         , 0b00000
.set ISA_MSK_LD_YP   , 0b10111
.set ISA_MSK_LD_YM   , 0b10111
.set ISA_MSK         , 0b00000
.set ISA_MSK         , 0b00000
.set ISA_MSK         , 0b00000
.set ISA_MSK         , 0b00000
.set ISA_MSK_POP     , 0b10111
.set ISA_MSK_STS     , 0b00111
.set ISA_MSK_ST_ZP   , 0b10111
.set ISA_MSK_ST_ZM   , 0b10111
.set ISA_MSK         , 0b00000
.set ISA_MSK_XCH     , 0b00001
.set ISA_MSK_LAS     , 0b00001
.set ISA_MSK_LAC     , 0b00001
.set ISA_MSK_LAT     , 0b00001
.set ISA_MSK         , 0b00000
.set ISA_MSK_ST_YP   , 0b10111
.set ISA_MSK_ST_YM   , 0b10111
.set ISA_MSK         , 0b00000
.set ISA_MSK_ST_X    , 0b10111
.set ISA_MSK_ST_XP   , 0b10111
.set ISA_MSK_ST_XM   , 0b10111
.set ISA_MSK_PUSH    , 0b10111
.set ISA_MSK_COM     , 0b11111
.set ISA_MSK_NEG     , 0b11111
.set ISA_MSK_SWAP    , 0b11111
.set ISA_MSK_INC     , 0b11111
.set ISA_MSK         , 0b00000
.set ISA_MSK_ASR     , 0b11111
.set ISA_MSK_LSR     , 0b11111
.set ISA_MSK_ROR     , 0b11111
.set ISA_MSK_BSET    , 0b11111
.set ISA_MSK_BCLR    , 0b11111
.set ISA_MSK_RET     , 0b11111
.set ISA_MSK_RETI    , 0b11111
.set ISA_MSK         , 0b00000
.set ISA_MSK         , 0b00000
.set ISA_MSK         , 0b00000
.set ISA_MSK         , 0b00000
.set ISA_MSK         , 0b00000
.set ISA_MSK         , 0b00000
.set ISA_MSK_SLEEP   , 0b11111
.set ISA_MSK_BREAK   , 0b10011
.set ISA_MSK_WDR     , 0b11111
.set ISA_MSK         , 0b00000
.set ISA_MSK_LPM     , 0b01111
.set ISA_MSK_ELPM    , 0b00011
.set ISA_MSK_SPM     , 0b00111
.set ISA_MSK_SPM_ZP  , 0b00011
.set ISA_MSK_IJMP    , 0b11111
.set ISA_MSK_EIJMP   , 0b00111
.set ISA_MSK_ICALL   , 0b11111
.set ISA_MSK_EICALL  , 0b00111
.set ISA_MSK_DEC     , 0b11111
.set ISA_MSK_DES     , 0b00001
.set ISA_MSK_JMP     , 0b00111
.set ISA_MSK_CALL    , 0b00111
.set ISA_MSK_ADIW    , 0b00011
.set ISA_MSK_SBIW    , 0b00011
.set ISA_MSK_CBI     , 0b11111
.set ISA_MSK_SBIC    , 0b11111
.set ISA_MSK_SBI     , 0b11111
.set ISA_MSK_SBIS    , 0b11111
.set ISA_MSK_MUL     , 0b00011
.set ISA_MSK_IN      , 0b11111
.set ISA_MSK_OUT     , 0b11111
.set ISA_MSK_RJMP    , 0b11111
.set ISA_MSK_RCALL   , 0b11111
.set ISA_MSK_LDI     , 0b11111
.set ISA_MSK_BRBS    , 0b11111
.set ISA_MSK_BRBC    , 0b11111
.set ISA_MSK_BLD     , 0b11111
.set ISA_MSK_BST     , 0b11111
.set ISA_MSK_SBRC    , 0b11111
.set ISA_MSK_SBRS    , 0b11111

;-------------------------------------------------------------------------------
; register usage
;-------------------------------------------------------------------------------

                ; R0  multiplication target
                ; R1  multiplication target
.set CL_00 , 2  ; R2  lower  constants 0x00
.set CL_FF , 3  ; R3  lower  constants 0xff
.set CL_80 , 4  ; R4  lower  constants 0x80
.set CL_01 , 5  ; R5  lower  constants 0x01
.set DL_8  , 8  ; R6  lower  8 bit destination
                ; R7
.set DL_16L, 10 ; R8  lower  16 bit destination LO
.set DL_16H, 11 ; R9  lower  16 bit destination HI
                ; R10
                ; R11
                ; R12
                ; R13
                ; R14
                ; R15
                ; R16
                ; R17
.set CH_00 , 18 ; R18 higher constants 0x00
.set CH_FF , 19 ; R19 higher constants 0xff
.set CH_80 , 20 ; R20 higher constants 0x80
.set CH_01 , 21 ; R21 higher constants 0x01
.set DH_8  , 22 ; R22 higher 8 bit destination
                ; R23
.set DH_16L, 24 ; R24 higher 16 bit destination LO
.set DH_16H, 25 ; R25 higher 16 bit destination HI
                ; R26 lo(X)
                ; R27 hi(X)
                ; R28 lo(Y)
                ; R29 hi(Y)
                ; R30 lo(Z)
                ; R31 hi(Z)

; TODO check if this 16 bit register targets are correct
.set CL_16, 2
.set CH_16, 18
.set DL_16, 10
.set DH_16, 24

;-------------------------------------------------------------------------------
; main is calling instruction tests one by one
;-------------------------------------------------------------------------------

.org 0

; TODO missing interrupt routines

main:
  rcall init_clr

  rcall test_nop
  rcall test_movw
  rcall test_muls
  rcall test_mulsu
  rcall test_fmul
  rcall test_fmuls
  rcall test_fmulsu
  rcall test_cpc
  rcall test_sbc
  rcall test_add
  rcall test_cpse
  rcall test_cp
  rcall test_sub
  rcall test_adc
  rcall test_and
  rcall test_eor
  rcall test_or
  rcall test_mov
  rcall test_cpi
  rcall test_sbci
  rcall test_subi
  rcall test_ori
  rcall test_andi
  sleep

;-------------------------------------------------------------------------------
; tests for all supported instructions by architecture
; where instructions provide register addresses or constants,
; it is attempted to test edge values (max/min sometimes all)
; TODO: in arithmetic instructions results should cover all edge values
; and affect all flags
;-------------------------------------------------------------------------------

test_nop:
.if (SUBISA & ISA_MSK_NOP   )
  nop
;.word 0b0000000011111111
.endif
  ret

test_movw:
.if (SUBISA & ISA_MSK_MOVW  )
  rcall  init_const
  movw   DL_16,CL_16
  movw   DH_16,CH_16
.endif
  ret

test_muls:
.if (SUBISA & ISA_MSK_MULS  )
  rcall  init_const
  muls   CH_00,CH_00 ; should set Z flag
  muls   CH_FF,CH_FF ; should set C flag
.endif
  ret

test_mulsu:
.if (SUBISA & ISA_MSK_MULSU )
  rcall  init_const
  muls   CH_00,CH_00 ; should set Z flag
  muls   CH_FF,CH_FF ; should set C flag
.endif
  ret

test_fmul:
.if (SUBISA & ISA_MSK_FMUL  )
  rcall  init_const
  fmul   CH_00,CH_00 ; should set Z flag
  fmul   CH_FF,CH_FF ; should set C flag
.endif
  ret

test_fmuls:
.if (SUBISA & ISA_MSK_FMULS )
  fmuls  CH_00,CH_00 ; should set Z flag
  fmuls  CH_FF,CH_FF ; should set C flag
.endif
  ret

test_fmulsu:
.if (SUBISA & ISA_MSK_FMULSU)
  fmulsu CH_00,CH_00 ; should set Z flag
  fmulsu CH_FF,CH_FF ; should set C flag
.endif
  ret

test_cpc:
.if (SUBISA & ISA_MSK_CPC   )
  rcall  init_const  ; instruction CPL does not modify registers
  clc                ; clear C
  cpc    CH_00,CH_00 ; (0x00-0x00-0) should set ....Z. flags
  clc                ; clear C
  cpc    CH_00,CH_FF ; (0x00-0xff-0) should set .....C flags
  clc                ; clear C
  cpc    CH_FF,CH_00 ; (0xff-0x00-0) should set .S.N.. flags
  clc                ; clear C
  cpc    CH_FF,CH_FF ; (0xff-0xff-0) should set HSV.Z. flags
  sec                ; set C
  cpc    CH_00,CH_00 ; (0x00-0x00-1) should set ..VN.C flags
  sec                ; set C
  cpc    CH_00,CH_FF ; (0x00-0xff-1) should set H...ZC flag
  sec                ; set C
  cpc    CH_FF,CH_00 ; (0xff-0x00-1) should set .S.N.. flags
  sec                ; set C
  cpc    CH_FF,CH_FF ; (0xff-0xff-1) should set HS.N.C flags
.endif
  ret

test_sbc:
.if (SUBISA & ISA_MSK_SBC   )
  rcall  init_const  ;
  clc                ; clear C
  sbc    CH_00,CH_00 ; (0x00-0x00-0) should set ....Z. flags
  clr    CH_00       ; 0x00
  clc                ; clear C
  sbc    CH_00,CH_FF ; (0x00-0xff-0) should set .....C flags
  clr    CH_00       ; 0x00
  clc                ; clear C
  sbc    CH_FF,CH_00 ; (0xff-0x00-0) should set .S.N.. flags
  ser    CH_FF       ; 0xff
  clc                ; clear C
  sbc    CH_FF,CH_FF ; (0xff-0xff-0) should set HSV.Z. flags
  ser    CH_FF       ; 0xff
  sec                ; set C
  sbc    CH_00,CH_00 ; (0x00-0x00-1) should set ..VN.C flags
  clr    CH_00       ; 0x00
  sec                ; set C
  sbc    CH_00,CH_FF ; (0x00-0xff-1) should set H...ZC flag
  clr    CH_00       ; 0x00
  sec                ; set C
  sbc    CH_FF,CH_00 ; (0xff-0x00-1) should set .S.N.. flags
  ser    CH_FF       ; 0xff
  sec                ; set C
  sbc    CH_FF,CH_FF ; (0xff-0xff-1) should set HS.N.C flags
  ser    CH_FF       ; 0xff
.endif
  ret

test_add:
.if (SUBISA & ISA_MSK_ADD   )
  rcall  init_const  ;
  add    CH_00,CH_00 ; (0x00+0x00) should set ....Z. flags
  clr    CH_00       ; 0x00
  add    CH_00,CH_FF ; (0x00+0xff) should set .S.N.. flags
  clr    CH_00       ; 0x00
  add    CH_80,CH_80 ; (0x8f+0x80) should set .SV.ZC flags
  ldi    CH_80,0x80  ; 0x80
  add    CH_FF,CH_FF ; (0xff+0xff) should set HS.N.C flags
  ser    CH_FF       ; 0xff
.endif
  ret

test_cpse:
.if (SUBISA & ISA_MSK_CPSE  )
  rcall  init_const  ; test skip (or not) 16bit instruction
  cpse   CL_FF,CL_FF ; should skip
  ldi    DH_8,0xA5
  cpse   CL_80,CL_01 ; should not skip
  ldi    DH_8,0x5A
  nop
  rcall  init_const  ; test skip (or not) 32bit instruction
  ldi    DH_8,0xA5
  sts    0x0100,DH_8 ; load 0xA5 into location 0x0100
  clr    DH_8        ; clear register
  cpse   CL_FF,CL_FF ; should skip
  lds    DH_8,0x0100 ; the second instruction word is "MOVW r1:r0,r1:r0"
  nop
  clr    r18         ; clear register
  cpse   CL_80,CL_01 ; should not skip
  lds    DH_8,0x0100
  nop
.endif
  ret

test_cp:
.if (SUBISA & ISA_MSK_CP    )
  ; same code as for cpc, except ignore cary
  rcall  init_const  ;
  cp     CH_00,CH_00 ; (0x00-0x00) should set ....Z. flags
  cp     CH_00,CH_FF ; (0x00-0xff) should set .....C flags
  cp     CH_FF,CH_00 ; (0xff-0x00) should set .S.N.. flags
  cp     CH_FF,CH_00 ; (0xff-0xff) should set HSV.Z. flags
.endif
  ret

test_sub:
.if (SUBISA & ISA_MSK_SUB   )
  ; same code as for sbc, except ignore cary
  rcall  init_const  ;
  sub    CH_00,CH_00 ; (0x00-0x00) should set ....Z. flags
  clr    CH_00       ; 0x00
  sub    CH_00,CH_FF ; (0x00-0xff) should set .....C flags
  clr    CH_00       ; 0x00
  sub    CH_FF,CH_00 ; (0xff-0x00) should set .S.N.. flags
  ser    CH_FF       ; 0xff
  sub    CH_FF,CH_00 ; (0xff-0xff) should set HSV.Z. flags
  ser    CH_FF       ; 0xff
.endif
  ret

test_adc:
.if (SUBISA & ISA_MSK_ADC   )
  rcall  init_const  ;
  clc                ; clear C
  adc    CH_00,CH_00 ; (0x00+0x00+0) should set ....Z. flags
  clr    CH_00       ; 0x00
  clc                ; clear C
  adc    CH_00,CH_FF ; (0x00+0xff+0) should set .S.N.. flags
  clr    CH_00       ; 0x00
  clc                ; clear C
  adc    CH_80,CH_80 ; (0x8f+0x80+0) should set .SV.ZC flags
  ldi    CH_80,0x80  ; 0x80
  clc                ; clear C
  adc    CH_FF,CH_FF ; (0xff+0xff+0) should set HS.N.C flags
  ser    CH_FF       ; 0xff
  sec                ; set C
  adc    CH_00,CH_00 ; (0x00+0x00+1) should set ...... flags
  clr    CH_00       ; 0x00
  sec                ; set C
  adc    CH_00,CH_FF ; (0x00+0xff+1) should set H...ZC flags
  clr    CH_00       ; 0x00
  sec                ; set C
  adc    CH_80,CH_80 ; (0x8f+0x80+1) should set .SV..C flags
  ldi    CH_80,0x80  ; 0x80
  sec                ; set C
  adc    CH_FF,CH_FF ; (0xff+0xff+1) should set HS.N.C flags
  ser    CH_FF       ; 0xff
.endif
  ret

test_and:
.if (SUBISA & ISA_MSK_AND   )
  rcall  init_const  ;
  and    CH_FF,CH_00 ; (0xff & 0x00) should set _...Z_ flags
  ser    CH_FF       ; 0xff
  and    CH_FF,CH_FF ; (0xff & 0xff) should set _S.N._ flags
  ser    CH_FF       ; 0xff
.endif
  ret

test_eor:
.if (SUBISA & ISA_MSK_EOR   )
  rcall  init_const  ;
  eor    CH_FF,CH_00 ; (0xff & 0x00) should set _S.N._ flags
  ser    CH_FF       ; 0xff
  eor    CH_FF,CH_FF ; (0xff & 0xff) should set _...Z_ flags
  ser    CH_FF       ; 0xff
.endif
  ret

test_or:
.if (SUBISA & ISA_MSK_OR    )
  rcall  init_const  ;
  or     CH_00,CH_00 ; (0x00 & 0x00) should set _...Z_ flags
  clr    CH_00       ; 0x00
  or     CH_00,CH_FF ; (0x00 & 0xff) should set _S.N._ flags
  clr    CH_00       ; 0x00
.endif
  ret

test_mov:
.if (SUBISA & ISA_MSK_MOV   )
  rcall  init_const ;
  ldi    DH_8,0xa5  ;
  mov    DL_8,DH_8  ;
.endif
  ret

test_cpi:
.if (SUBISA & ISA_MSK_CPI   )
  ; same code as for cp, except immediate instead of second register
  rcall  init_const ;
  cpi    CH_00,0x00 ; (0x00-0x00) should set ....Z. flags
  cpi    CH_00,0xff ; (0x00-0xff) should set .....C flags
  cpi    CH_FF,0x00 ; (0xff-0x00) should set .S.N.. flags
  cpi    CH_FF,0xff ; (0xff-0xff) should set HSV.Z. flags
.endif
  ret

test_sbci:
.if (SUBISA & ISA_MSK_SBCI  )
  rcall  init_const ;
  clc               ; clear C
  sbci   CH_00,0x00 ; (0x00-0x00-0) should set ....Z. flags
  clr    CH_00      ; 0x00
  clc               ; clear C
  sbci   CH_00,0xff ; (0x00-0xff-0) should set .....C flags
  clr    CH_00      ; 0x00
  clc               ; clear C
  sbci   CH_FF,0x00 ; (0xff-0x00-0) should set .S.N.. flags
  ser    CH_FF      ; 0xff
  clc               ; clear C
  sbci   CH_FF,0xff ; (0xff-0xff-0) should set HSV.Z. flags
  ser    CH_FF      ; 0xff
  sec               ; set C
  sbci   CH_00,0x00 ; (0x00-0x00-1) should set ..VN.C flags
  clr    CH_00      ; 0x00
  sec               ; set C
  sbci   CH_00,0xff ; (0x00-0xff-1) should set H...ZC flag
  clr    CH_00      ; 0x00
  sec               ; set C
  sbci   CH_FF,0x00 ; (0xff-0x00-1) should set .S.N.. flags
  ser    CH_FF      ; 0xff
  sec               ; set C
  sbci   CH_FF,0xff ; (0xff-0xff-1) should set HS.N.C flags
  ser    CH_FF      ; 0xff
.endif
  ret

test_subi:
.if (SUBISA & ISA_MSK_SUBI  )
  rcall  init_const ;
  subi   CH_00,0x00 ; (0x00-0x00-0) should set ....Z. flags
  clr    CH_00      ; 0x00
  subi   CH_00,0xff ; (0x00-0xff-0) should set .....C flags
  clr    CH_00      ; 0x00
  subi   CH_FF,0x00 ; (0xff-0x00-0) should set .S.N.. flags
  ser    CH_FF      ; 0xff
  subi   CH_FF,0xff ; (0xff-0xff-0) should set HSV.Z. flags
  ser    CH_FF      ; 0xff
.endif
  ret

test_ori:
.if (SUBISA & ISA_MSK_ORI   )
  rcall  init_const ;
  ori    CH_00,0x00 ; (0x00 & 0x00) should set _...Z_ flags
  clr    CH_00      ; 0x00
  ori    CH_00,0xff ; (0x00 & 0xff) should set _S.N._ flags
  clr    CH_00      ; 0x00
.endif
  ret

test_andi:
.if (SUBISA & ISA_MSK_ANDI  )
  rcall  init_const ;
  andi   CH_FF,0x00 ; (0xff & 0x00) should set _...Z_ flags
  ser    CH_FF      ; 0xff
  andi   CH_FF,0xff ; (0xff & 0xff) should set _S.N._ flags
  ser    CH_FF      ; 0xff
.endif
  ret

test_lds_16:
.if (SUBISA & ISA_MSK_LDS_16  )
  rcall  init_const ;
  ldi DH_8,0xa5 ;
  sts 0x00,DH_8 ;
  ldi DH_8,0xFF ;
  lds DH_8,0x00 ;
  ldi DH_8,0x5a ;
  sts 0x7f,DH_8 ;
  ldi DH_8,0xFF ;
  lds DH_8,0x7f ;
.endif
  ret

.if (SUBISA & ISA_MSK_LD_Z    )
.endif
  ret

.if (SUBISA & ISA_MSK_LDD_Z   )
.endif
  ret

.if (SUBISA & ISA_MSK_LD_Y    )
.endif
  ret

.if (SUBISA & ISA_MSK_LDD_Y   )
.endif
  ret

.if (SUBISA & ISA_MSK_ST_Z    )
.endif
  ret

.if (SUBISA & ISA_MSK_STD_Z   )
.endif
  ret

.if (SUBISA & ISA_MSK_ST_Y    )
.endif
  ret

.if (SUBISA & ISA_MSK_STD_Y   )
.endif
  ret

.if (SUBISA & ISA_MSK_LDS_32  )
.endif
  ret

.if (SUBISA & ISA_MSK_LD_ZP   )
.endif
  ret

.if (SUBISA & ISA_MSK_LD_ZM   )
.endif
  ret

.if (SUBISA & ISA_MSK         )
.endif
  ret

.if (SUBISA & ISA_MSK_LPM_RZ  )
.endif
  ret

.if (SUBISA & ISA_MSK_LPM_RZP )
.endif
  ret

.if (SUBISA & ISA_MSK_ELPM_RZ )
.endif
  ret

.if (SUBISA & ISA_MSK_ELPM_RZP)
.endif
  ret

.if (SUBISA & ISA_MSK         )
.endif
  ret

.if (SUBISA & ISA_MSK_LD_YP   )
.endif
  ret

.if (SUBISA & ISA_MSK_LD_YM   )
.endif
  ret

.if (SUBISA & ISA_MSK         )
.endif
  ret

.if (SUBISA & ISA_MSK         )
.endif
  ret

.if (SUBISA & ISA_MSK         )
.endif
  ret

.if (SUBISA & ISA_MSK         )
.endif
  ret

.if (SUBISA & ISA_MSK_POP     )
.endif
  ret

;-------------------------------------------------------------------------------
; initialize registers to zero
;-------------------------------------------------------------------------------

init_clr:
;  clr r0
;  clr r1
;  clr r2
;  clr r3
;  clr r4
;  clr r5
;  clr r6
;  clr r7
;  clr r8
;  clr r9
;  clr r10
;  clr r11
;  clr r12
;  clr r13
;  clr r14
;  clr r15
;  clr r16
;  clr r17
;  clr r18
;  clr r19
;  clr r20
;  clr r21
;  clr r22
;  clr r23
;  clr r24
;  clr r25
;  clr r26
;  clr r27
;  clr r28
;  clr r29
;  clr r30
;  clr r31
  ldi r16,0x00
  ldi r17,0x00
  ldi r18,0x00
  ldi r19,0x00
  ldi r20,0x00
  ldi r21,0x00
  ldi r22,0x00
  ldi r23,0x00
  ldi r24,0x00
  ldi r25,0x00
  ldi r26,0x00
  ldi r27,0x00
  ldi r28,0x00
  ldi r29,0x00
  ldi r30,0x00
  ldi r31,0x00
  mov r0 ,r16
  mov r1 ,r17
  mov r2 ,r18
  mov r3 ,r19
  mov r4 ,r20
  mov r5 ,r21
  mov r6 ,r22
  mov r7 ,r23
  mov r8 ,r24
  mov r9 ,r25
  mov r10,r26
  mov r11,r27
  mov r12,r28
  mov r13,r29
  mov r14,r30
  mov r15,r31
  ret

init_const:
  ldi CH_00,0x00  ; R18 higher constants 0x00
  ldi CH_FF,0xff  ; R19 higher constants 0xff
  ldi CH_80,0x80  ; R20 higher constants 0x80
  ldi CH_01,0x01  ; R21 higher constants 0x01
  mov CL_00,CH_00 ; R2  lower  constants 0x00
  mov CL_FF,CH_FF ; R3  lower  constants 0xff
  mov CL_80,CH_80 ; R4  lower  constants 0x80
  mov CL_01,CH_01 ; R5  lower  constants 0x01
  ret

.end
