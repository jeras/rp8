.set SUBISA, 0b1000

;-------------------------------------------------------------------------------
; table defining availability of instructions for various ISA subsets
;-------------------------------------------------------------------------------

; mnemonic                |  core  |
; ----------------------- | ------ |
.set ISA_MSK_NOP   , 0b1111
.set ISA_MSK_MOVW  , 0b0011
.set ISA_MSK_MULS  , 0b0011
.set ISA_MSK_MULSU , 0b0011
.set ISA_MSK_FMUL  , 0b0011
.set ISA_MSK_FMULS , 0b0011
.set ISA_MSK_FMULSU, 0b0011
.set ISA_MSK_CPC   , 0b1111
.set ISA_MSK_SBC   , 0b1111
.set ISA_MSK_ADD   , 0b1111
.set ISA_MSK_CPSE  , 0b1111
.set ISA_MSK_CP    , 0b1111
.set ISA_MSK_SUB   , 0b1111
.set ISA_MSK_ADC   , 0b1111
.set ISA_MSK_AND   , 0b1111
.set ISA_MSK_EOR   , 0b1111
.set ISA_MSK_OR    , 0b1111
.set ISA_MSK_MOV   , 0b1111
.set ISA_MSK_CPI   , 0b1111
.set ISA_MSK_SBCI  , 0b1111
.set ISA_MSK_SUBI  , 0b1111
.set ISA_MSK_ORI   , 0b1111
.set ISA_MSK_ANDI  , 0b1111
; `LD     Rd,Z`       , 0b1111
; `LDD    Rd,Z+q`     , 0b0011
; `LD     Rd,Y`       , 0b1111
; `LDD    Rd,Y+q`     , 0b0011
; `ST     Rd,Z`       , 0b1111
; `STD    Rd,Z+q`     , 0b0011
; `ST     Rd,Y`       , 0b1111
; `STD    Rd,Y+q`     , 0b0011
; `LDS    Rd,0x????`  , 0b0011
; `LD     Rd,Z+`      , 0b1111
; `LD     Rd,-Z`      , 0b1111
; `LPM    Rd,Z`       , 0b0111
; `LPM    Rd,Z+`      , 0b0111
; `ELPM   Rd,Z`       , 0b0111
; `ELPM   Rd,Z+`      , 0b0111
; `LD     Rd,Y+`      , 0b1111
; `LD     Rd,-Y`      , 0b1111
.set ISA_MSK_POP   , 0b1111
.set ISA_MSK_STS   , 0b0011
; `ST     Z+,Rr`      , 0b1111
; `ST     -Z,Rr`      , 0b1111
.set ISA_M0001CH   , 0b0001
.set ISA_MSK_LAS   , 0b0001
.set ISA_MSK_LAC   , 0b0001
.set ISA_MSK_LAT   , 0b0001
; `ST     Y+,Rr`      , 0b1111
; `ST     -Y,Rr`      , 0b1111
; `ST  0001,Rr`       , 0b1111
; `ST  0001+,Rr`      , 0b1111
; `ST   0001,Rr`      , 0b1111
.set ISA_MSK_PUSH  , 0b1111
.set ISA_MSK_COM   , 0b1111
.set ISA_MSK_NEG   , 0b1111
.set ISA_MSK_SWAP  , 0b1111
.set ISA_MSK_INC   , 0b1111
.set ISA_MSK_ASR   , 0b1111
.set ISA_MSK_LSR   , 0b1111
.set ISA_MSK_ROR   , 0b1111
.set ISA_MSK_BSET  , 0b1111
.set ISA_MSK_BCLR  , 0b1111
.set ISA_MSK_RET   , 0b1111
.set ISA_MSK_RETI  , 0b1111
.set ISA_MSK_SLEEP , 0b1111
.set ISA_MSK_BREAK , 0b0011
.set ISA_MSK_WDR   , 0b1111
.set ISA_MSK_LPM   , 0b0111
.set ISA_MSK_ELPM  , 0b0011
; `SPM `              , 0b0011
; `SPM Z+`            , 0b0011
.set ISA_MSK_IJMP  , 0b1111
.set ISA_MSK_EIJMP , 0b0111
.set ISA_MSK_ICALL , 0b1111
.set ISA_MSK_EICALL, 0b0111
.set ISA_MSK_DEC   , 0b1111
.set ISA_MSK_DES   , 0b0001
.set ISA_MSK_JMP   , 0b0111
.set ISA_MSK_CALL  , 0b0111
.set ISA_MSK_ADIW  , 0b0011
.set ISA_MSK_SBIW  , 0b0011
.set ISA_MSK_CBI   , 0b1111
.set ISA_MSK_SBIC  , 0b1111
.set ISA_MSK_SBI   , 0b1111
.set ISA_MSK_SBIS  , 0b1111
.set ISA_MSK_MUL   , 0b0011
.set ISA_MSK_IN    , 0b1111
.set ISA_MSK_OUT   , 0b1111
.set ISA_MSK_RJMP  , 0b1111
.set ISA_MSK_RCALL , 0b1111
.set ISA_MSK_LDI   , 0b1111
.set ISA_MSK_BRBS  , 0b1111
.set ISA_MSK_BRBC  , 0b1111
.set ISA_MSK_BLD   , 0b1111
.set ISA_MSK_BST   , 0b1111
.set ISA_MSK_SBRC  , 0b1111
.set ISA_MSK_SBRS  , 0b1111

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
  ldi r30,0x1e
  ldi r31,0x1f
  movw   r1 :r0 ,r31:r30
  movw   r29:r28,r1 :r0
.endif
  ret

test_muls:
.if (SUBISA & ISA_MSK_MULS  )
  clr    r16     ; clear to 0x00
  clr    r17     ; clear to 0x00
  muls   r16,r17 ; should set Z flag
  ser    r30     ; set to 0xff
  ser    r31     ; set to 0xff
  muls   r30,r31 ; should set C flag
.endif
  ret

test_mulsu:
.if (SUBISA & ISA_MSK_MULSU )
  clr    r16     ; clear to 0x00
  clr    r17     ; clear to 0x00
  muls   r16,r17 ; should set Z flag
  ser    r22     ; set to 0xff
  ser    r23     ; set to 0xff
  muls   r23,r22 ; should set C flag
.endif
  ret

test_fmul:
.if (SUBISA & ISA_MSK_FMUL  )
;TODO
  fmul   r16,r17
  fmul   r31,r30
.endif
  ret

test_fmuls:
.if (SUBISA & ISA_MSK_FMULS )
;TODO
  fmuls  r16,r17
  fmuls  r31,r30
.endif
  ret

test_fmulsu:
.if (SUBISA & ISA_MSK_FMULSU)
;TODO
  fmulsu r16,r17
  fmulsu r31,r30
.endif
  ret

test_cpc:
.if (SUBISA & ISA_MSK_CPC   )
  clc            ; clear carry
  cpc    r0 ,r1
  cpc    r31,r30
  sec            ; set carry
  cpc    r0 ,r1
  cpc    r31,r30
.endif
  ret

test_sbc:
.if (SUBISA & ISA_MSK_SBC   )
  sbc    r0 ,r1
  sbc    r31,r30
.endif
  ret

test_add:
.if (SUBISA & ISA_MSK_ADD   )
  add    r0 ,r1
  add    r31,r30
.endif
  ret

test_cpse:
.if (SUBISA & ISA_MSK_CPSE  )
; TODO test both skipping and not skipping situations, also test skipping 2 word instructions
  cpse   r0 ,r1
  nop
  cpse   r31,r30
  nop
.endif
  ret

test_cp:
.if (SUBISA & ISA_MSK_CP    )
  cp     r0 ,r1
  cp     r31,r30
.endif
  ret

test_sub:
.if (SUBISA & ISA_MSK_SUB   )
.endif
  ret

test_adc:
.if (SUBISA & ISA_MSK_ADC   )
.endif
  ret

test_and:
.if (SUBISA & ISA_MSK_AND   )
.endif
  ret

test_eor:
.if (SUBISA & ISA_MSK_EOR   )
.endif
  ret

test_or:
.if (SUBISA & ISA_MSK_OR    )
.endif
  ret

test_mov:
.if (SUBISA & ISA_MSK_MOV   )
.endif
  ret

test_cpi:
.if (SUBISA & ISA_MSK_CPI   )
.endif
  ret

test_sbci:
.if (SUBISA & ISA_MSK_SBCI  )
.endif
  ret

test_subi:
.if (SUBISA & ISA_MSK_SUBI  )
.endif
  ret

test_ori:
.if (SUBISA & ISA_MSK_ORI   )
.endif
  ret

test_andi:
.if (SUBISA & ISA_MSK_ANDI  )
.endif
  ret

;-------------------------------------------------------------------------------
; initialize registers to incrementing values
;-------------------------------------------------------------------------------

init_clr:
  clr r0
  clr r1
  clr r2
  clr r3
  clr r4
  clr r5
  clr r6
  clr r7
  clr r8
  clr r9
  clr r10
  clr r11
  clr r12
  clr r13
  clr r14
  clr r15
  clr r16
  clr r17
  clr r18
  clr r19
  clr r20
  clr r21
  clr r22
  clr r23
  clr r24
  clr r25
  clr r26
  clr r27
  clr r28
  clr r29
  clr r30
  clr r31
  ret

.end
