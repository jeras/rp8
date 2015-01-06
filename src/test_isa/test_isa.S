.set SUBISA, 0b1000

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

.section .text
.global main

main:

  rcall init_inc
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
  movw   r1 :r0 ,r17:r16
  movw   r15:r14,r31:r30
.endif
  ret

test_muls:
.if (SUBISA & ISA_MSK_MULS  )
  muls   r16,r17
  muls   r31,r30
.endif
  ret

test_mulsu:
.if (SUBISA & ISA_MSK_MULSU )
  mulsu  r16,r17
  mulsu  r31,r30
.endif
  ret

test_fmul:
.if (SUBISA & ISA_MSK_FMUL  )
  fmul   r16,r17
  fmul   r31,r30
.endif
  ret

test_fmuls:
.if (SUBISA & ISA_MSK_FMULS )
  fmuls  r16,r17
  fmuls  r31,r30
  ret
.endif

test_fmulsu:
.if (SUBISA & ISA_MSK_FMULSU)
  fmulsu r16,r17
  fmulsu r31,r30
.endif
  ret

test_cpc:
.if (SUBISA & ISA_MSK_CPC   )
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

init_inc:
  ldi r16,0x00
  ldi r17,0x01
  ldi r18,0x02
  ldi r19,0x03
  ldi r20,0x04
  ldi r21,0x05
  ldi r22,0x06
  ldi r23,0x07
  ldi r24,0x08
  ldi r25,0x09
  ldi r26,0x0a
  ldi r27,0x0b
  ldi r28,0x0c
  ldi r29,0x0d
  ldi r30,0x0e
  ldi r31,0x0f

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

  ldi r16,0x10
  ldi r17,0x11
  ldi r18,0x12
  ldi r19,0x13
  ldi r20,0x14
  ldi r21,0x15
  ldi r22,0x16
  ldi r23,0x17
  ldi r24,0x18
  ldi r25,0x19
  ldi r26,0x1a
  ldi r27,0x1b
  ldi r28,0x1c
  ldi r29,0x1d
  ldi r30,0x1e
  ldi r31,0x1f

  ret
.end
