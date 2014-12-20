#include <avr/io.h>

work = 16 ; Note [2]
tmp = 17
inttmp = 19
intsav = 0
SQUARE = PD6 ; Note [3]
; Note [4]:
tmconst= 10700000 / 200000 ; 100 kHz => 200000 edges/s
fuzz= 8 ; # clocks in ISR until TCNT0 is set

.section .text

.global main ; Note [5]
main:
  rcall ioinit
1:
  rjmp 1b ; Note [6]
.global TIMER0_OVF_vect ; Note [7]
TIMER0_OVF_vect:
  ldi inttmp, 256 - tmconst + fuzz
  out _SFR_IO_ADDR(TCNT0), inttmp ; Note [8]
  in intsav, _SFR_IO_ADDR(SREG) ; Note [9]
  sbic _SFR_IO_ADDR(PORTD), SQUARE
  rjmp 1f
  sbi _SFR_IO_ADDR(PORTD), SQUARE
  rjmp 2f
1: cbi _SFR_IO_ADDR(PORTD), SQUARE
2:
  out _SFR_IO_ADDR(SREG), intsav
  reti
ioinit:
  sbi _SFR_IO_ADDR(DDRD), SQUARE
  ldi work, _BV(TOIE0)
  out _SFR_IO_ADDR(TIMSK), work
  ldi work, _BV(CS00) ; tmr0: CK/1
  out _SFR_IO_ADDR(TCCR0), work
  ldi work, 256 - tmconst
  out _SFR_IO_ADDR(TCNT0), work
  sei
  ret
.global __vector_default ; Note [10]
__vector_default:
  reti
.end
