all:
	avr-gcc -mmcu=atmega128 -Wall -Os -o test_ldi.elf test_ldi.S
	avr-objcopy -j .text -j .data -O binary test_ldi.elf test_ldi.bin
	avr-objcopy -j .text -j .data -O ihex   test_ldi.elf test_ldi.hex
	srec_cat test_ldi.hex -Intel -byte-swap 2 -Output test_ldi.vmem -VMem 16
	avr-objdump -j .sec1 -d -m avr1 test_ldi.hex > test_ldi.dump
