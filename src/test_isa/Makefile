all:
	#avr-gcc -mmcu=atmega128 -Wall -Os -o test_isa.elf test_isa.S
	#avr-objcopy -j .text -j .data -O binary test_isa.elf test_isa.bin
	#avr-objcopy -j .text -j .data -O ihex   test_isa.elf test_isa.hex
	avr-as -mmcu=avr31 test_isa.S -o test_isa.elf
	avr-objcopy -j .text -j .data -O binary test_isa.elf test_isa.bin
	avr-objcopy -j .text -j .data -O ihex   test_isa.elf test_isa.hex
	srec_cat test_isa.hex -Intel -byte-swap 2 -Output test_isa.vmem -VMem 16
	avr-objdump -j .sec1 -d -m avr31 test_isa.hex > test_isa.dump
