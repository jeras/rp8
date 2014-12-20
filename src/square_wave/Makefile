all:
	avr-gcc -mmcu=at90s1200 -Wall -Os -o square_wave.elf square_wave.S
	avr-objcopy -j .text -j .data -O ihex square_wave.elf square_wave.hex
	avr-objdump -j .sec1 -d -m avr1 square_wave.hex > square_wave.dump
