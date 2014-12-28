## Core configurations:

1. reduced
2. minimal
4. clasic
6. enhanced
7. xmega

### Instruction set options
- enable ADIW, SBIW
- enable \*MUL\* and MOVW

| core     |    |    |
| -------- | -- | -- |
| reduced  |    |    |
| clasic   |    |    |
| enhanced |    |    |
| xmega    |    |    |

### Program address space size options

The size of the addressable program memory is defined by `parameter int unsigned PAW` (Program Address Width). The program memory is addressible in 2 byte words and not single bytes, so the available program address space depending on the address width is:
```SystemVerilog
address_space_size = 2**(PAW*1)
```
There is no lower limit to the program address space size, but testing was only done for sizes 128 bytes on (`PAW>=6`). The upper limit is defined by the direct program addressing mode, which provides 22 bits to be loaded into the program counter, this limits the program address space to 8MB.

There are some aspects of the CPU core that change depending on `PAW`.

1. Program counter size and call/interrupt latency

   The program counter `PC[PAW-1:0]` width is the same as the program memory address bus. During subroutine and interrupt calls the program counter is stored on the stack. The number of bytes stored on entry and retrived on return depends on the size of the program counter. Since the data memory the stack resides in requires a clock cycle for each read/write byte transfer, the call latency depends on the byte size of the program counter. Although the program counter can be a single byte for program address space up to 512 Bytes, the Atmel AVR core will push/pop two bytes each call/return. The RP_8bit (TODO) differs from the original here, by only storing a single byte.

2. Instructions for subroutine calls and jumps, and extended registers

   Different program addressing modes (call and jump intructions) have different reach inside the program address space.

   1. relative

      Instructions `RCALL` and `RJMP` provide a 12 bit constant `k` which specifies a relative offset from -2048 to 2047 program words (2 bytes) from the current PC. This covers the whole address space if `PAW <= 12` which gives a program address space of up to 8kB. So for program memory sizes up to 8kB there is no need for a dedicated direct addressing mode.

   2. indirect

      Instructions `ICALL` and `IJMP` use the 16 bit `Z` register to provide a 16 bit absolute address to be loaded into PC. This allows for addressing program memory of up to 128kB. So for program memory sizes up to 128kB there is no need for an extended indirect addressing mode.

   3. extended indirect

      Instructions `EICALL` and `EIJMP` use the 16 bit `Z` register and the 6 bit `EIMD` register to provide a 22 bit address to be loaded into PC. This allows for addressing program memory of up to 8MB.

   4. direct

      Instructions `CALL` and `JMP` are two words long and provide a 22 bit constant `k` to be loaded into PC. This allows for addressing program memory of up to 8MB.

3. Instructions for reading/writing data from/to program memory




| size     | PAW     | instructions                         | program counter | extended indirect register | 
| -------- | ------- | ------------------------------------ | --------------- | -------------------------- |
| size     | PAW     | instructions                         | program counter | extended indirect register | 
| -------- | ------- | ------------------------------------ | --------------- | -------------------------- |
| <=  512B |  0 -  8 | RCALL/RJMP                           | PC[PAW-1:0]     |                            |
| <=   8kB |  8 - 12 | RCALL/RJMP                           | PC[PAW-1:0]     |                            |
| <= 128kB | 13 - 16 | RCALL/RJMP, ICALL/IJMP               | PC[PAW-1:0]     |                            |
| <=   8MB | 17 - 22 | RCALL/RJMP, ICALL/IJMP, EICALL/EIJMP | PC[PAW-1:0]     | EIND [PAW-16-1:0], RAMPZ   |

