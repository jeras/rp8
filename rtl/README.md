## Core configurations:

### Instruction set options
- enable ADIW, SBIW
- enable \*MUL\* and MOVW

| core     |    |    |
| -------- | -- | -- |
| reduced  |    |    |
| classic  |    |    |
| enhanced |    |    |
| xmega    |    |    |



### Program address space size options

The size of the addressable program memory is defined by `parameter int unsigned PAW` (Program Address Width). The program memory is addressable in 2 byte words and not single bytes, so the available program address space depending on the address width is:
```SystemVerilog
address_space_size = 2**(PAW+1)
```
There is no lower limit to the program address space size, but testing was only done for sizes 128 bytes on (`PAW>=6`). The upper limit is defined by the direct program addressing mode, which provides 22 bits to be loaded into the program counter, this limits the program address space to 8MB.

There are some aspects of the CPU core that change depending on `PAW`.

1. Program counter size and call/interrupt latency

   The program counter `PC[PAW-1:0]` width is the same as the program memory address bus. During subroutine and interrupt calls the program counter is stored on the stack. The number of bytes stored on entry and retrieved on return depends on the size of the program counter. Since the data memory the stack resides in requires a clock cycle for each read/write byte transfer, the call latency depends on the byte size of the program counter. Although the program counter can be a single byte for program address space up to 512 Bytes, the Atmel AVR core will push/pop two bytes each call/return. The RP_8bit (TODO) differs from the original here, by only storing a single byte.

2. Instructions for subroutine calls and jumps, and extended registers

   Different program addressing modes (call and jump instructions) have different reach inside the program address space.

   1. relative

      Instructions `RCALL` and `RJMP` provide a 12 bit constant `k` which specifies a relative offset from -2048 to 2047 program words (2 bytes) from the current PC. This covers the whole address space if `PAW <= 12` which gives a program address space of up to 8kB. So for program memory sizes up to 8kB there is no need for a dedicated direct addressing mode.

   2. indirect

      Instructions `ICALL` and `IJMP` use the 16 bit `Z` register to provide a 16 bit absolute address to be loaded into PC. This allows for addressing program memory of up to 128kB. So for program memory sizes up to 128kB there is no need for an extended indirect addressing mode.

   3. extended indirect

      Instructions `EICALL` and `EIJMP` use the 16 bit `Z` register and the 6 bit `EIMD` register to provide a 22 bit address to be loaded into PC. This allows for addressing program memory of up to 8MB.

   4. direct

      Instructions `CALL` and `JMP` are two words long and provide a 22 bit constant `k` to be loaded into PC. This allows for addressing program memory of up to 8MB.

3. Instructions for reading/writing data from/to program memory

   Instructions `LPM`, `ELPM` and `SPM` are used to load/store from/to program memory. All this instructions use the `Z` register for indirect addressing with byte granularity. Load operations transfer a single byte, the `Z` register LSB bit selects between the instruction word bytes (`0`-lower, `1`-higher). Store operations transfer an entire instruction word, the `Z` register LSB bit should be cleared. If there is more then 64kB of program memory the `Z` pointer must be extended by the `RAMPZ` register. The `ELPM` instruction used to access locations above 64kB. There is no dedicated extended instruction for stores, `RAMPZ` is used if available.

   With RP_8bit read access is compatible with Atmel AVR definitions while write access is simplified to offer word writes to memory, without protection or a Flash memory controller, but this can be added as additional peripherals by the user.


| size  | PAW   | program fetch      | program ld/st | PC | EIND   | RAMPZ | 
| -----:|:-----:| ------------------ | ------------- | --:| ------ | ----- |
|  512B |  0-8  | rel, ind           | ind           | 1B | /      | /     |
|   8kB |  8-11 | rel, ind           | ind           | 2B | /      | /     |
|  64kB | 12-15 | rel, ind, dir      | ind,          | 2B | /      | /     |
| 128kB | 16    | rel, ind, dir      | ind, ext      | 2B | /      | 8 bit |
|   8MB | 17-22 | rel, ind, dir, ext | ind, ext      | 3B | PAW-16 | 8 bit |

### Data address space size options

The size of the addressable data memory is defined by `parameter int unsigned DAW` (Data Address Width). The data memory is byte addressable, so the available program address space depending on the address width is:
```SystemVerilog
address_space_size = 2**PAW
```
The data address space contains the register file, primary and extended I/O space internal SRAM and external memories.

| data address        | I/O address | size      | contents              | addressing mode                   |
|:-------------------:|:-----------:| ---------:| --------------------- | --------------------------------- |
|   `0x0000-0x001f`   |             |       32B | register file         | register direct                   |
|   `0x0020-0x003f`   | `0x00-0x1f` |       32B | I/O space             | I/O direct (bit-addressable)      |
|   `0x0040-0x005f`   | `0x20-0x3f` |       32B | I/O space             | I/O direct                        |
|   `0x0060-0x00ff`   |             |      160B | extended I/O space    | data direct and indirect          |
|   `0x0100-0xffff`   |             | 64kB-256B | internal/external RAM | data direct and indirect          |
| `0x010000-0xffffff` |             | 16MB-64kB | internal/external RAM | extended data direct and indirect |

The minimal data address space contains 128B (`DAW>=7`). The upper limit is 64kB (`DAW<=16`) without extended registers and 16MB (`DAW<=24`) with extended registers.

Specifying a data address space larger then 64kB (`DAW>16`) will create extended registers.

1. direct addressing mode extended register `RAMPD`

   There are no special extended versions of direct addressing mode instructions. Instructions `LDS` and `STS` will use the `RAMPD` register if it is present.

2. indirect addressing mode extended registers `RAMPX`, `RAMPY`, `RAMPZ`

   There are no special extended versions of indirect addressing mode instructions. Instructions `LD` and `ST` will use the `RAMPD` register if it is present. Post-increment and pre-decrement modes will increment the concatenation of the `RAMPZ`, `RAMPY`, `RAMPZ` registers with the basic index registers `X`, `Y`, `Z`.

### TODO

1. It is not yet clear how many bits do `RAMPX`, `RAMPY`, `RAMPZ` have if `DAW-16` is greater then 0 but less then 8.
2. It is not yet clear how many bits of `X`, `Y`, `Z` are changed by pre-decrement post-increment instructions, if `DAW<16`.
