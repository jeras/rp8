NOTE, TODO: most of the listed configuration options are not yet implemented, for now this is used as documentation for a test plan.

## Core configurations:

The rp_8bit core can be configured to support some standard subsets of the full instruction set, program and data address space sizes can also be configured.

The next table specifies configuration options supported by the `gcc-avr` compiler.

TODO

### Reference devices

To achive compatibility with GNU AVR toolchain, configuration options are based on actual AVR devices.

| reference device     | core type | `PAW`  | `DAW` |
| -------------------- | --------- | ------ | ----- |
| ATtiny4/5/9/10/20/40 | reduced   | `8:11` |       |
| ATtiny11/12          | minimal   |||
| ATtiny13             | minimal   |||
| AT90S2313            | classic   | `10`   ||
| ATmega103            | classic   | `16`   ||
| ATmega8              | enhanced  | `12`   ||
| ATmega128            | enhanced  | `16`   ||

### Instruction set options

A subset of core types provided by Atmel is available. The minimal core will not be implemented, instead the reduced core should be used.

| core type |     |
| --------- | --- |
| reduced   | `R` |
| minimal   | `M` |
| classic   | `C` |
| enhanced  | `E` |
| xmega     | `X` |

The next table provides a list of all instructions and specifies which instruction is supported by which core.

| OPCODE                | mnemonic                |  core   | notes |
| --------------------- | ----------------------- | ------- | ----- |
| `0000_0000_0000_0000` | `NOP`                   | `RMCEX` ||
| `0000_0000_????_????` | undefined               | `.....` | if not `NOP` |
| `0000_0001_dddd_rrrr` | `MOVW   Rd+1:Rd,Rr+1Rr` | `...EX` ||
| `0000_0010_dddd_rrrr` | `MULS   Rd,Rr`          | `...EX` ||
| `0000_0011_0ddd_0rrr` | `MULSU  Rd,Rr`          | `...EX` ||
| `0000_0011_0ddd_1rrr` | `FMUL   Rd,Rr`          | `...EX` ||
| `0000_0011_1ddd_0rrr` | `FMULS  Rd,Rr`          | `...EX` ||
| `0000_0011_1ddd_1rrr` | `FMULSU Rd,Rr`          | `...EX` ||
| `0000_01rd_dddd_rrrr` | `CPC    Rd,Rr`          | `RMCEX` | also `LSL    Rd` |
| `0000_10rd_dddd_rrrr` | `SBC    Rd,Rr`          | `RMCEX` ||
| `0000_11rd_dddd_rrrr` | `ADD    Rd,Rr`          | `RMCEX` ||
| `0001_00rd_dddd_rrrr` | `CPSE   Rd,Rr`          | `RMCEX` ||
| `0001_01rd_dddd_rrrr` | `CP     Rd,Rr`          | `RMCEX` ||
| `0001_10rd_dddd_rrrr` | `SUB    Rd,Rr`          | `RMCEX` ||
| `0001_11rd_dddd_rrrr` | `ADC    Rd,Rr`          | `RMCEX` | also `ROL    Rd` |
| `0010_00rd_dddd_rrrr` | `AND    Rd,Rr`          | `RMCEX` | also `TST    Rd` |
| `0010_01rd_dddd_rrrr` | `EOR    Rd,Rr`          | `RMCEX` | also `CLR    Rd` |
| `0010_10rd_dddd_rrrr` | `OR     Rd,Rr`          | `RMCEX` ||
| `0010_11rd_dddd_rrrr` | `MOV    Rd,Rr`          | `RMCEX` ||
| `0011_KKKK_dddd_KKKK` | `CPI    Rd,K`           | `RMCEX` ||
| `0100_KKKK_dddd_KKKK` | `SBCI   Rd,K`           | `RMCEX` ||
| `0101_KKKK_dddd_KKKK` | `SUBI   Rd,K`           | `RMCEX` ||
| `0110_KKKK_dddd_KKKK` | `ORI    Rd,K`           | `RMCEX` | also `SBR    Rd,K` |
| `0111_KKKK_dddd_KKKK` | `ANDI   Rd,K`           | `RMCEX` | also `CBR    Rd,K` |
| `1000_000d_dddd_0000` | `LD     Rd,Z`           | `RMCEX` |                             |
| `1010_0kkk_dddd_kkkk` | `LDS    Rd,k`           | `R....` | instruction conflict        |
| `10q0_qq0d_dddd_0qqq` | `LDD    Rd,Z+q`         | `..CEX` | also `LD     Rd,Z` if `q=0` |
| `1000_000d_dddd_1000` | `LD     Rd,Y`           | `R.CEX` |                             |
| `10q0_qq0d_dddd_1qqq` | `LDD    Rd,Y+q`         | `..CEX` | also `LD     Rd,Y` if `q=0` |
| `1000_001r_rrrr_0000` | `ST     Z,Rr`           | `RMCEX` |                             |
| `1010_1kkk_rrrr_kkkk` | `STS    k,Rr`           | `R....` | instruction conflict        |
| `10q0_qq1r_rrrr_0qqq` | `STD    Z+q,Rr`         | `..CEX` | also `ST     Rd,Z` if `q=0` |
| `1000_001r_rrrr_1000` | `ST     Y,Rr`           | `R.CEX` |                             |
| `10q0_qq1r_rrrr_1qqq` | `STD    Y+q,Rr`         | `..CEX` | also `ST     Rd,Y` if `q=0` |
| `1001_000d_dddd_0000` | `LDS    Rd,0x????`      | `..CEX` | a 16 bit constant follows the instruction |
| `1001_000d_dddd_0001` | `LD     Rd,Z+`          | `R.CEX` ||
| `1001_000d_dddd_0010` | `LD     Rd,-Z`          | `R.CEX` ||
| `1001_000d_dddd_0011` | undefined               | `.....` ||
| `1001_000d_dddd_0100` | `LPM    Rd,Z`           | `...EX` ||
| `1001_000d_dddd_0101` | `LPM    Rd,Z+`          | `...EX` ||
| `1001_000d_dddd_0110` | `ELPM   Rd,Z`           | `...EX` ||
| `1001_000d_dddd_0111` | `ELPM   Rd,Z+`          | `...EX` ||
| `1001_000d_dddd_1000` | undefined               | `.....` ||
| `1001_000d_dddd_1001` | `LD     Rd,Y+`          | `R.CEX` ||
| `1001_000d_dddd_1010` | `LD     Rd,-Y`          | `R.CEX` ||
| `1001_000d_dddd_1011` | undefined               | `.....` ||
| `1001_000d_dddd_1100` | undefined               | `.....` ||
| `1001_000d_dddd_1101` | undefined               | `.....` ||
| `1001_000d_dddd_1110` | undefined               | `.....` ||
| `1001_000d_dddd_1111` | `POP    Rd`             | `R.CEX` ||
| `1001_001r_rrrr_0000` | `STS    0x????,Rr`      | `..CEX` | a 16 bit constant follows the instruction |
| `1001_001r_rrrr_0001` | `ST     Z+,Rr`          | `R.CEX` ||
| `1001_001r_rrrr_0010` | `ST     -Z,Rr`          | `R.CEX` ||
| `1001_001r_rrrr_0011` | undefined               | `.....` ||
| `1001_001r_rrrr_0100` | `XCH    Z,Rr`           | `....X` ||
| `1001_001r_rrrr_0101` | `LAS    Z,Rr`           | `....X` ||
| `1001_001r_rrrr_0110` | `LAC    Z,Rr`           | `....X` ||
| `1001_001r_rrrr_0111` | `LAT    Z,Rr`           | `....X` ||
| `1001_001r_rrrr_1000` | undefined               | `.....` ||
| `1001_001r_rrrr_1001` | `ST     Y+,Rr`          | `R.CEX` ||
| `1001_001r_rrrr_1010` | `ST     -Y,Rr`          | `R.CEX` ||
| `1001_001r_rrrr_1011` | undefined               | `.....` ||
| `1001_001r_rrrr_1100` | `ST     X,Rr`           | `R.CEX` ||
| `1001_001r_rrrr_1101` | `ST     X+,Rr`          | `R.CEX` ||
| `1001_001r_rrrr_1110` | `ST     -X,Rr`          | `R.CEX` ||
| `1001_001r_rrrr_1111` | `PUSH   Rr`             | `R.CEX` ||
| `1001_010d_dddd_0000` | `COM    Rd`             | `RMCEX` ||
| `1001_010d_dddd_0001` | `NEG    Rd`             | `RMCEX` ||
| `1001_010d_dddd_0010` | `SWAP   Rd`             | `RMCEX` ||
| `1001_010d_dddd_0011` | `INC    Rd`             | `RMCEX` ||
| `1001_010d_dddd_0100` | undefined               | `.....` ||
| `1001_010d_dddd_0101` | `ASR    Rd`             | `RMCEX` ||
| `1001_010d_dddd_0110` | `LSR    Rd`             | `RMCEX` ||
| `1001_010d_dddd_0111` | `ROR    Rd`             | `RMCEX` ||
| `1001_0100_0bbb_1000` | `BSET   b`              | `RMCEX` | also `SE[ITHSVNZC]` |
| `1001_0100_1bbb_1000` | `BCLR   b`              | `RMCEX` | also `CL[ITHSVNZC]` |
| `1001_0101_0000_1000` | `RET`                   | `RMCEX` ||
| `1001_0101_0001_1000` | `RETI`                  | `RMCEX` ||
| `1001_0101_0010_1000` | undefined               | `.....` ||
| `1001_0101_0011_1000` | undefined               | `.....` ||
| `1001_0101_0100_1000` | undefined               | `.....` ||
| `1001_0101_0101_1000` | undefined               | `.....` ||
| `1001_0101_0110_1000` | undefined               | `.....` ||
| `1001_0101_0111_1000` | undefined               | `.....` ||
| `1001_0101_1000_1000` | `SLEEP`                 | `RMCEX` ||
| `1001_0101_1001_1000` | `BREAK`                 | `R..EX` ||
| `1001_0101_1010_1000` | `WDR`                   | `RMCEX` ||
| `1001_0101_1011_1000` | undefined               | `.....` ||
| `1001_0101_1100_1000` | `LPM`                   | `.MCEX` ||
| `1001_0101_1101_1000` | `ELPM`                  | `...EX` | depends program memory size |
| `1001_0101_1110_1000` | `SPM `                  | `...EX` ||
| `1001_0101_1111_1000` | `SPM Z+`                | `...EX` ||
| `1001_0100_0000_1001` | `IJMP`                  | `RMCEX` ||
| `1001_0100_0001_1001` | `EIJMP`                 | `..CEX` | depends program memory size |
| `1001_0101_0000_1001` | `ICALL`                 | `RMCEX` ||
| `1001_0101_0001_1001` | `EICALL`                | `..CEX` | depends program memory size |
| `1001_010d_dddd_1010` | `DEC    Rd`             | `RMCEX` ||
| `1001_0100_KKKK_1011` | `DES    K`              | `....X` ||
| `1001_010K_KKKK_110K` | `JMP    0xK????`        | `..CEX` | TODO, depends program memory size |
| `1001_010K_KKKK_111K` | `CALL   0xK????`        | `..CEX` | TODO, depends program memory size |
| `1001_0110_KKdd_KKKK` | `ADIW   Rd+1:Rd,K`      | `...EX` ||
| `1001_0111_KKdd_KKKK` | `SBIW   Rd+1:Rd,K`      | `...EX` ||
| `1001_1000_AAAA_?bbb` | `CBI    A,b`            | `RMCEX` ||
| `1001_1001_AAAA_?bbb` | `SBIC   A,b`            | `RMCEX` ||
| `1001_1010_AAAA_?bbb` | `SBI    A,b`            | `RMCEX` ||
| `1001_1011_AAAA_?bbb` | `SBIS   A,b`            | `RMCEX` ||
| `1001_11rd_dddd_rrrr` | `MUL    Rd,Rr`          | `...EX` ||
| `1011_0AAd_dddd_AAAA` | `IN     Rd,A`           | `RMCEX` ||
| `1011_1AAd_dddd_AAAA` | `OUT    A,Rd`           | `RMCEX` ||
| `1100_kkkk_kkkk_kkkk` | `RJMP   k`              | `RMCEX` ||
| `1101_kkkk_kkkk_kkkk` | `RCALL  k`              | `RMCEX` ||
| `1110_KKKK_dddd_KKKK` | `LDI    Rd,K`           | `RMCEX` | also `SER    Rd` |
| `1111_00kk_kkkk_kbbb` | `BRBS   b,k`            | `RMCEX` | also `BR[ITHSVNZC]S` |
| `1111_01kk_kkkk_kbbb` | `BRBC   b,k`            | `RMCEX` | also `BR[ITHSVNZC]C` |
| `1111_100d_dddd_0bbb` | `BLD    Rd,b`           | `RMCEX` ||
| `1111_101d_dddd_0bbb` | `BST    Rd,b`           | `RMCEX` ||
| `1111_110d_dddd_0bbb` | `SBRC   Rd,b`           | `RMCEX` ||
| `1111_111d_dddd_0bbb` | `SBRS   Rd,b`           | `RMCEX` ||

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
