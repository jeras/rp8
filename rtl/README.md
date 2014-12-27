Core configurations:

1. reduced
2. minimal
4. clasic
6. enhanced
7. xmega

Instruction configuration:
- enable ADIW, SBIW
- enable \*MUL\* and MOVW

| core     |    |    |
| -------- | -- | -- |
| reduced  |    |    |
| minimal  |    |    |
| clasic   |    |    |
| enhanced |    |    |
| xmega    |    |    |

Program memory options:

There are instructions and I/O registers which depend on the size of the Program memory:
it also affects subrutine call/return times and interrupt response times.

| memory size        | param PAW | instructions                            | program counter | extended indirect register | 
| ------------------ | --------- | --------------------------------------- | --------------- | -------------------------- |
|   0KB < x <=   1kB |  0 -  8   | RCALL, RJMP                             | PC[PAW-1:0]     |                            |
|   1KB < x <=   8kB |  8 - 12   | RCALL, RJMP                             | PC[PAW-1:0]     |                            |
|   8kB < x <= 128kB | 13 - 16   | RCALL, RJMP, ICALL, IJMP                | PC[PAW-1:0]     |                            |
| 128kB < x <=   4MB | 17 - 22   | RCALL, RJMP, ICALL, IJMP, EICALL, EIJMP | PC[PAW-1:0]     | EIND [PAW-16-1:0], RAMPZ   |

