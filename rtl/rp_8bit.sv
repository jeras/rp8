/*
 * RP8 processor core
 * Copyright (C) 2014, 2014 Iztok Jeras
 * Copyright (C) 2007, 2008, 2009, 2010 Sebastien Bourdeauducq
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

module rp_8bit #(
  parameter int unsigned IRW =  8,  // interrupt request width
  parameter int unsigned PAW = 11,  // program address width (16 bit words)
  parameter int unsigned DAW = 13,  // data    address width ( 8 bit bytes)
  parameter int unsigned SAW = DAW, // stack   address width (can be less then DAW)
//  parameter bit [16-1:0] SPR = 16'hffff   // stack pointer reset value
  parameter bit [16-1:0] SPR = 16'h10ff   // stack pointer reset value
)(
  // system signals
  input  logic           clk,
  input  logic           rst,
  // program bus
  output logic           bp_vld, // valid (address, write enable, write data)
  output logic           bp_wen, // write enable
  output logic [PAW-1:0] bp_adr, // address
  output logic  [16-1:0] bp_wdt, // write data
  input  logic  [16-1:0] bp_rdt, // read data
  input  logic [PAW-1:0] bp_npc, // new PC
  input  logic           bp_jmp, // debug jump request
  input  logic           bp_rdy, // ready (read data, new PC, debug jump request)
  // data bus
  output logic           bd_req, // request
  output logic [DAW-1:0] bd_adr, // address
  output logic           bd_wen, // write enable
  output logic   [6-1:0] bd_wid, // write identification
  output logic   [8-1:0] bd_wdt, // write data
  input  logic   [8-1:0] bd_rdt, // read data
  input  logic   [6-1:0] bd_rid, // read identification
  input  logic           bd_ren, // read enable
  input  logic           bd_ack, // acknowledge
  // I/O peripheral bus
  output logic           io_wen, // write enable
  output logic           io_ren, // read  enable
  output logic   [6-1:0] io_adr, // address
  output logic   [8-1:0] io_wdt, // write data
  output logic   [8-1:0] io_msk, // write mask
  input  logic   [8-1:0] io_rdt, // read data
  // interrupt
  input  logic [IRW-1:0] irq_req,
  output logic [IRW-1:0] irq_ack,
  // control outputs
  output logic           ctl_slp, // sleep
  output logic           ctl_brk, // break
  output logic           ctl_wdr  // watch dog reset
);

////////////////////////////////////////////////////////////////////////////////
// generic type constants
////////////////////////////////////////////////////////////////////////////////

// bit constants
localparam logic          CX = 1'bx;
localparam logic          C0 = 1'b0;
localparam logic          C1 = 1'b1;

// byte (8 bit) constants
localparam logic  [8-1:0] BX = 8'hxx;
localparam logic  [8-1:0] B0 = 8'h00;
localparam logic  [8-1:0] BF = 8'hff;

// word (16 bit) constants
localparam logic [16-1:0] WX = 16'hxxxx;
localparam logic [16-1:0] W0 = 16'h0000;
localparam logic [16-1:0] WF = 16'hffff;

// extended (24 bit) constants
localparam logic [24-1:0] EX = 24'hxxxxxx;
localparam logic [24-1:0] E0 = 24'h000000;
localparam logic [24-1:0] EF = 24'hffffff;

////////////////////////////////////////////////////////////////////////////////
// helper functions
////////////////////////////////////////////////////////////////////////////////

// binary to one hot
function logic [8-1:0] b2o (input logic [3-1:0] b);
  b2o = 8'h01 << b;
endfunction: b2o

// extend byte
// TODO: undefined bits are causing trouble in ALU
function logic [24-1:0] feb (input logic [8-1:0] b);
  feb = {15'bx, 1'b0, b};
endfunction: feb

// extend word
// TODO: undefined bits are causing trouble in ALU
function logic [24-1:0] few (input logic [16-1:0] w);
  few = {7'bx, 1'b0, w};
endfunction: few

////////////////////////////////////////////////////////////////////////////////
// type definitions
////////////////////////////////////////////////////////////////////////////////

// register file structure
typedef union packed {
  logic [32-1:0] [8-1:0] idx;
  struct packed {
    logic             [16-1:0] z, y, x;
    logic [32-2*3-1:0] [8-1:0] r;
  } nam;
} gpr_t;

// program counter type (program memory address)
typedef struct packed {
  logic [6-1:0] e;
  logic [8-1:0] h;
  logic [8-1:0] l;
} pc_t;

// load/store type (data memory address)
typedef struct packed {
  logic [8-1:0] e;
  logic [8-1:0] h;
  logic [8-1:0] l;
} ls_t;

// stack pointer (data memory address)
typedef struct packed {
  logic [8-1:0] h;
  logic [8-1:0] l;
} sp_t;

// general purpose register address
typedef logic [5-1:0] gpr_adr_t;

// I/O space address
typedef logic [6-1:0] iou_adr_t;

// data transfer identification
typedef struct packed {
  logic         m; // mode (0 - GPR, 1 - PC)
  logic [5-1:0] i; // index (R[31:0] or PC[2:0])
} id_t;

// status register
typedef struct packed {logic i, t, h, s, v, n, z, c;} sreg_t;

// core state machine
typedef enum logic [3-1:0] {
  RST = 3'b000, // reset
  NRM = 3'b001, // normal
  SKP = 3'b010, // skip
  P32 = 3'b011, // instruction 32 bit long
  STL = 3'b100, // stall
  WBK = 3'b101  // writeback
} stm_t;

// general purpose registers decode structure
typedef struct packed {
  // write access
  logic          we; // write enable
  logic          ww; // write word (0 - 8 bit mode, 1 - 16 bit mode)
  logic [16-1:0] wd; // write data 16 bit
  gpr_adr_t      wa; // write address 
  gpr_adr_t      rw; // read word (16 bit) address
  gpr_adr_t      rb; // read byte (8 bit) address
} gpr_dec_t;

// arithmetic logic unit decode structure
typedef struct packed {
  // TODO: check if a different encoding might be more optimal, better aligned to instruction decoder
  enum logic [3-1:0] {
    ADD = 3'b000, // addition
    SUB = 3'b001, // subtraction
    ADW = 3'b010, // addition    for word or address
    SBW = 3'b011, // subtraction for word or address
    AND = 3'b100, // logic and
    OR  = 3'b101, // logic or
    EOR = 3'b110, // logic eor
    SHR = 3'b111  // shift right
  } m;               // alu modes
  logic [24-1:0] d; // destination operand value
  logic [24-1:0] r; // source      operand value
  logic          c; // carry input
} alu_dec_t;

// multiplier decode structure
typedef struct packed {
  struct packed {
    logic f; // fractional
    logic d; // destination (0 - unsigned, 1 - signed)
    logic r; // source      (0 - unsigned, 1 - signed)
  } m;               // adder modes
  logic  [8-1:0] d; // destination operand value
  logic  [8-1:0] r; // source      operand value
} mul_dec_t;

// status register decode structure
typedef struct packed {
  sreg_t         s; // status
  sreg_t         m; // mask
} srg_dec_t;

// instruction fetch unit decode structure
typedef struct packed {
  logic          ii; // instruction immediate
  logic          sk; // skip
  logic          be; // branch enable
  pc_t           ad; // address
  logic          we; // write enable (for SPM instruction)
  logic [16-1:0] wd; // write data   (for SPM instruction)
} ifu_dec_t;

// input/output unit decode structure
typedef struct packed {
  logic          we; // write enable
  logic          re; // read  enable
  iou_adr_t      ad; // address
  logic  [8-1:0] wd; // write data
  logic  [8-1:0] ms; // write mask
} iou_dec_t;

// load/store unit decode structure
typedef struct packed {
  logic          en; // enable
  logic          we; // write enable
  logic          st; // stack push/pop
  logic          sb; // subroutine/interrupt call/return
  logic [24-1:0] ad; // address
  logic  [8-1:0] wd; // write data
  gpr_adr_t      dr; // destination register (if not PC)
} lsu_dec_t;

// control decode structure
typedef struct packed {
  logic          slp; // sleep
  logic          brk; // break
  logic          wdr; // watchdog reset
} ctl_dec_t;

// entire decode structure
typedef struct packed {
  gpr_dec_t gpr; // general purpose registers
  alu_dec_t alu; // arithmetic logic unit
  mul_dec_t mul; // multiplier
  srg_dec_t srg; // status register
  ifu_dec_t ifu; // instruction fetch unit
  iou_dec_t iou; // input/output unit
  lsu_dec_t lsu; // load/store unit
  ctl_dec_t ctl; // control
} dec_t;

////////////////////////////////////////////////////////////////////////////////
// calculated parameters
////////////////////////////////////////////////////////////////////////////////

// maximum address width
localparam int unsigned MAW = PAW > DAW ? PAW : DAW;

// adder destination width
localparam int unsigned AW = MAW > 16 ? MAW : 16;

// program (instruction fetch) address mask
localparam pc_t PAM = (1<<PAW)-1;
// data (load/store) address mask
localparam pc_t DAM = (1<<DAW)-1;
// stack address mask

// TODO
localparam bit [2-1:0] PCN = 2'd2;

////////////////////////////////////////////////////////////////////////////////
// AVR architecture constants
////////////////////////////////////////////////////////////////////////////////

// general purpose register addresses
localparam gpr_adr_t RX = 5'hxx; // undefined register
localparam gpr_adr_t R0 = 5'h00; // R1:R0 used for multiplication destination address
localparam gpr_adr_t DX = 5'h1a; // index register X
localparam gpr_adr_t DY = 5'h1c; // index register Y
localparam gpr_adr_t DZ = 5'h1e; // index register Z

// I/O register addresses
localparam iou_adr_t IOA_RAMPD = 6'h30 + 6'h08;
localparam iou_adr_t IOA_RAMPX = 6'h30 + 6'h09;
localparam iou_adr_t IOA_RAMPY = 6'h30 + 6'h0a;
localparam iou_adr_t IOA_RAMPZ = 6'h30 + 6'h0b;
localparam iou_adr_t IOA_EIND  = 6'h30 + 6'h0c;
localparam iou_adr_t IOA_SPL   = 6'h30 + 6'h0d;
localparam iou_adr_t IOA_SPH   = 6'h30 + 6'h0e;
localparam iou_adr_t IOA_SREG  = 6'h30 + 6'h0f;

////////////////////////////////////////////////////////////////////////////////
// local variables
////////////////////////////////////////////////////////////////////////////////

// core state machine
stm_t stm_sts; // current state

// core state registers
pc_t           pc;    // program counter
pc_t           pcn;   // program counter next (PC+1)
pc_t           pcs;   // program counter from stack
sp_t           sp;    // stack pointer
sp_t           spi;   // stack pointer increment
sp_t           spd;   // stack pointer decrement
sreg_t         sreg;  // status register
gpr_t          gpr;   // register file

// core stall
logic          stl;

// access conflicts:
// - arithmetic/logic/IO/move/bitmod operation conflicting with load writeback on GPR
// - arithmetic/logic/IO/move/bitmod operation conflicting with ?
// call/ret push/pop conflicting with SP I/O operations

// core state machine
logic          writeback;

// program word
logic [16-1:0] pw; // multiplexed
logic [16-1:0] pi; // input
logic [16-1:0] pr; // registered

// decoder structure
dec_t          dec;

// destination/source register address for:
gpr_adr_t      db, rb; // full space bytes (used by MOV and arithmetic)
gpr_adr_t      dw, rw; // full space words (used by MOVW)
gpr_adr_t      dh, rh; // high half space (used by MULS, arithmetic immediate, load store direct)
gpr_adr_t      dm, rm; // third quarter space (used by *MUL*)
gpr_adr_t      di;     // index registers (used by ADIW/SBIW)

// various immediate constans decoded from instruction word
logic         [8-1:0] kb; // byte (8 bit) immediate for ALU operations
logic         [6-1:0] kw; // word (6bit) for address adder
logic         [6-1:0] a ; // I/O address
logic signed [12-1:0] Kl;
logic signed  [7-1:0] Ks;
logic         [6-1:0] q ;
logic         [3-1:0] b ; // bit address

// reusable_results
logic Rd_b;

// read register values
logic  [8-1:0] Rd; // destination
logic  [8-1:0] Rr; // source
logic [16-1:0] Rw; // word
logic  [8-1:0] Rs; // nibble swap of Rd

// I/O read value
logic  [8-1:0] id;

// ALU results
logic [24-1:0] alu_t ; // result (full width plus carry)
logic  [8-1:0] alu_rb; // result for byte operations ( 8 bit)
logic [16-1:0] alu_rw; // result for word operations (16 bit)
sreg_t         alu_sb; // status for byte operations ( 8 bit)
sreg_t         alu_sw; // status for word operations (16 bit)
sreg_t         alu_s ; // status

// multiplication results
logic [18-1:0] mul_t; // result tmp
logic [16-1:0] mul_r; // result
sreg_t         mul_s;

// SPR (special purpose registers)
logic  [8-1:0] rampd;
logic  [8-1:0] rampx;
logic  [8-1:0] rampy;
logic  [8-1:0] rampz;
logic  [6-1:0] eind ;

// instruction fetch unit status
logic          ifu_blk; // block
logic          ifu_con; // continue

// load store unit status
logic          lsu_req;
logic          lsu_blk; // block execution
logic          lsu_end; // transfer counter end
logic  [2-1:0] lsu_cnt; // transfer counter (used for pushing PC to stack)
logic [24-1:8] lsu_buf; // temporary buffer for PC value during CALL/RET

// extended data memory direct/indirect addressing
ls_t           ed, ex, ey, ez; // register concatenations
ls_t           ea;             // output from ALU
// extended program memory indirect addressing
pc_t           ei;

////////////////////////////////////////////////////////////////////////////////
// register addresses and immediates
////////////////////////////////////////////////////////////////////////////////

// destination/source register address for full space bytes (used by MOV and arithmetic)
assign db =         pw[8:4] ;
assign rb = {pw[9], pw[3:0]};
// destination/source register address for full space words (used by MOVW)
assign dw =        {pw[7:4], 1'b0};
assign rw =        {pw[3:0], 1'b0};
// destination/source register address for high half space (used by MULS, arithmetic immediate, load store direct)
assign dh =  {1'b1, pw[7:4]};
assign rh =  {1'b1, pw[3:0]};
// destination/source register address for third quarter space (used by *MUL*)
assign dm = {2'b10, pw[6:4]};
assign rm = {2'b10, pw[2:0]};
// destination register address for index registers (used by ADIW/SBIW)
assign di = {2'b11, pw[5:4], 1'b0};

// byte (8 bit) immediate for ALU operations
assign kb = {pw[11:8], pw[3:0]};
// word (6bit) for address adder
assign kw = {pw[7:6], pw[3:0]};

// I/O address
assign a = {pw[10:9], pw[3:0]};

assign Kl = pw[11:0];
assign Ks = pw[ 9:3];
assign q = {pw[13], pw[11:10], pw[2:0]};

// bit address
assign b  = pw[2:0];

// reusable_results
assign Rd_b = Rd[b];

////////////////////////////////////////////////////////////////////////////////
// instruction decoder
////////////////////////////////////////////////////////////////////////////////

// constants for idling units
localparam gpr_dec_t GPR = '{we: C0, ww: C0, wd: WX, wa: RX, rb: RX, rw: RX};
localparam alu_dec_t ALU = '{m: 3'bxxx, d: EX, r: EX, c: CX};
localparam mul_dec_t MUL = '{m: 3'bxxx, d: BX, r: BX};
localparam srg_dec_t SRG = '{s: BX, m: B0};
localparam ifu_dec_t IFU = '{ii: C0, sk: C0, be: C0, ad: 22'hx, we: C0, wd: WX};
localparam iou_dec_t IOU = '{we: C0, re: C0, ad: 6'hxx, wd: BX, ms: BX};
localparam lsu_dec_t LSU = '{en: C0, we: CX, st: CX, sb: CX, ad: EX, wd: BX, dr: RX};
localparam ctl_dec_t CTL = '{slp: C0, brk: C0, wdr: C0};

always_comb
unique casez (pw)
  // no operation, same as default
  16'b0000_0000_0000_0000: begin dec = '{ GPR, ALU, MUL, SRG, IFU, IOU, LSU, CTL }; end // NOP
  // control instructions
  //                                    {                                     ctl          }
  //                                    {                                     {slp brk wdr}}
  16'b1001_0101_1000_1000: begin dec = '{ GPR, ALU, MUL, SRG, IFU, IOU, LSU, '{C1, C0, C0} }; end // SLEEP
  16'b1001_0101_1001_1000: begin dec = '{ GPR, ALU, MUL, SRG, IFU, IOU, LSU, '{C0, C1, C0} }; end // BREAK
  16'b1001_0101_1010_1000: begin dec = '{ GPR, ALU, MUL, SRG, IFU, IOU, LSU, '{C0, C0, C1} }; end // WDR
  // arithmetic
  //                                    {  gpr                                 alu                                    srg                                }
  //                                    {  {we, ww, wd         , wa, rw, rb}   {m  , d      , r      , c     }        {s    , m    }                     }
  16'b0000_01??_????_????: begin dec = '{ '{C0, CX, WX         , RX, db, rb}, '{SUB, feb(Rd), feb(Rr), sreg.c}, MUL, '{alu_s, 8'h3f}, IFU, IOU, LSU, CTL }; end // CPC
  16'b0000_10??_????_????: begin dec = '{ '{C1, C0, {2{alu_rb}}, db, db, rb}, '{SUB, feb(Rd), feb(Rr), sreg.c}, MUL, '{alu_s, 8'h3f}, IFU, IOU, LSU, CTL }; end // SBC
  16'b0000_11??_????_????: begin dec = '{ '{C1, C0, {2{alu_rb}}, db, db, rb}, '{ADD, feb(Rd), feb(Rr), C0    }, MUL, '{alu_s, 8'h3f}, IFU, IOU, LSU, CTL }; end // ADD
  16'b0001_01??_????_????: begin dec = '{ '{C0, C0, WX         , RX, db, rb}, '{SUB, feb(Rd), feb(Rr), C0    }, MUL, '{alu_s, 8'h3f}, IFU, IOU, LSU, CTL }; end // CP
  16'b0001_10??_????_????: begin dec = '{ '{C1, C0, {2{alu_rb}}, db, db, rb}, '{SUB, feb(Rd), feb(Rr), C0    }, MUL, '{alu_s, 8'h3f}, IFU, IOU, LSU, CTL }; end // SUB
  16'b0001_11??_????_????: begin dec = '{ '{C1, C0, {2{alu_rb}}, db, db, rb}, '{ADD, feb(Rd), feb(Rr), sreg.c}, MUL, '{alu_s, 8'h3f}, IFU, IOU, LSU, CTL }; end // ADC
  16'b0011_????_????_????: begin dec = '{ '{C0, CX, WX         , RX, dw, RX}, '{SUB, feb(Rd), feb(kb), C0    }, MUL, '{alu_s, 8'h3f}, IFU, IOU, LSU, CTL }; end // CPI
  16'b0100_????_????_????: begin dec = '{ '{C1, C0, {2{alu_rb}}, dw, dw, RX}, '{SUB, feb(Rd), feb(kb), sreg.c}, MUL, '{alu_s, 8'h3f}, IFU, IOU, LSU, CTL }; end // SBCI
  16'b0101_????_????_????: begin dec = '{ '{C1, C0, {2{alu_rb}}, dw, dw, RX}, '{SUB, feb(Rd), feb(kb), C0    }, MUL, '{alu_s, 8'h3f}, IFU, IOU, LSU, CTL }; end // SUBI
  16'b1001_010?_????_0000: begin dec = '{ '{C1, C0, {2{alu_rb}}, db, db, RX}, '{SUB, feb(BF), feb(Rd), C0    }, MUL, '{alu_s, 8'h1f}, IFU, IOU, LSU, CTL }; end // COM
  // TODO check the value of carry and overflow
  16'b1001_010?_????_0001: begin dec = '{ '{C1, C0, {2{alu_rb}}, db, db, RX}, '{SUB, feb(B0), feb(Rd), C0    }, MUL, '{alu_s, 8'h3f}, IFU, IOU, LSU, CTL }; end // NEG
  16'b1001_010?_????_0011: begin dec = '{ '{C1, C0, {2{alu_rb}}, db, db, RX}, '{ADD, feb(Rd), feb(B0), C1    }, MUL, '{alu_s, 8'h3e}, IFU, IOU, LSU, CTL }; end // INC
  16'b1001_010?_????_1010: begin dec = '{ '{C1, C0, {2{alu_rb}}, db, db, RX}, '{SUB, feb(Rd), feb(B0), C1    }, MUL, '{alu_s, 8'h3e}, IFU, IOU, LSU, CTL }; end // DEC
  // logic // TODO check flags
  16'b0010_00??_????_????: begin dec = '{ '{C1, C0, {2{alu_rb}}, db, db, rb}, '{AND, feb(Rd), feb(Rr), C0    }, MUL, '{alu_s, 8'h1e}, IFU, IOU, LSU, CTL }; end // AND
  16'b0111_????_????_????: begin dec = '{ '{C1, C0, {2{alu_rb}}, dw, dw, RX}, '{AND, feb(Rd), feb(kb), C0    }, MUL, '{alu_s, 8'h1e}, IFU, IOU, LSU, CTL }; end // ANDI
  16'b0010_10??_????_????: begin dec = '{ '{C1, C0, {2{alu_rb}}, db, db, rb}, '{OR , feb(Rd), feb(Rr), C0    }, MUL, '{alu_s, 8'h1e}, IFU, IOU, LSU, CTL }; end // OR
  16'b0110_????_????_????: begin dec = '{ '{C1, C0, {2{alu_rb}}, dw, dw, RX}, '{OR , feb(Rd), feb(kb), C0    }, MUL, '{alu_s, 8'h1e}, IFU, IOU, LSU, CTL }; end // ORI
  16'b0010_01??_????_????: begin dec = '{ '{C1, C0, {2{alu_rb}}, db, db, rb}, '{EOR, feb(Rd), feb(Rr), C0    }, MUL, '{alu_s, 8'h1e}, IFU, IOU, LSU, CTL }; end // EOR
  // shift right // TODO check flags
  16'b1001_010?_????_0110: begin dec = '{ '{C1, C0, {2{alu_rb}}, db, db, RX}, '{SHR, feb(Rd), feb(B0), C0    }, MUL, '{alu_s, 8'h1f}, IFU, IOU, LSU, CTL }; end // LSR
  16'b1001_010?_????_0111: begin dec = '{ '{C1, C0, {2{alu_rb}}, db, db, RX}, '{SHR, feb(Rd), feb(B0), sreg.c}, MUL, '{alu_s, 8'h1f}, IFU, IOU, LSU, CTL }; end // ROR
  16'b1001_010?_????_0101: begin dec = '{ '{C1, C0, {2{alu_rb}}, db, db, RX}, '{SHR, feb(Rd), feb(B0), Rd[7] }, MUL, '{alu_s, 8'h1f}, IFU, IOU, LSU, CTL }; end // ASR
  // 16 bit addition
  16'b1001_0110_????_????: begin dec = '{ '{C1, C1, alu_rw     , di, di, RX}, '{ADW, few(Rw), kw     , C0    }, MUL, '{alu_s, 8'h1f}, IFU, IOU, LSU, CTL }; end // ADIW
  16'b1001_0111_????_????: begin dec = '{ '{C1, C1, alu_rw     , di, di, RX}, '{SBW, few(Rw), kw     , C0    }, MUL, '{alu_s, 8'h1f}, IFU, IOU, LSU, CTL }; end // SBIW
  // multiplication
  //                                    {  gpr                                mul                       srg                                }
  //                                    {  {we, ww, wd   , wa, rw, rb}        {m{f , d , r }, d , r }   {s    , m    }                     }
  16'b1001_11??_????_????: begin dec = '{ '{C1, C1, mul_r, R0, db, rb}, ALU, '{'{C0, C0, C0}, Rd, Rr}, '{mul_s, 8'h03}, IFU, IOU, LSU, CTL }; end // MUL
  16'b0000_0010_????_????: begin dec = '{ '{C1, C1, mul_r, R0, dh, rh}, ALU, '{'{C0, C1, C1}, Rd, Rr}, '{mul_s, 8'h03}, IFU, IOU, LSU, CTL }; end // MULS
  16'b0000_0011_0???_0???: begin dec = '{ '{C1, C1, mul_r, R0, dm, rm}, ALU, '{'{C0, C1, C0}, Rd, Rr}, '{mul_s, 8'h03}, IFU, IOU, LSU, CTL }; end // MULSU
  16'b0000_0011_0???_1???: begin dec = '{ '{C1, C1, mul_r, R0, dm, rm}, ALU, '{'{C1, C0, C0}, Rd, Rr}, '{mul_s, 8'h03}, IFU, IOU, LSU, CTL }; end // FMUL
  16'b0000_0011_1???_0???: begin dec = '{ '{C1, C1, mul_r, R0, dm, rm}, ALU, '{'{C1, C1, C1}, Rd, Rr}, '{mul_s, 8'h03}, IFU, IOU, LSU, CTL }; end // FMULS
  16'b0000_0011_1???_1???: begin dec = '{ '{C1, C1, mul_r, R0, dm, rm}, ALU, '{'{C1, C1, C0}, Rd, Rr}, '{mul_s, 8'h03}, IFU, IOU, LSU, CTL }; end // FMULSU
  // register moves
  //                                    {  gpr                                                              }
  //                                    {  {we, ww, wd     , wa, rw, rb}                                    }
  16'b0000_0001_????_????: begin dec = '{ '{C1, C1, Rw     , dw, rw, RX}, ALU, MUL, SRG, IFU, IOU, LSU, CTL }; end // MOVW
  16'b0010_11??_????_????: begin dec = '{ '{C1, C0, {2{Rr}}, db, db, rb}, ALU, MUL, SRG, IFU, IOU, LSU, CTL }; end // MOV
  16'b1110_????_????_????: begin dec = '{ '{C1, C0, {2{kb}}, dh, RX, RX}, ALU, MUL, SRG, IFU, IOU, LSU, CTL }; end // LDI
  16'b1001_010?_????_0010: begin dec = '{ '{C1, C0, {2{Rs}}, db, db, RX}, ALU, MUL, SRG, IFU, IOU, LSU, CTL }; end // SWAP
  // SREG bit manipulation
  //                                    {                 srg                                    }
  //                                    {                 {s , m           }                     }
  16'b1001_0100_0???_1000: begin dec = '{ GPR, ALU, MUL, '{BF, b2o(pw[6:4])}, IFU, IOU, LSU, CTL }; end // BSET // create a common source instead of two b2o functions
  16'b1001_0100_1???_1000: begin dec = '{ GPR, ALU, MUL, '{B0, b2o(pw[6:4])}, IFU, IOU, LSU, CTL }; end // BCLR
  // GPR bit manipulation
  16'b1111_101?_????_0???: begin dec = '{ '{C0, CX, WX                                      , RX, db, RX}, ALU, MUL, '{{CX,Rd_b,6'hxx}, 8'h40}, IFU, IOU, LSU, CTL }; end // SBT
  16'b1111_100?_????_0???: begin dec = '{ '{C1, C0, {2{Rd & ~b2o(b) | {8{sreg.t}} & b2o(b)}}, db, db, RX}, ALU, MUL, SRG                      , IFU, IOU, LSU, CTL }; end // BLD  // TODO: ALU could be used
  // stack access
  //                                    {  gpr                                                 lsu                               }
  //                                    {  {we, ww, wd, wa, rw, rb}                            {en, we, st, sb, ad, wd, rd}      }
  16'b1001_000?_????_1111: begin dec = '{ GPR                      , ALU, MUL, SRG, IFU, IOU, '{C1, C0, C1, C0, 'x, BX, db}, CTL }; end // POP
  16'b1001_001?_????_1111: begin dec = '{ '{C0, CX, WX, RX, db, RX}, ALU, MUL, SRG, IFU, IOU, '{C1, C1, C1, C0, 'x, Rd, RX}, CTL }; end // PUSH
  // load store direct (32 bit instructions)
  //                                    {  gpr                                       ifu                             lsu                               }
  //                                    {  {we, ww, wd, wa, rw, rb}                  {ii, sk, be, ad, we, wd}        {en, we, st, sb, ad, wd, rd}      }
  16'b1001_000?_????_0000: begin dec = '{ GPR                      , ALU, MUL, SRG, '{C1, C0, C0, 'x, C0, WX}, IOU, '{C1, C0, C1, C0, ed, BX, db}, CTL }; end // LDS
  16'b1001_001?_????_0000: begin dec = '{ '{C0, CX, WX, RX, db, RX}, ALU, MUL, SRG, '{C1, C0, C0, 'x, C0, WX}, IOU, '{C1, C1, C1, C0, ed, Rd, RX}, CTL }; end // STS
  // load store indirect
  //                                    {  gpr                            alu                                     lsu                               }
  //                                    {  {we, ww, wd    , wa, rw, rb}   {m  , d , r , c }                       {en, we, st, sb, ad, wd, rd}      }
  16'b1001_000?_????_1100: begin dec = '{ '{C0, CX, WX    , RX, DX, RX}, ALU               , MUL, SRG, IFU, IOU, '{C1, C0, C0, C0, ex, BX, db}, CTL }; end // LD X
  16'b1001_001?_????_1100: begin dec = '{ '{C0, CX, WX    , RX, db, RX}, ALU               , MUL, SRG, IFU, IOU, '{C1, C1, C0, C0, ex, Rd, RX}, CTL }; end // ST X
  16'b1001_000?_????_1101: begin dec = '{ '{C1, C1, alu_rw, DX, DX, RX}, '{ADW, ex, E0, C1}, MUL, SRG, IFU, IOU, '{C1, C0, C0, C0, ex, BX, db}, CTL }; end // LD X+
  16'b1001_001?_????_1101: begin dec = '{ '{C1, C1, alu_rw, DX, db, RX}, '{ADW, ex, E0, C1}, MUL, SRG, IFU, IOU, '{C1, C1, C0, C0, ex, Rd, RX}, CTL }; end // ST X+
  16'b1001_000?_????_1110: begin dec = '{ '{C1, C1, alu_rw, DX, DX, RX}, '{SBW, ex, E0, C1}, MUL, SRG, IFU, IOU, '{C1, C0, C0, C0, ea, BX, db}, CTL }; end // LD -X
  16'b1001_001?_????_1110: begin dec = '{ '{C1, C1, alu_rw, DX, db, RX}, '{SBW, ex, E0, C1}, MUL, SRG, IFU, IOU, '{C1, C1, C0, C0, ea, Rd, RX}, CTL }; end // ST -X
  16'b10?0_??0?_????_1???: begin dec = '{ '{C0, CX, WX    , RX, DY, RX}, '{ADW, ey, q , C0}, MUL, SRG, IFU, IOU, '{C1, C0, C0, C0, ea, BX, db}, CTL }; end // LDD Y+q
  16'b10?0_??1?_????_1???: begin dec = '{ '{C0, CX, WX    , RX, db, RX}, '{ADW, ey, q , C0}, MUL, SRG, IFU, IOU, '{C1, C1, C0, C0, ea, Rd, RX}, CTL }; end // STD Y+q
  16'b1001_000?_????_1001: begin dec = '{ '{C1, C1, alu_rw, DY, DY, RX}, '{ADW, ey, E0, C1}, MUL, SRG, IFU, IOU, '{C1, C0, C0, C0, ey, BX, db}, CTL }; end // LD Y+
  16'b1001_001?_????_1001: begin dec = '{ '{C1, C1, alu_rw, DY, db, RX}, '{ADW, ey, E0, C1}, MUL, SRG, IFU, IOU, '{C1, C1, C0, C0, ey, Rd, RX}, CTL }; end // ST Y+
  16'b1001_000?_????_1010: begin dec = '{ '{C1, C1, alu_rw, DY, DY, RX}, '{SBW, ey, E0, C1}, MUL, SRG, IFU, IOU, '{C1, C0, C0, C0, ea, BX, db}, CTL }; end // LD -Y
  16'b1001_001?_????_1010: begin dec = '{ '{C1, C1, alu_rw, DY, db, RX}, '{SBW, ey, E0, C1}, MUL, SRG, IFU, IOU, '{C1, C1, C0, C0, ea, Rd, RX}, CTL }; end // ST -Y
  16'b10?0_??0?_????_0???: begin dec = '{ '{C0, CX, WX    , RX, DZ, RX}, '{ADW, ez, q , C0}, MUL, SRG, IFU, IOU, '{C1, C0, C0, C0, ea, BX, db}, CTL }; end // LDD Z+q
  16'b10?0_??1?_????_0???: begin dec = '{ '{C0, CX, WX    , RX, db, RX}, '{ADW, ez, q , C0}, MUL, SRG, IFU, IOU, '{C1, C1, C0, C0, ea, Rd, RX}, CTL }; end // STD Z+q
  16'b1001_000?_????_0001: begin dec = '{ '{C1, C1, alu_rw, DZ, DZ, RX}, '{ADW, ez, E0, C1}, MUL, SRG, IFU, IOU, '{C1, C0, C0, C0, ez, BX, db}, CTL }; end // LD Z+
  16'b1001_001?_????_0001: begin dec = '{ '{C1, C1, alu_rw, DZ, db, RX}, '{ADW, ez, E0, C1}, MUL, SRG, IFU, IOU, '{C1, C1, C0, C0, ez, Rd, RX}, CTL }; end // ST Z+
  16'b1001_000?_????_0010: begin dec = '{ '{C1, C1, alu_rw, DZ, DZ, RX}, '{SBW, ez, E0, C1}, MUL, SRG, IFU, IOU, '{C1, C0, C0, C0, ea, BX, db}, CTL }; end // LD -Z
  16'b1001_001?_????_0010: begin dec = '{ '{C1, C1, alu_rw, DZ, db, RX}, '{SBW, ez, E0, C1}, MUL, SRG, IFU, IOU, '{C1, C1, C0, C0, ea, Rd, RX}, CTL }; end // ST -Z
  // I/O instructions
  //                                    {  gpr                                            iou                                }
  //                                    {  {we, ww, wd, wa, rw, rb}                       {we, re, ad, wd, ms    }           }
  16'b1011_0???_????_????: begin dec = '{ '{C1, C0, id, db, RX, RX}, ALU, MUL, SRG, IFU, '{C0, C1, a , BX, BX    }, LSU, CTL }; end // IN
  16'b1011_1???_????_????: begin dec = '{ '{C0, C0, WX, RX, db, RX}, ALU, MUL, SRG, IFU, '{C1, C0, a , Rd, BF    }, LSU, CTL }; end // OUT
  16'b1001_1000_????_????: begin dec = '{ GPR                      , ALU, MUL, SRG, IFU, '{C1, C0, a , B0, b2o(b)}, LSU, CTL }; end // CBI
  16'b1001_1010_????_????: begin dec = '{ GPR                      , ALU, MUL, SRG, IFU, '{C1, C0, a , BF, b2o(b)}, LSU, CTL }; end // SBI
  // skips
  //                                    {  gpr                        alu                                     ifu                                iou                            }
  //                                    {  {we, ww, wd, wa, rw, rb}   {m  , d      , r      , c }             {ii, sk        , be, ad, we, wd}   {we, re, ad, wd, ms}           }
  16'b0001_00??_????_????: begin dec = '{ '{C0, CX, WX, RX, db, rb}, '{SUB, feb(Rd), feb(Rr), C0}, MUL, SRG, '{C0,  alu_s.z  , C0, 'x, C0, WX}, IOU                  , LSU, CTL }; end // CPSE
  16'b1001_1001_????_????: begin dec = '{ GPR                      , ALU                         , MUL, SRG, '{C0, ~io_rdt[b], C0, 'x, C0, WX}, '{C0, C1, a , BX, BX}, LSU, CTL }; end // SBIC
  16'b1001_1011_????_????: begin dec = '{ GPR                      , ALU                         , MUL, SRG, '{C0,  io_rdt[b], C0, 'x, C0, WX}, '{C0, C1, a , BX, BX}, LSU, CTL }; end // SBIS
  16'b1111_110?_????_0???: begin dec = '{ '{C0, CX, WX, RX, db, RX}, ALU                         , MUL, SRG, '{C0, ~Rd_b     , C0, 'x, C0, WX}, '{C0, C1, a , BX, BX}, LSU, CTL }; end // SBRC
  16'b1111_111?_????_0???: begin dec = '{ '{C0, CX, WX, RX, db, RX}, ALU                         , MUL, SRG, '{C0,  Rd_b     , C0, 'x, C0, WX}, '{C0, C1, a , BX, BX}, LSU, CTL }; end // SBRS
  // flow control
  //                                    {  gpr                        alu                                    ifu                                             lsu                               }
  //                                    {  {we, ww, wd, wa, rw, rb}   {m  , d         , r , c }             {ii, sk, be, ad                , we, wd}        {en, we, st, sb, ad, wd, rd}      }
  16'b1100_????_????_????: begin dec = '{ GPR                      , '{ADW, {2'b00,pc}, Kl, C1}, MUL, SRG, '{C0, C0, C1, alu_t[22-1:0]     , C0, WX}, IOU, LSU                          , CTL }; end // RJMP
  16'b1101_????_????_????: begin dec = '{ GPR                      , '{ADW, {2'b00,pc}, Kl, C1}, MUL, SRG, '{C0, C0, C1, alu_t[22-1:0]     , C0, WX}, IOU, '{C1, C1, C1, C1, 'x, BX, RX}, CTL }; end // RCALL
  16'b1001_0100_0000_1001: begin dec = '{ '{C0, CX, WX, RX, DZ, RX}, ALU                       , MUL, SRG, '{C0, C0, C1, {6'h00,Rw}        , C0, WX}, IOU, LSU                          , CTL }; end // IJMP
  16'b1001_0101_0000_1001: begin dec = '{ '{C0, CX, WX, RX, DZ, RX}, ALU                       , MUL, SRG, '{C0, C0, C1, {6'h00,Rw}        , C0, WX}, IOU, '{C1, C1, C1, C1, 'x, BX, RX}, CTL }; end // ICALL
  16'b1001_0100_0001_1001: begin dec = '{ '{C0, CX, WX, RX, DZ, RX}, ALU                       , MUL, SRG, '{C0, C0, C1, ei                , C0, WX}, IOU, LSU                          , CTL }; end // EIJMP
  16'b1001_0101_0001_1001: begin dec = '{ '{C0, CX, WX, RX, DZ, RX}, ALU                       , MUL, SRG, '{C0, C0, C1, ei                , C0, WX}, IOU, '{C1, C1, C1, C1, 'x, BX, RX}, CTL }; end // EICALL
  16'b1001_010?_????_110?: begin dec = '{ GPR                      , ALU                       , MUL, SRG, '{C1, C0, C1, {pr[8:4],pr[0],pi}, C0, WX}, IOU, LSU                          , CTL }; end // JMP
  16'b1001_010?_????_111?: begin dec = '{ GPR                      , ALU                       , MUL, SRG, '{C1, C0, C1, {pr[8:4],pr[0],pi}, C0, WX}, IOU, '{C1, C1, C1, C1, 'x, BX, RX}, CTL }; end // CALL
  // TODO pw (program word) and pr (program register) are not the same
  16'b1001_0101_0000_1000: begin dec = '{ GPR                      , ALU                       , MUL, SRG, '{C0, C0, C1, pcs               , C0, WX}, IOU, '{C1, C0, C1, C1, 'x, BX, RX}, CTL }; end // RET
  16'b1001_0101_0001_1000: begin dec = '{ GPR                      , ALU                       , MUL, SRG, '{C0, C0, C1, pcs               , C0, WX}, IOU, '{C1, C0, C1, C1, 'x, BX, RX}, CTL }; end // RETI
  // TODO, add interrupt status to ifu_cfg_t
  // branches
  //                                    {       alu                                   ifu                                                }
  //                                    {       {m  , d         , r , c }             {ii. sk, be, ad           , we, wd}                }
  16'b1111_00??_????_????: begin dec = '{ GPR, '{ADW, {2'b00,pc}, Ks, C1}, MUL, SRG, '{C0, C0, C1, alu_t[22-1:0], C0, WX}, IOU, LSU, CTL }; end // BRBS
  16'b1111_01??_????_????: begin dec = '{ GPR, '{ADW, {2'b00,pc}, Ks, C1}, MUL, SRG, '{C0, C0, C1, alu_t[22-1:0], C0, WX}, IOU, LSU, CTL }; end // BRBC

//  16'b1001_0101_1100_1000: begin	/* LPM */	pmem_selz = 1'b1;	pmem_ce = 1'b1;	next_state = LPM; end
  // no operation, same as NOP
  // TODO: it might make sense to assign X
  default:                 begin dec = '{ GPR, ALU, MUL, SRG, IFU, IOU, LSU, CTL }; end
endcase

////////////////////////////////////////////////////////////////////////////////
// register file access
////////////////////////////////////////////////////////////////////////////////

// GPR write access
always_ff @ (posedge clk)
if (writeback) begin
  // TODO, add writeback option from load/store unit
end else if (~stl) begin
  if (dec.gpr.we) begin
    // TODO recode this, so it is appropriate for a register file or at least optimized
    if (dec.gpr.ww) gpr.idx [{dec.gpr.wa[5-1:1], 1'b0}+:2] <= dec.gpr.wd;
    else            gpr.idx [ dec.gpr.wa                 ] <= dec.gpr.wd[8-1:0];
  end
end

// read word access
assign Rw = gpr.idx [{dec.gpr.rw[5-1:1], 1'b0}+:2];
assign Rd = gpr.idx [ dec.gpr.rw                 ];

// read byte access
assign Rr = gpr.idx [ dec.gpr.rb];

// swap of Rd
assign Rs = {Rd[3:0], Rd[7:4]};

////////////////////////////////////////////////////////////////////////////////
// ALU (8 bit)
////////////////////////////////////////////////////////////////////////////////

// adder
// TODO optimize adder
always_comb
unique casez (dec.alu.m)
  3'b??0: alu_t = dec.alu.d + dec.alu.r + 24'(dec.alu.c);
  3'b??1: alu_t = dec.alu.d - dec.alu.r - 24'(dec.alu.c);
endcase

// ALU mode selection
// TODO optimize mode encoding to allign with opcode bits for ADD/SUB
always_comb
unique case (dec.alu.m)
  ADD: begin  alu_s = alu_sb;  alu_rb = alu_t[8-1:0];                         alu_rw = 'x;             end
  SUB: begin  alu_s = alu_sb;  alu_rb = alu_t[8-1:0];                         alu_rw = 'x;             end
  ADW: begin  alu_s = alu_sw;  alu_rb = 'x;                                   alu_rw = alu_t[16-1:0];  end
  SBW: begin  alu_s = alu_sw;  alu_rb = 'x;                                   alu_rw = alu_t[16-1:0];  end
  AND: begin  alu_s = alu_sb;  alu_rb = dec.alu.d[8-1:0] & dec.alu.r[8-1:0];  alu_rw = 'x;             end
  OR : begin  alu_s = alu_sb;  alu_rb = dec.alu.d[8-1:0] | dec.alu.r[8-1:0];  alu_rw = 'x;             end
  EOR: begin  alu_s = alu_sb;  alu_rb = dec.alu.d[8-1:0] ^ dec.alu.r[8-1:0];  alu_rw = 'x;             end
  SHR: begin  alu_s = alu_sb;  alu_rb = {dec.alu.c, dec.alu.d[7:1]};          alu_rw = 'x;             end
endcase

// status for ( 8 bit byte operations)
assign alu_sb.i = 1'bx;
assign alu_sb.t = 1'bx;
assign alu_sb.h = (dec.alu.m == SUB) ? ~dec.alu.d[3] & dec.alu.r[3] | dec.alu.r[3] &  alu_rb[3] |  alu_rb[3] & ~dec.alu.d[3]
                                     :  dec.alu.d[3] & dec.alu.r[3] | dec.alu.r[3] & ~alu_rb[3] | ~alu_rb[3] &  dec.alu.d[3];
assign alu_sb.s = alu_sb.n ^ alu_sb.v;
assign alu_sb.v = (dec.alu.m [2]) ? 1'b0 :
                  (dec.alu.m == SUB) ? dec.alu.d[7] & ~dec.alu.r[7] & ~alu_rb[7] | ~dec.alu.d[7] &  dec.alu.r[7] & alu_rb[7]
                                     : dec.alu.d[7] &  dec.alu.r[7] & ~alu_rb[7] | ~dec.alu.d[7] & ~dec.alu.r[7] & alu_rb[7];
assign alu_sb.n = alu_rb[7];
assign alu_sb.z = (dec.alu.m == SUB) ? ~|alu_rb & sreg.z
                                     : ~|alu_rb;
assign alu_sb.c = (dec.alu.m == SHR) ? dec.alu.d[0] : alu_t[8];

// status for (16 bit word operations)
assign alu_sw.i = 1'bx;
assign alu_sw.t = 1'bx;
assign alu_sw.h = 1'bx;
assign alu_sw.s = alu_sw.n ^ alu_sw.v;
assign alu_sw.v = ~dec.alu.d[15] & dec.alu.r[15];
assign alu_sw.n = alu_rw[15];
assign alu_sw.z = ~|alu_rw;
assign alu_sw.c = alu_t[16];

////////////////////////////////////////////////////////////////////////////////
// multiplier (8 bit * 8 bit)
////////////////////////////////////////////////////////////////////////////////

assign mul_t = $signed({dec.mul.m.d & dec.mul.d[7], dec.mul.d})
             * $signed({dec.mul.m.r & dec.mul.r[7], dec.mul.r});

assign mul_r = dec.mul.m.f ? {mul_t[14:0], C0} : mul_t[15:0];

assign mul_s.i = 1'bx;
assign mul_s.t = 1'bx;
assign mul_s.h = 1'bx;
assign mul_s.s = 1'bx;
assign mul_s.v = 1'bx;
assign mul_s.n = 1'bx;
assign mul_s.z = ~|mul_r;
assign mul_s.c = mul_t[15];

////////////////////////////////////////////////////////////////////////////////
// core state machine
////////////////////////////////////////////////////////////////////////////////

// instruction fetch state machine
always_ff @(posedge clk, posedge rst)
if (rst) stm_sts <= RST;
else begin
  stm_sts <= stl ? STL : NRM;
end

// program counter
// - on reset it is loaded with ones so the incremented value points to zero
// - on clock it stores the address of the current instruction, or debugger jump
always_ff @(posedge clk, posedge rst)
if (rst)         pc <= '1;
else if (bp_vld) pc <= bp_jmp ? pc_t'(bp_npc) : pc_t'(bp_adr);

// program counter increment
assign pcn = pc + 22'd1;

// stall can be caused by ALU (not in this implementation), IFU or LSU
assign stl = lsu_blk;

////////////////////////////////////////////////////////////////////////////////
// instruction fetch unit
////////////////////////////////////////////////////////////////////////////////

// program address
assign bp_adr = dec.ifu.be ? dec.ifu.ad [PAW-1:0] : pcn [PAW-1:0];

// program memory enable
assign bp_vld = (stm_sts != RST) & ~stl;

// TODO: this should be more complex
// TODO: some skip code should probably be here
assign ifu_blk = 1'b0;
assign ifu_con = 1'b1;

// program memory write enable
always_ff @(posedge clk, posedge rst)
if (rst) bp_wen <= 1'b0;
else     bp_wen <= dec.ifu.we;

// program memory write data
always_ff @(posedge clk)
bp_wdt <= dec.ifu.wd;

logic ifu_vld;

// program word (just a short variable name)
assign pi = bp_rdt;
assign pw = (stm_sts == P32) | ~ifu_vld ? pr : pi;

always_ff @(posedge clk, posedge rst)
if (rst)  ifu_vld <= 1'b0;
else      ifu_vld <= bp_vld;

always_ff @(posedge clk, posedge rst)
if (rst)                   pr <= 16'h0000;
else if (dec.ifu.ii | stl) pr <= pi;

// TODO exception code should be here too

////////////////////////////////////////////////////////////////////////////////
// special purpose register access
////////////////////////////////////////////////////////////////////////////////

// SPR write access
always_ff @ (posedge clk, posedge rst)
if (rst) begin
  rampd <= 8'h00;
  rampx <= 8'h00;
  rampy <= 8'h00;
  rampz <= 8'h00;
  eind  <= 6'h00;
  sp.l  <= SPR[8*0+:8];
  sp.h  <= SPR[8*1+:8];
  sreg  <= 8'h00;
  sreg  <= 8'h00;
end else begin
  if (~stl & dec.iou.we) begin
    case (dec.iou.ad)
      IOA_RAMPD: rampd <= dec.iou.wd;
      IOA_RAMPX: rampx <= dec.iou.wd;
      IOA_RAMPY: rampy <= dec.iou.wd;
      IOA_RAMPZ: rampz <= dec.iou.wd;
      IOA_EIND : eind  <= dec.iou.wd[6-1:0];
      IOA_SPL  : sp.l  <= dec.iou.wd;
      IOA_SPH  : sp.h  <= dec.iou.wd;
      IOA_SREG : sreg  <= dec.iou.wd;
    endcase
  end else begin
    // TODO access to extended registers and SP
    // SP incrementing/decrementing
    if (lsu_req) begin
      if (dec.lsu.we)  sp <= spd;
      else             sp <= spi;
    end
    // SREG is updated by aritmetic/logic and dedicated instructions
    sreg <= (dec.srg.s & dec.srg.m) | (sreg & ~dec.srg.m);
  end
end

// stack pointer increment/decrement
assign spi = sp + 16'd1;
assign spd = sp - 16'd1;

// I/O write access (the top 8 loactions are reserved for internal SPR)
assign io_wen = dec.iou.we & dec.iou.ad !=? 6'b111???;
assign io_ren = dec.iou.re & dec.iou.ad !=? 6'b111???;
assign io_adr = dec.iou.ad;
assign io_wdt = dec.iou.wd;
assign io_msk = dec.iou.ms;

// I/O read access
always_comb
if (dec.iou.we) begin
  unique case (dec.iou.ad)
    IOA_RAMPD: id = rampd;
    IOA_RAMPX: id = rampx;
    IOA_RAMPY: id = rampy;
    IOA_RAMPZ: id = rampz;
    IOA_EIND : id = 8'(eind);
    IOA_SPL  : id = sp.l;
    IOA_SPH  : id = sp.h;
    IOA_SREG : id = sreg;
    default  : id = io_rdt;
  endcase
end

// extended data memory addresses
assign ed = {rampd, pw}; // extended direct address
assign ex = {rampx, Rw}; // extended indirect address using X pointer
assign ey = {rampy, Rw}; // extended indirect address using Y pointer
assign ez = {rampz, Rw}; // extended indirect address using Z pointer
assign ea = alu_t;       // extended indirect address comming from ALU

// extended instruction memory address
assign ei = {eind , Rw}; // extended indirect address using Z pointer

////////////////////////////////////////////////////////////////////////////////
// load/store unit
////////////////////////////////////////////////////////////////////////////////

// LSU request
always_comb
if (dec.lsu.sb) begin
  lsu_req = dec.lsu.en & (dec.lsu.we ? 1'b1 : ~(bd_req & (bd_wid ==? 6'b1?_??00)) & ~(bd_ren & (bd_rid ==? 6'b1?_??00)));
end else begin
  lsu_req = dec.lsu.en & (dec.lsu.we ? 1'b1 :  ~bd_req                                                                 );
end

always_ff @(posedge clk, posedge rst)
if (rst)           lsu_cnt <= 2'd0;
else if (lsu_req)  lsu_cnt <= lsu_end ? 2'd0 : lsu_cnt + 2'd1;

// end of load store sequence, only soubroutine calls returns are longer then one byte transfer
assign lsu_end = lsu_cnt == PCN - 2'd1;

always_ff @(posedge clk, posedge rst)
if (rst)  bd_req <= 1'b0;
else      bd_req <= lsu_req;

always_ff @(posedge clk)
if (lsu_req) begin
  bd_wen <= dec.lsu.we;
//bd_adr <= dec.lsu.st ? (dec.lsu.we ? DAW'(sp) : DAW'(spi)) : DAW'(dec.lsu.ad);
// Verilator: Unsupported: Size-changing cast on non-basic data type
  bd_adr <= dec.lsu.st ? (dec.lsu.we ? sp : spi) : DAW'(dec.lsu.ad);
  if (dec.lsu.we)
  bd_wdt <= dec.lsu.sb ? pcn [lsu_cnt*8+:8] : dec.lsu.wd;
  // write identification
  if (dec.lsu.sb) begin
    bd_wid <= dec.lsu.we ? {1'b1, 3'b000,              lsu_cnt}
                         : {1'b1, 3'b000, PCN - 2'd1 - lsu_cnt};
  end else begin
    bd_wid <= {1'b0, dec.lsu.dr};
  end
end

// PC after a return
// TODO, this should be extended to support 2 and 3 byte PC
assign pcs = {lsu_buf, bd_rdt} & 24'h00ffff;

always_ff @(posedge clk)
if (bd_ren) begin
  if (bd_rid ==? 6'b1?_??01)  lsu_buf[15: 8] <= bd_rdt;
  if (bd_rid ==? 6'b1?_??10)  lsu_buf[23:16] <= bd_rdt;
end

// stall on instructions reading from memory load/pull/ret
// TODO: reading should stall only if a dirty register is accessed or if there is a writeback conflict

// LSU block
always_comb
if (dec.lsu.sb) begin
  lsu_blk = dec.lsu.en & (dec.lsu.we ? ~lsu_end : ~(bd_ren & (bd_rid ==? 6'b1?_??00)));
end else begin
  lsu_blk = dec.lsu.en & (dec.lsu.we ? 1'b0     :  ~bd_ren                           );
end

////////////////////////////////////////////////////////////////////////////////
// control outputs
////////////////////////////////////////////////////////////////////////////////

assign ctl_slp = dec.ctl.slp; // sleep
assign ctl_brk = dec.ctl.brk; // break
assign ctl_wdr = dec.ctl.wdr; // watch dog reset

////////////////////////////////////////////////////////////////////////////////
// exceptions
////////////////////////////////////////////////////////////////////////////////

logic [IRW-1:0] next_irq_ack;

always_comb
casez (irq_req)
  8'b????_???1: next_irq_ack = 8'b0000_0001;
  8'b????_??10: next_irq_ack = 8'b0000_0010;
  8'b????_?100: next_irq_ack = 8'b0000_0100;
  8'b????_1000: next_irq_ack = 8'b0000_1000;
  8'b???1_0000: next_irq_ack = 8'b0001_0000;
  8'b??10_0000: next_irq_ack = 8'b0010_0000;
  8'b?100_0000: next_irq_ack = 8'b0100_0000;
  8'b1000_0000: next_irq_ack = 8'b1000_0000;
  default:      next_irq_ack = 8'b0000_0000;
endcase

logic irq_ack_en;

always_ff @(posedge clk, posedge rst)
if (rst) irq_ack <= '0;
else     irq_ack <= irq_ack_en ? next_irq_ack : '0;

/* Priority encoder */

logic [$clog2(IRW)-1:0] PC_ex;

always_comb
casez (irq_req)
  8'b????_???1: PC_ex = 3'h0;
  8'b????_??10: PC_ex = 3'h1;
  8'b????_?100: PC_ex = 3'h2;
  8'b????_1000: PC_ex = 3'h3;
  8'b???1_0000: PC_ex = 3'h4;
  8'b??10_0000: PC_ex = 3'h5;
  8'b?100_0000: PC_ex = 3'h6;
  8'b1000_0000: PC_ex = 3'h7;
  default:      PC_ex = 3'h0;
endcase

/* AVR cores always execute at least one instruction after an IRET.
 * Therefore, the I bit is only valid one clock after it has been set. */

logic I_r;

always_ff @(posedge clk, posedge rst)
if (rst) I_r <= 1'b0;
else     I_r <= sreg.i;

wire irq_asserted = |irq_req;
wire irq_request = sreg.i & I_r & irq_asserted;

////////////////////////////////////////////////////////////////////////////////
// __verilator__ specific bench code
////////////////////////////////////////////////////////////////////////////////

`ifdef verilator

function void dump_state_core (
  output bit [32-1:0] [8-1:0] dump_gpr ,
  output int                  dump_pc  ,
  output int                  dump_sp  ,
  output bit          [8-1:0] dump_sreg
);
/*verilator public*/
  dump_gpr  = gpr.idx;
  dump_pc   = {10'd0, pc};
  dump_sp   = {16'd0, sp};
  dump_sreg = sreg;
endfunction: dump_state_core

`endif

////////////////////////////////////////////////////////////////////////////////
// VCD specific bench code
////////////////////////////////////////////////////////////////////////////////

/* verilator lint_off UNUSED */

// general purpose registers decode structure
logic          dec_gpr_we; // write enable
logic          dec_gpr_ww; // write word (0 - 8 bit mode, 1 - 16 bit mode)
logic [16-1:0] dec_gpr_wd; // write data 16 bit
gpr_adr_t      dec_gpr_wa; // write address 
gpr_adr_t      dec_gpr_rw; // read address for word (16 bit)
gpr_adr_t      dec_gpr_rb; // read address for byte (8 bit)
// arithmetic logic unit decode structure
logic  [3-1:0] dec_alu_m; // alu modes
logic [24-1:0] dec_alu_d; // destination operand value
logic [24-1:0] dec_alu_r; // source      operand value
logic          dec_alu_c; // carry input
// multiplier decode structure
logic          dec_mul_m_f; // fractional
logic          dec_mul_m_d; // destination (0 - unsigned, 1 - signed)
logic          dec_mul_m_r; // source      (0 - unsigned, 1 - signed)
logic  [8-1:0] dec_mul_d; // destination operand value
logic  [8-1:0] dec_mul_r; // source      operand value
// status register decode structure
sreg_t         dec_srg_s; // status
sreg_t         dec_srg_m; // mask
// instruction fetch unit decode structure
logic          dec_ifu_ii; // instruction immediate
logic          dec_ifu_sk; // skip
logic          dec_ifu_be; // branch enable
pc_t           dec_ifu_ad; // address
logic          dec_ifu_we; // write enable (for SPM instruction)
logic [16-1:0] dec_ifu_wd; // write data   (for SPM instruction)
// input/output unit decode structure
logic          dec_iou_we; // write enable
logic          dec_iou_re; // read  enable
iou_adr_t      dec_iou_ad; // address
logic  [8-1:0] dec_iou_wd; // write data
logic  [8-1:0] dec_iou_ms; // write mask
// load/store unit decode structure
logic          dec_lsu_en; // enable
logic          dec_lsu_we; // write enable
logic          dec_lsu_st; // stack push/pop
logic          dec_lsu_sb; // subroutine/interrupt call/return
ls_t           dec_lsu_ad; // address
logic  [8-1:0] dec_lsu_wd; // write data
gpr_adr_t      dec_lsu_dr; // destination register (if not PC)
// control decode structure
logic          dec_ctl_slp; // sleep
logic          dec_ctl_brk; // break
logic          dec_ctl_wdr; // watchdog reset

assign {dec_gpr_we,
        dec_gpr_ww,
        dec_gpr_wd,
        dec_gpr_wa,
        dec_gpr_rw,
        dec_gpr_rb,
        dec_alu_m,
        dec_alu_d,
        dec_alu_r,
        dec_alu_c,
        dec_mul_m_f,
        dec_mul_m_d,
        dec_mul_m_r,
        dec_mul_d,
        dec_mul_r,
        dec_srg_s,
        dec_srg_m,
        dec_ifu_ii,
        dec_ifu_sk,
        dec_ifu_be,
        dec_ifu_ad,
        dec_ifu_we,
        dec_ifu_wd,
        dec_iou_we,
        dec_iou_re,
        dec_iou_ad,
        dec_iou_wd,
        dec_iou_ms,
        dec_lsu_en,
        dec_lsu_we,
        dec_lsu_st,
        dec_lsu_sb,
        dec_lsu_ad,
        dec_lsu_wd,
        dec_lsu_dr,
        dec_ctl_slp,
        dec_ctl_brk,
        dec_ctl_wdr} = dec;

logic [8-1:0] GPR_R00;
logic [8-1:0] GPR_R01;
logic [8-1:0] GPR_R02;
logic [8-1:0] GPR_R03;
logic [8-1:0] GPR_R04;
logic [8-1:0] GPR_R05;
logic [8-1:0] GPR_R06;
logic [8-1:0] GPR_R07;
logic [8-1:0] GPR_R08;
logic [8-1:0] GPR_R09;
logic [8-1:0] GPR_R10;
logic [8-1:0] GPR_R11;
logic [8-1:0] GPR_R12;
logic [8-1:0] GPR_R13;
logic [8-1:0] GPR_R14;
logic [8-1:0] GPR_R15;
logic [8-1:0] GPR_R16;
logic [8-1:0] GPR_R17;
logic [8-1:0] GPR_R18;
logic [8-1:0] GPR_R19;
logic [8-1:0] GPR_R20;
logic [8-1:0] GPR_R21;
logic [8-1:0] GPR_R22;
logic [8-1:0] GPR_R23;
logic [8-1:0] GPR_R24;
logic [8-1:0] GPR_R25;
logic [8-1:0] GPR_R26;
logic [8-1:0] GPR_R27;
logic [8-1:0] GPR_R28;
logic [8-1:0] GPR_R29;
logic [8-1:0] GPR_R30;
logic [8-1:0] GPR_R31;

assign GPR_R00 = gpr.idx[00];
assign GPR_R01 = gpr.idx[01];
assign GPR_R02 = gpr.idx[02];
assign GPR_R03 = gpr.idx[03];
assign GPR_R04 = gpr.idx[04];
assign GPR_R05 = gpr.idx[05];
assign GPR_R06 = gpr.idx[06];
assign GPR_R07 = gpr.idx[07];
assign GPR_R08 = gpr.idx[08];
assign GPR_R09 = gpr.idx[09];
assign GPR_R10 = gpr.idx[10];
assign GPR_R11 = gpr.idx[11];
assign GPR_R12 = gpr.idx[12];
assign GPR_R13 = gpr.idx[13];
assign GPR_R14 = gpr.idx[14];
assign GPR_R15 = gpr.idx[15];
assign GPR_R16 = gpr.idx[16];
assign GPR_R17 = gpr.idx[17];
assign GPR_R18 = gpr.idx[18];
assign GPR_R19 = gpr.idx[19];
assign GPR_R20 = gpr.idx[20];
assign GPR_R21 = gpr.idx[21];
assign GPR_R22 = gpr.idx[22];
assign GPR_R23 = gpr.idx[23];
assign GPR_R24 = gpr.idx[24];
assign GPR_R25 = gpr.idx[25];
assign GPR_R26 = gpr.idx[26];
assign GPR_R27 = gpr.idx[27];
assign GPR_R28 = gpr.idx[28];
assign GPR_R29 = gpr.idx[29];
assign GPR_R30 = gpr.idx[30];
assign GPR_R31 = gpr.idx[31];


/* verilator lint_on UNUSED */

endmodule: rp_8bit
