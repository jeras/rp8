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
  parameter int unsigned IRW =  8, // interrupt request width
  parameter int unsigned PAW = 11, // 16 bit words
  parameter int unsigned DAW = 13  //  8 bit bytes
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
  output logic           bd_wen, // write enable
  output logic [DAW-1:0] bd_adr, // address
  output logic   [8-1:0] bd_wdt, // write data
  input  logic   [8-1:0] bd_rdt, // read  data
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
// AVR architecture constants
////////////////////////////////////////////////////////////////////////////////

// I/O register addresses
localparam bit [6-1:0] IOA_RAMPD = 6'h30 + 6'h08;
localparam bit [6-1:0] IOA_RAMPX = 6'h30 + 6'h09;
localparam bit [6-1:0] IOA_RAMPY = 6'h30 + 6'h0a;
localparam bit [6-1:0] IOA_RAMPZ = 6'h30 + 6'h0b;
localparam bit [6-1:0] IOA_EIND  = 6'h30 + 6'h0c;
localparam bit [6-1:0] IOA_SPL   = 6'h30 + 6'h0d;
localparam bit [6-1:0] IOA_SPH   = 6'h30 + 6'h0e;
localparam bit [6-1:0] IOA_SREG  = 6'h30 + 6'h0f;

////////////////////////////////////////////////////////////////////////////////
// calculated parameters
////////////////////////////////////////////////////////////////////////////////

// maximum address width
localparam int unsigned MAW = PAW > DAW ? PAW : DAW;

// adder destination width
localparam int unsigned AW = MAW > 16 ? MAW : 16;

////////////////////////////////////////////////////////////////////////////////
// helper functions
////////////////////////////////////////////////////////////////////////////////

// binary to one hot
function logic [8-1:0] b2o (input logic [3-1:0] b);
  b2o = 8'h01 << b;
endfunction: b2o

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

// program memory pointer
typedef struct packed {
  logic [8-1:0] e;
  logic [8-1:0] h;
  logic [8-1:0] l;
} ifu_ptr_t;

// data memory pointer
typedef struct packed {
  logic [8-1:0] e;
  logic [8-1:0] h;
  logic [8-1:0] l;
} lsu_ptr_t;

// status register
typedef struct packed {logic i, t, h, s, v, n, z, c;} sreg_t;

// general purpose registers decode structure
typedef struct packed {
  // write access
  logic           we; // write enable
  logic           ww; // write word (0 - 8 bit mode, 1 - 16 bit mode)
  logic  [16-1:0] wd; // write data 16 bit
  logic   [5-1:0] wa; // write address 
  logic   [5-1:0] rw; // read word (16 bit) address
  logic   [5-1:0] rb; // read byte (8 bit) address
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
  logic  [AW-1:0] d; // destination operand value
  logic  [AW-1:0] r; // source      operand value
  logic           c; // carry input
} alu_dec_t;

// multiplier decode structure
typedef struct packed {
  struct packed {
    logic f; // fractional
    logic d; // destination (0 - unsigned, 1 - signed)
    logic r; // source      (0 - unsigned, 1 - signed)
  } m;               // adder modes
  logic   [8-1:0] d; // destination operand value
  logic   [8-1:0] r; // source      operand value
} mul_dec_t;

// status register decode structure
typedef struct packed {
  sreg_t          s; // status
  sreg_t          m; // mask
} srg_dec_t;

// instruction fetch unit decode structure
typedef struct packed {
  logic           ii; // instruction immediate
  logic           sk; // skip
  logic           be; // branch enable
  logic [PAW-1:0] ad; // address
  logic           we; // write enable (for SPM instruction)
  logic  [16-1:0] wd; // write data   (for SPM instruction)
} ifu_dec_t;

// input/output unit decode structure
typedef struct packed {
  logic           we; // write enable
  logic           re; // read  enable
  logic   [6-1:0] ad; // address
  logic   [8-1:0] wd; // write data
  logic   [8-1:0] ms; // write mask
} iou_dec_t;

// load/store unit decode structure
typedef struct packed {
  logic           en; // enable
  logic           we; // write enable
  logic           st; // stack push/pop
  logic           sb; // subroutine/interrupt call/return
  logic [DAW-1:0] ad; // address
  logic   [8-1:0] wd; // write data
  logic   [5-1:0] dr; // destination register (if not PC)
} lsu_dec_t;

// control decode structure
typedef struct packed {
  logic           slp; // sleep
  logic           brk; // break
  logic           wdr; // watchdog reset
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

dec_t dec;

////////////////////////////////////////////////////////////////////////////////
// local variables
////////////////////////////////////////////////////////////////////////////////

// program word
logic [16-1:0] pw; // multiplexed
logic [16-1:0] pi; // input
logic [16-1:0] pr; // registered

// destination/source register address for:
logic [5-1:0] db, rb; // full space bytes (used by MOV and arithmetic)
logic [5-1:0] dw, rw; // full space words (used by MOVW)
logic [5-1:0] dh, rh; // high half space (used by MULS, arithmetic immediate, load store direct)
logic [5-1:0] dm, rm; // third quarter space (used by *MUL*)
logic [5-1:0] di;     // index registers (used by ADIW/SBIW)

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

// core state registers
ifu_ptr_t      pc;    // program counter
ifu_ptr_t      pcn;   // program counter next (PC+1)
logic [16-1:0] sp;    // stack pointer
sreg_t         sreg;  // status register
gpr_t          gpr;   // register file

// various sources of core stall
logic          stall;
logic          stall_ifu;
logic          stall_lsu;
logic          stall_lsu_cmb;
logic          stall_lsu_reg;

// core state machine
logic          writeback;

// ALU results
logic [AW-0:0] alu_t ; // result (full width plus carry)
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
logic  [8-1:0] eind ;

// extended addressing registers
logic [24-1:0] ed, ex, ey, ez, ea;

// load store unit
ifu_ptr_t      lsu_pc;    // program counter

////////////////////////////////////////////////////////////////////////////////
// register addresses and immediates
////////////////////////////////////////////////////////////////////////////////

// bit constants
localparam logic          CX = 1'bx;
localparam logic          C0 = 1'b0;
localparam logic          C1 = 1'b1;

// byte (8 bit) constants
localparam logic  [8-1:0] KX = 8'hxx;
localparam logic  [8-1:0] K0 = 8'h00;
localparam logic  [8-1:0] KF = 8'hff;

// word (16 bit) constants
localparam logic [16-1:0] WX = 16'hxxxx;
localparam logic [16-1:0] W0 = 16'h0000;
localparam logic [16-1:0] WF = 16'hffff;

// register address constant
localparam logic [5-1:0] RX = 5'hxx;
localparam logic [5-1:0] R0 = 5'h00; // R1:R0 used for multiplication destination address
localparam logic [5-1:0] DX = 5'h1a; // index register X
localparam logic [5-1:0] DY = 5'h1c; // index register Y
localparam logic [5-1:0] DZ = 5'h1e; // index register Z

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
localparam gpr_dec_t GPR = '{we: C0, ww: C0, wd: WX, wa: 5'hxx, rb: RX, rw: RX};
localparam alu_dec_t ALU = '{m: 3'bxxx, d: KX, r: KX, c: CX};
localparam mul_dec_t MUL = '{m: 3'bxxx, d: KX, r: KX};
localparam srg_dec_t SRG = '{s: KX, m: K0};
localparam ifu_dec_t IFU = '{ii: C0, sk: C0, be: C0, ad: {PAW{CX}}, we: C0, wd: WX};
localparam iou_dec_t IOU = '{we: C0, re: C0, ad: 6'hxx, wd: KX, ms: KX};
localparam lsu_dec_t LSU = '{en: C0, we: CX, st: CX, sb: CX, ad: {DAW{CX}}, wd: KX, dr: RX};
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
  //                                    {  gpr                                 alu                          srg                                }
  //                                    {  {we, ww, wd         , wa, rw, rb}   {m  , d , r , c     }        {s    , m    }                     }
  16'b0000_01??_????_????: begin dec = '{ '{C0, CX, WX         , RX, db, rb}, '{SUB, Rd, Rr, sreg.c}, MUL, '{alu_s, 8'h3f}, IFU, IOU, LSU, CTL }; end // CPC
  16'b0000_10??_????_????: begin dec = '{ '{C1, C0, {2{alu_rb}}, db, db, rb}, '{SUB, Rd, Rr, sreg.c}, MUL, '{alu_s, 8'h3f}, IFU, IOU, LSU, CTL }; end // SBC
  16'b0000_11??_????_????: begin dec = '{ '{C1, C0, {2{alu_rb}}, db, db, rb}, '{ADD, Rd, Rr, C0    }, MUL, '{alu_s, 8'h3f}, IFU, IOU, LSU, CTL }; end // ADD
  16'b0001_01??_????_????: begin dec = '{ '{C0, C0, WX         , RX, db, rb}, '{SUB, Rd, Rr, C0    }, MUL, '{alu_s, 8'h3f}, IFU, IOU, LSU, CTL }; end // CP
  16'b0001_10??_????_????: begin dec = '{ '{C1, C0, {2{alu_rb}}, db, db, rb}, '{SUB, Rd, Rr, C0    }, MUL, '{alu_s, 8'h3f}, IFU, IOU, LSU, CTL }; end // SUB
  16'b0001_11??_????_????: begin dec = '{ '{C1, C0, {2{alu_rb}}, db, db, rb}, '{ADD, Rd, Rr, sreg.c}, MUL, '{alu_s, 8'h3f}, IFU, IOU, LSU, CTL }; end // ADC
  16'b0011_????_????_????: begin dec = '{ '{C0, CX, WX         , RX, dw, RX}, '{SUB, Rd, kb, C0    }, MUL, '{alu_s, 8'h3f}, IFU, IOU, LSU, CTL }; end // CPI
  16'b0100_????_????_????: begin dec = '{ '{C1, C0, {2{alu_rb}}, dw, dw, RX}, '{SUB, Rd, kb, sreg.c}, MUL, '{alu_s, 8'h3f}, IFU, IOU, LSU, CTL }; end // SBCI
  16'b0101_????_????_????: begin dec = '{ '{C1, C0, {2{alu_rb}}, dw, dw, RX}, '{SUB, Rd, kb, C0    }, MUL, '{alu_s, 8'h3f}, IFU, IOU, LSU, CTL }; end // SUBI
  16'b1001_010?_????_0000: begin dec = '{ '{C1, C0, {2{alu_rb}}, db, db, RX}, '{SUB, KF, Rd, C0    }, MUL, '{alu_s, 8'h1f}, IFU, IOU, LSU, CTL }; end // COM
  // TODO check the value of carry and overflow
  16'b1001_010?_????_0001: begin dec = '{ '{C1, C0, {2{alu_rb}}, db, db, RX}, '{SUB, K0, Rd, C0    }, MUL, '{alu_s, 8'h3f}, IFU, IOU, LSU, CTL }; end // NEG
  16'b1001_010?_????_0011: begin dec = '{ '{C1, C0, {2{alu_rb}}, db, db, RX}, '{ADD, Rd, K0, C1    }, MUL, '{alu_s, 8'h3e}, IFU, IOU, LSU, CTL }; end // INC
  16'b1001_010?_????_1010: begin dec = '{ '{C1, C0, {2{alu_rb}}, db, db, RX}, '{SUB, Rd, K0, C1    }, MUL, '{alu_s, 8'h3e}, IFU, IOU, LSU, CTL }; end // DEC
  // logic // TODO check flags
  16'b0010_00??_????_????: begin dec = '{ '{C1, C0, {2{alu_rb}}, db, db, rb}, '{AND, Rd, Rr, C0    }, MUL, '{alu_s, 8'h1e}, IFU, IOU, LSU, CTL }; end // AND
  16'b0111_????_????_????: begin dec = '{ '{C1, C0, {2{alu_rb}}, dw, dw, RX}, '{AND, Rd, kb, C0    }, MUL, '{alu_s, 8'h1e}, IFU, IOU, LSU, CTL }; end // ANDI
  16'b0010_10??_????_????: begin dec = '{ '{C1, C0, {2{alu_rb}}, db, db, rb}, '{OR , Rd, Rr, C0    }, MUL, '{alu_s, 8'h1e}, IFU, IOU, LSU, CTL }; end // OR
  16'b0110_????_????_????: begin dec = '{ '{C1, C0, {2{alu_rb}}, dw, dw, RX}, '{OR , Rd, kb, C0    }, MUL, '{alu_s, 8'h1e}, IFU, IOU, LSU, CTL }; end // ORI
  16'b0010_01??_????_????: begin dec = '{ '{C1, C0, {2{alu_rb}}, db, db, rb}, '{EOR, Rd, Rr, C0    }, MUL, '{alu_s, 8'h1e}, IFU, IOU, LSU, CTL }; end // EOR
  // shift right // TODO check flags
  16'b1001_010?_????_0110: begin dec = '{ '{C1, C0, {2{alu_rb}}, db, db, RX}, '{SHR, Rd, K0, C0    }, MUL, '{alu_s, 8'h1f}, IFU, IOU, LSU, CTL }; end // LSR
  16'b1001_010?_????_0111: begin dec = '{ '{C1, C0, {2{alu_rb}}, db, db, RX}, '{SHR, Rd, K0, sreg.c}, MUL, '{alu_s, 8'h1f}, IFU, IOU, LSU, CTL }; end // ROR
  16'b1001_010?_????_0101: begin dec = '{ '{C1, C0, {2{alu_rb}}, db, db, RX}, '{SHR, Rd, K0, Rd[7] }, MUL, '{alu_s, 8'h1f}, IFU, IOU, LSU, CTL }; end // ASR
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
  16'b0010_11??_????_????: begin dec = '{ '{C1, C0, {2{Rr}}, db, db, rb}, ALU, MUL, SRG, IFU, IOU, LSU, CTL }; end // MOV
  16'b1110_????_????_????: begin dec = '{ '{C1, C0, {2{kb}}, dh, RX, RX}, ALU, MUL, SRG, IFU, IOU, LSU, CTL }; end // LDI
  16'b1001_010?_????_0010: begin dec = '{ '{C1, C0, {2{Rs}}, db, db, RX}, ALU, MUL, SRG, IFU, IOU, LSU, CTL }; end // SWAP
  // bit manipulation
  //                                    {                 srg                                    }
  //                                    {                 {s , m           }                     }
  16'b1001_010?_0???_1000: begin dec = '{ GPR, ALU, MUL, '{KF, b2o(pw[6:4])}, IFU, IOU, LSU, CTL }; end // BSET // create a common source instead of two b2o functions
  16'b1001_010?_1???_1000: begin dec = '{ GPR, ALU, MUL, '{K0, b2o(pw[6:4])}, IFU, IOU, LSU, CTL }; end // BCLR
  // 16-24 bit adder
  16'b1001_0110_????_????: begin dec = '{ '{C1, C1, alu_rw, di, di, RX}, '{ADW, Rw, kw, C0}, MUL, '{alu_s, 8'h1f}, IFU, IOU, LSU, CTL }; end // ADIW
  16'b1001_0111_????_????: begin dec = '{ '{C1, C1, alu_rw, di, di, RX}, '{SBW, Rw, kw, C0}, MUL, '{alu_s, 8'h1f}, IFU, IOU, LSU, CTL }; end // SBIW
  // bit manipulation
  16'b1111_101?_????_0???: begin dec = '{ '{C0, CX, WX                                      , RX, db, RX}, ALU, MUL, '{{CX,Rd_b,6'hxx}, 8'h40}, IFU, IOU, LSU, CTL }; end // SBT
  16'b1111_100?_????_0???: begin dec = '{ '{C1, C0, {2{Rd & ~b2o(b) | {8{sreg.t}} & b2o(b)}}, db, db, RX}, ALU, MUL, SRG                      , IFU, IOU, LSU, CTL }; end // BLD  // TODO: ALU could be used
  // stack access
  //                                    {  gpr                                                 lsu                               }
  //                                    {  {we, ww, wd, wa, rw, rb}                            {en, we, st, sb, ad, wd, rd}      }
  16'b1001_000?_????_1111: begin dec = '{ GPR                      , ALU, MUL, SRG, IFU, IOU, '{C1, C0, C1, C0, 'x, KX, db}, CTL }; end // POP
  16'b1001_001?_????_1111: begin dec = '{ '{C0, CX, WX, RX, db, RX}, ALU, MUL, SRG, IFU, IOU, '{C1, C1, C1, C0, 'x, Rd, RX}, CTL }; end // PUSH
  // load store direct (32 bit instructions)
  //                                    {  gpr                                       ifu                             lsu                               }
  //                                    {  {we, ww, wd, wa, rw, rb}                  {ii, sk, be, ad, we, wd}        {en, we, st, sb, ad, wd, rd}      }
  16'b1001_000?_????_0000: begin dec = '{ GPR                      , ALU, MUL, SRG, '{C1, C0, C0, 'x, C0, WX}, IOU, '{C1, C0, C1, C0, ed, KX, db}, CTL }; end // LDS
  16'b1001_001?_????_0000: begin dec = '{ '{C0, CX, WX, RX, db, RX}, ALU, MUL, SRG, '{C1, C0, C0, 'x, C0, WX}, IOU, '{C1, C1, C1, C0, ed, Rd, RX}, CTL }; end // STS
  // load store indirect
  //                                    {  gpr                            alu                                     lsu                               }
  //                                    {  {we, ww, wd    , wa, rw, rb}   {m  , d , r , c }                       {en, we, st, sb, ad, wd, rd}      }
  16'b1001_000?_????_1100: begin dec = '{ '{C0, CX, WX    , RX, DX, RX}, ALU               , MUL, SRG, IFU, IOU, '{C1, C0, C0, C0, ex, KX, db}, CTL }; end // LD X
  16'b1001_001?_????_1100: begin dec = '{ '{C0, CX, WX    , RX, db, RX}, ALU               , MUL, SRG, IFU, IOU, '{C1, C1, C0, C0, ex, Rd, RX}, CTL }; end // ST X
  16'b1001_000?_????_1101: begin dec = '{ '{C1, C1, alu_rw, DX, DX, RX}, '{ADW, Rd, K0, C1}, MUL, SRG, IFU, IOU, '{C1, C0, C0, C0, ex, KX, db}, CTL }; end // LD X+
  16'b1001_001?_????_1101: begin dec = '{ '{C1, C1, alu_rw, DX, db, RX}, '{ADW, Rd, K0, C1}, MUL, SRG, IFU, IOU, '{C1, C1, C0, C0, ex, Rd, RX}, CTL }; end // ST X+
  16'b1001_000?_????_1110: begin dec = '{ '{C1, C1, alu_rw, DX, DX, RX}, '{SBW, Rd, K0, C1}, MUL, SRG, IFU, IOU, '{C1, C0, C0, C0, ea, KX, db}, CTL }; end // LD -X
  16'b1001_001?_????_1110: begin dec = '{ '{C1, C1, alu_rw, DX, db, RX}, '{SBW, Rd, K0, C1}, MUL, SRG, IFU, IOU, '{C1, C1, C0, C0, ea, Rd, RX}, CTL }; end // ST -X
  16'b10?0_??0?_????_1???: begin dec = '{ '{C0, CX, WX    , RX, DY, RX}, '{ADW, Rd, q , C0}, MUL, SRG, IFU, IOU, '{C1, C0, C0, C0, ea, KX, db}, CTL }; end // LDD Y+q
  16'b10?0_??1?_????_1???: begin dec = '{ '{C0, CX, WX    , RX, db, RX}, '{ADW, Rd, q , C0}, MUL, SRG, IFU, IOU, '{C1, C1, C0, C0, ea, Rd, RX}, CTL }; end // STD Y+q
  16'b1001_000?_????_1001: begin dec = '{ '{C1, C1, alu_rw, DY, DY, RX}, '{ADW, Rd, K0, C1}, MUL, SRG, IFU, IOU, '{C1, C0, C0, C0, ey, KX, db}, CTL }; end // LD Y+
  16'b1001_001?_????_1001: begin dec = '{ '{C1, C1, alu_rw, DY, db, RX}, '{ADW, Rd, K0, C1}, MUL, SRG, IFU, IOU, '{C1, C1, C0, C0, ey, Rd, RX}, CTL }; end // ST Y+
  16'b1001_000?_????_1010: begin dec = '{ '{C1, C1, alu_rw, DY, DY, RX}, '{SBW, Rd, K0, C1}, MUL, SRG, IFU, IOU, '{C1, C0, C0, C0, ea, KX, db}, CTL }; end // LD -Y
  16'b1001_001?_????_1010: begin dec = '{ '{C1, C1, alu_rw, DY, db, RX}, '{SBW, Rd, K0, C1}, MUL, SRG, IFU, IOU, '{C1, C1, C0, C0, ea, Rd, RX}, CTL }; end // ST -Y
  16'b10?0_??0?_????_0???: begin dec = '{ '{C0, CX, WX    , RX, DZ, RX}, '{ADW, Rd, q , C0}, MUL, SRG, IFU, IOU, '{C1, C0, C0, C0, ea, KX, db}, CTL }; end // LDD Z+q
  16'b10?0_??1?_????_0???: begin dec = '{ '{C0, CX, WX    , RX, db, RX}, '{ADW, Rd, q , C0}, MUL, SRG, IFU, IOU, '{C1, C1, C0, C0, ea, Rd, RX}, CTL }; end // STD Z+q
  16'b1001_000?_????_0001: begin dec = '{ '{C1, C1, alu_rw, DZ, DZ, RX}, '{ADW, Rd, K0, C1}, MUL, SRG, IFU, IOU, '{C1, C0, C0, C0, ez, KX, db}, CTL }; end // LD Z+
  16'b1001_001?_????_0001: begin dec = '{ '{C1, C1, alu_rw, DZ, db, RX}, '{ADW, Rd, K0, C1}, MUL, SRG, IFU, IOU, '{C1, C1, C0, C0, ez, Rd, RX}, CTL }; end // ST Z+
  16'b1001_000?_????_0010: begin dec = '{ '{C1, C1, alu_rw, DZ, DZ, RX}, '{SBW, Rd, K0, C1}, MUL, SRG, IFU, IOU, '{C1, C0, C0, C0, ea, KX, db}, CTL }; end // LD -Z
  16'b1001_001?_????_0010: begin dec = '{ '{C1, C1, alu_rw, DZ, db, RX}, '{SBW, Rd, K0, C1}, MUL, SRG, IFU, IOU, '{C1, C1, C0, C0, ea, Rd, RX}, CTL }; end // ST -Z
  // I/O instructions
  //                                    {  gpr                                            iou                                }
  //                                    {  {we, ww, wd, wa, rw, rb}                       {we, re, ad, wd, ms    }           }
  16'b1011_0???_????_????: begin dec = '{ '{C1, C0, id, db, RX, RX}, ALU, MUL, SRG, IFU, '{C0, C1, a , KX, KX    }, LSU, CTL }; end // IN
  16'b1011_1???_????_????: begin dec = '{ '{C0, C0, WX, RX, db, RX}, ALU, MUL, SRG, IFU, '{C1, C0, a , Rd, KF    }, LSU, CTL }; end // OUT
  16'b1001_1000_????_????: begin dec = '{ GPR                      , ALU, MUL, SRG, IFU, '{C1, C0, a , K0, b2o(b)}, LSU, CTL }; end // CBI
  16'b1001_1010_????_????: begin dec = '{ GPR                      , ALU, MUL, SRG, IFU, '{C1, C0, a , KF, b2o(b)}, LSU, CTL }; end // SBI
  // skips
  //                                    {  gpr                        alu                           ifu                                iou                            }
  //                                    {  {we, ww, wd, wa, rw, rb}   {m  , d , r , c }             {ii, sk        , be, ad, we, wd}   {we, re, ad, wd, ms}           }
  16'b0001_00??_????_????: begin dec = '{ '{C0, CX, WX, RX, db, rb}, '{SUB, Rd, Rr, C0}, MUL, SRG, '{C0,  alu_s.z  , C0, 'x, C0, WX}, IOU                  , LSU, CTL }; end // CPSE
  16'b1001_1001_????_????: begin dec = '{ GPR                      , ALU               , MUL, SRG, '{C0, ~io_rdt[b], C0, 'x, C0, WX}, '{C0, C1, a , KX, KX}, LSU, CTL }; end // SBIC
  16'b1001_1011_????_????: begin dec = '{ GPR                      , ALU               , MUL, SRG, '{C0,  io_rdt[b], C0, 'x, C0, WX}, '{C0, C1, a , KX, KX}, LSU, CTL }; end // SBIS
  16'b1111_110?_????_0???: begin dec = '{ '{C0, CX, WX, RX, db, RX}, ALU               , MUL, SRG, '{C0, ~Rd_b     , C0, 'x, C0, WX}, '{C0, C1, a , KX, KX}, LSU, CTL }; end // SBRC
  16'b1111_111?_????_0???: begin dec = '{ '{C0, CX, WX, RX, db, RX}, ALU               , MUL, SRG, '{C0,  Rd_b     , C0, 'x, C0, WX}, '{C0, C1, a , KX, KX}, LSU, CTL }; end // SBRS
  // flow control
  //                                    {  gpr                        alu                            ifu                                             lsu                               }
  //                                    {  {we, ww, wd, wa, rw, rb}   {m  , d  , r , c }             {ii, sk, be, ad                , we, wd}        {en, we, st, sb, ad, wd, rd}      }
  16'b1100_????_????_????: begin dec = '{ GPR                      , '{ADW, pcn, Kl, C1}, MUL, SRG, '{C0, C0, C1, alu_t[PAW-1:0]    , C0, WX}, IOU, LSU                          , CTL }; end // RJMP
  16'b1101_????_????_????: begin dec = '{ GPR                      , '{ADW, pcn, Kl, C1}, MUL, SRG, '{C0, C0, C1, alu_t[PAW-1:0]    , C0, WX}, IOU, '{C1, C1, C1, C1, 'x, KX, RX}, CTL }; end // RCALL
  16'b1001_0100_0000_1001: begin dec = '{ '{C0, CX, WX, RX, DZ, RX}, ALU                , MUL, SRG, '{C0, C0, C1, Rw                , C0, WX}, IOU, LSU                          , CTL }; end // IJMP
  16'b1001_0101_0000_1001: begin dec = '{ '{C0, CX, WX, RX, DZ, RX}, ALU                , MUL, SRG, '{C0, C0, C1, Rw                , C0, WX}, IOU, '{C1, C1, C1, C1, 'x, KX, RX}, CTL }; end // ICALL
  16'b1001_0100_0001_1001: begin dec = '{ '{C0, CX, WX, RX, DZ, RX}, ALU                , MUL, SRG, '{C0, C0, C1, ez                , C0, WX}, IOU, LSU                          , CTL }; end // EIJMP
  16'b1001_0101_0001_1001: begin dec = '{ '{C0, CX, WX, RX, DZ, RX}, ALU                , MUL, SRG, '{C0, C0, C1, ez                , C0, WX}, IOU, '{C1, C1, C1, C1, 'x, KX, RX}, CTL }; end // EICALL
  16'b1001_010?_????_110?: begin dec = '{ GPR                      , ALU                , MUL, SRG, '{C1, C0, C1, {pr[8:4],pr[0],pi}, C0, WX}, IOU, LSU                          , CTL }; end // JMP
  16'b1001_010?_????_111?: begin dec = '{ GPR                      , ALU                , MUL, SRG, '{C1, C0, C1, {pr[8:4],pr[0],pi}, C0, WX}, IOU, '{C1, C1, C1, C1, 'x, KX, RX}, CTL }; end // CALL
  // TODO pw (program word) and pr (program register) are not the same
  16'b1001_0101_0000_1000: begin dec = '{ GPR                      , ALU                , MUL, SRG, '{C0, C0, C1, lsu_pc            , C0, WX}, IOU, '{C1, C0, C1, C1, 'x, KX, RX}, CTL }; end // RET
  16'b1001_0101_0001_1000: begin dec = '{ GPR                      , ALU                , MUL, SRG, '{C0, C0, C1, lsu_pc            , C0, WX}, IOU, '{C1, C0, C1, C1, 'x, KX, RX}, CTL }; end // RETI
  // TODO, add interrupt status to ifu_cfg_t
  // branches
  //                                    {       alu                            ifu                                                 }
  //                                    {       {m  , d  , r , c }             {ii. sk, be, ad            , we, wd}                }
  16'b1111_00??_????_????: begin dec = '{ GPR, '{ADW, pcn, Ks, C1}, MUL, SRG, '{C0, C0, C1, alu_t[PAW-1:0], C0, WX}, IOU, LSU, CTL }; end // BRBS
  16'b1111_01??_????_????: begin dec = '{ GPR, '{ADW, pcn, Ks, C1}, MUL, SRG, '{C0, C0, C1, alu_t[PAW-1:0], C0, WX}, IOU, LSU, CTL }; end // BRBC

//  16'b1001_0101_1100_1000: begin	/* LPM */	pmem_selz = 1'b1;	pmem_ce = 1'b1;	next_state = LPM; end
  // no operation, same as NOP
  // TODO: it might make sense to assign X
  default:                 begin dec = '{ GPR, ALU, MUL, SRG, IFU, IOU, LSU, CTL }; end // NOP
endcase

////////////////////////////////////////////////////////////////////////////////
// register file access
////////////////////////////////////////////////////////////////////////////////

// stall can be caused by ALU (not in this implementation) IFU or LSU
assign stall = stall_ifu | stall_lsu;

// GPR write access
always_ff @ (posedge clk)
if (writeback) begin
  // TODO, add writeback option from load/store unit
end else if (~stall) begin
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
  3'b??0: alu_t = dec.alu.d + dec.alu.r + dec.alu.c;
  3'b??1: alu_t = dec.alu.d - dec.alu.r - dec.alu.c;
endcase

// ALU mode selection
// TODO optimize mode encoding to allign with opcode bits for ADD/SUB
always_comb
unique case (dec.alu.m)
  ADD: begin  alu_s = alu_sb;  alu_rb = alu_t[8-1:0];                 alu_rw = 'x;             end
  SUB: begin  alu_s = alu_sb;  alu_rb = alu_t[8-1:0];                 alu_rw = 'x;             end
  ADW: begin  alu_s = alu_sw;  alu_rb = 'x;                           alu_rw = alu_t[16-1:0];  end
  SBW: begin  alu_s = alu_sw;  alu_rb = 'x;                           alu_rw = alu_t[16-1:0];  end
  AND: begin  alu_s = alu_sb;  alu_rb = dec.alu.d & dec.alu.r;        alu_rw = 'x;             end
  OR : begin  alu_s = alu_sb;  alu_rb = dec.alu.d | dec.alu.r;        alu_rw = 'x;             end
  EOR: begin  alu_s = alu_sb;  alu_rb = dec.alu.d ^ dec.alu.r;        alu_rw = 'x;             end
  SHR: begin  alu_s = alu_sb;  alu_rb = {dec.alu.c, dec.alu.d[7:1]};  alu_rw = 'x;             end
endcase

// status for ( 8 bit byte operations)
assign alu_sb.i = 1'bx;
assign alu_sb.t = 1'bx;
assign alu_sb.h = dec.alu.d[3] & dec.alu.r[3] | dec.alu.r[3] & ~alu_rb[3] | ~alu_rb[3] & dec.alu.d[3];
assign alu_sb.s = alu_sb.n ^ alu_sb.v;
assign alu_sb.v = dec.alu.d[7] & dec.alu.r[7] & ~alu_rb[7] | ~dec.alu.d[7] & ~dec.alu.r[7] & alu_rb[7];
assign alu_sb.n = alu_rb[7];
assign alu_sb.z = ~|alu_rb;
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
// instruction fetch unit
////////////////////////////////////////////////////////////////////////////////

// reset status
logic ifu_rst;

always_ff @(posedge clk, posedge rst)
if (rst) ifu_rst <= 1'b1;
else     ifu_rst <= 1'b0;

// instruction fetch state
enum logic [2-1:0] {
  NRM = 2'b00, // normal
  SKP = 2'b01, // skip
  WBK = 2'b10, // writeback
  P32 = 2'b11  // instruction 32 bit long
} ifu_sts;

// instruction fetch state machine
always_ff @(posedge clk, posedge rst)
if (rst) ifu_sts <= NRM;
else begin
  ifu_sts <= NRM;
end

// program counter
// - on reset it is loaded with ones so the incremented value points to zero
// - on clock it stores the address of the current instruction, or debugger jump
always_ff @(posedge clk, posedge rst)
if (rst)         pc <= '1;
else if (bp_rdy) pc <= bp_jmp ? bp_npc : bp_adr;

// program address
assign bp_adr = dec.ifu.be ? dec.ifu.ad : pcn;

// program counter increment
assign pcn = pc + 'd1;

// program memory enable
assign bp_vld = ~ifu_rst;

// TODO: this should be more complex
// TODO: some skip code should probably be here
assign stall_ifu = 1'b0;

// program memory write enable
always_ff @(posedge clk, posedge rst)
if (rst) bp_wen <= 1'b0;
else     bp_wen <= dec.ifu.we;

// program memory write data
always_ff @(posedge clk)
bp_wdt <= dec.ifu.wd;

// program word (just a short variable name)
// TODO, there should be a registered version of the previous instruction for 32bit instructions
assign pi = bp_rdt;
assign pw = (ifu_sts == P32) ? pr : pi;

always_ff @(posedge clk)
if (dec.ifu.ii) pr <= pi;


// TODO exception code should be here too

////////////////////////////////////////////////////////////////////////////////
// status register access
////////////////////////////////////////////////////////////////////////////////

// SPR write access
always_ff @ (posedge clk, posedge rst)
if (rst) begin
  rampd      <= 8'h00;
  rampx      <= 8'h00;
  rampy      <= 8'h00;
  rampz      <= 8'h00;
  eind       <= 8'h00;
  sp[8*0+:8] <= 8'h00;
  sp[8*1+:8] <= 8'h00;
  sreg       <= 8'h00;
  sreg       <= 8'h00;
end else if (~stall) begin
  if (dec.iou.we) begin
    unique case (dec.iou.ad)
      IOA_RAMPD: rampd      <= dec.iou.wd;
      IOA_RAMPX: rampx      <= dec.iou.wd;
      IOA_RAMPY: rampy      <= dec.iou.wd;
      IOA_RAMPZ: rampz      <= dec.iou.wd;
      IOA_EIND : eind       <= dec.iou.wd;
      IOA_SPL  : sp[8*0+:8] <= dec.iou.wd;
      IOA_SPH  : sp[8*1+:8] <= dec.iou.wd;
      IOA_SREG : sreg       <= dec.iou.wd;
    endcase
  end else begin
    // TODO access to extended registers and SP
    // SP incrementing/decrementing
    // SREG is updated by aritmetic/logic and dedicated instructions
    sreg <= (dec.srg.s & dec.srg.m) | (sreg & ~dec.srg.m);
  end
end

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
    IOA_RAMPD: id = rampd     ;
    IOA_RAMPX: id = rampx     ;
    IOA_RAMPY: id = rampy     ;
    IOA_RAMPZ: id = rampz     ;
    IOA_EIND : id = eind      ;
    IOA_SPL  : id = sp[8*0+:8];
    IOA_SPH  : id = sp[8*1+:8];
    IOA_SREG : id = sreg      ;
    default  : id = io_rdt    ;
  endcase
end

// TODO: add comment
assign ed = {rampd, pw}; // extended direct address
assign ex = {rampx, Rw};
assign ey = {rampy, Rw};
assign ez = {rampz, Rw};
assign ea = alu_t;

////////////////////////////////////////////////////////////////////////////////
// load/store unit
////////////////////////////////////////////////////////////////////////////////

always_ff @(posedge clk, posedge rst)
if (rst) begin
  bd_req <= 1'b0;
end else begin
  bd_req <= dec.lsu.en;
end

always_ff @(posedge clk)
if (dec.lsu.en) begin
  bd_wen <= dec.lsu.we;
  bd_adr <= dec.lsu.st ? sp         : dec.lsu.ad;
  if (dec.lsu.we)
  bd_wdt <= dec.lsu.sb ? pc[0*8+:8] : dec.lsu.wd;
end

// stall on instructions reading from memory load/pull/ret
// TODO: reading should stall only if a dirty register is accessed or if there is a writeback conflict

assign stall_lsu = stall_lsu_cmb | stall_lsu_reg;

assign stall_lsu_cmb = dec.lsu.en & ~dec.lsu.we;

always_ff @(posedge clk, posedge rst)
if (rst) begin
  stall_lsu_reg <= 1'b0;
end else begin
  stall_lsu_reg <= stall_lsu_cmb;
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
  dump_pc   = pc;
  dump_sp   = sp;
  dump_sreg = sreg;
endfunction: dump_state_core

`endif

////////////////////////////////////////////////////////////////////////////////
// VCD specific bench code
////////////////////////////////////////////////////////////////////////////////

/* verilator lint_off UNUSED */

// general purpose registers decode structure
logic           dec_gpr_we; // write enable
logic           dec_gpr_ww; // write word (0 - 8 bit mode, 1 - 16 bit mode)
logic  [16-1:0] dec_gpr_wd; // write data 16 bit
logic   [5-1:0] dec_gpr_wa; // write address 
logic   [5-1:0] dec_gpr_rw; // read address for word (16 bit)
logic   [5-1:0] dec_gpr_rb; // read address for byte (8 bit)
// arithmetic logic unit decode structure
logic   [3-1:0] dec_alu_m; // alu modes
logic  [AW-1:0] dec_alu_d; // destination operand value
logic  [AW-1:0] dec_alu_r; // source      operand value
logic           dec_alu_c; // carry input
// multiplier decode structure
logic           dec_mul_m_f; // fractional
logic           dec_mul_m_d; // destination (0 - unsigned, 1 - signed)
logic           dec_mul_m_r; // source      (0 - unsigned, 1 - signed)
logic   [8-1:0] dec_mul_d; // destination operand value
logic   [8-1:0] dec_mul_r; // source      operand value
// status register decode structure
sreg_t          dec_srg_s; // status
sreg_t          dec_srg_m; // mask
// instruction fetch unit decode structure
logic           dec_ifu_ii; // instruction immediate
logic           dec_ifu_sk; // skip
logic           dec_ifu_be; // branch enable
logic [PAW-1:0] dec_ifu_ad; // address
logic           dec_ifu_we; // write enable (for SPM instruction)
logic  [16-1:0] dec_ifu_wd; // write data   (for SPM instruction)
// input/output unit decode structure
logic           dec_iou_we; // write enable
logic           dec_iou_re; // read  enable
logic   [6-1:0] dec_iou_ad; // address
logic   [8-1:0] dec_iou_wd; // write data
logic   [8-1:0] dec_iou_ms; // write mask
// load/store unit decode structure
logic           dec_lsu_en; // enable
logic           dec_lsu_we; // write enable
logic           dec_lsu_st; // stack push/pop
logic           dec_lsu_sb; // subroutine/interrupt call/return
logic [DAW-1:0] dec_lsu_ad; // address
logic   [8-1:0] dec_lsu_wd; // write data
logic   [5-1:0] dec_lsu_dr; // destination register (if not PC)
// control decode structure
logic           dec_ctl_slp; // sleep
logic           dec_ctl_brk; // break
logic           dec_ctl_wdr; // watchdog reset

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

/* verilator lint_on UNUSED */

endmodule: rp_8bit
