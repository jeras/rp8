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
  parameter int unsigned BI_IW =  8, // interrupt request width
  parameter int unsigned PAW = 11, // 16 bit words
  parameter int unsigned DAW = 13  //  8 bit bytes
)(
  // system signals
  input  logic           clk,
  input  logic           rst,
  // instruction bus
  output logic           pmem_ce,
  output logic [PAW-1:0] pmem_a ,
  input  logic  [16-1:0] pmem_d ,
  // data bus
  output logic           dmem_we,
  output logic [DAW-1:0] dmem_a ,
  output logic   [8-1:0] dmem_do,
  input  logic   [8-1:0] dmem_di,
  // peripheral bus
  output logic           io_re,
  output logic           io_we,
  output logic   [6-1:0] io_a ,
  output logic   [8-1:0] io_do,
  input  logic   [8-1:0] io_di,
  // interrupt
  input  logic [BI_IW-1:0] irq,
  output logic [BI_IW-1:0] irq_ack
);

////////////////////////////////////////////////////////////////////////////////
// calculated parameters
////////////////////////////////////////////////////////////////////////////////

// maximum address width
localparam int unsigned MAW = PAW > DAW ? PAW : DAW;

// adder destination width
localparam int unsigned ADW = MAW > 16 ? MAW : 16;

////////////////////////////////////////////////////////////////////////////////
// helper functions
////////////////////////////////////////////////////////////////////////////////

// binary to one hot
function logic [8-1:0] b2o (input logic [3-1:0] b);
  b2o = 8'h01 << b;
endfunction

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
} prg_ptr_t;

// data memory pointer
typedef struct packed {
  logic [8-1:0] e;
  logic [8-1:0] h;
  logic [8-1:0] l;
} dat_ptr_t;

// status register
typedef struct packed {logic i, t, h, s, v, n, z, c;} sreg_t;

// general purpose registers control structure
typedef struct packed {
  // write access
  logic          we; // write enable
  logic          ww; // write word (0 - 8 bit mode, 1 - 16 bit mode)
  logic [16-1:0] wd; // write data 16 bit
  logic  [5-1:0] wa; // write address 
  // read word (16 bit) access
  logic  [5-1:0] rw; // read word address
  // read byte (8 bit) access
  logic  [5-1:0] rb; // read byte address
} gpr_ctl_t;

// arithmetic logic unit control structure
typedef struct packed {
  enum logic [2:0] {
    ADD = 3'b000, // addition
    SUB = 3'b001, // subtraction
    ADW = 3'b010, // addition    for word or address
    SBW = 3'b011, // subtraction for word or address
    AND = 3'b100, // logic and
    OR  = 3'b101, // logic or
    EOR = 3'b110, // logic eor
    SHR = 3'b111  // shift right
  } m;             // alu modes
  logic [8-1:0] d; // destination register value
  logic [8-1:0] r; // source      register value
  logic         c; // carry input
} alu_ctl_t;

// multiplier control structure
struct packed {
  struct packed {
    logic f; // fractional
    logic d; // destination (0 - unsigned, 1 - signed)
    logic r; // source      (0 - unsigned, 1 - signed)
  } m;             // adder modes
  logic [8-1:0] d; // destination register value
  logic [8-1:0] r; // source      register value
} mul_cfg_t;

// status register control structure
typedef struct packed {
  sreg_t s; // status
  sreg_t m; // mask
} srg_ctl_t;

// entire control structure
typedef struct packed {
  gpr_ctl_t gpr; // general purpose registers
  alu_ctl_t alu; // arithmetic logic unit
  alu_ctl_t add; // address adder
  srg_ctl_t srg; // status register
  prg_ctl_t prg; // program unit
  iou_ctl_t iou; // input/output unit
  lsu_ctl_t lsu; // load/store unit
} ctl_t;
  
////////////////////////////////////////////////////////////////////////////////
// local variables
////////////////////////////////////////////////////////////////////////////////

// program word
logic [16-1:0] pw;

// read register values
logic  [8-1:0] Rd; // destination
logic  [8-1:0] Rr; // source
logic [16-1:0] Rw; // word
logic  [8-1:0] Rs; // nibble swap of Rd

// program counter
prg_ptr_t pc;

// stack pointer
dat_ptr_t sp;

// status register
sreg_t sreg;

// register file
gpr_t gpr_f;

// various sources of core stall
logic stall;

////////////////////////////////////////////////////////////////////////////////
// register addresses and immediates
////////////////////////////////////////////////////////////////////////////////

// program word (just a short variable name)
assign pw = pmem_d;

// destination/source register address for full space bytes (used by MOV and arithmetic)
logic [5-1:0] db =         pw[8:4] ;
logic [5-1:0] rb = {pw[9], pw[3:0]};
// destination/source register address for full space words (used by MOVW)
logic [5-1:0] dw =        {pw[7:4], 1'b0};
logic [5-1:0] rw =        {pw[3:0], 1'b0};
// destination/source register address for high half space (used by MULS, arithmetic immediate, load store direct)
logic [5-1:0] dh =  {1'b1, pw[7:4]};
logic [5-1:0] rh =  {1'b1, pw[3:0]};
// destination/source register address for third quarter space (used by *MUL*)
logic [5-1:0] dm = {2'b10, pw[6:4]};
logic [5-1:0] rm = {2'b10, pw[2:0]};
// destination register address for index registers (used by ADIW/SBIW)
logic [5-1:0] di = {2'b11, pw[5:4], 1'b0};

// register address constant
localparam logic [5-1:0] R0 = 5'h00; // used for multiplication destination address
localparam logic [5-1:0] RX = 5'hxx;

// bit address
logic [3-1:0] b  = pw[2:0];

// bit constants
localparam logic         CX = 1'bx;
localparam logic         C0 = 1'b0;
localparam logic         C1 = 1'b1;

// byte (8 bit) constants
localparam logic [8-1:0] KX = 8'hxx;
localparam logic [8-1:0] KF = 8'hff;
localparam logic [8-1:0] K0 = 8'h00;
localparam logic [8-1:0] K1 = 8'h01;

// byte (8 bit) immediate for ALU operations
logic [8-1:0] kb = {pw[11:8], pw[3:0]};
// word (6bit) for address adder
logic [6-1:0] kw = {pw[7:6], pw[3:0]};

logic signed [12-1:0] Kl = pw[11:0];
logic signed  [7-1:0] Ks = pw[ 9:3];
logic         [6-1:0] q = {pw[13], pw[11:10], pw[2:0]};

// reusable_results
logic Rd_b = Rd[b];

////////////////////////////////////////////////////////////////////////////////
// instruction decoder
////////////////////////////////////////////////////////////////////////////////

// constants for idling units
localparam gpr_ctl_t GPR_I = `{we: 1'b0, ww: 1'b0, wd: 16'hxxxx, wa: 5'hxx, rb: RX, rw: RX};
localparam alu_ctl_t ALU_I = `{m: 3'bxxx, d: 8'hxx, r: 8'hxx, c: 1'bx};
localparam alu_ctl_t MUL_I = `{m: 3'bxxx, d: 8'hxx, r: 8'hxx, c};
localparam srg_ctl_t SRG_I = `{s: KX, m: 8'h00};
localparam prg_ctl_t PRG_I = `{};
localparam iou_ctl_t IOU_I = `{};
localparam lsu_ctl_t LSU_I = `{};

always_comb
casez (pw)
  // no operation, same as default
  16'b0000_0000_0000_0000: begin cfg = '{ GPR_I, ALU_I, SRG_I, PRG_I, IOU_I, LSU_I }; end // NOP
  // arithmetic
  //                                    {  gpr                               alu                            srg                                        }
  //                                    {  {we, ww, wd       , wa, rw, rb}   {m  , d , r , c     }          {s    , m    }                             }
  16'b0000_01??_????_????: begin cfg = '{ '{C0, C0, {2{alu_r}, db, db, rb}, '{SUB, Rd, Rr, sreg.c}, MUL_I, '{alu_s, 8'h3f}, PRG_I, IOU_I, LSU_I }; end // CPC
  16'b0000_10??_????_????: begin cfg = '{ '{C1, C0, {2{alu_r}, db, db, rb}, '{SUB, Rd, Rr, sreg.c}, MUL_I, '{alu_s, 8'h3f}, PRG_I, IOU_I, LSU_I }; end // SBC
  16'b0000_11??_????_????: begin cfg = '{ '{C1, C0, {2{alu_r}, db, db, rb}, '{ADD, Rd, Rr, C0    }, MUL_I, '{alu_s, 8'h3f}, PRG_I, IOU_I, LSU_I }; end // ADD
  16'b0001_01??_????_????: begin cfg = '{ '{C0, C0, {2{alu_r}, db, db, rb}, '{SUB, Rd, Rr, C0    }, MUL_I, '{alu_s, 8'h3f}, PRG_I, IOU_I, LSU_I }; end // CP
  16'b0001_10??_????_????: begin cfg = '{ '{C1, C0, {2{alu_r}, db, db, rb}, '{SUB, Rd, Rr, C0    }, MUL_I, '{alu_s, 8'h3f}, PRG_I, IOU_I, LSU_I }; end // SUB
  16'b0001_11??_????_????: begin cfg = '{ '{C1, C0, {2{alu_r}, db, db, rb}, '{ADD, Rd, Rr, sreg.c}, MUL_I, '{alu_s, 8'h3f}, PRG_I, IOU_I, LSU_I }; end // ADC
  16'b0011_????_????_????: begin cfg = '{ '{C0, C0, {2{alu_r}, dw, dw, RX}, '{SUB, Rd, kb, C0    }, MUL_I, '{alu_s, 8'h3f}, PRG_I, IOU_I, LSU_I }; end // CPI
  16'b0100_????_????_????: begin cfg = '{ '{C1, C0, {2{alu_r}, dw, dw, RX}, '{SUB, Rd, kb, sreg.c}, MUL_I, '{alu_s, 8'h3f}, PRG_I, IOU_I, LSU_I }; end // SBCI
  16'b0101_????_????_????: begin cfg = '{ '{C1, C0, {2{alu_r}, dw, dw, RX}, '{SUB, Rd, kb, C0    }, MUL_I, '{alu_s, 8'h3f}, PRG_I, IOU_I, LSU_I }; end // SUBI
  16'b1001_010?_????_0000: begin cfg = '{ '{C1, C0, {2{alu_r}, db, db, RX}, '{SUB, KF, Rd, C0    }, MUL_I, '{alu_s, 8'h1f}, PRG_I, IOU_I, LSU_I }; end // COM
  // TODO check the value of carry and overflow
  16'b1001_010?_????_0001: begin cfg = '{ '{C1, C0, {2{alu_r}, db, db, RX}, '{SUB, K0, Rd, C0    }, MUL_I, '{alu_s, 8'h3f}, PRG_I, IOU_I, LSU_I }; end // NEG
  16'b1001_010?_????_0011: begin cfg = '{ '{C1, C0, {2{alu_r}, db, db, RX}, '{ADD, Rd, K0, C1    }, MUL_I, '{alu_s, 8'h3e}, PRG_I, IOU_I, LSU_I }; end // INC
  16'b1001_010?_????_1010: begin cfg = '{ '{C1, C0, {2{alu_r}, db, db, RX}, '{SUB, Rd, K0, C1    }, MUL_I, '{alu_s, 8'h3e}, PRG_I, IOU_I, LSU_I }; end // DEC
  // logic // TODO check flags
  16'b0010_00??_????_????: begin cfg = '{ '{C1, C0, {2{alu_r}, db, db, rb}, '{AND, Rd, Rr, C0    }, MUL_I, '{alu_s, 8'h1e}, PRG_I, IOU_I, LSU_I }; end // AND
  16'b0111_????_????_????: begin cfg = '{ '{C1, C0, {2{alu_r}, dw, dw, RX}, '{AND, Rd, kb, C0    }, MUL_I, '{alu_s, 8'h1e}, PRG_I, IOU_I, LSU_I }; end // ANDI
  16'b0010_10??_????_????: begin cfg = '{ '{C1, C0, {2{alu_r}, db, db, rb}, '{OR , Rd, Rr, C0    }, MUL_I, '{alu_s, 8'h1e}, PRG_I, IOU_I, LSU_I }; end // OR
  16'b0110_????_????_????: begin cfg = '{ '{C1, C0, {2{alu_r}, dw, dw, RX}, '{OR , Rd, kb, C0    }, MUL_I, '{alu_s, 8'h1e}, PRG_I, IOU_I, LSU_I }; end // ORI
  16'b0010_01??_????_????: begin cfg = '{ '{C1, C0, {2{alu_r}, db, db, rb}, '{XOR, Rd, Rr, C0    }, MUL_I, '{alu_s, 8'h1e}, PRG_I, IOU_I, LSU_I }; end // EOR
  // shift right // TODO check flags
  16'b1001_010?_????_0110: begin cfg = '{ '{C1, C0, {2{alu_r}, db, db, RX}, '{SHR, Rd, K0, C0    }, MUL_I, '{alu_s, 8'h1f}, PRG_I, IOU_I, LSU_I }; end // LSR
  16'b1001_010?_????_0111: begin cfg = '{ '{C1, C0, {2{alu_r}, db, db, RX}, '{SHR, Rd, K0, sreg.c}, MUL_I, '{alu_s, 8'h1f}, PRG_I, IOU_I, LSU_I }; end // ROR
  16'b1001_010?_????_0101: begin cfg = '{ '{C1, C0, {2{alu_r}, db, db, RX}, '{SHR, Rd, K0, Rd[7] }, MUL_I, '{alu_s, 8'h1f}, PRG_I, IOU_I, LSU_I }; end // ASR
  // multiplication
  //                                    {  gpr                                  mul                       srg                                 }
  //                                    {  {we, ww, wd   , wa, rw, rb}          {m{f , d , r }, d , r }   {s    , m    }                      }
  16'b1001_11??_????_????: begin cfg = '{ '{C1, C1, mul_r, R0, db, rb}, ALU_I, '{'{C0, C0, C0}, Rd, Rr}, '{mul_s, 8'h03}, PRG_I, IOU_I, LSU_I }; end // MUL
  16'b0000_0010_????_????: begin cfg = '{ '{C1, C1, mul_r, R0, dh, rh}, ALU_I, '{'{C0, C1, C1}, Rd, Rr}, '{mul_s, 8'h03}, PRG_I, IOU_I, LSU_I }; end // MULS
  16'b0000_0011_0???_0???: begin cfg = '{ '{C1, C1, mul_r, R0, dm, rm}, ALU_I, '{'{C0, C1, C0}, Rd, Rr}, '{mul_s, 8'h03}, PRG_I, IOU_I, LSU_I }; end // MULSU
  16'b0000_0011_0???_1???: begin cfg = '{ '{C1, C1, mul_r, R0, dm, rm}, ALU_I, '{'{C1, C0, C0}, Rd, Rr}, '{mul_s, 8'h03}, PRG_I, IOU_I, LSU_I }; end // FMUL
  16'b0000_0011_1???_0???: begin cfg = '{ '{C1, C1, mul_r, R0, dm, rm}, ALU_I, '{'{C1, C1, C1}, Rd, Rr}, '{mul_s, 8'h03}, PRG_I, IOU_I, LSU_I }; end // FMULS
  16'b0000_0011_1???_1???: begin cfg = '{ '{C1, C1, mul_r, R0, dm, rm}, ALU_I, '{'{C1, C1, C0}, Rd, Rr}, '{mul_s, 8'h03}, PRG_I, IOU_I, LSU_I }; end // FMULSU
  // register moves
  16'b0010_11??_????_????: begin cfg = '{ '{C1, C0, {2{Rr}}, db, rb}, ALU_I, MUL_I, SRG_I, PRG_I, IOU_I, LSU_I }; end // MOV
  16'b1110_????_????_????: begin cfg = '{ '{C1, C0, {2{kb}}, dw, RX}, ALU_I, MUL_I, SRG_I, PRG_I, IOU_I, LSU_I }; end // LDI
  16'b1001_010?_????_0010: begin cfg = '{ '{C1, C0, {2{Rs}}, db, RX}, ALU_I, MUL_I, SRG_I, PRG_I, IOU_I, LSU_I }; end // SWAP
  // bit manipulation
  16'b1001_010?_0???_1000: begin cfg = '{ GPR_I, ALU_I, MUL_I, '{KF, b2o(pw[6:4])}, PRG_I, IOU_I, LSU_I }; end // BSET // create a common source instead of two functions
  16'b1001_010?_1???_1000: begin cfg = '{ GPR_I, ALU_I, MUL_I, '{K0, b2o(pw[6:4])}, PRG_I, IOU_I, LSU_I }; end // BCLR
  // 16-24 bit adder
  16'b1001_0110_????_????: begin cfg = '{ '{C1, C1, alu_r, di, RX}, ALU_I, '{ADD, Rw, kw}, '{alu_s, 8'h1f}, PRG_I, IOU_I, LSU_I }; end // ADIW
  16'b1001_0111_????_????: begin cfg = '{ '{C1, C1, alu_r, di, RX}, ALU_I, '{SUB, Rw, kw}, '{alu_s, 8'h1f}, PRG_I, IOU_I, LSU_I }; end // SBIW
  // bit manipulation
  16'b1111_101?_????_0???: begin cfg = '{ '{C0, C0, 16'hxxxx                                  , db, RX}, ALU_I, MUL_I, srg = '{{8{Rd_b}}, 8'h40}, PRG_I, IOU_I, LSU_I }; end // SBT
  16'b1111_100?_????_0???: begin cfg = '{ '{C1, C0, {2{Rd & ~b2o(b)) | {8{sreg.t}} & b20(b))}}, db, RX}, ALU_I, MUL_I, srg = '{KX,        8'h00}, PRG_I, IOU_I, LSU_I }; end // BLD

  /* TODO: SLEEP is not implemented */
  /* TODO: WDR is not implemented */
  16'b1001_00??_????_1111, /* PUSH/POP */
  16'b1001_00??_????_1100, /*  X   */
  16'b1001_00??_????_1101, /*  X+  */
  16'b1001_00??_????_1110, /* -X   */
  16'b1001_00??_????_1001, /*  Y+  */
  16'b1001_00??_????_1010, /* -Y   */
  16'b10?0_????_????_1???, /*  Y+q */
  16'b1001_00??_????_0001, /*  Z+  */
  16'b1001_00??_????_0010, /* -Z   */
  16'b10?0_????_????_0???: /*  Z+q */
  begin
  	/* LD - POP (run from state WRITEBACK) */
  	R = dmem_di;
  end
  16'b1011_0???_????_????: begin
  	/* IN (run from state WRITEBACK) */
  	case(io_sel)
  		IO_SEL_STACK: R = io_sp;
  		IO_SEL_SREG:  R = sreg;
  		default: R = io_di;
  	endcase
  end
  16'b1100_????_????_????: begin /* RJMP */pc_sel = PC_SEL_KL;	next_state = STALL;end
  16'b1101_????_????_????: begin/* RCALL */dmem_sel = DMEM_SEL_SP_PCL;dmem_we = 1'b1;push = 1'b1;next_state = RCALL;end
  16'b0001_00??_????_????: begin/* CPSE */pc_sel = PC_SEL_INC;pmem_ce = 1'b1; if(reg_equal) next_state = SKIP; end
  16'b1111_11??_????_0???: begin/* SBRC - SBRS */	pc_sel = PC_SEL_INC;pmem_ce = 1'b1; if(Rd_b == pmem_d[9])next_state = SKIP;end
  /* SBIC, SBIS, SBI, CBI are not implemented TODO*/
  16'b1111_0???_????_????: begin 	/* BRBS - BRBC */pmem_ce = 1'b1;
  	if (sreg[b] ^ pmem_d[10]) begin pc_sel = PC_SEL_KS;next_state = STALL; end
  	else				pc_sel = PC_SEL_INC;
  end
  16'b1001_00??_????_1100, /*  X   */
  16'b1001_00??_????_1101, /*  X+  */
  16'b1001_00??_????_1110, /* -X   */
  16'b1001_00??_????_1001, /*  Y+  */
  16'b1001_00??_????_1010, /* -Y   */
  16'b10?0_????_????_1???, /*  Y+q */
  16'b1001_00??_????_0001, /*  Z+  */
  16'b1001_00??_????_0010, /* -Z   */
  16'b10?0_????_????_0???: /*  Z+q */
  begin
    casez ({pmem_d[12], pmem_d[3:0]})
      5'b1_1100: dmem_sel = DMEM_SEL_X;
      5'b1_1101: dmem_sel = DMEM_SEL_XPLUS;
      5'b1_1110: dmem_sel = DMEM_SEL_XMINUS;
      5'b1_1001: dmem_sel = DMEM_SEL_YPLUS;
      5'b1_1010: dmem_sel = DMEM_SEL_YMINUS;
      5'b0_1???: dmem_sel = DMEM_SEL_YQ;
      5'b1_0001: dmem_sel = DMEM_SEL_ZPLUS;
      5'b1_0010: dmem_sel = DMEM_SEL_ZMINUS;
      5'b0_0???: dmem_sel = DMEM_SEL_ZQ;
    endcase
    if(pmem_d[9]) begin /* ST */pc_sel = PC_SEL_INC;pmem_ce = 1'b1;	dmem_we = 1'b1; end
    else begin/* LD */		next_state = WRITEBACK;			end
  end
  16'b1011_0???_????_????: begin/* IN */	io_re = 1'b1; next_state = WRITEBACK; end
  16'b1011_1???_????_????: begin/* OUT */ io_we = 1'b1; pc_sel = PC_SEL_INC; pmem_ce = 1'b1;end
  16'b1001_00??_????_1111: begin
  	if(pmem_d[9]) begin/* PUSH */push = 1'b1;dmem_sel = DMEM_SEL_SP_R;dmem_we = 1'b1;pc_sel = PC_SEL_INC; pmem_ce = 1'b1; end
  	else begin	/* POP */pop = 1'b1;dmem_sel = DMEM_SEL_SP_R;next_state = WRITEBACK;end
  end
  16'b1001_00??_????_0000: begin	pc_sel = PC_SEL_INC; pmem_ce = 1'b1; next_state = (pmem_d[9]) ? STS : LDS1;end
  16'b1001_0101_000?_1000: begin	/* RET / RETI */ dmem_sel = DMEM_SEL_SP_PCH;pop = 1'b1;	next_state = (pmem_d[4] == 1'b0) ? RET1 : RETI1;end
  16'b1001_0101_1100_1000: begin	/* LPM */	pmem_selz = 1'b1;	pmem_ce = 1'b1;	next_state = LPM; end
  16'b1001_0100_0000_1001: begin	/* IJMP */	pc_sel = PC_SEL_Z;	next_state = STALL;	end
  16'b1001_0101_0000_1001: begin	/* ICALL */	dmem_sel = DMEM_SEL_SP_PCL;dmem_we = 1'b1;push = 1'b1;	next_state = ICALL;end
  default: begin					pc_sel = PC_SEL_INC;	normal_en = 1'b1; pmem_ce = 1'b1; end
endcase
      	end /* if(normal_en) */
      	if(lds_writeback) begin
      		R = dmem_di;
      		writeback = 1'b1;
      	end
      	if(lpm_en) begin
      		R = gpr.nam.z[0] ? pmem_d[15:8] : pmem_d[7:0];
      		writeback = 1'b1;
      	end
      	if(io_we & (io_a == 6'b111111))
      		sreg <= io_do[7:0];
      	if(I_clr)
      		sreg.i <= 1'b0;
      	if(writeback) begin
      		if(mode16) begin
      			// $display("REG WRITE(16): %d < %d", Rw, R16);
      			//gpr.idx [{2'b11,Rw,1'b0}+:2] <= R16;  // TODO
      			{gpr.idx [{2'b11,Rw,1'b1}], gpr.idx [{2'b11,Rw,1'b0}]} <= R16;
      		end else begin
      			// $display("REG WRITE: %d < %d", Rd, R);
      			gpr.idx [write_dest] <= R;
      		end
      	end else begin /* if(writeback) */
      		case(dmem_sel)
      			DMEM_SEL_XPLUS:		gpr.nam.x <= gpr.nam.x + 16'd1;
      			DMEM_SEL_XMINUS:	gpr.nam.x <= gpr.nam.x - 16'd1;
      			DMEM_SEL_YPLUS:		gpr.nam.y <= gpr.nam.y + 16'd1;
      			DMEM_SEL_YMINUS:	gpr.nam.y <= gpr.nam.y - 16'd1;
      			DMEM_SEL_ZPLUS:		gpr.nam.z <= gpr.nam.z + 16'd1;
      			DMEM_SEL_ZMINUS:	gpr.nam.z <= gpr.nam.z - 16'd1;
      			default:;
      		endcase
      	end
      end /* if(rst) ... else */
end

////////////////////////////////////////////////////////////////////////////////
// register file access
////////////////////////////////////////////////////////////////////////////////

// GPR control structure
gpr_ctl_t gpr = ctl.gpr;

// GPR write access
always_ff @ (posedge clk)
if (gpr.we) begin
  // TODO recode this, so it is appropriate for a register file or at least optimized
  if (gpr.ww) gpr_f.idx [{wa[5-1:1], 1'b0}+:2] <= wd;
  else        gpr_f.idx [ wa                 ] <= wd[8-1:0];
end

// read word access
assign Rw = gpr_f.idx [{rw[5-1:1], 1'b0}+:2];
assign Rd = gpr_f.idx [rw];

// read byte access
assign Rr = gpr_f.idx [rb];

// swap of Rd
assign Rs = {Rd[3:0], Rd[7:4]};

////////////////////////////////////////////////////////////////////////////////
// ALU (8 bit)
////////////////////////////////////////////////////////////////////////////////

// ALU control structure
alu_ctl_t alu = ctl.alu;

// results
logic [ADW-0:0] alu_t; // result (full width plus carry)
logic   [8-1:0] alu_r; // result (8 bit byte)
sreg_t          alu_b; // status for ( 8 bit byte operations)
sreg_t          alu_w; // status for (16 bit word operations)
sreg_t          alu_s; // status

// adder
// TODO optimize adder
always_comb
casez (alu.m)
  3'b??0: alu_t = alu.d + alu.r + alu.c;
  3'b??1: alu_t = alu.d - alu.r - alu.c;
endcase

// ALU mode selection
// TODO optimize mode encoding to allign with opcode bits for ADD/SUB
always_comb
case (alu.md)
  ADD: begin  alu_s = alu_b;  alu_r = alu_t[8-1:0];         end
  SUB: begin  alu_s = alu_b;  alu_r = alu_t[8-1:0];         end
  ADW: begin  alu_s = alu_w;  alu_r = 'x;                   end
  SBW: begin  alu_s = alu_w;  alu_r = 'x;                   end
  AND: begin  alu_s = alu_b;  alu_r = alu.d & alu.r;        end
  OR : begin  alu_s = alu_b;  alu_r = alu.d | alu.r;        end
  EOR: begin  alu_s = alu_b;  alu_r = alu.d ^ alu.r;        end
  SHR: begin  alu_s = alu_b;  alu_r = {alu.c, alu.d[7:1]};  end
endcase

// status for ( 8 bit byte operations)
assign alu_b.i = 1'bx;
assign alu_b.t = 1'bx;
assign alu_b.h = alu.d[3] & alu.r[3] | alu.r[3] & ~alu_r[3] | ~alu_r[3] & alu.d[3];
assign alu_b.s = alu_b.n ^ alu_b.v;
assign alu_b.v = alu.d[7] & alu.r[7] & ~alu_r[7] | ~alu.d[7] & ~alu.r[7] & alu_r[7];
assign alu_b.n = alu_r[7];
assign alu_b.z = ~|alu_r;
assign alu_b.c = (alu.m == SHR) ? alu.d[0] : alu_t[8];

// status for (16 bit word operations)
assign alu_w.i = 1'bx;
assign alu_w.t = 1'bx;
assign alu_w.h = 1'bx;
assign alu_w.s = alu_w.n ^ alu_w.v;
assign alu_w.v = ~alu.d[15] & alu.r[15];
assign alu_w.n = alu_t[15];
assign alu_w.z = ~|alu_t[15:0];
assign alu_w.c = alu_t[16];

////////////////////////////////////////////////////////////////////////////////
// multiplier (8 bit * 8 bit)
////////////////////////////////////////////////////////////////////////////////

// adder control structure
mul_ctl_t mul = ctl.mul;

logic [18-1:0] mul_t; // result tmp
logic [16-1:0] mul_r; // result
sreg_t         mul_s;

assign mul_t = $signed({mul.m.d & mul.d[7], mul.d}) * $signed({mul.m.r & mul.r[7], add.r});

assign mul_r = mul.m.f ? {mul_t[14:0], C0} : mul_t[15:0];

assign mul_s.i = 1'bx;
assign mul_s.t = 1'bx;
assign mul_s.h = 1'bx;
assign mul_s.s = 1'bx;
assign mul_s.v = 1'bx;
assign mul_s.n = 1'bx;
assign mul_s.z = ~|mul_r;
assign mul_s.c = mul_t[15];

////////////////////////////////////////////////////////////////////////////////
// status register access
////////////////////////////////////////////////////////////////////////////////

// SREG write access
always_ff @ (posedge clk, posedge rst)
if (rst) sreg <= 8'b00000000;
else     sreg <= (sreg_tmp & sreg_msk) | (sreg & ~sreg_msk);

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

always_ff @(posedge clk, posedge rst)
if (rst)        PC            <= '1;
else case (pc_sel)
  PC_SEL_NOP  :;
  PC_SEL_INC  : PC            <= pc_inc;
  // !!! WARNING !!! replace with PC <= PC + {{PAW-12{Kl[11]}}, Kl}; if PAW>12
  PC_SEL_KL   : PC            <= PC + Kl;
  PC_SEL_KS   : PC            <= PC + {{PAW-7{Ks[6]}}, Ks};
  PC_SEL_DMEML: PC[7:0]       <= dmem_di;
  PC_SEL_DMEMH: PC[PAW-1:8] <= dmem_di[PAW-8-1:0];
  PC_SEL_DEC  : PC            <= PC - 1;
  PC_SEL_Z    : PC            <= gpr.nam.z - 1;
  PC_SEL_EX   : PC            <= PC_ex;
endcase

reg pmem_selz;
assign pmem_a = pmem_selz ? gpr.nam.z[15:1] : pc_inc;

/* Load/store operations */
reg [3:0] dmem_sel;

localparam DMEM_SEL_UNDEFINED	= 3'bxxx;
localparam DMEM_SEL_X		= 4'd0;
localparam DMEM_SEL_XPLUS	= 4'd1;
localparam DMEM_SEL_XMINUS	= 4'd2;
localparam DMEM_SEL_YPLUS	= 4'd3;
localparam DMEM_SEL_YMINUS	= 4'd4;
localparam DMEM_SEL_YQ		= 4'd5;
localparam DMEM_SEL_ZPLUS	= 4'd6;
localparam DMEM_SEL_ZMINUS	= 4'd7;
localparam DMEM_SEL_ZQ		= 4'd8;
localparam DMEM_SEL_SP_R	= 4'd9;
localparam DMEM_SEL_SP_PCL	= 4'd10;
localparam DMEM_SEL_SP_PCH	= 4'd11;
localparam DMEM_SEL_PMEM	= 4'd12;

/* ALU */

reg normal_en;
reg lds_writeback;

wire [4:0] write_dest = lds_writeback ? Rd_r : Rd;

logic I_clr;
logic I_set;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/* I/O port */
assign io_a = {pmem_d[10:9], pmem_d[3:0]};
assign io_do = Rd;

/* Data memory */
always_comb
case (dmem_sel)
	DMEM_SEL_X,
	DMEM_SEL_XPLUS:		dmem_a = gpr.nam.x;
	DMEM_SEL_XMINUS:	dmem_a = gpr.nam.x - 16'd1;
	DMEM_SEL_YPLUS:		dmem_a = gpr.nam.y;
	DMEM_SEL_YMINUS:	dmem_a = gpr.nam.y - 16'd1;
	DMEM_SEL_YQ:		dmem_a = gpr.nam.y + q;
	DMEM_SEL_ZPLUS:		dmem_a = gpr.nam.z;
	DMEM_SEL_ZMINUS:	dmem_a = gpr.nam.z - 16'd1;
	DMEM_SEL_ZQ:		dmem_a = gpr.nam.z + q;
	DMEM_SEL_SP_R,
	DMEM_SEL_SP_PCL,
	DMEM_SEL_SP_PCH:	dmem_a = SP + pop;
	DMEM_SEL_PMEM:		dmem_a = pmem_d;
	default:		dmem_a = {DAW{1'bx}};
endcase

assign pc_inc = PC + 1;
reg exception;
always_comb
case(dmem_sel)
	DMEM_SEL_X,
	DMEM_SEL_XPLUS,
	DMEM_SEL_XMINUS,
	DMEM_SEL_YPLUS,
	DMEM_SEL_YMINUS,
	DMEM_SEL_YQ,
	DMEM_SEL_ZPLUS,
	DMEM_SEL_ZMINUS,
	DMEM_SEL_ZQ,
	DMEM_SEL_SP_R:		dmem_do = Rd;
	DMEM_SEL_SP_PCL:	dmem_do = exception ? PC[    8-1:0] : pc_inc[    8-1:0];
	DMEM_SEL_SP_PCH:	dmem_do = exception ? PC[PAW-1:8] : pc_inc[PAW-1:8];
	DMEM_SEL_PMEM:		dmem_do = Rd_r;
	default:		dmem_do = 8'hxx;
endcase

/* Multi-cycle operation sequencer */

wire reg_equal = Rd == Rr;

logic [4:0] state;
logic [4:0] next_state;

localparam NORMAL	= 5'd0;
localparam RCALL	= 5'd1;
localparam ICALL	= 5'd2;
localparam STALL	= 5'd3;
localparam RET1		= 5'd4;
localparam RET2		= 5'd5;
localparam RET3		= 5'd6;
localparam LPM		= 5'd7;
localparam STS		= 5'd8;
localparam LDS1		= 5'd9;
localparam LDS2		= 5'd10;
localparam SKIP		= 5'd11;
localparam WRITEBACK	= 5'd12;
localparam EXCEPTION	= 5'd13;
localparam RETI1	= 5'd14;
localparam RETI2	= 5'd15;
localparam RETI3	= 5'd16;
localparam RETI4	= 5'd17;

always_ff @(posedge clk, posedge rst)
if (rst) state <= NORMAL;
else     state <= next_state;

always_comb begin
	next_state = state;

	pmem_ce = rsts;

	pc_sel = PC_SEL_NOP;
	normal_en = 1'b0;
	lpm_en = 1'b0;

	io_re = 1'b0;
	io_we = 1'b0;

	dmem_we = 1'b0;
	dmem_sel = DMEM_SEL_UNDEFINED;

	push = 1'b0;
	pop = 1'b0;

	exception = 1'b0;
	irq_ack_en = 1'b0;
	I_set = 1'b0;
	I_clr = 1'b0;

	pmem_selz = 1'b0;

	regmem_ce = 1'b1;
	lds_writeback = 1'b0;
	
	case(state)
		NORMAL: begin
			if(irq_request) begin
				dmem_sel = DMEM_SEL_SP_PCL;
				dmem_we = 1'b1;
				exception = 1'b1;
				push = 1'b1;
				irq_ack_en = 1'b1;
				I_clr = 1'b1;
				next_state = EXCEPTION;
			end else begin
				casez (pmem_d)
				endcase
			end
		end
		RCALL: begin dmem_sel = DMEM_SEL_SP_PCH;	dmem_we = 1'b1;	push = 1'b1;	pc_sel = PC_SEL_KL;	next_state = STALL;	end
		EXCEPTION: begin dmem_sel = DMEM_SEL_SP_PCH;	dmem_we = 1'b1;	exception = 1'b1;	push = 1'b1;	pc_sel = PC_SEL_EX;	next_state = STALL;	end
		ICALL: begin dmem_sel = DMEM_SEL_SP_PCH;	dmem_we = 1'b1;	push = 1'b1;	pc_sel = PC_SEL_Z;	next_state = STALL;	end
		RET1 : begin pc_sel = PC_SEL_DMEMH;	dmem_sel = DMEM_SEL_SP_PCL;	pop = 1'b1;	next_state = RET2; end
		RET2 : begin pc_sel = PC_SEL_DMEML;	next_state = RET3; end
		RET3 : begin pc_sel = PC_SEL_DEC;	next_state = STALL; end
		RETI1: begin pc_sel = PC_SEL_DMEMH;	dmem_sel = DMEM_SEL_SP_PCL; pop = 1'b1; next_state = RETI2; end
		RETI2: begin pc_sel = PC_SEL_DMEML;	next_state = RETI3; end
		RETI3: begin pc_sel = PC_SEL_DEC;	next_state = RETI4; end
		RETI4: begin pc_sel = PC_SEL_INC;	pmem_ce = 1'b1;	    I_set = 1'b1; next_state = NORMAL; end
		LPM  : begin lpm_en = 1'b1;		pc_sel = PC_SEL_INC; pmem_ce = 1'b1; next_state = NORMAL; end
		STS  : begin pc_sel = PC_SEL_INC;	pmem_ce = 1'b1;	     dmem_sel = DMEM_SEL_PMEM; dmem_we = 1'b1; next_state = NORMAL; end
		LDS1 : begin dmem_sel = DMEM_SEL_PMEM;	regmem_ce = 1'b0; next_state = LDS2; end
		LDS2 : begin pc_sel = PC_SEL_INC;	pmem_ce = 1'b1;	lds_writeback = 1'b1; next_state = NORMAL;end
		SKIP : begin pc_sel = PC_SEL_INC;	pmem_ce = 1'b1;
			/* test for STS and LDS */
			if((pmem_d[15:10] == 6'b100100) & (pmem_d[3:0] == 4'h0))
				next_state = STALL; /* 2-word instruction, skip the second word as well */
			else
				next_state = NORMAL; /* 1-word instruction */
		end
		STALL: begin pc_sel = PC_SEL_INC; pmem_ce = 1'b1; next_state = NORMAL; end
		WRITEBACK: begin pmem_ce = 1'b1; pc_sel = PC_SEL_INC; normal_en = 1'b1; next_state = NORMAL; end
	endcase
end

////////////////////////////////////////////////////////////////////////////////
// exceptions
////////////////////////////////////////////////////////////////////////////////

logic [BI_IW-1:0] next_irq_ack;

always_comb
casez (irq)
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

logic [$clog2(BI_IW)-1:0] PC_ex;

always_comb
casez (irq)
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

wire irq_asserted = |irq;
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
  dump_pc   = PC;
  dump_sp   = SP;
  dump_sreg = sreg;
endfunction: dump_state_core

`endif

endmodule: rp_8bit
