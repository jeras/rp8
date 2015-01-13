/*
 * Milkymist SoC
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
  parameter BI_IW =  8, // interrupt request width
  parameter BI_AW = 11, /* < in 16-bit instructions */
  parameter BD_AW = 13  /* < in bytes */
)(
  // system signals
  input  logic             clk,
  input  logic             rst,
  // instruction bus
  output logic             pmem_ce,
  output logic [BI_AW-1:0] pmem_a ,
  input  logic    [16-1:0] pmem_d ,
  // data bus
  output logic             dmem_we,
  output logic [BD_AW-1:0] dmem_a ,
  output logic     [8-1:0] dmem_do,
  input  logic     [8-1:0] dmem_di,
  // peripheral bus
  output logic             io_re,
  output logic             io_we,
  output logic     [6-1:0] io_a ,
  output logic     [8-1:0] io_do,
  input  logic     [8-1:0] io_di,
  // interrupt
  input  logic [BI_IW-1:0] irq,
  output logic [BI_IW-1:0] irq_ack
);

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

// general purpose register control structure
typedef struct packed {
  // write access
  logic          wen; // write enable
  logic          w16; // write 16 bit mode (0 - 8 bit mode, 1 - 16 bit mode)
  logic [16-1:0] wdt; // write data 16 bit
  logic  [5-1:0] wad; // write address 
  // 8 bit read byte access
  logic          rbe; // read byte enable
  logic  [5-1:0] rba; // read byte address
  // 18 bit read word access
  logic          rwe; // read word enable
  logic  [5-1:0] rwa; // read word address
} gpr_ctl_t;

// arithmetic logic unit control structure
typedef struct packed {
  enum logic [2:0] {
    ADD = 3'b0x0, // addition
    SUB = 3'b0x1, // subtraction
    AND = 3'b100, // logic and
    OR  = 3'b101, // logic or
    EOR = 3'b110, // logic eor
    SHR = 3'b111  // shift right
  } m;             // alu modes
  logic [8-1:0] d; // destination register value
  logic [8-1:0] r; // source      register value
  logic         c; // carry input
} alu_ctl_t;

// status register control structure
typedef struct packed {
  sreg_t s; // status
  sreg_t m; // mask
} srg_ctl_t;

// entire control structure
typedef struct packed {
  gpr_ctl_t gpr;
  alu_ctl_t alu;
  srg_ctl_t srg;
  prg_ctl_t prg;
  iop_ctl_t iop;
  dat_ctl_t dat;
  spt_ctl_t spt;
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

// program counter
prg_ptr_t pc;

// stack pointer
dat_ptr_t sp;

// status register
sreg_t sreg;

// register file
gpr_t gpr_f;

////////////////////////////////////////////////////////////////////////////////
// register addresses and immediates
////////////////////////////////////////////////////////////////////////////////

// program word (just a short variable name)
assign pw = pmem_d;

// destination/source register address for full space bytes (used by MOV and arithmetic)
logic [5-1:0] db = pw[8:4];  
logic [5-1:0] rb = {pw[9] pw[3:0]};
// destination/source register address for full space words (used by MOVW)
logic [5-1:0] dw = {pw[7:4], 1'b0};
logic [5-1:0] rw = {pw[3:0], 1'b0};
// destination/source register address for high half space (used by MULS, arithmetic immediate, load store direct)
logic [5-1:0] dh = {1'b1, pw[7:4]};
logic [5-1:0] rh = {1'b1, pw[3:0]};
// destination/source register address for third quarter space (used by *MUL*)
logic [5-1:0] dm = {2'b10, pw[6:4]};
logic [5-1:0] rm = {2'b10, pw[2:0]};
// destination register address for index registers (used by ADIW/SBIW)
logic [5-1:0] di = {2'b11, pw[5:4], 1'b0};

// bit address
logic [3-1:0] b  = pw[2:0];

// 8 bit common constants
localparam logic [8-1:0] KX = 8'hxx;
localparam logic [8-1:0] KF = 8'hff;
localparam logic [8-1:0] K0 = 8'h00;
localparam logic [8-1:0] K1 = 8'h01;

// 8 bit immediate for arithmetic operations
logic [8-1:0] k8 = {pw[11:8], pw[3:0]};
//

wire [11:0] Kl = pmem_d[11:0];
wire [6:0] Ks = pmem_d[9:3];
wire [1:0] Rd16 = pmem_d[5:4];
wire [5:0] K16 = {pmem_d[7:6], pmem_d[3:0]};
wire [5:0] q = {pmem_d[13], pmem_d[11:10], pmem_d[2:0]};

logic [16-1:0] Rd16;

wire Rd_b = Rd[b];

//assign Rd16 = gpr.idx [{2'b11,Rd16,1'b0}+:2];
assign Rd16 = {gpr.idx [{2'b11,Rd16,1'b1}], gpr.idx [{2'b11,Rd16,1'b0}]};

/* Memorize values to support 16-bit instructions */
reg regmem_ce;

reg [4:0] Rd_r;
reg [7:0] Rd_r;
always_ff @(posedge clk) begin
	if(regmem_ce)
		Rd_r <= Rd; /* < control with regmem_ce */
	Rd_r <= Rd; /* < always loaded */
end

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

always_ff @(posedge clk, posedge rst)
if (rst)        PC            <= '1;
else case (pc_sel)
  PC_SEL_NOP  :;
  PC_SEL_INC  : PC            <= pc_inc;
  // !!! WARNING !!! replace with PC <= PC + {{BI_AW-12{Kl[11]}}, Kl}; if BI_AW>12
  PC_SEL_KL   : PC            <= PC + Kl;
  PC_SEL_KS   : PC            <= PC + {{BI_AW-7{Ks[6]}}, Ks};
  PC_SEL_DMEML: PC[7:0]       <= dmem_di;
  PC_SEL_DMEMH: PC[BI_AW-1:8] <= dmem_di[BI_AW-8-1:0];
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
// ALU (8 bit)
////////////////////////////////////////////////////////////////////////////////

// ALU control structure
alu_ctl_t alu = ctl.alu;

logic [8-1:0] alu_r; // result
sreg_t        alu_s; // status

always_comb
case (alu.md)
  ALU_ADD: {alu_s.c, alu_r} = alu.d + alu.r + alu.c;
  ALU_SUB: {alu_s.c, alu_r} = alu.d - alu.r - alu.c;
  ALU_AND: {alu_s.c, alu_r} = {1'bx, alu.d & alu.r};
  ALU_OR : {alu_s.c, alu_r} = {1'bx, alu.d | alu.r};
  ALU_EOR: {alu_s.c, alu_r} = {1'bx, alu.d ^ alu.r};
  ALU_SHR: {alu_r, alu_s.c} = {alu.c, alu.d};
endcase

assign alu_s.i = 1'bx;
assign alu_s.t = 1'bx;
assign alu_s.h = alu.d[3] & alu.r[3] | alu.r[3] & ~alu_r[3] | ~alu_r[3] & alu.d[3];
assign alu_s.s = alu_s.n ^ alu_s.v;
assign alu_s.v = alu.d[7] & alu.r[7] & ~alu_r[7] | ~alu.d[7] & ~alu.r[7] & alu_r[7];
assign alu_s.n = alu_r[7];
assign alu_s.z = ~|alu_r;

////////////////////////////////////////////////////////////////////////////////
// address adder (16 bit - 24 bit)
////////////////////////////////////////////////////////////////////////////////

struct packed {
  enum logic {
    ADD_ADD = 1'b0, // addition
    ADD_SUB = 1'b1  // subtraction
  } md;              // alu modes
  logic [16-1:0] rd; // destination register value
  logic [16-1:0] rr; // source      register value
} add;
logic [16-1:0] add_ro; // result
sreg_t         add_sr;

always_comb
case (alu.md)
  ALU_ADD: {add.s.c, add_ro} = add.rd + add.rr;
  ALU_SUB: {add.s.c, add_ro} = add.rd - add.rr;
endcase

assign add_s.i = 1'bx;
assign add_s.t = 1'bx;
assign add_s.h = 1'bx;
assign add_s.s = alu_sr.n ^ alu_sr.v;
assign add_s.v = ~alu.rd[15] & alu.rr[15];
assign add_s.n = alu_ro[15];
assign add_s.z = ~|alu_ro;

////////////////////////////////////////////////////////////////////////////////
// instruction decoder
////////////////////////////////////////////////////////////////////////////////

// constants for idling units
localparam gpr_ctl_t GPR_IDL = `{wen: 1'b0, w16: 1'b0, wdt: 16'hxxxx, wad: 5'hxx, rbe: 1'b0, rba: 5'hxx, rwe: 1'b0, rwa: 5'hxx};
localparam alu_ctl_t ALU_IDL = `{m: 3'bxxx, d: 8'hxx, r: 8'hxx, c: 1'bx};
localparam srg_ctl_t SRG_IDL = `{s: KX, m: 8'h00};
localparam prg_ctl_t PRG_IDL = `{};
localparam iop_ctl_t IOP_IDL = `{};
localparam dat_ctl_t DAT_IDL = `{};
localparam spt_ctl_t SPT_IDL = `{};

always_comb
casez (pw)
  // no operation, same as default
  16'b0000_0000_0000_0000: begin cfg = '{  GPR_IDL, ALU_IDL, SRG_IDL, PRG_IDL, IOP_IDL, DAT_IDL, SPT_IDL }; end // NOP
  // arithmetic
  //                                    {  gpr                             alu                     srg            }
  //                                       {wen , w16 , wdt      , , , }   {m  , d , r , c     }   {s    , m    } }
  16'b0000_01??_????_????: begin cfg = '{ '{1'b0, 1'b0, {2{alu_r}, , , }, '{SUB, Rd, Rr, sreg.c}, '{alu_s, 8'h3f}, PRG_IDL, IOP_IDL, DAT_IDL, SPT_IDL }; end // CPC
  16'b0000_10??_????_????: begin cfg = '{ '{1'b1, 1'b0, {2{alu_r}, , , }, '{SUB, Rd, Rr, sreg.c}, '{alu_s, 8'h3f}, PRG_IDL, IOP_IDL, DAT_IDL, SPT_IDL }; end // SBC
  16'b0000_11??_????_????: begin cfg = '{ '{1'b1, 1'b0, {2{alu_r}, , , }, '{ADD, Rd, Rr, 1'b0  }, '{alu_s, 8'h3f}, PRG_IDL, IOP_IDL, DAT_IDL, SPT_IDL }; end // ADD
  16'b0001_01??_????_????: begin cfg = '{ '{1'b0, 1'b0, {2{alu_r}, , , }, '{SUB, Rd, Rr, 1'b0  }, '{alu_s, 8'h3f}, PRG_IDL, IOP_IDL, DAT_IDL, SPT_IDL }; end // CP
  16'b0001_10??_????_????: begin cfg = '{ '{1'b1, 1'b0, {2{alu_r}, , , }, '{SUB, Rd, Rr, 1'b0  }, '{alu_s, 8'h3f}, PRG_IDL, IOP_IDL, DAT_IDL, SPT_IDL }; end // SUB
  16'b0001_11??_????_????: begin cfg = '{ '{1'b1, 1'b0, {2{alu_r}, , , }, '{ADD, Rd, Rr, sreg.c}, '{alu_s, 8'h3f}, PRG_IDL, IOP_IDL, DAT_IDL, SPT_IDL }; end // ADC
  16'b0011_????_????_????: begin cfg = '{ '{1'b0, 1'b0, {2{alu_r}, , , }, '{SUB, Rd, k8, 1'b0  }, '{alu_s, 8'h3f}, PRG_IDL, IOP_IDL, DAT_IDL, SPT_IDL }; end // CPI
  16'b0100_????_????_????: begin cfg = '{ '{1'b1, 1'b0, {2{alu_r}, , , }, '{SUB, Rd, k8, sreg.c}, '{alu_s, 8'h3f}, PRG_IDL, IOP_IDL, DAT_IDL, SPT_IDL }; end // SBCI
  16'b0101_????_????_????: begin cfg = '{ '{1'b1, 1'b0, {2{alu_r}, , , }, '{SUB, Rd, k8, 1'b0  }, '{alu_s, 8'h3f}, PRG_IDL, IOP_IDL, DAT_IDL, SPT_IDL }; end // SUBI
  16'b1001_010?_????_0000: begin cfg = '{ '{1'b1, 1'b0, {2{alu_r}, , , }, '{SUB, KF, Rd, 1'b0  }, '{alu_s, 8'h1f}, PRG_IDL, IOP_IDL, DAT_IDL, SPT_IDL }; end // COM
  // TODO check the value of carry and overflow
  16'b1001_010?_????_0001: begin cfg = '{ '{1'b1, 1'b0, {2{alu_r}, , , }, '{SUB, K0, Rd, 1'b0  }, '{alu_s, 8'h3f}, PRG_IDL, IOP_IDL, DAT_IDL, SPT_IDL }; end // NEG
  16'b1001_010?_????_0011: begin cfg = '{ '{1'b1, 1'b0, {2{alu_r}, , , }, '{ADD, Rd, K0, 1'b1  }, '{alu_s, 8'h3e}, PRG_IDL, IOP_IDL, DAT_IDL, SPT_IDL }; end // INC
  16'b1001_010?_????_1010: begin cfg = '{ '{1'b1, 1'b0, {2{alu_r}, , , }, '{SUB, Rd, K0, 1'b1  }, '{alu_s, 8'h3e}, PRG_IDL, IOP_IDL, DAT_IDL, SPT_IDL }; end // DEC
  // logic // TODO check flags
  16'b0010_00??_????_????: begin cfg = '{ '{1'b1, 1'b0, {2{alu_r}, , , }, '{AND, Rd, Rr, 1'b0  }, '{alu_s, 8'h1e}, PRG_IDL, IOP_IDL, DAT_IDL, SPT_IDL }; end // AND
  16'b0111_????_????_????: begin cfg = '{ '{1'b1, 1'b0, {2{alu_r}, , , }, '{AND, Rd, k8, 1'b0  }, '{alu_s, 8'h1e}, PRG_IDL, IOP_IDL, DAT_IDL, SPT_IDL }; end // ANDI
  16'b0010_10??_????_????: begin cfg = '{ '{1'b1, 1'b0, {2{alu_r}, , , }, '{OR , Rd, Rr, 1'b0  }, '{alu_s, 8'h1e}, PRG_IDL, IOP_IDL, DAT_IDL, SPT_IDL }; end // OR
  16'b0110_????_????_????: begin cfg = '{ '{1'b1, 1'b0, {2{alu_r}, , , }, '{OR , Rd, k8, 1'b0  }, '{alu_s, 8'h1e}, PRG_IDL, IOP_IDL, DAT_IDL, SPT_IDL }; end // ORI
  16'b0010_01??_????_????: begin cfg = '{ '{1'b1, 1'b0, {2{alu_r}, , , }, '{XOR, Rd, Rr, 1'b0  }, '{alu_s, 8'h1e}, PRG_IDL, IOP_IDL, DAT_IDL, SPT_IDL }; end // EOR
  // shift right // TODO check flags
  16'b1001_010?_????_0110: begin cfg = '{ '{1'b1, 1'b0, {2{alu_r}, , , }, '{SHR, Rd, K0, 1'b0  }, '{alu_s, 8'h1e}, PRG_IDL, IOP_IDL, DAT_IDL, SPT_IDL }; end // LSR
  16'b1001_010?_????_0111: begin cfg = '{ '{1'b1, 1'b0, {2{alu_r}, , , }, '{SHR, Rd, K0, 1'b1  }, '{alu_s, 8'h1e}, PRG_IDL, IOP_IDL, DAT_IDL, SPT_IDL }; end // ROR
  16'b1001_010?_????_0101: begin cfg = '{ '{1'b1, 1'b0, {2{alu_r}, , , }, '{SHR, Rd, K0, Rd[7] }, '{alu_s, 8'h1e}, PRG_IDL, IOP_IDL, DAT_IDL, SPT_IDL }; end // ASR

  // register moves
  16'b0010_11??_????_????: begin cfg = '{ '{1'b1, 1'b0, {2{Rr}}              , , , }, ALU_IDL, SRG_IDL, PRG_IDL, IOP_IDL, DAT_IDL, SPT_IDL }; end // MOV
  16'b1110_????_????_????: begin cfg = '{ '{1'b1, 1'b0, {2{k8}}              , , , }, ALU_IDL, SRG_IDL, PRG_IDL, IOP_IDL, DAT_IDL, SPT_IDL }; end // LDI
  16'b1001_010?_????_0010: begin cfg = '{ '{1'b1, 1'b0, {2{Rd[3:0], Rd[7:4]}}, , , }, ALU_IDL, SRG_IDL, PRG_IDL, IOP_IDL, DAT_IDL, SPT_IDL }; end // SWAP
  // bit manipulation
  16'b1001_010?_0???_1000: begin cfg = '{ GPR_IDL, ALU_IDL, '{KF, 8'h01 << pw[6:4]}, PRG_IDL, IOP_IDL, DAT_IDL, SPT_IDL }; end // BSET
  16'b1001_010?_1???_1000: begin cfg = '{ GPR_IDL, ALU_IDL, '{K0, 8'h01 << pw[6:4]}, PRG_IDL, IOP_IDL, DAT_IDL, SPT_IDL }; end // BCLR
  // 16-24 bit adder
  16'b1001_0110_????_????: begin alu = 'x; add = '{ADD_ADD, Rd16, K16}; srg = '{add_sr, 8'h1f}; gpr_str = '{1'b1, 1'b1, add_ro}; end // ADIW
  16'b1001_0111_????_????: begin alu = 'x; add = '{ADD_SUB, Rd16, K16}; srg = '{add_sr, 8'h1f}; gpr_str = '{1'b1, 1'b1, add_ro}; end // SBIW

  16'b1111_101?_????_0???: begin alu = 'x; add = 'x; srg = '{{8{Rd_b}}, 8'h40}; gpr_str = '{1'b0, 1'b0, 16'hxxxx}; end // SBT
  16'b1111_100?_????_0???: begin alu = 'x; add = 'x; srg = '{KX, 8'h00}; gpr_str = '{1'b1, 1'b0, {2{Rd & ~(8'h01<<b) | {8{sreg.t}} & (8'h01<<b)}}}; end // BLD

  /* SLEEP is not implemented */
  /* WDR is not implemented */
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
      			// $display("REG WRITE(16): %d < %d", Rd16, R16);
      			//gpr.idx [{2'b11,Rd16,1'b0}+:2] <= R16;  // TODO
      			{gpr.idx [{2'b11,Rd16,1'b1}], gpr.idx [{2'b11,Rd16,1'b0}]} <= R16;
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
if (gpr.wen) begin
  // TODO recode this, so it is appropriate for a register file or at least optimized
  if (gpr.w16) gpr_f.idx [{wad[5-1:1], 1'b0}+:2] <= wdt;
  else         gpr_f.idx [ wad                 ] <= wdt[8-1:0];
end

// read word access
always_comb
if (gpr.rwe) Rw = gpr_f.idx [{rwa[5-1:1], 1'b0}+:2];
else         Rw = 16'hxxxx;
always_comb
if (gpr.rwe) Rd = gpr_f.idx [rwa];
else         Rd = KX;

// read byte access
always_comb
if (gpr.rbe) Rr = gpr_f.idx [rba];
else         Rr = KX;

////////////////////////////////////////////////////////////////////////////////
// status register access
////////////////////////////////////////////////////////////////////////////////

// SREG write access
always_ff @ (posedge clk, posedge rst)
if (rst) sreg <= 8'b00000000;
else     sreg <= (sreg_tmp & sreg_msk) | (sreg & ~sreg_msk);

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
	default:		dmem_a = {BD_AW{1'bx}};
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
	DMEM_SEL_SP_PCH:	dmem_do = exception ? PC[BI_AW-1:8] : pc_inc[BI_AW-1:8];
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
					16'b1100_????_????_????: begin /* RJMP */pc_sel = PC_SEL_KL;	next_state = STALL;end
					16'b1101_????_????_????: begin/* RCALL */dmem_sel = DMEM_SEL_SP_PCL;dmem_we = 1'b1;push = 1'b1;next_state = RCALL;					end
					16'b0001_00??_????_????: begin/* CPSE */pc_sel = PC_SEL_INC;pmem_ce = 1'b1; if(reg_equal) next_state = SKIP; end
					16'b1111_11??_????_0???: begin/* SBRC - SBRS */	pc_sel = PC_SEL_INC;pmem_ce = 1'b1; if(Rd_b == pmem_d[9])next_state = SKIP;					end
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
