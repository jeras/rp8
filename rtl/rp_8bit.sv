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

// program counter
logic [BI_AW-1:0] PC;
logic [BI_AW-1:0] pc_inc;

// Register file
union packed {
  logic [32-1:0] [8-1:0] idx;
  struct packed {
    logic             [16-1:0] z, y, x;
    logic [32-2*3-1:0] [8-1:0] r;
  } nam;
} gpr;

// status register
typedef struct packed {logic i, t, h, s, v, n, z, c;} sreg_t;

sreg_t sreg;      // register
sreg_t sreg_tmp;  // write input
sreg_t sreg_msk;  // write mask

/* Stack */
logic [7:0] io_sp;
logic [BD_AW-1:0] SP;
logic push;
logic pop;

// synchronous reset
logic rsts;
always_ff @(posedge clk, posedge rst)
if (rst) rsts <= 1'b1;
else     rsts <= 1'b0;


always_ff @(posedge clk, posedge rst)
if (rst) begin
  io_sp <= '0;
  SP    <= 16'h10ff;
end else begin
  io_sp <= io_a[0] ? SP[7:0] : SP[15:8];
  if ((io_a == 6'b111101) | (io_a == 6'b111110)) begin
    if (io_we) begin
      if (io_a[0]) SP[ 7:0] <= io_do;
      else         SP[15:8] <= io_do;
    end
  end
  if (push)  SP <= SP - 16'd1;
  if (pop)   SP <= SP + 16'd1;
end

/* I/O mapped registers */

localparam IO_SEL_EXT	= 2'd0;
localparam IO_SEL_STACK	= 2'd1;
localparam IO_SEL_SREG	= 2'd2;

logic [1:0] io_sel;
always_ff @(posedge clk, posedge rst)
if (rst)       io_sel <= IO_SEL_EXT;
else begin
  case(io_a)
    6'b111101,
    6'b111110: io_sel <= IO_SEL_STACK;
    6'b111111: io_sel <= IO_SEL_SREG;
    default:   io_sel <= IO_SEL_EXT;
  endcase
end


/* Register operations */
wire immediate = (pmem_d[14]
	| (pmem_d[15:12] == 4'b0011))		/* CPI */
	& (pmem_d[15:10] != 6'b111111)		/* SBRC - SBRS */
	& (pmem_d[15:10] != 6'b111110);		/* BST - BLD */
reg lpm_en;

wire [4:0] Rd = lpm_en ? 5'd0 : {immediate | pmem_d[8], pmem_d[7:4]};
wire [4:0] Rr = {pmem_d[9], pmem_d[3:0]};
wire [7:0] K = {pmem_d[11:8], pmem_d[3:0]};
wire [2:0] b = pmem_d[2:0];
wire [11:0] Kl = pmem_d[11:0];
wire [6:0] Ks = pmem_d[9:3];
wire [1:0] Rd16 = pmem_d[5:4];
wire [5:0] K16 = {pmem_d[7:6], pmem_d[3:0]};
wire [5:0] q = {pmem_d[13], pmem_d[11:10], pmem_d[2:0]};

logic  [8-1:0] GPR_Rd;
logic  [8-1:0] GPR_Rr;
logic [16-1:0] GPR_Rd16;

assign GPR_Rd = gpr.idx [Rd];
assign GPR_Rr = gpr.idx [Rr];

wire GPR_Rd_b = GPR_Rd[b];

//assign GPR_Rd16 = gpr.idx [{2'b11,Rd16,1'b0}+:2];
assign GPR_Rd16 = {gpr.idx [{2'b11,Rd16,1'b1}], gpr.idx [{2'b11,Rd16,1'b0}]};

/* Memorize values to support 16-bit instructions */
reg regmem_ce;

reg [4:0] Rd_r;
reg [7:0] GPR_Rd_r;
always_ff @(posedge clk) begin
	if(regmem_ce)
		Rd_r <= Rd; /* < control with regmem_ce */
	GPR_Rd_r <= GPR_Rd; /* < always loaded */
end

/* PC */

reg [3:0] pc_sel;

localparam PC_SEL_NOP		= 4'd0;
localparam PC_SEL_INC		= 4'd1;
localparam PC_SEL_KL		= 4'd2;
localparam PC_SEL_KS		= 4'd3;
localparam PC_SEL_DMEML		= 4'd4;
localparam PC_SEL_DMEMH		= 4'd6;
localparam PC_SEL_DEC		= 4'd7;
localparam PC_SEL_Z		= 4'd8;
localparam PC_SEL_EX		= 4'd9;

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

struct packed {
  enum logic [2:0] {
    ALU_ADD = 3'b0x0, // addition
    ALU_SUB = 3'b0x1, // subtraction
    ALU_AND = 3'b100, // logic and
    ALU_OR  = 3'b101, // logic or
    ALU_EOR = 3'b110, // logic eor
    ALU_SHR = 3'b111  // shift right
  } md;             // alu modes
  logic [8-1:0] rd; // destination register value
  logic [8-1:0] rr; // source      register value
  logic         ci; // carry input
} alu;
logic [8-1:0] alu_ro; // result
sreg_t        alu_sr;

always_comb
case (alu.md)
  ALU_ADD: {alu.sr.c, alu_ro} = alu.rd + alu.rr + alu.ci;
  ALU_SUB: {alu.sr.c, alu_ro} = alu.rd - alu.rr - alu.ci;
  ALU_AND: {alu.sr.c, alu_ro} = {1'bx, alu.rd & alu.rr};
  ALU_OR : {alu.sr.c, alu_ro} = {1'bx, alu.rd | alu.rr};
  ALU_EOR: {alu.sr.c, alu_ro} = {1'bx, alu.rd ^ alu.rr};
  ALU_SHR: {alu_ro, alu.sr.c} = {alu.ci, alu.rd};
endcase

assign alu_sr.i = 1'bx;
assign alu_sr.t = 1'bx;
assign alu_sr.h = alu.rd[3] & alu.rr[3] | alu.rr[3] & ~alu_ro[3] | ~alu_ro[3] & alu.rd[3];
assign alu_sr.s = alu_sr.n ^ alu_sr.v;
assign alu_sr.v = alu.rd[7] & alu.rr[7] & ~alu_ro[7] | ~alu.rd[7] & ~alu.rr[7] & alu_ro[7];
assign alu_sr.n = alu_ro[7];
assign alu_sr.z = ~|alu_ro;

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
  ALU_ADD: {add.sr.c, add_ro} = add.rd + add.rr;
  ALU_SUB: {add.sr.c, add_ro} = add.rd - add.rr;
endcase

assign alu_sr.i = 1'bx;
assign alu_sr.t = 1'bx;
assign alu_sr.h = 1'bx;
assign alu_sr.s = alu_sr.n ^ alu_sr.v;
assign alu_sr.v = ~alu.rd[15] & alu.rr[15];
assign alu_sr.n = alu_ro[15];
assign alu_sr.z = ~|alu_ro;

////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////

struct packed {
  sreg_t tmp;
  sreg_t msk;
} srg;

struct packed {
  // write access
  logic wen; // write enable
  logic w16; // 16bit write
  logic wdt; // write data 16bit
  // read access
} gpr_str;

////////////////////////////////////////////////////////////////////////////////
// instruction decoder
////////////////////////////////////////////////////////////////////////////////

always_ff @(posedge clk) begin
	end else begin
		if(I_set)
			sreg.i <= 1'b1;
  casez (pmem_d)
    // no operation, same as default
    16'b0000_0000_0000_0000: begin alu = 'x; add = 'x; srg = '{8'hxx, 8'h00}; gpr_str = '{1'b0, 1'b0, 16'hxx}; end // NOP
    // arithmetic
    //                                    {mode   , rd    , rr    , ci    }                   {tmp   , msk  }             {wen , w16 , wdt        }
    16'b0000_01??_????_????: begin alu = '{ALU_SUB, GPR_Rd, GPR_Rr, sreg.c}; add = 'x; srg = '{alu_sr, 8'h3f}; gpr_str = '{1'b0, 1'b0, {2{alu_ro}}}; end // CPC
    16'b0000_10??_????_????: begin alu = '{ALU_SUB, GPR_Rd, GPR_Rr, sreg.c}; add = 'x; srg = '{alu_sr, 8'h3f}; gpr_str = '{1'b1, 1'b0, {2{alu_ro}}}; end // SBC
    16'b0000_11??_????_????: begin alu = '{ALU_ADD, GPR_Rd, GPR_Rr, 1'b0  }; add = 'x; srg = '{alu_sr, 8'h3f}; gpr_str = '{1'b1, 1'b0, {2{alu_ro}}}; end // ADD
    16'b0001_01??_????_????: begin alu = '{ALU_SUB, GPR_Rd, GPR_Rr, 1'b0  }; add = 'x; srg = '{alu_sr, 8'h3f}; gpr_str = '{1'b0, 1'b0, {2{alu_ro}}}; end // CP
    16'b0001_10??_????_????: begin alu = '{ALU_SUB, GPR_Rd, GPR_Rr, 1'b0  }; add = 'x; srg = '{alu_sr, 8'h3f}; gpr_str = '{1'b1, 1'b0, {2{alu_ro}}}; end // SUB
    16'b0001_11??_????_????: begin alu = '{ALU_ADD, GPR_Rd, GPR_Rr, sreg.c}; add = 'x; srg = '{alu_sr, 8'h3f}; gpr_str = '{1'b1, 1'b0, {2{alu_ro}}}; end // ADC
    16'b0011_????_????_????: begin alu = '{ALU_SUB, GPR_Rd, K     , 1'b0  }; add = 'x; srg = '{alu_sr, 8'h3f}; gpr_str = '{1'b0, 1'b0, {2{alu_ro}}}; end // CPI
    16'b0100_????_????_????: begin alu = '{ALU_SUB, GPR_Rd, K     , sreg.c}; add = 'x; srg = '{alu_sr, 8'h3f}; gpr_str = '{1'b1, 1'b0, {2{alu_ro}}}; end // SBCI
    16'b0101_????_????_????: begin alu = '{ALU_SUB, GPR_Rd, K     , 1'b0  }; add = 'x; srg = '{alu_sr, 8'h3f}; gpr_str = '{1'b1, 1'b0, {2{alu_ro}}}; end // SUBI
    16'b1001_010?_????_0000: begin alu = '{ALU_SUB, 8'hff , GPR_Rd, 1'b0  }; add = 'x; srg = '{alu_sr, 8'h1f}; gpr_str = '{1'b1, 1'b0, {2{alu_ro}}}; end // COM // TODO check the value of carry and overflow
    16'b1001_010?_????_0001: begin alu = '{ALU_SUB, 8'h00 , GPR_Rd, 1'b0  }; add = 'x; srg = '{alu_sr, 8'h3f}; gpr_str = '{1'b1, 1'b0, {2{alu_ro}}}; end // NEG
    16'b1001_010?_????_0011: begin alu = '{ALU_ADD, GPR_Rd, 8'h00 , 1'b1  }; add = 'x; srg = '{alu_sr, 8'h3e}; gpr_str = '{1'b1, 1'b0, {2{alu_ro}}}; end // INC
    16'b1001_010?_????_1010: begin alu = '{ALU_SUB, GPR_Rd, 8'h00 , 1'b1  }; add = 'x; srg = '{alu_sr, 8'h3e}; gpr_str = '{1'b1, 1'b0, {2{alu_ro}}}; end // DEC
    // logic // TODO check flags
    16'b0010_00??_????_????: begin alu = '{ALU_AND, GPR_Rd, GPR_Rr, 1'b0  }; add = 'x; srg = '{alu_sr, 8'h1e}; gpr_str = '{1'b1, 1'b0, {2{alu_ro}}}; end // AND
    16'b0111_????_????_????: begin alu = '{ALU_AND, GPR_Rd, K     , 1'b0  }; add = 'x; srg = '{alu_sr, 8'h1e}; gpr_str = '{1'b1, 1'b0, {2{alu_ro}}}; end // ANDI
    16'b0010_10??_????_????: begin alu = '{ALU_OR , GPR_Rd, GPR_Rr, 1'b0  }; add = 'x; srg = '{alu_sr, 8'h1e}; gpr_str = '{1'b1, 1'b0, {2{alu_ro}}}; end // OR
    16'b0110_????_????_????: begin alu = '{ALU_OR , GPR_Rd, K     , 1'b0  }; add = 'x; srg = '{alu_sr, 8'h1e}; gpr_str = '{1'b1, 1'b0, {2{alu_ro}}}; end // ORI
    16'b0010_01??_????_????: begin alu = '{ALU_XOR, GPR_Rd, GPR_Rr, 1'b0  }; add = 'x; srg = '{alu_sr, 8'h1e}; gpr_str = '{1'b1, 1'b0, {2{alu_ro}}}; end // EOR
    // shift right // TODO check flags
    16'b1001_010?_????_0110: begin alu = '{ALU_SHR, GPR_Rd, 8'h00 , 1'b0  }; add = 'x; srg = '{alu_sr, 8'h1e}; gpr_str = '{1'b1, 1'b0, {2{alu_ro}}}; end // LSR
    16'b1001_010?_????_0111: begin alu = '{ALU_SHR, GPR_Rd, 8'h00 , 1'b1  }; add = 'x; srg = '{alu_sr, 8'h1e}; gpr_str = '{1'b1, 1'b0, {2{alu_ro}}}; end // ROR
    16'b1001_010?_????_0101: begin alu = '{ALU_SHR, GPR_Rd, 8'h00,GPR_Rd[7]};add = 'x; srg = '{alu_sr, 8'h1e}; gpr_str = '{1'b1, 1'b0, {2{alu_ro}}}; end // ASR

    // register moves
    16'b0010_11??_????_????: begin alu = 'x; add = 'x; srg = '{8'hxx, 8'h00};                gpr_str = '{1'b1, 1'b0, {2{GPR_Rr}}}; end // MOV
    16'b1110_????_????_????: begin alu = 'x; add = 'x; srg = '{8'hxx, 8'h00};                gpr_str = '{1'b1, 1'b0, {2{K     }}}; end // LDI
    16'b1001_010?_????_0010: begin alu = 'x; add = 'x; srg = '{8'hxx, 8'h00};                gpr_str = '{1'b1, 1'b0, {2{GPR_Rd[3:0], GPR_Rd[7:4]}}}; end // SWAP
    16'b1001_010?_0???_1000: begin alu = 'x; add = 'x; srg = '{8'hff, 8'h01 << pmem_d[6:4]}; gpr_str = '{1'b1, 1'b0, 16'hxxxx}; end // BSET
    // bit manipulation
    16'b1001_010?_1???_1000: begin alu = 'x; add = 'x; srg = '{8'h00, 8'h01 << pmem_d[6:4]}; gpr_str = '{1'b0, 1'b0, 16'hxxxx}; end // BCLR
    // 16-24 bit adder
    16'b1001_0110_????_????: begin alu = 'x; add = '{ADD_ADD, GPR_Rd16, K16}; srg = '{add_sr, 8'h1f}; gpr_str = '{1'b1, 1'b1, add_ro}; end // ADIW
    16'b1001_0111_????_????: begin alu = 'x; add = '{ADD_SUB, GPR_Rd16, K16}; srg = '{add_sr, 8'h1f}; gpr_str = '{1'b1, 1'b1, add_ro}; end // SBIW

    16'b1111_101?_????_0???: begin alu = 'x; add = 'x; srg = '{{8{GPR_Rd_b}}, 8'h40}; gpr_str = '{1'b0, 1'b0, 16'hxxxx}; end // SBT
    16'b1111_100?_????_0???: begin alu = 'x; add = 'x; srg = '{8'hxx, 8'h00}; gpr_str = '{1'b1, 1'b0, {2{GPR_Rd & ~(8'h01<<b) | {8{sreg.t}} & (8'h01<<b)}}}; end // BLD

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
// register access
////////////////////////////////////////////////////////////////////////////////

// SREG write access
always_ff @ (posedge clk, posedge rst)
if (rst) sreg <= 8'b00000000;
else     sreg <= (sreg_tmp & sreg_msk) | (sreg & ~sreg_msk);

// GPR write access
always_ff @ (posedge clk)
if (gpr_wen) begin
  if (gpr_w16) gpr.idx [Rd+:2] <= R16;
  else         gpr.idx [Rd]    <= R8;
end

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/* I/O port */
assign io_a = {pmem_d[10:9], pmem_d[3:0]};
assign io_do = GPR_Rd;

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
	DMEM_SEL_SP_R:		dmem_do = GPR_Rd;
	DMEM_SEL_SP_PCL:	dmem_do = exception ? PC[    8-1:0] : pc_inc[    8-1:0];
	DMEM_SEL_SP_PCH:	dmem_do = exception ? PC[BI_AW-1:8] : pc_inc[BI_AW-1:8];
	DMEM_SEL_PMEM:		dmem_do = GPR_Rd_r;
	default:		dmem_do = 8'hxx;
endcase

/* Multi-cycle operation sequencer */

wire reg_equal = GPR_Rd == GPR_Rr;

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
					16'b1111_11??_????_0???: begin/* SBRC - SBRS */	pc_sel = PC_SEL_INC;pmem_ce = 1'b1; if(GPR_Rd_b == pmem_d[9])next_state = SKIP;					end
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
