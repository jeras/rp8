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

// Register file
logic [32-1:0] [8-1:0] gpr;
logic  [8-1:0] GPR [0:23];
logic [16-1:0] U;	/* < R24-R25 */
logic [16-1:0] pX;	/* < R26-R27 */
logic [16-1:0] pY;	/* < R28-R29 */
logic [16-1:0] pZ;	/* < R30-R31 */

// special purpose registers
struct packed {logic i, t, h, s, v, n, z, c;} spr_flg;

/* Stack */
logic [7:0] io_sp;
logic [15:0] SP;
logic push;
logic pop;

always @(posedge clk, posedge rst)
if (rst) begin
  io_sp <= 8'd0;
  SP <= 16'd0;
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
always @(posedge clk, posedge rst)
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

assign gpr [ 0] = GPR [ 0];
assign gpr [ 1] = GPR [ 1];
assign gpr [ 2] = GPR [ 2];
assign gpr [ 3] = GPR [ 3];
assign gpr [ 4] = GPR [ 4];
assign gpr [ 5] = GPR [ 5];
assign gpr [ 6] = GPR [ 6];
assign gpr [ 7] = GPR [ 7];
assign gpr [ 8] = GPR [ 8];
assign gpr [ 9] = GPR [ 9];
assign gpr [10] = GPR [10];
assign gpr [11] = GPR [11];
assign gpr [12] = GPR [12];
assign gpr [13] = GPR [13];
assign gpr [14] = GPR [14];
assign gpr [15] = GPR [15];
assign gpr [16] = GPR [16];
assign gpr [17] = GPR [17];
assign gpr [18] = GPR [18];
assign gpr [19] = GPR [19];
assign gpr [20] = GPR [20];
assign gpr [21] = GPR [21];
assign gpr [22] = GPR [22];
assign gpr [23] = GPR [23];
assign gpr [24] =  U[ 7:0];
assign gpr [25] =  U[15:8];
assign gpr [26] = pX[ 7:0];
assign gpr [27] = pX[15:8];
assign gpr [28] = pY[ 7:0];
assign gpr [29] = pY[15:8];
assign gpr [30] = pZ[ 7:0];
assign gpr [31] = pZ[15:8];

logic  [8-1:0] GPR_Rd;
logic  [8-1:0] GPR_Rr;
logic [16-1:0] GPR_Rd16;

assign GPR_Rd = gpr [Rd];
assign GPR_Rr = gpr [Rr];

wire GPR_Rd_b = GPR_Rd[b];

//assign GPR_Rd16 = gpr [{2'b11,Rd16,1'b0}+:2];
assign GPR_Rd16 = {gpr [{2'b11,Rd16,1'b1}], gpr [{2'b11,Rd16,1'b0}]};

/* Memorize values to support 16-bit instructions */
reg regmem_ce;

reg [4:0] Rd_r;
reg [7:0] GPR_Rd_r;
always @(posedge clk) begin
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

always @(*)
casex(irq)
  8'bxxxx_xxx1: next_irq_ack = 8'b0000_0001;
  8'bxxxx_xx10: next_irq_ack = 8'b0000_0010;
  8'bxxxx_x100: next_irq_ack = 8'b0000_0100;
  8'bxxxx_1000: next_irq_ack = 8'b0000_1000;
  8'bxxx1_0000: next_irq_ack = 8'b0001_0000;
  8'bxx10_0000: next_irq_ack = 8'b0010_0000;
  8'bx100_0000: next_irq_ack = 8'b0100_0000;
  8'b1000_0000: next_irq_ack = 8'b1000_0000;
  default:      next_irq_ack = 8'b0000_0000;
endcase

logic irq_ack_en;

always @(posedge clk, posedge rst)
if (rst) irq_ack <= '0;
else     irq_ack <= irq_ack_en ? next_irq_ack : '0;

/* Priority encoder */

logic [$clog2(BI_IW)-1:0] PC_ex;

always @(*)
casex(irq)
  8'bxxxx_xxx1: PC_ex = 4'h0;
  8'bxxxx_xx10: PC_ex = 4'h1;
  8'bxxxx_x100: PC_ex = 4'h2;
  8'bxxxx_1000: PC_ex = 4'h3;
  8'bxxx1_0000: PC_ex = 4'h4;
  8'bxx10_0000: PC_ex = 4'h5;
  8'bx100_0000: PC_ex = 4'h6;
  8'b1000_0000: PC_ex = 4'h7;
  default:      PC_ex = 4'h0;
endcase

/* AVR cores always execute at least one instruction after an IRET.
 * Therefore, the I bit is only valid one clock after it has been set. */

logic I_r;

always @(posedge clk, posedge rst)
if (rst) I_r <= 1'b0;
else     I_r <= spr_flg.i;

wire irq_asserted = |irq;
wire irq_request = spr_flg.i & I_r & irq_asserted;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

always @(posedge clk, posedge rst)
if (rst)        PC <= 0;
else case (pc_sel)
  PC_SEL_NOP  :;
  PC_SEL_INC  : PC <= PC + 1;
  // !!! WARNING !!! replace with PC <= PC + {{BI_AW-12{Kl[11]}}, Kl}; if BI_AW>12
  PC_SEL_KL   : PC <= PC + Kl;
  PC_SEL_KS   : PC <= PC + {{BI_AW-7{Ks[6]}}, Ks};
  PC_SEL_DMEML: PC[7:0] <= dmem_di;
  PC_SEL_DMEMH: PC[BI_AW-1:8] <= dmem_di;
  PC_SEL_DEC  : PC <= PC - 1;
  PC_SEL_Z    : PC <= pZ - 1;
  PC_SEL_EX   : PC <= PC_ex;
endcase

reg pmem_selz;
assign pmem_a = rst ? 0 : (pmem_selz ? pZ[15:1] : PC + 1);

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

`define INIT_REGS
`ifdef INIT_REGS
integer i_rst_regf;
`endif
reg [7:0] R;
reg writeback;
reg update_svnz;
reg change_z;
reg [15:0] R16;
reg mode16;
reg _N;
reg _V;
reg _C;
reg I_clr;
reg I_set;
always @(posedge clk) begin
	R = 8'hxx;
	writeback = 1'b0;
	update_svnz = 1'b0;
	change_z = 1'b1;
	R16 = 16'hxxxx;
	mode16 = 1'b0;
	if(rst) begin
`ifndef REGRESS
		/*
		 * Not resetting the register file enables the use of more efficient
		 * distributed block RAM.
		 */
`ifdef INIT_REGS
		for(i_rst_regf=0;i_rst_regf<24;i_rst_regf=i_rst_regf+1)
			GPR[i_rst_regf] <= 8'd0;
		U <= 16'd0;
		pX <= 16'd0;
		pY <= 16'd0;
		pZ <= 16'd0;
`endif
		spr_flg <= 8'h00;
		_V = 1'b0;
		_N = 1'b0;
		_C = 1'b0;
`endif
	end else begin
		if(I_set)
			spr_flg.i <= 1'b1;
		if(normal_en) begin
			writeback = 1'b1;
			update_svnz = 1'b1;
			casex(pmem_d)
				16'b000x_11xx_xxxx_xxxx: begin
					/* ADD - ADC */
					{_C, R} = GPR_Rd + GPR_Rr + (pmem_d[12] & spr_flg.c);
					spr_flg.c <= _C;
					spr_flg.h <= (GPR_Rd[3] & GPR_Rr[3])|(GPR_Rr[3] & ~R[3])|(~R[3] & GPR_Rd[3]);
					_V = (GPR_Rd[7] & GPR_Rr[7] & ~R[7])|(~GPR_Rd[7] & ~GPR_Rr[7] & R[7]);
				end
				16'b000x_10xx_xxxx_xxxx, /* subtract */
				16'b000x_01xx_xxxx_xxxx: /* compare  */ begin
					/* SUB - SBC / CP - CPC */
					{_C, R} = GPR_Rd - GPR_Rr - (~pmem_d[12] & spr_flg.c);
					spr_flg.c <= _C;
					spr_flg.h <= (~GPR_Rd[3] & GPR_Rr[3])|(GPR_Rr[3] & R[3])|(R[3] & ~GPR_Rd[3]);
					_V = (GPR_Rd[7] & ~GPR_Rr[7] & ~R[7])|(~GPR_Rd[7] & GPR_Rr[7] & R[7]);
					if(~pmem_d[12])
						change_z = 1'b0;
					writeback = pmem_d[11];
				end
				16'b010x_xxxx_xxxx_xxxx, /* subtract */
				16'b0011_xxxx_xxxx_xxxx: /* compare  */ begin
					/* SUBI - SBCI / CPI */
					{_C, R} = GPR_Rd - K - (~pmem_d[12] & spr_flg.c);
					spr_flg.c <= _C;
					spr_flg.h <= (~GPR_Rd[3] & K[3])|(K[3] & R[3])|(R[3] & ~GPR_Rd[3]);
					_V = (GPR_Rd[7] & ~K[7] & ~R[7])|(~GPR_Rd[7] & K[7] & R[7]);
					if(~pmem_d[12])
						change_z = 1'b0;
					writeback = pmem_d[14];
				end
				16'b0010_00xx_xxxx_xxxx: begin
					/* AND */
					R = GPR_Rd & GPR_Rr;
					_V = 1'b0;
				end
				16'b0111_xxxx_xxxx_xxxx: begin
					/* ANDI */
					R = GPR_Rd & K;
					_V = 1'b0;
				end
				16'b0010_10xx_xxxx_xxxx: begin
					/* OR */
					R = GPR_Rd | GPR_Rr;
					_V = 1'b0;
				end
				16'b0110_xxxx_xxxx_xxxx: begin
					/* ORI */
					R = GPR_Rd | K;
					_V = 1'b0;
				end
				16'b0010_01xx_xxxx_xxxx: begin
					/* EOR */
					R = GPR_Rd ^ GPR_Rr;
					_V = 1'b0;
				end
				16'b1001_010x_xxxx_0000: begin
					/* COM */
					R = ~GPR_Rd;
					_V = 1'b0;
					spr_flg.c <= 1'b1;
				end
				16'b1001_010x_xxxx_0001: begin
					/* NEG */
					{_C, R} = 8'h00 - GPR_Rd;
					spr_flg.c <= _C;
					spr_flg.h <= R[3] | GPR_Rd[3];
					_V = R == 8'h80;
				end
				16'b1001_010x_xxxx_0011: begin
					/* INC */
					R = GPR_Rd + 8'd1;
					_V = R == 8'h80;
				end
				16'b1001_010x_xxxx_1010: begin
					/* DEC */
					R = GPR_Rd - 8'd1;
					_V = R == 8'h7f;
				end
				16'b1001_010x_xxxx_011x: begin
					/* LSR - ROR */
					R = {pmem_d[0] & spr_flg.c, GPR_Rd[7:1]};
					spr_flg.c <= GPR_Rd[0];
					_V = R[7] ^ GPR_Rd[0];
				end
				16'b1001_010x_xxxx_0101: begin
					/* ASR */
					R = {GPR_Rd[7], GPR_Rd[7:1]};
					spr_flg.c <= GPR_Rd[0];
					_V = R[7] ^ GPR_Rd[0];
				end
				16'b1001_010x_xxxx_0010: begin
					/* SWAP */
					R = {GPR_Rd[3:0], GPR_Rd[7:4]};
					update_svnz = 1'b0;
				end
				16'b1001_010x_xxxx_1000: begin
					/* BSET - BCLR */
					spr_flg [pmem_d[6:4]] <= ~pmem_d[7];
					update_svnz = 1'b0;
					writeback = 1'b0;
				end
				16'b1001_011x_xxxx_xxxx: begin
					mode16 = 1'b1;
					if(pmem_d[8]) begin
						/* SBIW */
						{_C, R16} = GPR_Rd16 - K16;
						spr_flg.c <= _C;
						_V = GPR_Rd16[15] & ~R16[15];
					end else begin
						/* ADIW */
						{_C, R16} = GPR_Rd16 + K16;
						spr_flg.c <= _C;
						_V = ~GPR_Rd16[15] & R16[15];
					end
				end
				/* SBR and CBR are replaced with ORI and ANDI */
				/* TST is replaced with AND */
				/* CLR and SER are replaced with EOR and LDI */
				16'b0010_11xx_xxxx_xxxx: begin
					/* MOV */
					R = GPR_Rr;
					update_svnz = 1'b0;
				end
				16'b1110_xxxx_xxxx_xxxx: begin
					/* LDI */
					R = K;
					update_svnz = 1'b0;
				end
				/* LSL is replaced with ADD */
				/* ROL is replaced with ADC */
				16'b1111_10xx_xxxx_0xxx: begin
					if(pmem_d[9]) begin
						/* BST */
						spr_flg.t <= GPR_Rd_b;
						writeback = 1'b0;
					end else begin
						/* BLD */
						case(b)
							3'd0: R = {GPR_Rd[7:1], spr_flg.t             };
							3'd1: R = {GPR_Rd[7:2], spr_flg.t, GPR_Rd[  0]};
							3'd2: R = {GPR_Rd[7:3], spr_flg.t, GPR_Rd[1:0]};
							3'd3: R = {GPR_Rd[7:4], spr_flg.t, GPR_Rd[2:0]};
							3'd4: R = {GPR_Rd[7:5], spr_flg.t, GPR_Rd[3:0]};
							3'd5: R = {GPR_Rd[7:6], spr_flg.t, GPR_Rd[4:0]};
							3'd6: R = {GPR_Rd[7  ], spr_flg.t, GPR_Rd[5:0]};
							3'd7: R = {             spr_flg.t, GPR_Rd[6:0]};
						endcase
					end
					update_svnz = 1'b0;
				end
				/* SEC, CLC, SEN, CLN, SEZ, CLZ, SEI, CLI, SES, CLS, SEV, CLV, SET, CLT, SEH, CLH
				 * are replaced with BSET and BCLR */
				16'b0000_0000_0000_0000: begin
					/* NOP */
					update_svnz = 1'b0;
					writeback = 1'b0;
				end
				/* SLEEP is not implemented */
				/* WDR is not implemented */
				16'b1001_00xx_xxxx_1111, /* PUSH/POP */
				16'b1001_00xx_xxxx_1100, /*  X   */
				16'b1001_00xx_xxxx_1101, /*  X+  */
				16'b1001_00xx_xxxx_1110, /* -X   */
				16'b1001_00xx_xxxx_1001, /*  Y+  */
				16'b1001_00xx_xxxx_1010, /* -Y   */
				16'b10x0_xxxx_xxxx_1xxx, /*  Y+q */
				16'b1001_00xx_xxxx_0001, /*  Z+  */
				16'b1001_00xx_xxxx_0010, /* -Z   */
				16'b10x0_xxxx_xxxx_0xxx: /*  Z+q */
				begin
					/* LD - POP (run from state WRITEBACK) */
					R = dmem_di;
					update_svnz = 1'b0;
				end
				16'b1011_0xxx_xxxx_xxxx: begin
					/* IN (run from state WRITEBACK) */
					case(io_sel)
						IO_SEL_EXT:   R = io_di;
						IO_SEL_STACK: R = io_sp;
						IO_SEL_SREG:  R = spr_flg;
						default: R = 8'hxx;
					endcase
					update_svnz = 1'b0;
				end
			endcase
		end /* if(normal_en) */
		if(lds_writeback) begin
			R = dmem_di;
			writeback = 1'b1;
		end
		if(lpm_en) begin
			R = pZ[0] ? pmem_d[15:8] : pmem_d[7:0];
			writeback = 1'b1;
		end
		if(update_svnz) begin
			_N = mode16 ? R16[15] : R[7];
			spr_flg.s <= _N ^ _V;
			spr_flg.v <= _V;
			spr_flg.n <= _N;
			spr_flg.z <= mode16 ? R16 == 16'h0000 : ((R == 8'h00) & (change_z|spr_flg.z));
		end
		if(io_we & (io_a == 6'b111111))
			spr_flg <= io_do[7:0];
		if(I_clr)
			spr_flg.i <= 1'b0;
		if(writeback) begin
			if(mode16) begin
				// $display("REG WRITE(16): %d < %d", Rd16, R16);
				case(Rd16)
					2'd0: U <= R16;
					2'd1: pX <= R16;
					2'd2: pY <= R16;
					2'd3: pZ <= R16;
				endcase
			end else begin
				// $display("REG WRITE: %d < %d", Rd, R);
				case(write_dest)
					default: GPR[write_dest] <= R;
					5'd24: U[7:0] <= R;
					5'd25: U[15:8] <= R;
					5'd26: pX[7:0] <= R;
					5'd27: pX[15:8] <= R;
					5'd28: pY[7:0] <= R;
					5'd29: pY[15:8] <= R;
					5'd30: pZ[7:0] <= R;
					5'd31: pZ[15:8] <= R;
				endcase
			end
		end else begin /* if(writeback) */
			case(dmem_sel)
				DMEM_SEL_XPLUS:		pX <= pX + 16'd1;
				DMEM_SEL_XMINUS:	pX <= pX - 16'd1;
				DMEM_SEL_YPLUS:		pY <= pY + 16'd1;
				DMEM_SEL_YMINUS:	pY <= pY - 16'd1;
				DMEM_SEL_ZPLUS:		pZ <= pZ + 16'd1;
				DMEM_SEL_ZMINUS:	pZ <= pZ - 16'd1;
				default:;
			endcase
		end
	end /* if(rst) ... else */
end

/* I/O port */
assign io_a = {pmem_d[10:9], pmem_d[3:0]};
assign io_do = GPR_Rd;

/* Data memory */
always @(*) begin
	case(dmem_sel)
		DMEM_SEL_X,
		DMEM_SEL_XPLUS:		dmem_a = pX;
		DMEM_SEL_XMINUS:	dmem_a = pX - 16'd1;
		DMEM_SEL_YPLUS:		dmem_a = pY;
		DMEM_SEL_YMINUS:	dmem_a = pY - 16'd1;
		DMEM_SEL_YQ:		dmem_a = pY + q;
		DMEM_SEL_ZPLUS:		dmem_a = pZ;
		DMEM_SEL_ZMINUS:	dmem_a = pZ - 16'd1;
		DMEM_SEL_ZQ:		dmem_a = pZ + q;
		DMEM_SEL_SP_R,
		DMEM_SEL_SP_PCL,
		DMEM_SEL_SP_PCH:	dmem_a = SP + pop;
		DMEM_SEL_PMEM:		dmem_a = pmem_d;
		default:		dmem_a = {BD_AW{1'bx}};
	endcase
end

wire [BI_AW-1:0] PC_inc = PC + 1;
reg exception;
always @(*) begin
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
		DMEM_SEL_SP_PCL:	dmem_do = exception ? PC[7:0] : PC_inc[7:0];
		DMEM_SEL_SP_PCH:	dmem_do = exception ? PC[BI_AW-1:8] : PC_inc[BI_AW-1:8];
		DMEM_SEL_PMEM:		dmem_do = GPR_Rd_r;
		default:		dmem_do = 8'hxx;
	endcase
end

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

always @(posedge clk, posedge rst)
if (rst) state <= NORMAL;
else     state <= next_state;

always @(*) begin
	next_state = state;

	pmem_ce = rst;

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
				casex(pmem_d)
					16'b1100_xxxx_xxxx_xxxx: begin
						/* RJMP */
						pc_sel = PC_SEL_KL;
						next_state = STALL;
					end
					16'b1101_xxxx_xxxx_xxxx: begin
						/* RCALL */
						dmem_sel = DMEM_SEL_SP_PCL;
						dmem_we = 1'b1;
						push = 1'b1;
						next_state = RCALL;
					end
					16'b0001_00xx_xxxx_xxxx: begin
						/* CPSE */
						pc_sel = PC_SEL_INC;
						pmem_ce = 1'b1;
						if(reg_equal)
							next_state = SKIP;
					end
					16'b1111_11xx_xxxx_0xxx: begin
						/* SBRC - SBRS */
						pc_sel = PC_SEL_INC;
						pmem_ce = 1'b1;
						if(GPR_Rd_b == pmem_d[9])
							next_state = SKIP;
					end
					/* SBIC, SBIS, SBI, CBI are not implemented */
					16'b1111_0xxx_xxxx_xxxx: begin
						/* BRBS - BRBC */
						pmem_ce = 1'b1;
						if (spr_flg[b] ^ pmem_d[10]) begin
							pc_sel = PC_SEL_KS;
							next_state = STALL;
						end else
							pc_sel = PC_SEL_INC;
					end
					/* BREQ, BRNE, BRCS, BRCC, BRSH, BRLO, BRMI, BRPL, BRGE, BRLT,
					 * BRHS, BRHC, BRTS, BRTC, BRVS, BRVC, BRIE, BRID are replaced
					 * with BRBS/BRBC */
					16'b1001_00xx_xxxx_1100, /*  X   */
					16'b1001_00xx_xxxx_1101, /*  X+  */
					16'b1001_00xx_xxxx_1110, /* -X   */
					16'b1001_00xx_xxxx_1001, /*  Y+  */
					16'b1001_00xx_xxxx_1010, /* -Y   */
					16'b10x0_xxxx_xxxx_1xxx, /*  Y+q */
					16'b1001_00xx_xxxx_0001, /*  Z+  */
					16'b1001_00xx_xxxx_0010, /* -Z   */
					16'b10x0_xxxx_xxxx_0xxx: /*  Z+q */
					begin
						casex({pmem_d[12], pmem_d[3:0]})
							5'b1_1100: dmem_sel = DMEM_SEL_X;
							5'b1_1101: dmem_sel = DMEM_SEL_XPLUS;
							5'b1_1110: dmem_sel = DMEM_SEL_XMINUS;
							5'b1_1001: dmem_sel = DMEM_SEL_YPLUS;
							5'b1_1010: dmem_sel = DMEM_SEL_YMINUS;
							5'b0_1xxx: dmem_sel = DMEM_SEL_YQ;
							5'b1_0001: dmem_sel = DMEM_SEL_ZPLUS;
							5'b1_0010: dmem_sel = DMEM_SEL_ZMINUS;
							5'b0_0xxx: dmem_sel = DMEM_SEL_ZQ;
						endcase
						if(pmem_d[9]) begin
							/* ST */
							pc_sel = PC_SEL_INC;
							pmem_ce = 1'b1;
							dmem_we = 1'b1;
						end else begin
							/* LD */
							next_state = WRITEBACK;
						end
					end
					16'b1011_0xxx_xxxx_xxxx: begin
						/* IN */
						io_re = 1'b1;
						next_state = WRITEBACK;
					end
					16'b1011_1xxx_xxxx_xxxx: begin
						/* OUT */
						io_we = 1'b1;
						pc_sel = PC_SEL_INC;
						pmem_ce = 1'b1;
					end
					16'b1001_00xx_xxxx_1111: begin
						if(pmem_d[9]) begin
							/* PUSH */
							push = 1'b1;
							dmem_sel = DMEM_SEL_SP_R;
							dmem_we = 1'b1;
							pc_sel = PC_SEL_INC;
							pmem_ce = 1'b1;
						end else begin
							/* POP */
							pop = 1'b1;
							dmem_sel = DMEM_SEL_SP_R;
							next_state = WRITEBACK;
						end
					end
					16'b1001_00xx_xxxx_0000: begin
						pc_sel = PC_SEL_INC;
						pmem_ce = 1'b1;
						if(pmem_d[9])
							/* STS */
							next_state = STS;
						else
							/* LDS */
							next_state = LDS1;
					end
					16'b1001_0101_000x_1000: begin
						/* RET / RETI */
						dmem_sel = DMEM_SEL_SP_PCH;
						pop = 1'b1;
						if(pmem_d[4] == 1'b0)
							next_state = RET1;
						else
							next_state = RETI1;
					end
					16'b1001_0101_1100_1000: begin
						/* LPM */
						pmem_selz = 1'b1;
						pmem_ce = 1'b1;
						next_state = LPM;
					end
					16'b1001_0100_0000_1001: begin
						/* IJMP */
						pc_sel = PC_SEL_Z;
						next_state = STALL;
					end
					16'b1001_0101_0000_1001: begin
						/* ICALL */
						dmem_sel = DMEM_SEL_SP_PCL;
						dmem_we = 1'b1;
						push = 1'b1;
						next_state = ICALL;
					end
					default: begin
						pc_sel = PC_SEL_INC;
						normal_en = 1'b1;
						pmem_ce = 1'b1;
					end
				endcase
			end
		end
		RCALL: begin
			dmem_sel = DMEM_SEL_SP_PCH;
			dmem_we = 1'b1;
			push = 1'b1;
			pc_sel = PC_SEL_KL;
			next_state = STALL;
		end
		EXCEPTION: begin
			dmem_sel = DMEM_SEL_SP_PCH;
			dmem_we = 1'b1;
			exception = 1'b1;
			push = 1'b1;
			pc_sel = PC_SEL_EX;
			next_state = STALL;
		end
		ICALL: begin
			dmem_sel = DMEM_SEL_SP_PCH;
			dmem_we = 1'b1;
			push = 1'b1;
			pc_sel = PC_SEL_Z;
			next_state = STALL;
		end
		RET1: begin
			pc_sel = PC_SEL_DMEMH;
			dmem_sel = DMEM_SEL_SP_PCL;
			pop = 1'b1;
			next_state = RET2;
		end
		RET2: begin
			pc_sel = PC_SEL_DMEML;
			next_state = RET3;
		end
		RET3: begin
			pc_sel = PC_SEL_DEC;
			next_state = STALL;
		end
		RETI1: begin
			pc_sel = PC_SEL_DMEMH;
			dmem_sel = DMEM_SEL_SP_PCL;
			pop = 1'b1;
			next_state = RETI2;
		end
		RETI2: begin
			pc_sel = PC_SEL_DMEML;
			next_state = RETI3;
		end
		RETI3: begin
			pc_sel = PC_SEL_DEC;
			next_state = RETI4;
		end
		RETI4: begin
			pc_sel = PC_SEL_INC;
			pmem_ce = 1'b1;
			I_set = 1'b1;
			next_state = NORMAL;
		end
		LPM: begin
			lpm_en = 1'b1;
			pc_sel = PC_SEL_INC;
			pmem_ce = 1'b1;
			next_state = NORMAL;
		end
		STS: begin
			pc_sel = PC_SEL_INC;
			pmem_ce = 1'b1;
			dmem_sel = DMEM_SEL_PMEM;
			dmem_we = 1'b1;
			next_state = NORMAL;
		end
		LDS1: begin
			dmem_sel = DMEM_SEL_PMEM;
			regmem_ce = 1'b0;
			next_state = LDS2;
		end
		LDS2: begin
			pc_sel = PC_SEL_INC;
			pmem_ce = 1'b1;
			lds_writeback = 1'b1;
			next_state = NORMAL;
		end
		SKIP: begin
			pc_sel = PC_SEL_INC;
			pmem_ce = 1'b1;
			/* test for STS and LDS */
			if((pmem_d[15:10] == 6'b100100) & (pmem_d[3:0] == 4'h0))
				next_state = STALL; /* 2-word instruction, skip the second word as well */
			else
				next_state = NORMAL; /* 1-word instruction */
		end
		STALL: begin
			pc_sel = PC_SEL_INC;
			pmem_ce = 1'b1;
			next_state = NORMAL;
		end
		WRITEBACK: begin
			pmem_ce = 1'b1;
			pc_sel = PC_SEL_INC;
			normal_en = 1'b1;
			next_state = NORMAL;
		end
	endcase
end

`ifdef REGRESS
integer i;
integer cycles;
always @(posedge clk) begin
	if(~rst & (state == NORMAL) & (cycles != 0)) begin
		$display("DUMP REGISTERS");
		for(i=0;i<24;i=i+1)
			$display("%x", GPR[i]);
		$display("%x", U[7:0]);
		$display("%x", U[15:8]);
		$display("%x", pX[7:0]);
		$display("%x", pX[15:8]);
		$display("%x", pY[7:0]);
		$display("%x", pY[15:8]);
		$display("%x", pZ[7:0]);
		$display("%x", pZ[15:8]);
		$display("%x", spr_flg);
		$display("%x", SP[15:8]);
		$display("%x", SP[7:0]);
		$display("%x", PC[BI_AW-1:7]);
		$display("%x", {PC[6:0], 1'b0});
		tb_regress.dump;
		$finish;
	end
	if(rst)
		cycles = 0;
	else
		cycles = cycles + 1;
end

reg [7:0] SPR[0:12];
initial begin
	$readmemh("gpr.rom", GPR);
	$readmemh("spr.rom", SPR);
	U = {SPR[1], SPR[0]};
	pX = {SPR[3], SPR[2]};
	pY = {SPR[5], SPR[4]};
	pZ = {SPR[7], SPR[6]};
	spr_flg = SPR[8];
	SP = {SPR[9], SPR[10]};
	PC = {SPR[11], SPR[12]}/2;
end
`endif

endmodule: rp_8bit