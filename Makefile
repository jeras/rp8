all:
	iverilog -g2012 -o rp_8bit.out rtl/rp_8bit.sv tbn/rp_8bit_tb.sv
	vvp rp_8bit.out
