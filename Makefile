all:
	iverilog -g2012 rtl/rp_8bit.sv tbn/rp_8bit_tb.sv
