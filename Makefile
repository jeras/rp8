MODELSIM_BIN="$(HOME)/altera/14.0/modelsim_ase/bin"

all: modelsim

iverilog:
	iverilog -g2012 -o rp_8bit.out rtl/rp_8bit.sv tbn/rp_8bit_tb.sv
	vvp rp_8bit.out

verilator:
	verilator

modelsim:
	$(MODELSIM_BIN)/vlib work
	$(MODELSIM_BIN)/vlog rtl/rp_8bit.sv tbn/rp_8bit_tb.sv
	$(MODELSIM_BIN)/vsim -c -do 'run -all;quit' rp_8bit_tb

