MODELSIM_BIN=$(HOME)/altera/14.0/modelsim_ase/bin

RTL=rtl/rp_8bit.sv
TBN=tbn/rp_8bit_disasm.sv tbn/rp_8bit_tb.sv

all: modelsim

iverilog:
	iverilog -g2012 -o $(RTL) $(TBN)
	vvp rp_8bit.out

verilator:
	verilator $(RTL) $(TBN)

modelsim:
	$(MODELSIM_BIN)/vlib work
	$(MODELSIM_BIN)/vlog $(RTL) $(TBN)
	$(MODELSIM_BIN)/vsim -c -do 'run -all;quit' rp_8bit_tb

