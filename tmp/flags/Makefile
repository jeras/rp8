MODELSIM_BIN=$(HOME)/altera/14.1/modelsim_ase/bin

SRC=multiplication.sv

all: modelsim

iverilog:
	iverilog -g2012 multiplication.sv -o multiplication.out
	vvp multiplication.out

modelsim:
	$(MODELSIM_BIN)/vlib work
	$(MODELSIM_BIN)/vlog multiplication.sv
	$(MODELSIM_BIN)/vsim -c -do 'run -all;quit' multiplication

