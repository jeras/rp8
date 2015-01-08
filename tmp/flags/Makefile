MODELSIM_BIN=$(HOME)/altera/14.1/modelsim_ase/bin

SRC=multiplication.sv arithmetic.sv

all: modelsim

modelsim:
	$(MODELSIM_BIN)/vlib work
	$(MODELSIM_BIN)/vlog $(SRC)
	$(MODELSIM_BIN)/vsim -c -do 'run -all;quit' multiplication

multiplication:
	iverilog -g2012 multiplication.sv -o multiplication.out
	vvp multiplication.out

arithmetic:
	iverilog -g2012 arithmetic.sv -o arithmetic.out
	vvp arithmetic.out

