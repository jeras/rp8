MODELSIM_BIN=$(HOME)/altera/14.1/modelsim_ase/bin

SRC=isa_multiplication.sv isa_arithmetic.sv isa_logic.sv

all: modelsim

modelsim:
	$(MODELSIM_BIN)/vlib work
	$(MODELSIM_BIN)/vlog $(SRC)
	$(MODELSIM_BIN)/vsim -c -do 'run -all;quit' isa_multiplication

multiplication:
	iverilog -g2012 isa_multiplication.sv -o isa_multiplication.out
	vvp isa_multiplication.out

arithmetic:
	iverilog -g2012 isa_arithmetic.sv -o isa_arithmetic.out
	vvp isa_arithmetic.out

logic:
	iverilog -g2012 isa_logic.sv -o isa_logic.out
	vvp isa_logic.out

