MODELSIM_BIN=$(HOME)/altera/14.0/modelsim_ase/bin

RTL=../rtl/rp_8bit.sv ../rtl/rp_8bit_gpio.sv
TBN=../tbn/rp_8bit_disasm.sv ../tbn/rp_8bit_tb.sv

all: modelsim

iverilog:
	iverilog -g2012 -o $(RTL) $(TBN)
	vvp rp_8bit.out

verilator:
	# TODO generalize this
	cp ../sim/test_ldi.elf .
	cp ../sim/test_ldi.vmem .
	# compile SystemVerilog sources
	verilator -Wall --cc --trace --exe ../tbn/rp_8bit_verilator.cpp --top-module rp_8bit_verilator \
	-Wno-fatal \
        -y ../tbn/ \
        -y /home/izi/Workplace/rp_8bit/tbn \
	-CFLAGS -I/usr/local/include/simavr/ \
	-LDFLAGS -lsimavr \
	../tbn/rp_8bit_verilator.sv \
	$(RTL)
	# build C++ project
	make -j -C obj_dir/ -f Vrp_8bit_verilator.mk Vrp_8bit_verilator
	# run executable simulation
	LD_LIBRARY_PATH=/usr/local/lib/ obj_dir/Vrp_8bit_verilator

modelsim:
	$(MODELSIM_BIN)/vlib work
	$(MODELSIM_BIN)/vlog $(RTL) $(TBN)
	$(MODELSIM_BIN)/vsim -c -do 'run -all;quit' rp_8bit_tb

