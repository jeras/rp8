// verilator includes
#include "Vrp_8bit_verilator.h"
#include "verilated.h"
#include "verilated_vcd_c.h"
// simavr includes
#include "sim_avr.h"
#include "sim_elf.h"
//#include "sim_core.h"

void avr_dump_state(avr_t * avr) {
  printf("->> r[0:31] = {");
  for (int i = 0; i < 32; i++)
    printf("%02x, ", avr->data[i]);
  printf("}\n");
}



int main(int argc, char **argv, char **env) {
  // verilator initialization
  Verilated::commandArgs(argc, argv);
  // init top verilog instance
  Vrp_8bit_verilator* top = new Vrp_8bit_verilator;
  // init trace dump
  Verilated::traceEverOn(true);
  VerilatedVcdC* tfp = new VerilatedVcdC;
  top->trace (tfp, 99);
  tfp->open ("rp_8bit_verilator.vcd");

  // AVR simulator initialization
  avr_t *avr = NULL;
  // load firmware
  elf_firmware_t fw;
  elf_read_firmware ("test_ldi.elf", &fw);
  // initialize structure and load firmware
  avr = avr_make_mcu_by_name ("atmega128");
  //avr = avr_make_mcu_by_name (fw.mmcu);
  avr_init (avr);
  avr_load_firmware (avr, &fw);
  //
  int avr_state = cpu_Running;

  // initialize simulation inputs
  top->clk = 1;
  top->rst = 1;
  // run simulation for 100 clock periods
  for (int unsigned i=0; i<100; i++) {
    top->rst = (i < 2);
    // dump variables into VCD file and toggle clock
    for (int unsigned clk=0; clk<2; clk++) {
      tfp->dump (2*i+clk);
      top->clk = !top->clk;
      top->eval ();
      avr_state = avr_run (avr);
      avr_dump_state (avr);
    }
    if (Verilated::gotFinish())  exit(0);
  }
  tfp->close();

  exit(0);
}
