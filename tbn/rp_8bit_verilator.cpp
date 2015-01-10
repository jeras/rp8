// verilator includes
#include "Vrp_8bit_verilator.h"
#include "verilated.h"
#include "verilated_vcd_c.h"
// DPI interface
//#include "svdpi.h"
//#include "Vrp_8bit_verilator__Dpi.h"
// accessing public variables/functions
#include "Vrp_8bit_verilator_rp_8bit_verilator.h"
#include "Vrp_8bit_verilator_rp_8bit.h"
// simavr includes
#include "sim_avr.h"
#include "sim_elf.h"
//#include "sim_core.h"

void print_state_avrsim (avr_t * avr) {
  printf("avrsim -> ");
  printf("r[31:0] = {");
  for (int i=32-1; i>=0; i--)
    printf("%02x", avr->data[i]);
  printf("} ");
  printf("io[63:0] = {");
  for (int i=64-1; i>=0; i--)
    printf("%02x", avr->data[32+i]);
  printf("} ");
  printf("\n");
}

void print_state_avrrtl (
  uint8_t dump_gpr[32],
  uint8_t dump_io [64]
) {
  printf("avrrtl -> ");
  printf("r[31:0] = {");
  for (int i=32-1; i>=0; i--)
    printf("%02x", dump_gpr[i]);
  printf("} ");
  printf("io[63:0] = {");
  for (int i=64-1; i>=0; i--)
    printf("%02x", dump_io[i]);
  printf("} ");
  printf("\n");
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

  // verilator public function access
  union {
    uint32_t word[8];
    uint8_t  byte[32];
  } dump_gpr;
  uint32_t dump_pc;
  uint32_t dump_sp;
  uint32_t dump_sreg_precast;
  uint8_t  dump_sreg;
  union {
    uint32_t word[16];
    uint8_t  byte[64];
  } dump_io;
  //svSetScope (svGetScopeFromName ("DUT"));

  // AVR simulator initialization
  avr_t *avr = NULL;
  // load firmware
  elf_firmware_t fw;
  elf_read_firmware ("test_isa.elf", &fw);
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
  for (int unsigned cyc=0; cyc<40; cyc++) {
    top->rst = (cyc<2);
    // dump variables into VCD file and toggle clock
    for (int unsigned clk=0; clk<2; clk++) {
      tfp->dump (2*cyc+clk);
      top->clk = !top->clk;
      top->eval ();
    }
    if (!top->rst) {
      // DUT internal state
      top->v->DUT->dump_state_core (dump_gpr.word, dump_pc, dump_sp, dump_sreg_precast);
      top->v->     dump_state_io   (dump_io.word);
      // simavr internal state
      avr_state = avr_run (avr);
      // TODO: a faster comparison might be done by casting to 64bit)
      // compare GPR
      for (unsigned int i=0; i<32; i++) {
        if (dump_gpr.byte[i] != avr->data[i])
          printf ("ERROR: GPR[%02d] mismatch\n", i);
      }
      // compare IO
      for (unsigned int i=0; i<64; i++) {
        if (dump_io.byte[i] != avr->data[32+i])
          printf ("ERROR: I/O[%02d] mismatch\n", i);
      }
      // compare RAM
      // TODO simulator should report the changed address
//      for (unsigned int i=0; i<32; i++) {
//        if (dump_gpr.byte[i] != avr->data[i])
//          printf ("ERROR: GPR[%02d] mismatch", i);
//      }

      // debug print
      print_state_avrsim (avr);
      print_state_avrrtl (dump_gpr.byte, dump_io.byte);
      printf("\n");
    }
    // check for end of simulation
    if (Verilated::gotFinish())  exit(0);
  }
  tfp->close();

  exit(0);
}
