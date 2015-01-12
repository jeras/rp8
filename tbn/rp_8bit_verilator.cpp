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
  uint32_t sp  =  (avr->data[32+0x3d] << 0)
               |  (avr->data[32+0x3e] << 8);
  uint8_t sreg = ((avr->sreg[0] << 0) & 0x01)
               | ((avr->sreg[1] << 1) & 0x01)
               | ((avr->sreg[2] << 2) & 0x01)
               | ((avr->sreg[3] << 3) & 0x01)
               | ((avr->sreg[4] << 4) & 0x01)
               | ((avr->sreg[5] << 5) & 0x01)
               | ((avr->sreg[6] << 6) & 0x01)
               | ((avr->sreg[7] << 7) & 0x01);
  printf("avrsim -> ");
  printf("pc = %06x ", avr->pc/2);
  printf("sp = %06x ", sp);
  printf("sreg = %02x ", sreg);
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
  uint32_t dump_pc,
  uint32_t dump_sp,
  uint8_t  dump_sreg,
  uint8_t  dump_gpr[32],
  uint8_t  dump_io [64]
) {
  printf("avrrtl -> ");
  printf("pc = %06x ", dump_pc);
  printf("sp = %06x ", dump_sp);
  printf("sreg = %02x ", dump_sreg);
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
  for (int unsigned cyc=0; cyc<12; cyc++) {
    // dump variables into VCD file and toggle clock
    for (int unsigned clk=0; clk<2; clk++) {
      tfp->dump (2*cyc+clk);
      top->clk = !top->clk;
      top->eval ();
    }
    // set reset
    top->rst = (cyc<2);
    // check if a new instruction is beeing executed
    uint32_t dump_pmem_ce;
    top->v->     dump_state_pmem_ce (dump_pmem_ce);
    // DUT internal state
    top->v->DUT->dump_state_core (dump_gpr.word, dump_pc, dump_sp, dump_sreg_precast);
    top->v->     dump_state_io   (dump_io.word);
    dump_sreg = dump_sreg_precast;
    // simavr internal state
    uint32_t sp  =  (avr->data[32+0x3d] << 0)
                 |  (avr->data[32+0x3e] << 8);
    uint8_t sreg = ((avr->sreg[0] << 0) & 0x01)
                 | ((avr->sreg[1] << 1) & 0x01)
                 | ((avr->sreg[2] << 2) & 0x01)
                 | ((avr->sreg[3] << 3) & 0x01)
                 | ((avr->sreg[4] << 4) & 0x01)
                 | ((avr->sreg[5] << 5) & 0x01)
                 | ((avr->sreg[6] << 6) & 0x01)
                 | ((avr->sreg[7] << 7) & 0x01);
    // only make compatisons, when RTL requests a new instruction,
    // so the execution of the last one is finished
    if (dump_pmem_ce) {
      // TODO: a faster comparison might be done by casting to 64bit)
      // compare PC
      if (dump_pc != avr->pc/2)
        printf ("ERROR: PC mismatch - rtl: 0x%06x, sim: 0x%06x\n", dump_pc, avr->pc/2);
      // compare SP
      if (dump_sp != sp)
        printf ("ERROR: SP mismatch - rtl: 0x%06x, sim: 0x%06x\n", dump_sp, sp);
      // compare SP
      if (dump_sreg != sreg)
        printf ("ERROR: SREG mismatch - rtl: 0x%02x, sim: 0x%02x\n", dump_sreg, sreg);
      // compare GPR
      for (unsigned int i=0; i<32; i++) {
        if (dump_gpr.byte[i] != avr->data[i])
          printf ("ERROR: GPR[0x%02x] mismatch\n", i);
      }
      // compare IO
      for (unsigned int i=0; i<64; i++) {
        if (dump_io.byte[i] != avr->data[32+i])
          printf ("ERROR: I/O[0x%02x] mismatch\n", i);
      }
      // compare RAM
      // TODO simulator should report the changed address
  //    for (unsigned int i=0; i<32; i++) {
  //      if (dump_gpr.byte[i] != avr->data[i])
  //        printf ("ERROR: GPR[%02d] mismatch", i);
  //    }
  
      // debug print
      print_state_avrsim (avr);
      print_state_avrrtl (dump_pc, dump_sp, dump_sreg, dump_gpr.byte, dump_io.byte);
      printf("\n");
    }

    if (!top->rst && dump_pmem_ce)
    avr_state = avr_run (avr);

    // check for end of simulation
    if (Verilated::gotFinish())  exit(0);
  }
  tfp->close();

  exit(0);
}
