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
//// simavr includes
//#include "sim_avr.h"
//#include "sim_elf.h"
////#include "sim_core.h"

// verilator public function access
typedef struct {
  union {
    uint32_t word[8];
    uint8_t  byte[32];
  } gpr;
  uint32_t pc;
  uint32_t sp;
  uint32_t sreg_precast;
  uint8_t  sreg;
  union {
    uint32_t word[16];
    uint8_t  byte[64];
  } io;
} dump_t;

//void print_state_avrsim (avr_t * avr) {
//  uint32_t sp  =  (avr->data[32+0x3d] << 0)
//               |  (avr->data[32+0x3e] << 8);
//  uint8_t sreg = ((avr->sreg[0] & 0x01) << 0)
//               | ((avr->sreg[1] & 0x01) << 1)
//               | ((avr->sreg[2] & 0x01) << 2)
//               | ((avr->sreg[3] & 0x01) << 3)
//               | ((avr->sreg[4] & 0x01) << 4)
//               | ((avr->sreg[5] & 0x01) << 5)
//               | ((avr->sreg[6] & 0x01) << 6)
//               | ((avr->sreg[7] & 0x01) << 7);
//  printf("avrsim -> ");
//  printf("pc = %06x ", avr->pc/2);
//  printf("sp = %06x ", sp);
//  printf("sreg = %02x ", sreg);
//  printf("r[31:0] = {");
//  for (int i=32-1; i>=0; i--)
//    printf("%02x", avr->data[i]);
//  printf("} ");
//  printf("io[63:0] = {");
//  for (int i=64-1; i>=0; i--)
//    printf("%02x", avr->data[32+i]);
//  printf("} ");
//  printf("\n");
//}

void print_state_avrrtl (
  dump_t *dump
) {
  printf("avrrtl -> ");
  printf("pc = %06x ", dump->pc);
  printf("sp = %06x ", dump->sp);
  printf("sreg = %02x ", dump->sreg);
  printf("r[31:0] = {");
  for (int i=32-1; i>=0; i--)
    printf("%02x", dump->gpr.byte[i]);
  printf("} ");
  printf("io[63:0] = {");
  for (int i=64-1; i>=0; i--)
    printf("%02x", dump->io.byte[i]);
  printf("} ");
  printf("\n");
}

//int compare_state (
//  unsigned int cyc,
//  avr_t *avr,
//  dump_t *dump,
//  uint32_t prev_pc
//) {
//  unsigned int error = 0;
//  // simavr internal state
//  uint32_t sp  =  (avr->data[32+0x3d] << 0)
//               |  (avr->data[32+0x3e] << 8);
//  uint8_t sreg = ((avr->sreg[0] & 0x01) << 0)
//               | ((avr->sreg[1] & 0x01) << 1)
//               | ((avr->sreg[2] & 0x01) << 2)
//               | ((avr->sreg[3] & 0x01) << 3)
//               | ((avr->sreg[4] & 0x01) << 4)
//               | ((avr->sreg[5] & 0x01) << 5)
//               | ((avr->sreg[6] & 0x01) << 6)
//               | ((avr->sreg[7] & 0x01) << 7);
//  // TODO: a faster comparison might be done by casting to 64bit)
//  // compare PC
//  if (prev_pc != avr->pc/2) {
//    printf ("ERROR: PC mismatch - rtl: 0x%06x, sim: 0x%06x\n", prev_pc, avr->pc/2);
//    error++;
//  }
//  // compare SP
//  if (dump->sp != sp) {
//    printf ("ERROR: SP mismatch - rtl: 0x%06x, sim: 0x%06x\n", dump->sp, sp);
//    error++;
//  }
//  // compare SP
//  if (dump->sreg != sreg) {
//    printf ("ERROR: SREG mismatch - rtl: 0x%02x, sim: 0x%02x\n", dump->sreg, sreg);
//    error++;
//  }
//  // compare GPR
//  for (unsigned int i=0; i<32; i++) {
//    if (dump->gpr.byte[i] != avr->data[i]) {
//      printf ("ERROR: GPR[0x%02x] mismatch\n", i);
//      error++;
//    }
//  }
//  // compare IO
////  for (unsigned int i=0; i<64; i++) {
////    if (dump->io.byte[i] != avr->data[32+i]) {
////      printf ("ERROR: I/O[0x%02x] mismatch\n", i);
////      error++;
////    }
////  }
//  // compare RAM
//  // TODO simulator should report the changed address
////  for (unsigned int i=0; i<32; i++) {
////    if (dump->gpr.byte[i] != avr->data[i]) {
////      printf ("ERROR: GPR[%02d] mismatch", i);
////      error++;
////    }
////  }
//
//  // debug print
//  if (error) {
//    printf ("cyc = %d\n", cyc);
//    print_state_avrsim (avr);
//    print_state_avrrtl (dump);
//  }
//  return (error);
//}

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

  dump_t dump;

  // variables used to handle RTL pipeline (synchronous instruction memory)
  uint32_t prev_pc = 0;
  uint32_t prev_ce = 0;

//  // AVR simulator initialization
//  avr_t *avr = NULL;
//  // load firmware
//  elf_firmware_t fw;
//  elf_read_firmware ("test_isa.elf", &fw);
//  // initialize structure and load firmware
//  avr = avr_make_mcu_by_name ("atmega128");
//  //avr = avr_make_mcu_by_name (fw.mmcu);
//  avr_init (avr);
//  avr_load_firmware (avr, &fw);
//  //
//  int avr_state = cpu_Running;

  // initialize simulation inputs
  top->clk = 1;
  top->rst = 1;
  top->eval ();
  // check reset state
  // compare_state (avr, &dump);
  printf("RESET\n");
  // run simulation for 100 clock periods
  for (int unsigned cyc=0; cyc<10; cyc++) {
    if (cyc>=2) {
      // set reset
      top->rst = 0;
      // check if a new instruction is beeing executed
      uint32_t dump_bp_adr;
      uint32_t dump_bp_vld;
      top->v->     dump_state_bp  (dump_bp_adr, dump_bp_vld);
      // DUT internal state
      top->v->DUT->dump_state_core (dump.gpr.word, dump.pc, dump.sp, dump.sreg_precast);
      top->v->     dump_state_io   (dump.io.word);
      dump.sreg = dump.sreg_precast;
      
      // only make compatisons, when RTL requests a new instruction,
      // so the execution of the last one is finished
//      if (prev_ce) {
//        compare_state (cyc, avr, &dump, prev_pc);
//        // simavr should process another instruction
//        avr_state = avr_run (avr);
//      }
//      prev_pc = dump_bp_adr ;
//      prev_ce = dump_bp_vld;
    }

    // dump variables into VCD file and toggle clock
    tfp->dump (2*cyc+0);
    top->clk = 0;
    top->eval ();
    tfp->dump (2*cyc+1);
    top->clk = 1;
    top->eval ();

    // check for end of simulation
    if (Verilated::gotFinish())  exit(0);
  }
  tfp->close();

  exit(0);
}
