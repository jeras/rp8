////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

module rp_8bit_verilator (
  // ststem signals
  input logic clk,
  input logic rst
);

////////////////////////////////////////////////////////////////////////////////
// local signals
////////////////////////////////////////////////////////////////////////////////

localparam IRW =  8; // bus instruction - interrupt width
localparam PAW = 11; // bus instruction - address   width
localparam DAW = 13; // bus data        - address   width

// system signals
logic           clk; // clock
logic           rst; // reset

// program bus
logic           bp_vld; // valid (address, write enable, write data)
logic           bp_wen; // write enable
logic [PAW-1:0] bp_adr; // address
logic  [16-1:0] bp_wdt; // write data
logic  [16-1:0] bp_rdt; // read data
logic [PAW-1:0] bp_npc; // new PC
logic           bp_jmp; // debug jump request
logic           bp_rdy; // ready (read data, new PC, debug jump request)
// data bus
logic           bd_req;
logic           bd_wen;
logic [DAW-1:0] bd_adr;
logic   [8-1:0] bd_wdt;
logic   [8-1:0] bd_rdt;
logic           bd_ack;
// I/O peripheral bus
logic           io_wen; // write enable
logic           io_ren; // read  enable
logic   [6-1:0] io_adr; // address
logic   [8-1:0] io_wdt; // write data
logic   [8-1:0] io_msk; // write mask
logic   [8-1:0] io_rdt; // read data
// interrupt
logic [IRW-1:0] irq_req;
logic [IRW-1:0] irq_ack;

int unsigned cyc = 0;
always_ff @ (posedge clk)
cyc++;

////////////////////////////////////////////////////////////////////////////////
// RTL DUT instance
////////////////////////////////////////////////////////////////////////////////

rp_8bit #(
  .BI_IW (BI_IW),
  .BI_AW (BI_AW),
  .BD_AW (BD_AW)
) DUT (
  // system signals
  .clk     (clk),
  .rst     (rst),
  // instruction bus
  .pmem_ce (pmem_ce),
  .pmem_a  (pmem_a ),
  .pmem_d  (pmem_d ),
  // data bus
  .dmem_we (dmem_we),
  .dmem_a  (dmem_a ),
  .dmem_di (dmem_di),
  .dmem_do (dmem_do),
  // peripheral bus
  .io_re   (io_re  ),
  .io_we   (io_we  ),
  .io_a    (io_a   ),
  .io_do   (io_do  ),
  .io_di   (io_di  ),
  // interrupt
  .irq     (irq    ),
  .irq_ack (irq_ack)
);

////////////////////////////////////////////////////////////////////////////////
// instruction memory
////////////////////////////////////////////////////////////////////////////////

mem #(
  .FN ("test_isa.vmem"),
  .SZ (0:2**PAW-1),
  .DW (16)
) bp_mem (
  .clk (clk),
  .ena (bp_ena),
  .wen (bp_wen),
  .adr (bp_adr),
  .wdt (bp_wdt),
  .rdt (bp_rdt),
);

//string str;
//bit [0:32-1] [8-1:0] asm;
//
//always_comb begin
//  str = rp_8bit_disasm::disasm(pmem_d);
//  asm = '0;
//  for (int i=0; i<str.len(); i++) begin
//    asm [i] = (8)'(str[i]);
//  end
//end

////////////////////////////////////////////////////////////////////////////////
// data memory
////////////////////////////////////////////////////////////////////////////////

mem #(
  .FN ("test_isa.vmem"),
  .SZ (0:2**PAW-1),
  .DW (16)
) bd_mem (
  .clk (clk),
  .ena (bd_ena),
  .wen (bd_wen),
  .adr (bd_adr),
  .wdt (bd_wdt),
  .rdt (bd_rdt),
);

module #(
  // 1kB by default
  string       FN = ""          // initialization file name
  int unsigned SZ = 2**10,      // memory size in words
  int unsigned AW = $clog2(SZ), // address width
  int unsigned DW = 8           // data width
) mem (
  input logic           clk, // clock
  input logic           ena, // write or read enable
  input logic           wen, // write enable
  input logic  [AW-1:0] adr, // address
  input logic  [DW-1:0] wdt, // write data
  output logic [DW-1:0] rdt  // read data
);

////////////////////////////////////////////////////////////////////////////////
// periphery
////////////////////////////////////////////////////////////////////////////////

logic [8-1:0] io_mem [0:64-1];

always @(posedge clk, posedge rst)
if (rst) begin
  for (int unsigned i=0; i<64; i++)
    io_mem[io_adr] <= 8'h00;
end else begin
  if (io_wen) io_mem[io_wdt] <= io_wdt;
  if (io_ren) io_rdt <= io_mem[io_adr];
end

////////////////////////////////////////////////////////////////////////////////
// interrupts
////////////////////////////////////////////////////////////////////////////////

assign irq = '0;

////////////////////////////////////////////////////////////////////////////////
// DPI
////////////////////////////////////////////////////////////////////////////////

function void dump_state_pmem (
  output int dump_pmem_a,
  output int dump_pmem_ce
);
/*verilator public*/
  dump_pmem_a  = pmem_a ;
  dump_pmem_ce = pmem_ce;
endfunction: dump_state_pmem

function void dump_state_io (
  output bit [64-1:0] [8-1:0] dump_iomem
);
  /*verilator public*/
  for (int unsigned i=0; i<64; i++)
    dump_iomem[i] = iomem[i];
endfunction: dump_state_io

endmodule: rp_8bit_verilator
