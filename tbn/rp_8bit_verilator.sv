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

localparam int unsigned IRW =  8; // bus instruction - interrupt width
localparam int unsigned PAW = 11; // bus instruction - address   width
localparam int unsigned DAW = 13; // bus data        - address   width
localparam bit [16-1:0] SPR = 16'h10ff;

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
// control
logic           ctl_slp; // sleep
logic           ctl_brk; // break
logic           ctl_wdr; // watch dog reset

int unsigned cyc = 0;
always_ff @ (posedge clk)
cyc <= cyc+1;

////////////////////////////////////////////////////////////////////////////////
// RTL DUT instance
////////////////////////////////////////////////////////////////////////////////

rp_8bit #(
  .IRW (IRW),
  .PAW (PAW),
  .DAW (DAW),
  .SPR (SPR)
) DUT (
  // system signals
  .clk     (clk),
  .rst     (rst),
  // program bus
  .bp_vld  (bp_vld),
  .bp_wen  (bp_wen),
  .bp_adr  (bp_adr),
  .bp_wdt  (bp_wdt),
  .bp_rdt  (bp_rdt),
  .bp_npc  (bp_npc),
  .bp_jmp  (bp_jmp),
  .bp_rdy  (bp_rdy),
  // data bus
  .bd_req  (bd_req),
  .bd_wen  (bd_wen),
  .bd_adr  (bd_adr),
  .bd_wdt  (bd_wdt),
  .bd_rdt  (bd_rdt),
  .bd_ack  (bd_ack),
  // I/O peripheral bus
  .io_wen  (io_wen),
  .io_ren  (io_ren),
  .io_adr  (io_adr),
  .io_wdt  (io_wdt),
  .io_msk  (io_msk),
  .io_rdt  (io_rdt),
  // interrupts
  .irq_req (irq_req),
  .irq_ack (irq_ack),
  // control
  .ctl_slp (ctl_slp),
  .ctl_brk (ctl_brk),
  .ctl_wdr (ctl_wdr)
);

////////////////////////////////////////////////////////////////////////////////
// instruction memory
////////////////////////////////////////////////////////////////////////////////

mem #(
//  .FN ("test_isa.vmem"),
  .SZ (2**PAW),
  .DW (16)
) bp_mem (
  .clk (clk),
  .ena (bp_vld),
  .wen (bp_wen),
  .adr (bp_adr),
  .wdt (bp_wdt),
  .rdt (bp_rdt)
);

initial
$readmemh("test_isa.vmem", bp_mem.mem);

//string str;
//bit [0:32-1] [8-1:0] asm;
//
//always_comb begin
//  str = rp_8bit_disasm::disasm(bp_rdt);
//  asm = '0;
//  for (int i=0; i<str.len(); i++) begin
//    asm [i] = (8)'(str[i]);
//  end
//end

////////////////////////////////////////////////////////////////////////////////
// data memory
////////////////////////////////////////////////////////////////////////////////

mem #(
  .SZ (2**PAW),
  .DW (16)
) bd_mem (
  .clk (clk),
  .ena (bd_req),
  .wen (bd_wen),
  .adr (bd_adr),
  .wdt (bd_wdt),
  .rdt (bd_rdt)
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
  if (io_wen) io_mem[io_wdt] <= io_wdt & io_msk | io_mem[io_adr] & ~io_msk;
  if (io_ren) io_rdt <= io_mem[io_adr];
end

////////////////////////////////////////////////////////////////////////////////
// interrupts
////////////////////////////////////////////////////////////////////////////////

assign irq_req = '0;

////////////////////////////////////////////////////////////////////////////////
// DPI
////////////////////////////////////////////////////////////////////////////////

function void dump_state_bp (
  output int dump_bp_adr,
  output int dump_bp_vld
);
/*verilator public*/
  dump_bp_adr = bp_adr;
  dump_bp_vld = bp_vld;
endfunction: dump_state_bp

function void dump_state_io (
  output bit [64-1:0] [8-1:0] dump_io_mem
);
  /*verilator public*/
  for (int unsigned i=0; i<64; i++)
    dump_io_mem[i] = io_mem[i];
endfunction: dump_state_io

endmodule: rp_8bit_verilator
