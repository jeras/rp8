////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

module rp8_tb;

////////////////////////////////////////////////////////////////////////////////
// local signals
////////////////////////////////////////////////////////////////////////////////

localparam int unsigned IRW =  8; // bus instruction - interrupt width
localparam int unsigned PAW = 11; // bus instruction - address   width
localparam int unsigned DAW = 13; // bus data        - address   width
localparam bit [16-1:0] SPR = 16'h10ff;

// test class
class test_class;
  rand bit [16-1:0] code;

  constraint valid {
    code inside {
      16'b0000_0000_0000_0000,  // NOP
      16'b0000_0001_????_????,  // MOVW
      16'b0000_0010_????_????,  // MULS
      16'b0000_0011_????_????,  // MULSU FMUL FMULS FMULSU
      16'b0000_01??_????_????,  // CPC  
      16'b0000_10??_????_????,  // SBC  
      16'b0000_11??_????_????,  // ADD  
      16'b0001_00??_????_????,  // CPSE 
      16'b0001_01??_????_????,  // CP   
      16'b0001_10??_????_????,  // SUB  
      16'b0001_11??_????_????,  // ADC  
      16'b0010_00??_????_????,  // AND  
      16'b0010_01??_????_????,  // EOR  
      16'b0010_10??_????_????,  // OR   
      16'b0010_11??_????_????  // MOV  
      
    };
  }
endclass: test_class

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
logic   [6-1:0] bd_wid;
logic   [8-1:0] bd_wdt;
logic   [8-1:0] bd_rdt;
logic   [6-1:0] bd_rid;
logic           bd_ren;
logic           bd_ack;
// I/O peripheral bus
logic           io_wen; // write enable
logic           io_ren; // read  enable
logic   [6-1:0] io_adr; // address
logic   [8-1:0] io_wdt; // write data
logic   [8-1:0] io_msk; // write mask
logic   [8-1:0] io_rdt; // read data
// interrupts
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

rp8 #(
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
  .bd_wid  (bd_wid),
  .bd_wdt  (bd_wdt),
  .bd_rdt  (bd_rdt),
  .bd_rid  (bd_rid),
  .bd_ren  (bd_ren),
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
// clocking
////////////////////////////////////////////////////////////////////////////////

initial    clk = 1'b1;
always #50 clk = ~clk;

////////////////////////////////////////////////////////////////////////////////

test_class test_instance;

initial begin
  test_instance = new();
  rst = 1'b1;
  repeat (2) @ (posedge clk);
  rst = 1'b0;
  repeat (2000) begin
//    test_instance.randomize();
//    $display ("%016b", test_instance.code);
    @ (posedge clk);
  end
  $finish;
end

////////////////////////////////////////////////////////////////////////////////
// instruction memory
////////////////////////////////////////////////////////////////////////////////

mem #(
  .FN ("test_isa.vmem"),
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

// TODO: for now there will be no delays on the program bus
always @ (posedge clk)
bp_rdy <= bp_vld;

// TODO, debugger code is not yet available
assign bp_npc = 'x;
assign bp_jmp = 1'b0;

////////////////////////////////////////////////////////////////////////////////
// instruction decoder
////////////////////////////////////////////////////////////////////////////////

string str;
bit [0:32-1] [8-1:0] asm;

always_comb begin
  str = avr_disasm::disasm(bp_rdt);
  asm = '0;
  for (int i=0; i<str.len(); i++) begin
    asm [i] = (8)'(str[i]);
  end
end

////////////////////////////////////////////////////////////////////////////////
// data memory
////////////////////////////////////////////////////////////////////////////////

// synchronous RAM
mem #(
  .SZ (2**DAW),
  .DW (8)
) bd_mem (
  .clk (clk),
  .ena (bd_req),
  .wen (bd_wen),
  .adr (bd_adr),
  .wdt (bd_wdt),
  .rdt (bd_rdt)
);

// read identification
always_ff @(posedge clk)
if (bd_req & ~bd_wen) bd_rid <= bd_wid;

// read enable
always_ff @(posedge clk, posedge rst)
if (rst)  bd_ren <= 1'b0;
else      bd_ren <= bd_req & ~bd_wen;

////////////////////////////////////////////////////////////////////////////////
// periphery
////////////////////////////////////////////////////////////////////////////////

logic [8-1:0] io_mem [0:64-1];

// I/O write (synchronous)
always @(posedge clk, posedge rst)
if (rst) begin
  for (int unsigned i=0; i<64; i++)
    io_mem[io_adr] <= 8'h00;
end else begin
  if (io_wen) io_mem[io_adr] <= io_wdt & io_msk | io_mem[io_adr] & ~io_msk;
end

// I/O read (asynchronous)
assign io_rdt = io_ren ? io_mem[io_adr] : 8'hxx;

////////////////////////////////////////////////////////////////////////////////
// interrupts
////////////////////////////////////////////////////////////////////////////////

assign irq_req = '0;

////////////////////////////////////////////////////////////////////////////////
// waveforms
////////////////////////////////////////////////////////////////////////////////

initial begin
  $dumpfile("rp8_tb.vcd");
  $dumpvars(0, rp8_tb);
end

endmodule: rp8_tb
