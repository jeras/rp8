////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

module rp_8bit_tb;

localparam IRW =  8; // bus instruction - interrupt width
localparam PAW = 11; // bus instruction - address   width
localparam DAW = 13; // bus data        - address   width

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
// interrupts
logic [IRW-1:0] irq_req;
logic [IRW-1:0] irq_ack;

rp_8bit #(
  .IRW (IRW),
  .PAW (PAW),
  .DAW (DAW)
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
  .irq_ack (irq_ack)
);

////////////////////////////////////////////////////////////////////////////////
// clocking
////////////////////////////////////////////////////////////////////////////////

initial    clk = 1'b0;
always #50 clk = ~clk;

////////////////////////////////////////////////////////////////////////////////

test_class test_instance;

initial begin
  test_instance = new();
  rst = 1'b1;
  repeat (4) @ (posedge clk);
  rst = 1'b0;
  repeat (16) begin
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
  .SZ (0:2**PAW-1),
  .DW (16)
)(
  .clk (clk),
  .ena (bp_ena),
  .wen (bp_wen),
  .adr (bp_adr),
  .wdt (bp_wdt),
  .rdt (bp_rdt),
);

string str;
bit [0:32-1] [8-1:0] asm;

always_comb begin
  str = rp_8bit_disasm::disasm(bp_rdt);
  asm = '0;
  for (int i=0; i<str.len(); i++) begin
    asm [i] = (8)'(str[i]);
  end
end

////////////////////////////////////////////////////////////////////////////////
// data memory
////////////////////////////////////////////////////////////////////////////////

mem #(
  .SZ (0:2**DAW-1),
  .DW (8)
)(
  .clk (clk),
  .ena (bd_ena),
  .wen (bd_wen),
  .adr (bd_adr),
  .wdt (bd_wdt),
  .rdt (bd_rdt),
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

assign irq_req = '0;

////////////////////////////////////////////////////////////////////////////////
// waveforms
////////////////////////////////////////////////////////////////////////////////

initial begin
  $dumpfile("rp_8bit_tb.vcd");
  $dumpvars(0, rp_8bit_tb);
end

endmodule: rp_8bit_tb
