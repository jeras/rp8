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

localparam BI_IW =  8; // bus instruction - interrupt width
localparam BI_AW = 11; // bus instruction - address   width
localparam BD_AW = 13; // bus data        - address   width

// instruction bus
logic             pmem_ce;
logic [BI_AW-1:0] pmem_a ;
logic    [16-1:0] pmem_d ;
// data bus
logic             dmem_we;
logic [BD_AW-1:0] dmem_a ;
logic     [8-1:0] dmem_do;
logic     [8-1:0] dmem_di;
// peripheral bus
logic             io_re;
logic             io_we;
logic     [6-1:0] io_a ;
logic     [8-1:0] io_do;
logic     [8-1:0] io_di;
// interrupt
logic [BI_IW-1:0] irq;
logic [BI_IW-1:0] irq_ack;

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

logic [16-1:0] pmem [0:2**BI_AW-1];

initial begin
  $readmemh ("test_isa.vmem", pmem);
end

always @(posedge clk)
if (pmem_ce) begin
  pmem_d <= pmem[pmem_a];
end

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

logic  [8-1:0] dmem [0:2**BD_AW-1];

initial begin
  for (int i=0; i<2**BI_AW; i++)  dmem[i] = 0;
end

always @(posedge clk)
begin
  if (dmem_we) begin
    dmem[dmem_a] <= dmem_do;
  end else begin
    dmem_di <= dmem[dmem_a];
  end
end

////////////////////////////////////////////////////////////////////////////////
// periphery
////////////////////////////////////////////////////////////////////////////////

logic [8-1:0] iomem [0:64-1];

initial begin
  for (int i=0; i<2**BI_AW; i++)  dmem[i] = 0;
end

always @(posedge clk, posedge rst)
if (rst) begin
  for (int unsigned i=0; i<64; i++)
    iomem[io_a] <= 8'h00;
end else begin
  if (io_we) iomem[io_a] <= io_do;
  if (io_re) io_di <= iomem[io_a];
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
