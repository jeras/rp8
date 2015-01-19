////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

module mem #(
  // 1kB by default
  string       FN = "",         // initialization file name
  int unsigned SZ = 2**10,      // memory size in words
  int unsigned AW = $clog2(SZ), // address width
  int unsigned DW = 8           // data width
)(
  input  logic          clk, // clock
  input  logic          ena, // write or read enable
  input  logic          wen, // write enable
  input  logic [AW-1:0] adr, // address
  input  logic [DW-1:0] wdt, // write data
  output logic [DW-1:0] rdt  // read data
);

logic [DW-1:0] mem [0:SZ-1];

// write and read access
always @(posedge clk)
if (ena) begin
  if (wen)  mem[adr] <= wdt;
  else      rdt <= mem[adr];
end

// initialization
initial
if (FN!="") $readmemh(FN, mem);

endmodule: mem
