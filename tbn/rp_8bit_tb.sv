
module rp_8bit_tb;

localparam BI_IW =  8; // bus instruction - interrupt width
localparam BI_AW = 11; // bus instruction - address   width
localparam BD_AW = 13; // bus data        - address   width

// system signals
logic        clk;
logic        rst;

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

rp_8bit #(
  .BI_IW (BI_IW),
  .BI_AW (BI_AW),
  .BD_AW (BD_AW)
) UUT (
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

integer cycles;

initial begin
	clk <= 1;
	rst <= 1;
	cycles = 0;
	while (cycles < 8) begin
		#50; clk <= ~clk;
		cycles = cycles + 1;
		#50; clk <= ~clk;
	end
	rst <= #20 0;
	forever begin
		#50; clk <= ~clk;
		cycles = cycles + 1;
		#50; clk <= ~clk;
		if (cycles == 10000) begin
			$display("Reached limit of 10000 cpu cycles.");
			$finish;
		end
	end
end

////////////////////////////////////////////////////////////////////////////////
// instruction memory
////////////////////////////////////////////////////////////////////////////////

logic [16-1:0] pmem [0:2**BI_AW-1];

initial begin
  for (int i=0; i<2**BI_AW; i++)  pmem[i] = 0;
  $readmemh ("tbn/sieve.hex", pmem);
end

always @(posedge clk)
if (pmem_ce) begin
  $display("+LOG+ %t PR @%x %x", $time, pmem_a * 2, pmem[pmem_a]);
  pmem_d <= pmem[pmem_a];
end

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
    $display("+LOG+ %t DW @%x   %x", $time, dmem_a, dmem_do);
    dmem[dmem_a] <= dmem_do;
  end else begin
    dmem_di <= dmem[dmem_a];
  end
end

////////////////////////////////////////////////////////////////////////////////
// interrupts
////////////////////////////////////////////////////////////////////////////////

assign irq = '0;

////////////////////////////////////////////////////////////////////////////////
// periphery
////////////////////////////////////////////////////////////////////////////////

integer     output_idx;
logic [7:0] output_buf [1023:0];
event       output_eof;

initial begin
	output_idx = 0;
end

always @(posedge clk) begin
	if (io_we && io_a == 42) begin
		$display("+LOG+ %t IO @%x   %x  <---", $time, io_a, io_do);
		if (io_do == 0) begin
			-> output_eof;
		end else begin
			output_buf[output_idx] = io_do;
			output_idx = output_idx + 1;
		end
	end
	io_di <= 0;
end

always @(output_eof) begin
	#1001;
	$display("Got EOF marker on IO port.");
	for (int i=0; i<output_idx; i++) begin
		$display("+OUT+ %t %d", $time, output_buf[i]);
	end
	$finish;
end

initial begin
  $dumpfile("rp_8bit_tb.vcd");
  $dumpvars(0, rp_8bit_tb);
end

endmodule: rp_8bit_tb
