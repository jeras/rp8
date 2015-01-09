module isa_logic ();

// SREG: Status Register
logic C;  // Carry Flag
logic Z;  // Zero Flag
logic N;  // Negative Flag
logic V;  // Two’s complement overflow indicator
logic S;  // N^V, For signed tests
logic H;  // Half Carry Flag

logic       c;
logic [7:0] Rd, Rr, R;

task isa_and; R=Rd&Rr; flags; $display ("AND Rd,Rr ; 8'h%02x & 8'h%02x = 8'h%02x | HSVNZC=%6b", Rd, Rr, R, {H,S,V,N,Z,C}); endtask
task isa_eor; R=Rd^Rr; flags; $display ("EOR Rd,Rr ; 8'h%02x ^ 8'h%02x = 8'h%02x | HSVNZC=%6b", Rd, Rr, R, {H,S,V,N,Z,C}); endtask
task isa_or ; R=Rd|Rr; flags; $display ("EO  Rd,Rr ; 8'h%02x | 8'h%02x = 8'h%02x | HSVNZC=%6b", Rd, Rr, R, {H,S,V,N,Z,C}); endtask

task flags;
  V = 1'b0;
  N = R[7];
  Z = ~|R;
  S = N^V;
endtask

initial begin
  Rd=8'hff; Rr=8'h00;  isa_and;
  Rd=8'hff; Rr=8'hff;  isa_and;
  $display();
  Rd=8'hff; Rr=8'h00;  isa_eor;
  Rd=8'hff; Rr=8'hff;  isa_eor;
  $display();
  Rd=8'h00; Rr=8'h00;  isa_or;
  Rd=8'h00; Rr=8'hff;  isa_or;
  $display();
end

endmodule: isa_logic
