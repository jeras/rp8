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

task add; {C,R}=Rd+Rr  ; flags; $display ("ADD Rd,Rr ; 8'h%02x+8'h%02x    \  = 8'h%02x | HSVNZC=%6b", Rd, Rr,    R, {H,S,V,N,Z,C}); endtask
task adc; {C,R}=Rd+Rr+c; flags; $display ("ADC Rd,Rr ; 8'h%02x+8'h%02x+1'b%b = 8'h%02x | HSVNZC=%6b", Rd, Rr, c, R, {H,S,V,N,Z,C}); endtask
task sub; {C,R}=Rd-Rr  ; flags; $display ("SUB Rd,Rr ; 8'h%02x-8'h%02x    \  = 8'h%02x | HSVNZC=%6b", Rd, Rr,    R, {H,S,V,N,Z,C}); endtask
task sbc; {C,R}=Rd-Rr-c; flags; $display ("SBC Rd,Rr ; 8'h%02x-8'h%02x-1'b%b = 8'h%02x | HSVNZC=%6b", Rd, Rr, c, R, {H,S,V,N,Z,C}); endtask
task com; {C,R}=Rr-Rd  ; flags; $display ("COM Rd    ; 8'h%02x-8'h%02x    \  = 8'h%02x | HSVNZC=%6b", Rr, Rd,    R, {H,S,V,N,Z,C}); endtask
task neg; {C,R}=Rr-Rd  ; flags; $display ("NEG Rd    ; 8'h%02x-8'h%02x    \  = 8'h%02x | HSVNZC=%6b", Rr, Rd,    R, {H,S,V,N,Z,C}); endtask
task inc; {C,R}=Rd+Rr  ; flags; $display ("INC Rd    ; 8'h%02x+1'b1   \ \ \  = 8'h%02x | HSVNZC=%6b", Rd,        R, {H,S,V,N,Z,C}); endtask
task dec; {C,R}=Rd-Rr  ; flags; $display ("DEC Rd    ; 8'h%02x-1'b1   \ \ \  = 8'h%02x | HSVNZC=%6b", Rd,        R, {H,S,V,N,Z,C}); endtask

task flags;
  H = Rd[3]&Rr[3] | Rr[3]&~R[3] | ~R[3]&Rd[3];
  V = Rd[7]&Rr[7]&~R[7] | ~Rd[7]&~Rr[7]&R[7];
  N = R[7];
  Z = ~|R;
  S = N^V;
endtask

initial begin
  Rd=8'h00; Rr=8'h00;         add;
  Rd=8'h00; Rr=8'hff;         add;
  Rd=8'h80; Rr=8'h80;         add;
  Rd=8'hff; Rr=8'hff;         add;
  $display();
  Rd=8'h00; Rr=8'h00; c=1'b0; adc;
  Rd=8'h00; Rr=8'hff; c=1'b0; adc;
  Rd=8'h80; Rr=8'h80; c=1'b0; adc;
  Rd=8'hff; Rr=8'hff; c=1'b0; adc;
  Rd=8'h00; Rr=8'h00; c=1'b1; adc;
  Rd=8'h00; Rr=8'hff; c=1'b1; adc;
  Rd=8'h80; Rr=8'h80; c=1'b1; adc;
  Rd=8'hff; Rr=8'hff; c=1'b1; adc;
  $display();
  Rd=8'h00; Rr=8'h00;         sub;
  Rd=8'h00; Rr=8'hff;         sub;
  Rd=8'hff; Rr=8'h00;         sub;
  Rd=8'hff; Rr=8'hff;         sub;
  $display();
  Rd=8'h00; Rr=8'h00; c=1'b0; sbc;
  Rd=8'h00; Rr=8'hff; c=1'b0; sbc;
  Rd=8'hff; Rr=8'h00; c=1'b0; sbc;
  Rd=8'hff; Rr=8'hff; c=1'b0; sbc;
  Rd=8'h00; Rr=8'h00; c=1'b1; sbc;
  Rd=8'h00; Rr=8'hff; c=1'b1; sbc;
  Rd=8'hff; Rr=8'h00; c=1'b1; sbc;
  Rd=8'hff; Rr=8'hff; c=1'b1; sbc;
  $display();
  Rd=8'hff; Rr=8'hff;         com;
  Rd=8'h00; Rr=8'h00;         neg;
  Rd=8'h00; Rr=8'h01;         inc;
  Rd=8'h00; Rr=8'h01;         dec;
end

endmodule: isa_logic
