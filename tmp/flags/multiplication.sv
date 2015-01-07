module multiplication ();

logic [ 7:0] Rd, Rr;
logic [15:0] R;

logic [0:- 7] Fd, Fr;
logic [1:-14] F;

task mul   ; R=        Rd *              Rr  ; $display ("MUL    Rd,Rr ; %4d(8'h%02x) * %4d(8'h%02x) = %7d(16'h%04x) | C=%b Z=%b",         Rd , Rd,         Rr , Rr,         R , R, R[15], ~|R); endtask
task muls  ; R=$signed(Rd)*$signed(      Rr ); $display ("MULS   Rd,Rr ; %4d(8'h%02x) * %4d(8'h%02x) = %7d(16'h%04x) | C=%b Z=%b", $signed(Rd), Rd, $signed(Rr), Rr, $signed(R), R, R[15], ~|R); endtask
task mulsu ; R=$signed(Rd)*$signed({1'b0,Rr}); $display ("MULSU  Rd,Rr ; %4d(8'h%02x) * %4d(8'h%02x) = %7d(16'h%04x) | C=%b Z=%b", $signed(Rd), Rd,         Rr , Rr, $signed(R), R, R[15], ~|R); endtask

task fmul  ; F=        Fd *              Fr  ;  $display ("FMUL   Rd,Rr ; %f(8'h%02x) * %f(8'h%02x) = %f(16'h%04x) | C=%b Z=%b", $itor(        Fd )/128, Fd, $itor(        Fr )/128, Fr, $itor(        F )/128/128, F, F[1], ~|F[0:-14]); endtask
task fmuls ; F=$signed(Fd)*$signed(      Fr );  $display ("FMULS  Rd,Rr ; %f(8'h%02x) * %f(8'h%02x) = %f(16'h%04x) | C=%b Z=%b", $itor($signed(Fd))/128, Fd, $itor($signed(Fr))/128, Fr, $itor($signed(F))/128/128, F, F[1], ~|F[0:-14]); endtask
task fmulsu; F=$signed(Fd)*$signed({1'b0,Fr});  $display ("FMULSU Rd,Rr ; %f(8'h%02x) * %f(8'h%02x) = %f(16'h%04x) | C=%b Z=%b", $itor($signed(Fd))/128, Fd, $itor(        Fr )/128, Fr, $itor($signed(F))/128/128, F, F[1], ~|F[0:-14]); endtask

initial begin
  Rd=8'h00; Rr=8'h00;  mul;
  Rd=8'h01; Rr=8'h01;  mul;
  Rd=8'h01; Rr=8'hff;  mul;
  Rd=8'hff; Rr=8'hff;  mul;
  $display ();
  Rd=8'h00; Rr=8'h00;  muls;
  Rd=8'h7f; Rr=8'h7f;  muls;
  Rd=8'h7f; Rr=8'h80;  muls;
  Rd=8'hff; Rr=8'h01;  muls;
  Rd=8'h7f; Rr=8'h01;  muls;
  Rd=8'h80; Rr=8'h01;  muls;
  Rd=8'hff; Rr=8'hff;  muls;
  Rd=8'h7f; Rr=8'hff;  muls;
  Rd=8'h80; Rr=8'hff;  muls;
  $display ();
  Rd=8'h00; Rr=8'h00;  mulsu;
  Rd=8'h7f; Rr=8'h7f;  mulsu;
  Rd=8'h7f; Rr=8'h80;  mulsu;
  Rd=8'hff; Rr=8'h01;  mulsu;
  Rd=8'h7f; Rr=8'h01;  mulsu;
  Rd=8'h80; Rr=8'h01;  mulsu;
  Rd=8'hff; Rr=8'hff;  mulsu;
  Rd=8'h7f; Rr=8'hff;  mulsu;
  Rd=8'h80; Rr=8'hff;  mulsu;
  $display ();
  Fd=8'h00; Fr=8'h00;  fmul;
  Fd=8'h01; Fr=8'h01;  fmul;
  Fd=8'h01; Fr=8'hff;  fmul;
  Fd=8'h80; Fr=8'h01;  fmul;
  Fd=8'h80; Fr=8'h80;  fmul;
  Fd=8'h80; Fr=8'hff;  fmul;
  Fd=8'hff; Fr=8'hff;  fmul;
  $display ();
  Fd=8'h00; Fr=8'h00;  fmuls;
  Fd=8'h7f; Fr=8'h7f;  fmuls;
  Fd=8'h7f; Fr=8'h80;  fmuls;
  Fd=8'hff; Fr=8'h01;  fmuls;
  Fd=8'h7f; Fr=8'h01;  fmuls;
  Fd=8'h80; Fr=8'h01;  fmuls;
  Fd=8'hff; Fr=8'hff;  fmuls;
  Fd=8'h7f; Fr=8'hff;  fmuls;
  Fd=8'h80; Fr=8'hff;  fmuls;
  $display ();
  Fd=8'h00; Fr=8'h00;  fmulsu;
  Fd=8'h7f; Fr=8'h7f;  fmulsu;
  Fd=8'h7f; Fr=8'h80;  fmulsu;
  Fd=8'hff; Fr=8'h01;  fmulsu;
  Fd=8'h7f; Fr=8'h01;  fmulsu;
  Fd=8'h80; Fr=8'h01;  fmulsu;
  Fd=8'hff; Fr=8'hff;  fmulsu;
  Fd=8'h7f; Fr=8'hff;  fmulsu;
  Fd=8'h80; Fr=8'hff;  fmulsu;
end

endmodule: multiplication
