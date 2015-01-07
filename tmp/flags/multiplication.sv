module multiplication ();

logic  [8-1:0] Rd, Rr;
logic [16-1:0] R;

initial begin
  Rd = 8'h00;
  Rr = 8'h00;
  R  = Rd * Rr;
  $display ("MUL  Rd,Rr   ; %d(%02x) * %d(%02x) = %d(%04x)", Rd, Rd, Rr, Rr, R, R);
end

endmodule multiplication;
