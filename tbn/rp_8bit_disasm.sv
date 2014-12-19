package rp_8bit_disasm;

// TODO check proper syntax for constants and displacements

function string disasm (bit [16-1:0] code);
//function bit [32-1:0] [8-1:0] disasm (bit [16-1:0] code);
  // local variables
  bit unsigned [5-1:0] Rd, Rr;  // destination and source registers
  bit unsigned [8-1:0] K;       // 8-bit constant
  bit unsigned [6-1:0] q;       // 6-bit displacement
  string index;
  string str;

  // decoder
  casez (code)
    16'b0000_0000_0000_0000: begin
      str = $sformatf ("nop");
    end
    16'b0000_0001_????_????: begin
      {Rd[4:0], Rr[4:0]} = {code[7:4], 1'b0, code[3:0], 1'b0};
      str = $sformatf ("movw r%0d:%0d,r%0d,%0d", Rd+1, Rd, Rr+1, Rr);
    end
    16'b0000_0010_????_????: begin
      {Rd[4:0], Rr[4:0]} = {2'b1, code[7:4], 2'b1, code[3:0]};
      str = $sformatf ("muls  r%0d,r%0d", Rd, Rr);
    end
    16'b0000_001?_????_????: begin
      {Rd[4:0], Rr[4:0]} = {2'b10, code[6:4], 2'b10, code[2:0]};
      case ({code[7], code[3]})
        2'b00: str = $sformatf ("mulsu  r%0d,r%0d", Rd, Rr);
        2'b01: str = $sformatf ("fmul   r%0d,r%0d", Rd, Rr);
        2'b10: str = $sformatf ("fmuls  r%0d,r%0d", Rd, Rr);
        2'b11: str = $sformatf ("fmulsu r%0d,r%0d", Rd, Rr);
      endcase
    end
    16'b0000_01??_????_????,
    16'b0000_1???_????_????,
    16'b0001_????_????_????,
    16'b0010_????_????_????: begin
      {Rr[4], Rd[4:0], Rr[3:0]} = code [9:0];
      case (code[13:10])
        4'b0001:             str = $sformatf ("cpc  r%0d,r%0d", Rd, Rr);  // Compare with Carry
        4'b0010:             str = $sformatf ("sbc  r%0d,r%0d", Rd, Rr);  // Subtract with Carry
        4'b0011: if (Rd!=Rr) str = $sformatf ("add  r%0d,r%0d", Rd, Rr);  // Add without Carry
                 else        str = $sformatf ("lsl  r%0d"     , Rd    );  // Logical Shift Left
        4'b0100:             str = $sformatf ("cpse r%0d,r%0d", Rd, Rr);  // Compare Skip if Equal
        4'b0101:             str = $sformatf ("cp   r%0d,r%0d", Rd, Rr);  // Compare
        4'b0110:             str = $sformatf ("sub  r%0d,r%0d", Rd, Rr);  // Subtract without Carry
        4'b0111: if (Rd!=Rr) str = $sformatf ("adc  r%0d,r%0d", Rd, Rr);  // Add with Carry
                 else        str = $sformatf ("rol  r%0d"     , Rd    );  // Rotate Left trough Carry
        4'b1000: if (Rd!=Rr) str = $sformatf ("and  r%0d,r%0d", Rd, Rr);  // Logical AND
                 else        str = $sformatf ("tst  r%0d"     , Rd    );  // Test for Zero or Minus
        4'b1001: if (Rd!=Rr) str = $sformatf ("eor  r%0d,r%0d", Rd, Rr);  // Exclusive OR
                 else        str = $sformatf ("clr  r%0d"     , Rd    );  // Clear Register
        4'b1010:             str = $sformatf ("or   r%0d,r%0d", Rd, Rr);  // Logical OR
        4'b1011:             str = $sformatf ("mov  r%0d,r%0d", Rd, Rr);  // Copy Register
      endcase
    end
    16'b0011_????_????_????,
    16'b01??_????_????_????: begin
      Rd[4:0] = {1'b1, code [7:4]};
      K = {code[11:8], code [3:0]};
      case (code[14:12])
        4'b0001: str = $sformatf ("cpi  r%0d,0x%02x", Rd, K);  // Compare with Immediate
        4'b0100: str = $sformatf ("sbci r%0d,0x%02x", Rd, K);  // Subtract Immediate with Carry
        4'b0101: str = $sformatf ("subi r%0d,0x%02x", Rd, K);  // Subtract Immediate
        4'b0110: str = $sformatf ("ori  r%0d,0x%02x", Rd, K);  // Logical OR with Immediate
        4'b0111: str = $sformatf ("andi r%0d,0x%02x", Rd, K);  // Logical AND with Immediate
      endcase
    end
    16'b10?0_????_????_????: begin
      Rd[4:0] = code[8:4];
      q = {code[13], code[12:11], code[2:0]};
      index = code[3] ? "Y" : "Z";
      case (code[9])
        1'b0:  // Load Indirect from Data Space to Register using Index Y/Z
          if (~|q)  str = $sformatf ("ld  r%0d,%s"       , Rd, index   );  // Y/Z: Unchanged
          else      str = $sformatf ("ldd r%0d,%s+0x%02x", Rd, index, q);  // Y/Z: Unchanged, q: Displacement
        1'b1:  // Store Indirect From Register to Data Space using Index Y/Z
          if (~|q)  str = $sformatf ("st  %s,r%0d"       , Rr, index   );  // Y/Z: Unchanged
          else      str = $sformatf ("std %s+0x%02x,r%0d", Rr, index, q);  // Y/Z: Unchanged, q: Displacement
      endcase
    end
    16'b1001_00??_????_????: begin
      Rd[4:0] = {1'b1, code [7:4]};
      index = code[3] ? "Y" : "Z";
      case (code[9])
        1'b0:  // Load Indirect from Data Space to Register using Index Y/Z
          case (q)
            3'b001: str = $sformatf ("ld  r%0d,%s+", Rd, index);  // Y/Z: Post incremented
            3'b010: str = $sformatf ("ld  r%0d,-%s", Rd, index);  // Y/Z: Pre decremented
            default:str = $sformatf ("undefined");
          endcase
        1'b1:  // Store Indirect From Register to Data Space using Index Y/Z
          case (q)
            3'b001: str = $sformatf ("st  r%0d,%s+", Rd, index);  // Y/Z: Post incremented
            3'b010: str = $sformatf ("st  r%0d,-%s", Rd, index);  // Y/Z: Pre decremented
            default:str = $sformatf ("undefined");
          endcase
      endcase
    end
    16'b1001_010?_????_0???: begin
      Rd[4:0] = {1'b1, code [7:4]};
      case (code[2:0])
        3'b000: str = $sformatf ("com  r%0d", Rd);  // One’s Complement
        3'b001: str = $sformatf ("neg  r%0d", Rd);  // Two’s Complement
        3'b010: str = $sformatf ("swap r%0d", Rd);  // Swap Nibbles
        3'b011: str = $sformatf ("inc  r%0d", Rd);  // Increment
        3'b100: str = $sformatf ("undefined");  // TODO check
        3'b101: str = $sformatf ("asr  r%0d", Rd);  // Arithmetic Shift Right 
        3'b110: str = $sformatf ("lsr  r%0d", Rd);  // Logical Shift Right
        3'b111: str = $sformatf ("ror  r%0d", Rd);  // Rotate Right through Carry
      endcase
    end



    16'b1001_11??_????_????: begin
      {Rr[4], Rd[4:0], Rr[3:0]} = code [9:0];
      str = $sformatf ("mul   r%0d,r%0d", Rd, Rr);
    end
    
  endcase
      
  return (str);
endfunction: disasm

endpackage: rp_8bit_disasm
