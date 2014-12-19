package rp_8bit_disasm;

// TODO check proper syntax for constants and displacements

function string disasm (bit [16-1:0] code);
  // local variables
  bit unsigned [5-1:0] Rd, Rr;  // destination and source registers
  bit unsigned [8-1:0] K;       // 8-bit constant
  bit unsigned [6-1:0] q;       // 6-bit displacement
  string index;

  // decoder
  casez (code)
    16'b0000_0000_0000_0000: begin
      disasm = $sformatf ("nop");
    end
    16'b0000_0001_????_????: begin
      {Rd[4:0], Rr[4:0]} = {code[7:4], 1'b0, code[3:0], 1'b0};
      disasm = $sformatf ("movw r%d:%d,r%d,%d", Rd+1, Rd, Rr+1, Rr);
    end
    16'b0000_0010_????_????: begin
      {Rd[4:0], Rr[4:0]} = {2'b1, code[7:4], 2'b1, code[3:0]};
      disasm = $sformatf ("muls  r%d,r%d", Rd, Rr);
    end
    16'b0000_001?_????_????: begin
      {Rd[4:0], Rr[4:0]} = {2'b10, code[6:4], 2'b10, code[2:0]};
      case ({code[7], code[3]})
        2'b00: disasm = $sformatf ("mulsu  r%d,r%d", Rd, Rr);
        2'b01: disasm = $sformatf ("fmul   r%d,r%d", Rd, Rr);
        2'b10: disasm = $sformatf ("fmuls  r%d,r%d", Rd, Rr);
        2'b11: disasm = $sformatf ("fmulsu r%d,r%d", Rd, Rr);
      endcase
    end
    16'b0000_01??_????_????,
    16'b0000_1???_????_????,
    16'b0001_????_????_????,
    16'b0010_????_????_????: begin
      {Rr[4], Rd[4:0], Rr[3:0]} = code [9:0];
      case (code[13:10])
        4'b0001: disasm = $sformatf ("cpc  r%d,r%d", Rd, Rr);  // Compare with Carry
        4'b0010: disasm = $sformatf ("sbc  r%d,r%d", Rd, Rr);  // Subtract with Carry
        4'b0011: disasm = $sformatf ("add  r%d,r%d", Rd, Rr);  // Add without Carry
        // TODO LSL (see ADD Rd,Rd) // Logical Shift Left
        4'b0100: disasm = $sformatf ("cpse r%d,r%d", Rd, Rr);  // Compare Skip if Equal
        4'b0101: disasm = $sformatf ("cp   r%d,r%d", Rd, Rr);  // Compare
        4'b0110: disasm = $sformatf ("sub  r%d,r%d", Rd, Rr);  // Subtract without Carry
        4'b0111: disasm = $sformatf ("adc  r%d,r%d", Rd, Rr);  // Add with Carry
        // TODO ROL (ADC Rd,Rd) // Rotate Left trough Carry
        4'b1000: disasm = $sformatf ("and  r%d,r%d", Rd, Rr);  // Logical AND
        // TODO TST (and rd,rd) // Test for Zero or Minus
        4'b1001: disasm = $sformatf ("eor  r%d,r%d", Rd, Rr);  // Exclusive OR
        // TODO CLR (eor rd,rd) // Clear Register
        4'b1010: disasm = $sformatf ("or   r%d,r%d", Rd, Rr);  // Logical OR
        4'b1011: disasm = $sformatf ("mov  r%d,r%d", Rd, Rr);  // Copy Register
      endcase
    end
    16'b0011_????_????_????,
    16'b01??_????_????_????: begin
      Rd[4:0] = {1'b1, code [7:4]};
      K = {code[11:8], code [3:0]};
      case (code[14:12])
        4'b0001: disasm = $sformatf ("cpi  r%d,0x%02x", Rd, K);  // Compare with Immediate
        4'b0100: disasm = $sformatf ("sbci r%d,0x%02x", Rd, K);  // Subtract Immediate with Carry
        4'b0101: disasm = $sformatf ("subi r%d,0x%02x", Rd, K);  // Subtract Immediate
        4'b0110: disasm = $sformatf ("ori  r%d,0x%02x", Rd, K);  // Logical OR with Immediate
        4'b0111: disasm = $sformatf ("andi r%d,0x%02x", Rd, K);  // Logical AND with Immediate
      endcase
    end
    16'b10?0_????_????_????: begin
      Rd[4:0] = code[8:4];
      q = {code[13], code[12:11], code[2:0]};
      index = code[3] ? "Y" : "Z";
      case (code[9])
        1'b0:  // Load Indirect from Data Space to Register using Index Y/Z
          if (~|q)  disasm = $sformatf ("ld  r%d,%s"       , Rd, index   );  // Y/Z: Unchanged
          else      disasm = $sformatf ("ldd r%d,%s+0x%02x", Rd, index, q);  // Y/Z: Unchanged, q: Displacement
        1'b1:  // Store Indirect From Register to Data Space using Index Y/Z
          if (~|q)  disasm = $sformatf ("st  %s,r%d"       , Rr, index   );  // Y/Z: Unchanged
          else      disasm = $sformatf ("std %s+0x%02x,r%d", Rr, index, q);  // Y/Z: Unchanged, q: Displacement
      endcase
    end
    16'b1001_00??_????_????: begin
      Rd[4:0] = {1'b1, code [7:4]};
      index = code[3] ? "Y" : "Z";
      case (code[9])
        1'b0:  // Load Indirect from Data Space to Register using Index Y/Z
          case (q)
            3'b001: disasm = $sformatf ("ld  r%d,%s+", Rd, index);  // Y/Z: Post incremented
            3'b010: disasm = $sformatf ("ld  r%d,-%s", Rd, index);  // Y/Z: Pre decremented
            default:disasm = $sformatf ("undefined");
          endcase
        1'b1:  // Store Indirect From Register to Data Space using Index Y/Z
          case (q)
            3'b001: disasm = $sformatf ("st  r%d,%s+", Rd, index);  // Y/Z: Post incremented
            3'b010: disasm = $sformatf ("st  r%d,-%s", Rd, index);  // Y/Z: Pre decremented
            default:disasm = $sformatf ("undefined");
          endcase
      endcase
    end
    16'b1001_010?_????_0???: begin
      Rd[4:0] = {1'b1, code [7:4]};
      case (code[2:0])
        3'b000: disasm = $sformatf ("com  r%d", Rd);  // One’s Complement
        3'b001: disasm = $sformatf ("neg  r%d", Rd);  // Two’s Complement
        3'b010: disasm = $sformatf ("swap r%d", Rd);  // Swap Nibbles
        3'b011: disasm = $sformatf ("inc  r%d", Rd);  // Increment
        3'b100: disasm = $sformatf ("undefined");  // TODO check
        3'b101: disasm = $sformatf ("asr  r%d", Rd);  // Arithmetic Shift Right 
        3'b110: disasm = $sformatf ("lsr  r%d", Rd);  // Logical Shift Right
        3'b111: disasm = $sformatf ("ror  r%d", Rd);  // Rotate Right through Carry
      endcase
    end



    16'b1001_11??_????_????: begin
      {Rr[4], Rd[4:0], Rr[3:0]} = code [9:0];
      disasm = $sformatf ("mul   r%d,r%d", Rd, Rr);
    end
    
  endcase
endfunction: disasm

endpackage: rp_8bit_disasm
