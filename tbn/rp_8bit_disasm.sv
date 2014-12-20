package rp_8bit_disasm;

// TODO check proper syntax for constants and displacements

function string disasm (bit [16-1:0] code, bit deep=1'b0);
//function bit [32-1:0] [8-1:0] disasm (bit [16-1:0] code);
  // local variables
  bit unsigned [5-1:0] Rd, Rr;  // destination and source registers
  bit unsigned [8-1:0] K;       // 8-bit constant
  bit unsigned [6-1:0] q;       // 6-bit displacement
  bit unsigned [3-1:0] b;       // 3-bit bit index
  bit unsigned [6-1:0] A;       // 6-bit IO address
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
        4'b0001: str = $sformatf ("cpc  r%0d,r%0d", Rd, Rr);  // Compare with Carry
        4'b0010: str = $sformatf ("sbc  r%0d,r%0d", Rd, Rr);  // Subtract with Carry
        4'b0011: str = $sformatf ("add  r%0d,r%0d", Rd, Rr);  // Add without Carry
        4'b0100: str = $sformatf ("cpse r%0d,r%0d", Rd, Rr);  // Compare Skip if Equal
        4'b0101: str = $sformatf ("cp   r%0d,r%0d", Rd, Rr);  // Compare
        4'b0110: str = $sformatf ("sub  r%0d,r%0d", Rd, Rr);  // Subtract without Carry
        4'b0111: str = $sformatf ("adc  r%0d,r%0d", Rd, Rr);  // Add with Carry
        4'b1000: str = $sformatf ("and  r%0d,r%0d", Rd, Rr);  // Logical AND
        4'b1001: str = $sformatf ("eor  r%0d,r%0d", Rd, Rr);  // Exclusive OR
        4'b1010: str = $sformatf ("or   r%0d,r%0d", Rd, Rr);  // Logical OR
        4'b1011: str = $sformatf ("mov  r%0d,r%0d", Rd, Rr);  // Copy Register
      endcase
      if (deep && ((Rd==Rr))) begin
        case (code[13:10])
          4'b0011: str = $sformatf ("lsl  r%0d", Rd);  // Logical Shift Left
          4'b0111: str = $sformatf ("rol  r%0d", Rd);  // Rotate Left trough Carry
          4'b1000: str = $sformatf ("tst  r%0d", Rd);  // Test for Zero or Minus
          4'b1001: str = $sformatf ("clr  r%0d", Rd);  // Clear Register
        endcase
      end
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
    16'b1001_0100_????_1000: begin
      const bit [8-1:0] [8-1:0] sreg = "ithsvnzc";
      b = code [6:4];
      if (code[7]) str = $sformatf ("bclr r%0d", b);  // Bit Clear in SREG
      else         str = $sformatf ("bset r%0d", b);  // Bit Set in SREG
      // Flafs: Global Interrupt/T/Half Carry/Signed/Overflow/Negative/Zero/Carry
      if (deep) begin
        if (code[7]) str = $sformatf ("cl%s", sreg[b]);  // Clear * Flag
        else         str = $sformatf ("se%s", sreg[b]);  // Set * Flag
      end
    end
    16'b1001_0101_????_1000: begin
      case (code[7:4])
        4'b0000: str = $sformatf ("ret"   );  // Return from Subroutine
        4'b0001: str = $sformatf ("reti"  );  // Return from Interrupt
        4'b0010: str = $sformatf ("undefined");  // TODO check
        4'b0011: str = $sformatf ("undefined");  // TODO check
        4'b0100: str = $sformatf ("undefined");  // TODO check
        4'b0101: str = $sformatf ("undefined");  // TODO check
        4'b0110: str = $sformatf ("undefined");  // TODO check
        4'b0111: str = $sformatf ("undefined");  // TODO check
        4'b1000: str = $sformatf ("sleep" );  // Sleep
        4'b1001: str = $sformatf ("break" );  // Break
        4'b1010: str = $sformatf ("wdr"   );  // Watchdog Reset
        4'b1011: str = $sformatf ("undefined");  // TODO check
        4'b1100: str = $sformatf ("lpm"   );  // Load Program Memory
        4'b1101: str = $sformatf ("elpm"  );  // Extended Load Program Memory
        4'b1110: str = $sformatf ("spm"   );  // Store Program Memory
        4'b1111: str = $sformatf ("spm Z+");  // Store Program Memory #2 (Z post incremented)
      endcase
    end
    16'b1001_010?_000?_1001: begin
      case (code[8])
        1'b0: case (code[4])
          1'b0: str = $sformatf ("ijmp"  );  //          Indirect Jump
          1'b1: str = $sformatf ("eijmp" );  // Extended Indirect Jump
        endcase
        1'b1: case (code[4])
          1'b0: str = $sformatf ("icall" );  //          Indirect Call to Subroutine
          1'b1: str = $sformatf ("eicall");  // Extended Indirect Call to Subroutine
        endcase
      endcase
    end
    16'b1001_010?_????_1010: begin
      Rd[4:0] = code[8:4];
      str = $sformatf ("dec %0d", Rd);  // Decrement
    end
    16'b1001_0100_????_1011: begin
      K = {4'b0000, code[7:4]};
      str = $sformatf ("des 0x%01x", K[3:0]);  // Data Encryption Standard
    end
    16'b1001_010?_????_10??: begin
      K = {2'b00, code[8:4], code[0]};
      case (code[1])
        1'b0: str = $sformatf ("jmp  0x%02x????", K[5:0]);  // Jump
        1'b1: str = $sformatf ("call 0x%02x????", K[5:0]);  // Long Call to a Subroutine
      endcase
    end
    16'b1001_011?_????_????: begin
      Rd[4:0] = {2'b11, code[5:4], 1'b0};
      K = {2'b00, code[7:6], code[3:0]};
      // TODO: X/Y/Z registers could be named
      case (code[8])
        1'b0: str = $sformatf ("adiw r%0d:%0d,0x%02x", Rd+1, Rd, K);  // Add Immediate to Word
        1'b1: str = $sformatf ("movw r%0d:%0d,0x%02x", Rd+1, Rd, K);  // Subtract Immediate from Word
      endcase
    end
    16'b1001_10??_????_????: begin
      b = code[2:0];
      A = {1'b0, code[7:3]};
      case (code[8])
        1'b0: case (code[9])
          1'b0: str = $sformatf ("cbi 0x%02x,%0d", A, b);  // Clear Bit in I/O Register
          1'b1: str = $sformatf ("sbi 0x%02x,%0d", A, b);  // Set Bit in I/O Register
        endcase
        1'b1: case (code[9])
          1'b0: str = $sformatf ("sbic 0x%02x,%0d", A, b);  // Skip if Bit in I/O Register is Cleared
          1'b1: str = $sformatf ("sbis 0x%02x,%0d", A, b);  // Skip if Bit in I/O Register is Set
        endcase
      endcase
    end
    16'b1001_11??_????_????: begin
      {Rr[4], Rd[4:0], Rr[3:0]} = code[9:0];
      str = $sformatf ("mul   r%0d,r%0d", Rd, Rr);  // Multiply Unsigned
    end
    16'b1011_????_????_????: begin
      Rd[4:0] = code[8:4];
      A = {code[10:9], code[3:0]};
      case (code[11])
        1'b0: str = $sformatf ("in   r%0d,0x%02x", Rd, A);  // Load an I/O Location to Register
        1'b1: str = $sformatf ("out  0x%02x,r%0d", A, Rd);  // Store Register to I/O Location
      endcase
    end
    
  endcase
      
  return (str);
endfunction: disasm

endpackage: rp_8bit_disasm
