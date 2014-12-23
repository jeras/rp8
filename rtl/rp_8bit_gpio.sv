/*
 * rp_8bit (8 bit risc processor, AVR compatible)
 * rp_8bit_gpio (GPIO peripheral)
 * Copyright (C) 2014 Iztok Jeras
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

module rp_8bit_gpio #(
  int unsigned PDW = 8,     // port data width
  bit [6-1:0]  ADR = 6'h00  // I/O address
  
)(
  // system signals
  input  logic           clk,
  input  logic           rst,
  // peripheral bus
  input  logic   [3-1:0] io_re,
  input  logic   [3-1:0] io_we,
  input  logic   [8-1:0] io_dw,
  output logic   [8-1:0] io_dr,
  // configuration
  input logic            pud  ,  // pullup disable
  input logic            sleep,  // sleep
  // interrupt
  input  logic [PDW-1:0] irq,
  // gpio control
  output logic [PDW-1:0] gpio_pull,
  output logic [PDW-1:0] gpio_ddr ,
  output logic [PDW-1:0] gpio_port,
  input  logic [PDW-1:0] gpio_pin
);

// direction register write
always_ff @ (posedge clk, posedge rst)
if (rst)           gpio_ddr <= '0;
else if (io_we[2]) gpio_ddr <= io_dw[PDW-1:0];

// output register write
always_ff @ (posedge clk, posedge rst)
if (rst)           gpio_port <= '0;
else if (io_we[1]) gpio_port <= io_dw[PDW-1:0];

// GPIO pullup
assign gpio_pull = pud ? '0 : ~gpio_ddr & gpio_port;

// input metastability handler
logic [PDW-1:0] gpio_latch;
logic [PDW-1:0] gpio_meta;
// TODO the sleep handler is more complex in reality
always_latch
if (clk) gpio_latch <= sleep ? '0 : gpio_pin;
always_ff @ (posedge clk)
gpio_meta <= gpio_latch;

// read access
always_comb
case (io_re)
//  3'b001:  io_dr = PDW'(gpio_meta);
//  3'b010:  io_dr = PDW'(gpio_ddr );
//  3'b100:  io_dr = PDW'(gpio_port);
  3'b001:  io_dr = gpio_meta;
  3'b010:  io_dr = gpio_ddr ;
  3'b100:  io_dr = gpio_port;
  default: io_dr = 'x;
endcase

endmodule: rp_8bit_gpio
