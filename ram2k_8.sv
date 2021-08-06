`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Cal Poly
// Engineer: Paul Hummel
//
// Create Date: 06/07/2018 06:00:59 PM
// Design Name:
// Module Name: ram2k_8
// Project Name: VGA 40x30
// Target Devices: RAT MCU on Basys3
// Description: Framebuffer memory for VGA driver.
//              3 port memory, 2 for reading, 1 for writing
//              WA1 - first address for reading and writing,
//                    output is RD1, input is WD
//              WE  - write enable, only save data (WD to WA1) when high
//              RA2 - first address only for reading, output is RD2
//
// Revision:
// Revision 0.01 - File Created
//////////////////////////////////////////////////////////////////////////////////


module ram2k_8(
    input CLK,
    input WE,           // write enable
    input [10:0] WA1,   // write address 1
    input [10:0] RA2,   // read address 2
    input [7:0] WD,     // write data to address 1
    output [7:0] RD1,   // read data from address 1
    output [7:0] RD2    // read data from address 2
    );
    
    logic [7:0] r_memory [2047:0];
    
    // Initialize all memory to 0s
    initial begin
        int i;
        for (i=0; i<2048; i++) begin
            r_memory[i] = 8'h00;
        end
    end
    
    // only save data on rising edge
    always_ff @(posedge CLK) begin
        if (WE == 1'b1)
            r_memory[WA1] = WD;
    end
    
    assign RD2 = r_memory[RA2];
    assign RD1 = r_memory[WA1];
    
endmodule
