`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  J. Callenes
// 
// Create Date: 01/05/2019 12:17:57 AM
// Design Name: 
// Module Name: registerFile
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module OTTER_registerFile(Read1,Read2,WriteReg,WriteData,RegWrite,Data1,Data2,clock,
                          Read1_2,Read2_2,WriteReg_2,WriteData_2,RegWrite_2,Data1_2,Data2_2
                          );
    input [4:0] Read1,Read2,WriteReg; //the register numbers to read or write
    input [4:0] Read1_2, Read2_2, WriteReg_2;
    input [31:0] WriteData; //data to write
    input [31:0] WriteData_2;
    input RegWrite, //the write control
          RegWrite_2,
          clock;  // the clock to trigger write
    output logic [31:0] Data1, Data2; // the register values read
    output logic [31:0] Data1_2, Data2_2;
    
    logic [31:0] RF [31:0]; //32 registers each 32 bits long
    
    //assign Data1 = RF[Read1];
    //assign Data2 = RF[Read2];
    always_comb
    begin
    // S 1
        if(Read1==0) Data1 =0;
        else Data1 = RF[Read1];
    // S 2
        if(Read1_2==0) Data1_2 =0;
        else Data1_2 = RF[Read1_2];
    end
    always_comb
    begin
        if(Read2==0) Data2 =0;
        else Data2 = RF[Read2];
        
        if(Read2_2==0) Data2_2 =0;
        else Data2_2 = RF[Read2_2];
    end
    always@(negedge clock) begin // write the register with the new value if Regwrite is high
        if(RegWrite && WriteReg!=0) RF[WriteReg] <= WriteData;
        if(RegWrite_2 && WriteReg_2 !=0) RF[WriteReg_2] <= WriteData_2;
    end
 endmodule

