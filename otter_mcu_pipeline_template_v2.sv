`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  Filippo Cheein
// 
// Create Date: 01/04/2019 04:32:12 PM
// Design Name: 
// Module Name: PIPELINED_OTTER_CPU
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

  typedef enum logic [6:0] {
           LUI      = 7'b0110111,
           AUIPC    = 7'b0010111,
           JAL      = 7'b1101111,
           JALR     = 7'b1100111,
           BRANCH   = 7'b1100011,
           LOAD     = 7'b0000011,
           STORE    = 7'b0100011,
           OP_IMM   = 7'b0010011,
           OP       = 7'b0110011,
           SYSTEM   = 7'b1110011
 } opcode_t;
        
typedef struct packed{
    opcode_t opcode, opcode_2;
    logic [4:0] rs1_addr, rs1_addr_2;
    logic [4:0] rs2_addr, rs2_addr_2;
    logic [4:0] rd_addr, rd_addr_2;
    
    logic rs1_used;
    logic rs2_used;
    logic rd_used;
    logic [3:0] alu_fun, alu_fun_2;
    logic memWrite, memWrite_2;
    logic memRead1, memRead1_2;
    logic memRead2, memRead2_2;
    logic regWrite, regWrite_2;
    logic [1:0] rf_wr_sel, rf_wr_sel_2;
    logic [2:0] mem_type;  //sign, size
    
    logic [31:0] pc;
    logic [31:0] pc_2;
    logic [31:0] next_pc, next_pc_2;
    logic pcWrite;
    logic [3:0]  pc_sel, pc_sel_2;
    
    logic [31:0] rs1, rs1_2;
    logic [31:0] rs2, rs2_2;
    logic [31:0] aluRes, aluRes_2;
    
    
    
    logic [31:0] IR, IR_2;
    logic [31:0] I_immed, I_immed_2;
} instr_t;

module OTTER_MCU(input CLK,
                input INTR,
                input RESET,
                input [31:0] IOBUS_IN,
                output [31:0] IOBUS_OUT,
                output [31:0] IOBUS_ADDR,
                output logic IOBUS_WR 
);           
    wire [6:0] opcode;
    wire [31:0] pc, pc_2, pc_value, next_pc, jalr_pc, branch_pc, jump_pc, int_pc,A,B, A_2, B_2,
        I_immed,S_immed,U_immed, I_immed_2,S_immed_2,U_immed_2, aluBin,aluAin,aluResult, aluBin_2,aluAin_2,aluResult_2,
        rfIn, rfIn_2, csr_reg, mem_data, mem_data_2;
    
    wire [31:0] IR, IR_2;
    wire memRead1,memRead2, memRead1_2,memRead2_2;
    
    wire pcWrite,regWrite,memWrite, op1_sel,mem_op,IorD,pcWriteCond,memRead;
    wire regWrite_2, memWrite_2;
    wire [1:0] opB_sel, opB_sel_2, rf_wr_sel, rf_wr_sel_2, wb_sel, mSize;
    logic [3:0] pc_sel, ex_pc_sel, pc_sel_2;
    wire [3:0]alu_fun, alu_fun_2;
    wire opA_sel, opA_sel_2;
    
    wire mepcWrite, csrWrite,intCLR, mie, intTaken;
    wire [31:0] mepc, mtvec;
    
    logic br_lt, br_eq, br_ltu;
    logic br_lt_2, br_eq_2, br_ltu_2;
    
    logic [31:0] ex_jalr_pc, ex_branch_pc, ex_jump_pc, ex_next_pc;
    logic [31:0] ex_jalr_pc_2, ex_branch_pc_2, ex_jump_pc_2, ex_next_pc_2;
    logic  [31:0] wb_rfIn, wb_rfIn_2;
    logic wb_regWrite, wb_regWrite_2;
    logic [4:0] wb_rd_addr, wb_rd_addr_2;
    
//==== Instruction Fetch ===========================================
     instr_t if_de_inst;
    
    // assign pcWrite = 1'b1; 	//Hardwired high, assuming now hazards
    // assign memRead1 = 1'b1; 	//Fetch new instruction every cycle
     assign next_pc = pc + 8;
     
     // assign if_de_inst.pc = pc;
     // assign if_de_inst.next_pc = next_pc;
     
     always_ff @(posedge CLK) begin
                // write
               if_de_inst.pc <= pc;
               if_de_inst.pc_2 <= pc + 4;
               if_de_inst.next_pc <= next_pc;
               if_de_inst.next_pc_2 <= next_pc + 4;
     end
     
    // Creates a 2-to-1 multiplexor used to select the source of the next PC
     Mult7to1 PCdatasrc (next_pc, ex_jalr_pc, ex_branch_pc, ex_jump_pc, ex_jalr_pc_2, ex_branch_pc_2, ex_jump_pc_2, ex_pc_sel, pc_value);    

    ProgCount PC (.PC_CLK(CLK), .PC_RST(RESET), .PC_LD(de_ex_inst.pcWrite),
             .PC_DIN(pc_value), .PC_COUNT(pc)); 
    
                  
//==== Instruction Decode ===========================================
    logic de_br_lt, de_br_eq, de_br_ltu;
    // output
    logic [31:0] de_ex_opA, de_ex_opB;
    logic [31:0] de_ex_opA_2, de_ex_opB_2;
    
    logic [31:0] de_ex_I_immed;
    logic [31:0] de_ex_I_immed_2;
    
    instr_t de_ex_inst, de_inst;
    
    opcode_t OPCODE, OPCODE_2;
    assign OPCODE = opcode_t'(IR[6:0]);
    assign OPCODE_2 = opcode_t'(IR_2[6:0]);
    
    // Writing to de_ex register from if_de register
    assign de_inst.pc = if_de_inst.pc;
    assign de_inst.next_pc = if_de_inst.next_pc;
    
    assign de_inst.pc_2 = if_de_inst.pc_2;
    assign de_inst.next_pc_2 = if_de_inst.next_pc_2;
    
    assign de_inst.IR = IR;
    assign de_inst.rs1_addr = IR[19:15];
    assign de_inst.rs2_addr = IR[24:20];
    assign de_inst.rd_addr  = IR[11:7];
    assign de_inst.opcode   = OPCODE;
    
    assign de_inst.IR_2 = IR_2;
    assign de_inst.rs1_addr_2 = IR_2[19:15];
    assign de_inst.rs2_addr_2 = IR_2[24:20];
    assign de_inst.rd_addr_2  = IR_2[11:7];
    assign de_inst.opcode_2   = OPCODE_2;
   
    assign de_inst.rs1_used=    de_inst.rs1 != 0
                                && de_inst.opcode != LUI
                                && de_inst.opcode != AUIPC
                                && de_inst.opcode != JAL;
    
    
    //assign de_inst.next_pc = if_de_inst.next_pc; 

    always_ff @(posedge CLK)
    begin
 
        de_ex_inst    <= de_inst;
        de_ex_inst.pcWrite <= pcWrite;
        de_ex_inst.memRead1 <= memRead1;
        
        // write in de_ex register
        de_ex_opA <= aluAin;
        de_ex_opB <= aluBin;
        de_ex_inst.rs1 <= A;
        de_ex_inst.rs2 <= B;
        
        de_ex_opA_2 <= aluAin_2;
        de_ex_opB_2 <= aluBin_2;
        de_ex_inst.rs1_2 <= A_2;
        de_ex_inst.rs2_2 <= B_2;
        
        de_ex_inst.I_immed <= I_immed;
        de_ex_inst.I_immed_2 <= I_immed_2;
        
        // write to de_ex register
        de_ex_inst.regWrite  <= regWrite;
        de_ex_inst.memWrite  <= memWrite;
        de_ex_inst.memRead2  <= memRead2;
        de_ex_inst.alu_fun   <= alu_fun;
        de_ex_inst.rf_wr_sel <= rf_wr_sel;
        de_ex_inst.pc_sel    <= pc_sel;
        de_ex_inst.pc_sel_2    <= pc_sel_2;
        
        de_ex_inst.regWrite_2  <= regWrite_2;
        de_ex_inst.memWrite_2  <= memWrite_2;
        de_ex_inst.memRead2_2  <= memRead2_2;
        de_ex_inst.alu_fun_2   <= alu_fun_2;
        de_ex_inst.rf_wr_sel_2 <= rf_wr_sel_2;
 
    end
	
	// Branch Gen 1 
     always_comb
     begin
         br_lt = 0; br_eq = 0; br_ltu = 0;
         if($signed(A) < $signed(B)) br_lt=1;
         if(A == B) br_eq=1;
         if(A < B) br_ltu=1;
     end
     
     // Branch Gen 2 
      always_comb
      begin
          br_lt_2 = 0; br_eq_2 = 0; br_ltu_2 = 0;
          if($signed(A_2) < $signed(B_2)) br_lt_2=1;
          if(A_2 == B_2) br_eq_2=1;
          if(A_2 < B_2) br_ltu_2=1;
      end
     
	//  Generate immediates - 1
     assign S_immed = {{20{de_inst.IR[31]}},de_inst.IR[31:25],de_inst.IR[11:7]};
     assign I_immed = {{20{de_inst.IR[31]}},de_inst.IR[31:20]};
     assign U_immed = {de_inst.IR[31:12],{12{1'b0}}};
     
     //  Generate immediates - 2 
     assign S_immed_2 = {{20{de_inst.IR_2[31]}},de_inst.IR_2[31:25],de_inst.IR_2[11:7]};
     assign I_immed_2 = {{20{de_inst.IR_2[31]}},de_inst.IR_2[31:20]};
     assign U_immed_2 = {de_inst.IR_2[31:12],{12{1'b0}}};
     
     OTTER_CU_DECODER_PIPE CU_DECODER_PIPE(.CU_OPCODE(de_inst.opcode), .CU_FUNC3(de_inst.IR[14:12]),.CU_FUNC7(de_inst.IR[31:25]), 
                 .CU_BR_EQ(br_eq),.CU_BR_LT(br_lt),.CU_BR_LTU(br_ltu),.CU_PCSOURCE(pc_sel),
                 .CU_ALU_SRCA(opA_sel),.CU_ALU_SRCB(opB_sel),.CU_ALU_FUN(alu_fun),.CU_RF_WR_SEL(rf_wr_sel),.intTaken(intTaken),
                 .CU_PCWRITE(pcWrite), .CU_REGWRITE(regWrite), .CU_MEMWRITE(memWrite), .CU_MEMREAD1(memRead1), .CU_MEMREAD2(memRead2));
     
     OTTER_CU_DECODER_PIPE CU_DECODER_PIPE_2(.CU_OPCODE(de_inst.opcode_2), .CU_FUNC3(de_inst.IR_2[14:12]),.CU_FUNC7(de_inst.IR_2[31:25]), 
                                 .CU_BR_EQ(br_lt_2),.CU_BR_LT(br_lt_2),.CU_BR_LTU(br_ltu_2),.CU_PCSOURCE(pc_sel_2),
                                 .CU_ALU_SRCA(opA_sel_2),.CU_ALU_SRCB(opB_sel_2),.CU_ALU_FUN(alu_fun_2),.CU_RF_WR_SEL(rf_wr_sel_2),.intTaken(intTaken),
                                 .CU_PCWRITE(), .CU_REGWRITE(regWrite_2), .CU_MEMWRITE(memWrite_2), .CU_MEMREAD1(memRead1_2), .CU_MEMREAD2(memRead2_2));

     
     logic prev_INT=0;

     OTTER_registerFile RF (.Read1(de_inst.rs1_addr), .Read2(de_inst.rs2_addr), 
                            .WriteReg(wb_rd_addr), .WriteData(wb_rfIn), .RegWrite(wb_regWrite), 
                            .Data1(A), .Data2(B), .clock(CLK),
                            .Read1_2(de_inst.rs1_addr_2) , .Read2_2(de_inst.rs2_addr_2), 
                            .WriteReg_2(wb_rd_addr_2), .WriteData_2(wb_rfIn_2), .RegWrite_2(wb_regWrite_2),
                            .Data1_2(A_2), .Data2_2(B_2) ); // Register file

     // ALU muxes for - S_1
     Mult2to1 ALUAinput (A, U_immed, opA_sel, aluAin);
     Mult4to1 ALUBinput (B, I_immed, S_immed, de_inst.pc, opB_sel, aluBin);
    
     // ALU muxes for - S_1
     Mult2to1 ALUAinput_2 (A_2, U_immed_2, opA_sel_2, aluAin_2);
     Mult4to1 ALUBinput_2 (B_2, I_immed_2, S_immed_2, de_inst.pc_2, opB_sel_2, aluBin_2);
    
//==== Execute ======================================================
     
     logic [31:0] ex_opA_forwarded;
     logic [31:0] ex_opB_forwarded;
     logic [31:0] ex_opA, ex_opB, ex_opA_2, ex_opB_2;
     logic [31:0] ex_I_immed, ex_I_immed_2; 
    
     logic [31:0] ex_mem_aluRes = 0;
     instr_t ex_mem_inst, ex_inst;
     
     assign ex_inst = de_ex_inst;
  //   assign ex_pc_sel = de_ex_inst.pc_sel;
     assign ex_I_immed = de_ex_inst.I_immed;
     assign ex_I_immed_2 = de_ex_inst.I_immed_2;
     assign ex_opA = de_ex_opA;
     assign ex_opB = de_ex_opB;
     
     assign ex_opA_2 = de_ex_opA_2;
     assign ex_opB_2 = de_ex_opB_2;
    // assign ex_inst.aluRes = aluResult;
     
     always_comb
     begin
     if(de_ex_inst.pc_sel_2 == 0)
       ex_pc_sel = de_ex_inst.pc_sel;
        
     else if((de_ex_inst.pc_sel == 0) && (de_ex_inst.pc_sel_2 != 0))
        ex_pc_sel = 3 + de_ex_inst.pc_sel_2;
        
     else
        ex_pc_sel = 0;
     end
    
     always_ff @(posedge CLK)
     begin       
        
        ex_mem_inst <= ex_inst;
        ex_mem_inst.aluRes <= aluResult;
        ex_mem_inst.aluRes_2 <= aluResult_2;
        
     end
     
    // Target Gen 1
    //pc target calculations 
    assign ex_jalr_pc = ex_I_immed + ex_inst.rs1;
    assign ex_branch_pc = ex_inst.pc + {{20{ex_inst.IR[31]}},ex_inst.IR[7],ex_inst.IR[30:25],ex_inst.IR[11:8],1'b0};   //byte aligned addresses
    assign ex_jump_pc = de_inst.pc + {{12{ex_inst.IR[31]}}, ex_inst.IR[19:12], ex_inst.IR[20], ex_inst.IR[30:21],1'b0};
    assign int_pc = 0;
     
    // Target Gen 2
    //pc target calculations 
    assign ex_jalr_pc_2 = ex_I_immed_2 + ex_inst.rs1_2;
    assign ex_branch_pc_2 = ex_inst.pc_2 + {{20{ex_inst.IR_2[31]}},ex_inst.IR_2[7],ex_inst.IR_2[30:25],ex_inst.IR_2[11:8],1'b0};   //byte aligned addresses
    assign ex_jump_pc_2 = de_inst.pc_2 + {{12{ex_inst.IR_2[31]}}, ex_inst.IR_2[19:12], ex_inst.IR_2[20], ex_inst.IR_2[30:21],1'b0};
    assign int_pc = 0;
     
    // Creates a RISC-V ALU
     OTTER_ALU ALU (ex_inst.alu_fun, ex_opA, ex_opB, aluResult);
     OTTER_ALU ALU_2 (ex_inst.alu_fun_2, ex_opA_2, ex_opB_2, aluResult_2);
     
     always_ff @ (posedge CLK)
     begin
          if(INTR && mie)
             prev_INT=1'b1;
          if(intCLR || RESET)
             prev_INT=1'b0;
     end

//==== Memory ======================================================
      logic [31:0] mem_rs2, mem_rs2_2;
      logic [31:0] mem_aluRes, mem_aluRes_2;
      instr_t mem_inst, mem_wb_inst;
      logic [31:0] mem_wb_dout2, mem_wb_dout2_2;
      
      assign mem_inst = ex_mem_inst;
      
      assign mem_rs2 = mem_inst.rs2;
      assign mem_aluRes = ex_mem_inst.aluRes;
      
      assign mem_rs2_2 = mem_inst.rs2_2;
      assign mem_aluRes_2 = ex_mem_inst.aluRes_2;
      
     always_ff @(posedge CLK)
     begin
          
         mem_wb_dout2 <= mem_data;
         mem_wb_dout2_2 <= mem_data_2;
         
         mem_wb_inst <= mem_inst;
     end
         
        assign IOBUS_ADDR = mem_inst.aluRes;
        assign IOBUS_OUT = mem_rs2;
        
     
     OTTER_mem_byte #(14) memory  (.MEM_CLK(CLK),.MEM_ADDR1(pc), .MEM_ADDR1_2(pc+4), 
                                       .MEM_ADDR2(mem_aluRes), .MEM_ADDR2_2(mem_aluRes_2),
                                       .MEM_DIN2(mem_rs2), .MEM_DIN2_2(mem_rs2_2),
                                       .MEM_WRITE2(mem_inst.memWrite), .MEM_WRITE2_2(mem_inst.memWrite_2),
                                       .MEM_READ1(de_ex_inst.memRead1),
                                       .MEM_READ2(mem_inst.memRead2), .MEM_READ2_2(mem_inst.memRead2_2),
                                       .ERR(),.MEM_DOUT1(IR), .MEM_DOUT1_2(IR_2),
                                       .MEM_DOUT2(mem_data), .MEM_DOUT2_2(mem_data_2), 
                                       .IO_IN(IOBUS_IN), .IO_IN_2(),
                                       .IO_WR(IOBUS_WR), .IO_WR_2(),
                                       .MEM_SIZE(mem_inst.IR[14:12]),.MEM_SIZE_2(mem_inst.IR_2[14:12]),
                                       .MEM_SIGN(mem_inst.IR[14]), .MEM_SIGN_2(mem_inst.IR_2[14])
                                       );
                                        
     
//==== Write Back ==================================================
     logic [31:0] wb_dout2, wb_dout2_2;
     logic [31:0] wb_aluResult, wb_aluResult_2;
     logic [1:0] wb_rf_wr_sel, wb_rf_wr_sel_2;
     logic [31:0] wb_next_pc, wb_next_pc_2;
     
     assign wb_dout2 = mem_wb_dout2;
     assign wb_regWrite = mem_wb_inst.regWrite;
     assign wb_aluResult = mem_wb_inst.aluRes;
     assign wb_rf_wr_sel = mem_wb_inst.rf_wr_sel;
     assign wb_next_pc = mem_wb_inst.next_pc;
     assign wb_rd_addr = mem_wb_inst.rd_addr;
     assign wb_rfIn = rfIn;
 
     assign wb_dout2_2 = mem_wb_dout2_2;
     assign wb_regWrite_2 = mem_wb_inst.regWrite_2;
     assign wb_aluResult_2 = mem_wb_inst.aluRes_2;
     assign wb_rf_wr_sel_2 = mem_wb_inst.rf_wr_sel_2;
     assign wb_next_pc_2 = mem_wb_inst.next_pc_2;
     assign wb_rd_addr_2 = mem_wb_inst.rd_addr_2;
     assign wb_rfIn_2 = rfIn_2;
         
     //Creates 4-to-1 multiplexor used to select reg write back data
    Mult4to1 regWriteback (wb_next_pc, csr_reg, wb_dout2 , wb_aluResult, wb_rf_wr_sel, rfIn); 
    Mult4to1 regWriteback_2 (wb_next_pc_2, csr_reg, wb_dout2_2 , wb_aluResult_2, wb_rf_wr_sel_2, rfIn_2);     
    
    
    //****do not touch****//
     //CSR registers and interrupt logic
         CSR CSRs(.clk(CLK),.rst(RESET),.intTaken(intTaken),.addr(IR[31:20]),.next_pc(pc),.wd(aluResult),.wr_en(csrWrite),
           .rd(csr_reg),.mepc(mepc),.mtvec(mtvec),.mie(mie));
    // ******************* //    
                
endmodule
