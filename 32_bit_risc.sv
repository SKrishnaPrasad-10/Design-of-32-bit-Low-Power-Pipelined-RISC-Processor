// Code your design here
// Code your design here
// Code your design here
// Code your design here
// Code your design here
// Wire widths
`define WORD_LEN 32
`define REG_FILE_ADDR_LEN 5
`define EXE_CMD_LEN 4
`define FORW_SEL_LEN 2
`define OP_CODE_LEN 6

// Memory constants
`define DATA_MEM_SIZE 1024
`define INSTR_MEM_SIZE 1024
`define REG_FILE_SIZE 32
`define MEM_CELL_SIZE 8

// To be used inside controller.v
`define OP_NOP 6'b000000
`define OP_ADD 6'b000001
`define OP_SUB 6'b000011
`define OP_AND 6'b000101
`define OP_OR 6'b000110
`define OP_NOR 6'b000111
`define OP_XOR 6'b001000
`define OP_SLA 6'b001001
`define OP_SLL 6'b001010
`define OP_SRA 6'b001011
`define OP_SRL 6'b001100
`define OP_ADDI 6'b100000
`define OP_SUBI 6'b100001
`define OP_LD 6'b100100
`define OP_ST 6'b100101
`define OP_BEZ 6'b101000
`define OP_BNE 6'b101001
`define OP_JMP 6'b101010

// To be used in side ALU
`define EXE_ADD 4'b0000
`define EXE_SUB 4'b0010
`define EXE_AND 4'b0100
`define EXE_OR 4'b0101
`define EXE_NOR 4'b0110
`define EXE_XOR 4'b0111
`define EXE_SLA 4'b1000
`define EXE_SLL 4'b1000
`define EXE_SRA 4'b1001
`define EXE_SRL 4'b1010
`define EXE_NO_OPERATION 4'b1111 // for NOP, BEZ, BNQ, JMP

// To be used in conditionChecker
`define COND_JUMP 2'b10
`define COND_BEZ 2'b11
`define COND_BNE 2'b01
`define COND_NOTHING 2'b00

//`include "defines.v"

module MIPS_Processor (input CLOCK_50, input rst, input forward_EN);
	wire clock = CLOCK_50;
	wire [`WORD_LEN-1:0] PC_IF, PC_ID, PC_EXE, PC_MEM;
	wire [`WORD_LEN-1:0] inst_IF, inst_ID;
	wire [`WORD_LEN-1:0] reg1_ID, reg2_ID, ST_value_EXE, ST_value_EXE2MEM, ST_value_MEM;
	wire [`WORD_LEN-1:0] val1_ID, val1_EXE;
	wire [`WORD_LEN-1:0] val2_ID, val2_EXE;
	wire [`WORD_LEN-1:0] ALURes_EXE, ALURes_MEM, ALURes_WB;
	wire [`WORD_LEN-1:0] dataMem_out_MEM, dataMem_out_WB;
	wire [`WORD_LEN-1:0] WB_result;
	wire [`REG_FILE_ADDR_LEN-1:0] dest_EXE, dest_MEM, dest_WB; // dest_ID = instruction[25:21] thus nothing declared
	wire [`REG_FILE_ADDR_LEN-1:0] src1_ID, src2_regFile_ID, src2_forw_ID, src2_forw_EXE, src1_forw_EXE;
	wire [`EXE_CMD_LEN-1:0] EXE_CMD_ID, EXE_CMD_EXE;
	wire [`FORW_SEL_LEN-1:0] val1_sel, val2_sel, ST_val_sel;
	wire [1:0] branch_comm;
	wire Br_Taken_ID, IF_Flush, Br_Taken_EXE;
	wire MEM_R_EN_ID, MEM_R_EN_EXE, MEM_R_EN_MEM, MEM_R_EN_WB;
	wire MEM_W_EN_ID, MEM_W_EN_EXE, MEM_W_EN_MEM;
	wire WB_EN_ID, WB_EN_EXE, WB_EN_MEM, WB_EN_WB;
	wire hazard_detected, is_imm, ST_or_BNE;

	regFile regFile(
		// INPUTS
		.clk(clock),
		.rst(rst),
		.src1(src1_ID),
		.src2(src2_regFile_ID),
		.dest(dest_WB),
		.writeVal(WB_result),
		.writeEn(WB_EN_WB),
		// OUTPUTS
		.reg1(reg1_ID),
		.reg2(reg2_ID)
	);

	hazard_detection hazard (
		// INPUTS
		.forward_EN(forward_EN),
		.is_imm(is_imm),
		.ST_or_BNE(ST_or_BNE),
		.src1_ID(src1_ID),
		.src2_ID(src2_regFile_ID),
		.dest_EXE(dest_EXE),
		.dest_MEM(dest_MEM),
		.WB_EN_EXE(WB_EN_EXE),
		.WB_EN_MEM(WB_EN_MEM),
		.MEM_R_EN_EXE(MEM_R_EN_EXE),
		// OUTPUTS
		.branch_comm(branch_comm),
		.hazard_detected(hazard_detected)
	);

	forwarding_EXE forwrding_EXE (
		.src1_EXE(src1_forw_EXE),
		.src2_EXE(src2_forw_EXE),
		.ST_src_EXE(dest_EXE),
		.dest_MEM(dest_MEM),
		.dest_WB(dest_WB),
		.WB_EN_MEM(WB_EN_MEM),
		.WB_EN_WB(WB_EN_WB),
		.val1_sel(val1_sel),
		.val2_sel(val2_sel),
		.ST_val_sel(ST_val_sel)
	);

	//###########################
	//##### PIPLINE STAGES ######
	//###########################
	IFStage IFStage (
		// INPUTS
		.clk(clock),
		.rst(rst),
		.freeze(hazard_detected),
		.brTaken(Br_Taken_ID),
		.brOffset(val2_ID),
		// OUTPUTS
		.instruction(inst_IF),
		.PC(PC_IF)
	);

	IDStage IDStage (
		// INPUTS
		.clk(clock),
		.rst(rst),
		.hazard_detected_in(hazard_detected),
		.instruction(inst_ID),
		.reg1(reg1_ID),
		.reg2(reg2_ID),
		// OUTPUTS
		.src1(src1_ID),
		.src2_reg_file(src2_regFile_ID),
		.src2_forw(src2_forw_ID),
		.val1(val1_ID),
		.val2(val2_ID),
		.brTaken(Br_Taken_ID),
		.EXE_CMD(EXE_CMD_ID),
		.MEM_R_EN(MEM_R_EN_ID),
		.MEM_W_EN(MEM_W_EN_ID),
		.WB_EN(WB_EN_ID),
		.is_imm_out(is_imm),
		.ST_or_BNE_out(ST_or_BNE),
		.branch_comm(branch_comm)
	);

	EXEStage EXEStage (
		// INPUTS
		.clk(clock),
		.EXE_CMD(EXE_CMD_EXE),
		.val1_sel(val1_sel),
		.val2_sel(val2_sel),
		.ST_val_sel(ST_val_sel),
		.val1(val1_EXE),
		.val2(val2_EXE),
		.ALU_res_MEM(ALURes_MEM),
		.result_WB(WB_result),
		.ST_value_in(ST_value_EXE),
		// OUTPUTS
		.ALUResult(ALURes_EXE),
		.ST_value_out(ST_value_EXE2MEM)
	);

	MEMStage MEMStage (
		// INPUTS
		.clk(clock),
		.rst(rst),
		.MEM_R_EN(MEM_R_EN_MEM),
		.MEM_W_EN(MEM_W_EN_MEM),
		.ALU_res(ALURes_MEM),
		.ST_value(ST_value_MEM),
		// OUTPUTS
		.dataMem_out(dataMem_out_MEM)
	);

	WBStage WBStage (
		// INPUTS
		.MEM_R_EN(MEM_R_EN_WB),
		.memData(dataMem_out_WB),
		.aluRes(ALURes_WB),
		// OUTPUTS
		.WB_res(WB_result)
	);

	//############################
	//#### PIPLINE REGISTERS #####
	//############################
	IF2ID IF2IDReg (
		// INPUTS
		.clk(clock),
		.rst(rst),
		.flush(IF_Flush),
		.freeze(hazard_detected),
		.PCIn(PC_IF),
		.instructionIn(inst_IF),
		// OUTPUTS
		.PC(PC_ID),
		.instruction(inst_ID)
	);

	ID2EXE ID2EXEReg (
		.clk(clock),
		.rst(rst),
		// INPUTS
		.destIn(inst_ID[25:21]),
		.src1_in(src1_ID),
		.src2_in(src2_forw_ID),
		.reg2In(reg2_ID),
		.val1In(val1_ID),
		.val2In(val2_ID),
		.PCIn(PC_ID),
		.EXE_CMD_IN(EXE_CMD_ID),
		.MEM_R_EN_IN(MEM_R_EN_ID),
		.MEM_W_EN_IN(MEM_W_EN_ID),
		.WB_EN_IN(WB_EN_ID),
		.brTaken_in(Br_Taken_ID),
		// OUTPUTS
		.src1_out(src1_forw_EXE),
		.src2_out(src2_forw_EXE),
		.dest(dest_EXE),
		.ST_value(ST_value_EXE),
		.val1(val1_EXE),
		.val2(val2_EXE),
		.PC(PC_EXE),
		.EXE_CMD(EXE_CMD_EXE),
		.MEM_R_EN(MEM_R_EN_EXE),
		.MEM_W_EN(MEM_W_EN_EXE),
		.WB_EN(WB_EN_EXE),
		.brTaken_out(Br_Taken_EXE)
	);

	EXE2MEM EXE2MEMReg (
		.clk(clock),
		.rst(rst),
		// INPUTS
		.WB_EN_IN(WB_EN_EXE),
		.MEM_R_EN_IN(MEM_R_EN_EXE),
		.MEM_W_EN_IN(MEM_W_EN_EXE),
		.PCIn(PC_EXE),
		.ALUResIn(ALURes_EXE),
		.STValIn(ST_value_EXE2MEM),
		.destIn(dest_EXE),
		// OUTPUTS
		.WB_EN(WB_EN_MEM),
		.MEM_R_EN(MEM_R_EN_MEM),
		.MEM_W_EN(MEM_W_EN_MEM),
		.PC(PC_MEM),
		.ALURes(ALURes_MEM),
		.STVal(ST_value_MEM),
		.dest(dest_MEM)
	);

	MEM2WB MEM2WB(
		.clk(clock),
		.rst(rst),
		// INPUTS
		.WB_EN_IN(WB_EN_MEM),
		.MEM_R_EN_IN(MEM_R_EN_MEM),
		.ALUResIn(ALURes_MEM),
		.memReadValIn(dataMem_out_MEM),
		.destIn(dest_MEM),
		// OUTPUTS
		.WB_EN(WB_EN_WB),
		.MEM_R_EN(MEM_R_EN_WB),
		.ALURes(ALURes_WB),
		.memReadVal(dataMem_out_WB),
		.dest(dest_WB)
	);

	assign IF_Flush = Br_Taken_ID;
endmodule

//IF Stage

module IFStage (clk, rst, brTaken, brOffset, freeze, PC, instruction);
  input clk, rst, brTaken, freeze;
  input [`WORD_LEN-1:0] brOffset;
  output [`WORD_LEN-1:0] PC, instruction;

  wire [`WORD_LEN-1:0] adderIn1, adderOut, brOffserTimes4;

  mux #(.LENGTH(`WORD_LEN)) adderInput (
    .in1(32'd4),
    .in2(brOffserTimes4),
    .sel(brTaken),
    .out(adderIn1)
  );

  adder add4 (
    .in1(adderIn1),
    .in2(PC),
    .out(adderOut)
  );

  register PCReg (
    .clk(clk),
    .rst(rst),
    .writeEn(~freeze),
    .regIn(adderOut),
    .regOut(PC)
  );

  instructionMem instructions (
    .rst(rst),
    .addr(PC),
    .instruction(instruction)
  );

  assign brOffserTimes4 = brOffset << 2;
endmodule // IFStage


//ID Stage

module IDStage (clk, rst, hazard_detected_in, is_imm_out, ST_or_BNE_out, instruction, reg1, reg2, src1, src2_reg_file, src2_forw, val1, val2, brTaken, EXE_CMD, MEM_R_EN, MEM_W_EN, WB_EN, branch_comm);
  input clk, rst, hazard_detected_in;
  input [`WORD_LEN-1:0] instruction, reg1, reg2;
  output brTaken, MEM_R_EN, MEM_W_EN, WB_EN, is_imm_out, ST_or_BNE_out;
  output [1:0] branch_comm;
  output [`EXE_CMD_LEN-1:0] EXE_CMD;
  output [`REG_FILE_ADDR_LEN-1:0] src1, src2_reg_file, src2_forw;
  output [`WORD_LEN-1:0] val1, val2;

  wire CU2and, Cond2and;
  wire [1:0] CU2Cond;
  wire Is_Imm, ST_or_BNE;
  wire [`WORD_LEN-1:0] signExt2Mux;

  controller controller(
    // INPUT
    .opCode(instruction[31:26]),
    .branchEn(CU2and),
    // OUTPUT
    .EXE_CMD(EXE_CMD),
    .Branch_command(CU2Cond),
    .Is_Imm(Is_Imm),
    .ST_or_BNE(ST_or_BNE),
    .WB_EN(WB_EN),
    .MEM_R_EN(MEM_R_EN),
    .MEM_W_EN(MEM_W_EN),
    .hazard_detected(hazard_detected_in)
  );

  mux #(.LENGTH(`REG_FILE_ADDR_LEN)) mux_src2 ( // determins the register source 2 for register file
    .in1(instruction[15:11]),
    .in2(instruction[25:21]),
    .sel(ST_or_BNE),
    .out(src2_reg_file)
  );

  mux #(.LENGTH(`WORD_LEN)) mux_val2 ( // determins whether val2 is from the reg file or the immediate value
    .in1(reg2),
    .in2(signExt2Mux),
    .sel(Is_Imm),
    .out(val2)
  );

  mux #(.LENGTH(`REG_FILE_ADDR_LEN)) mux_src2_forw ( // determins the value of register source 2 for forwarding
    .in1(instruction[15:11]), // src2 = instruction[15:11]
    .in2(5'd0),
    .sel(Is_Imm),
    .out(src2_forw)
  );

  signExtend signExtend(
    .in(instruction[15:0]),
    .out(signExt2Mux)
  );

  conditionChecker conditionChecker (
    .reg1(reg1),
    .reg2(reg2),
    .cuBranchComm(CU2Cond),
    .brCond(Cond2and)
  );

  assign brTaken = CU2and && Cond2and;
  assign val1 = reg1;
  assign src1 = instruction[20:16];
  assign is_imm_out = Is_Imm;
  assign ST_or_BNE_out = ST_or_BNE;
  assign branch_comm = CU2Cond;
endmodule // IDStage

//EX Stage

module EXEStage (clk, EXE_CMD, val1_sel, val2_sel, ST_val_sel, val1, val2, ALU_res_MEM, result_WB, ST_value_in, ALUResult, ST_value_out);
  input clk;
  input [`FORW_SEL_LEN-1:0] val1_sel, val2_sel, ST_val_sel;
  input [`EXE_CMD_LEN-1:0] EXE_CMD;
  input [`WORD_LEN-1:0] val1, val2, ALU_res_MEM, result_WB, ST_value_in;
  output [`WORD_LEN-1:0] ALUResult, ST_value_out;

  wire [`WORD_LEN-1:0] ALU_val1, ALU_val2;

  mux_3input #(.LENGTH(`WORD_LEN)) mux_val1 (
    .in1(val1),
    .in2(ALU_res_MEM),
    .in3(result_WB),
    .sel(val1_sel),
    .out(ALU_val1)
  );

  mux_3input #(.LENGTH(`WORD_LEN)) mux_val2 (
    .in1(val2),
    .in2(ALU_res_MEM),
    .in3(result_WB),
    .sel(val2_sel),
    .out(ALU_val2)
  );

  mux_3input #(.LENGTH(`WORD_LEN)) mux_ST_value (
    .in1(ST_value_in),
    .in2(ALU_res_MEM),
    .in3(result_WB),
    .sel(ST_val_sel),
    .out(ST_value_out)
  );

  ALU ALU(
    .val1(ALU_val1),
    .val2(ALU_val2),
    .EXE_CMD(EXE_CMD),
    .aluOut(ALUResult)
  );
endmodule // EXEStage


//MEM Stage

module MEMStage (clk, rst, MEM_R_EN, MEM_W_EN, ALU_res, ST_value, dataMem_out);
  input clk, rst, MEM_R_EN, MEM_W_EN;
  input [`WORD_LEN-1:0] ALU_res, ST_value;
  output [`WORD_LEN-1:0]  dataMem_out;

  dataMem dataMem (
    .clk(clk),
    .rst(rst),
    .writeEn(MEM_W_EN),
    .readEn(MEM_R_EN),
    .address(ALU_res),
    .dataIn(ST_value),
    .dataOut(dataMem_out)
  );
endmodule // MEMStage


//WB Stage

module WBStage (MEM_R_EN, memData, aluRes, WB_res);
  input MEM_R_EN;
  input [`WORD_LEN-1:0] memData, aluRes;
  output [`WORD_LEN-1:0] WB_res;

  assign WB_res = (MEM_R_EN) ? memData : aluRes;
endmodule // WBStage

////Program_Counter_IF_Stage

module register (clk, rst, writeEn, regIn, regOut);
  input clk, rst, writeEn;
  input [`WORD_LEN-1:0] regIn;
  output reg [`WORD_LEN-1:0] regOut;
  wire clk_gate1;

  assign clk_gate1 = writeEn & clk;
  always @ (posedge clk_gate1) begin
    if (rst == 1) regOut <= 0;
    else  regOut <= regIn;
  end
   
endmodule // Program_Counter

///Adder_IF_Stage

module adder (in1, in2, out);
  input [`WORD_LEN-1:0] in1, in2;
  output [`WORD_LEN-1:0] out;

  assign out = in1 + in2;
endmodule // adder

//Instruction memory IF stage


module instructionMem (rst, addr, instruction);
  input rst;
  input [`WORD_LEN-1:0] addr;
  output [`WORD_LEN-1:0] instruction;

  wire [$clog2(`INSTR_MEM_SIZE)-1:0] address = addr[$clog2(`INSTR_MEM_SIZE)-1:0];
  reg [`MEM_CELL_SIZE-1:0] instMem [0:`INSTR_MEM_SIZE-1];

  always @ (*) begin
  	if (rst) begin
        // No nop added in between instructions since there is a hazard detection unit

        instMem[0] <= 8'b10000000; //-- Addi	r1,r0,10
        instMem[1] <= 8'b00100000;
        instMem[2] <= 8'b00000000;
        instMem[3] <= 8'b00001010;

        instMem[4] <= 8'b00000100; //-- Add 	r2,r0,r1
        instMem[5] <= 8'b01000000;
        instMem[6] <= 8'b00001000;
        instMem[7] <= 8'b00000000;

        instMem[8] <= 8'b00001100; //-- sub	r3,r0,r1
        instMem[9] <= 8'b01100000;
        instMem[10] <= 8'b00001000;
        instMem[11] <= 8'b00000000;

        instMem[12] <= 8'b00010100; //-- And	r4,r2,r3
        instMem[13] <= 8'b10000010;
        instMem[14] <= 8'b00011000;
        instMem[15] <= 8'b00000000;

        instMem[16] <= 8'b10000100; //-- Subi	r5,r0,564
        instMem[17] <= 8'b10100000;
        instMem[18] <= 8'b00000010;
        instMem[19] <= 8'b00110100;

        instMem[20] <= 8'b00011000; //-- or	r5,r5,r3
        instMem[21] <= 8'b10100101;
        instMem[22] <= 8'b00011000;
        instMem[23] <= 8'b00000000;

        instMem[24] <= 8'b00011100; //-- nor 	r6,r5,r0
        instMem[25] <= 8'b11000101;
        instMem[26] <= 8'b00000000;
        instMem[27] <= 8'b00000000;

        instMem[28] <= 8'b00100000; //-- xor	r0,r5,r1
        instMem[29] <= 8'b00000101;
        instMem[30] <= 8'b00001000;
        instMem[31] <= 8'b00000000;

        instMem[32] <= 8'b00100000; //-- xor	r7,r5,r0
        instMem[33] <= 8'b11100101;
        instMem[34] <= 8'b00001000;
        instMem[35] <= 8'b00000000;

        instMem[36] <= 8'b00100100; //-- sla	r7,r4,r2
        instMem[37] <= 8'b11100100;
        instMem[38] <= 8'b00010000;
        instMem[39] <= 8'b00000000;

        instMem[40] <= 8'b00101001; //-- sll	r8,r3,r2
        instMem[41] <= 8'b00000011;
        instMem[42] <= 8'b00010000;
        instMem[43] <= 8'b00000000;

        instMem[44] <= 8'b00101101; //-- sra	r9,r6,r2
        instMem[45] <= 8'b00100110;
        instMem[46] <= 8'b00010000;
        instMem[47] <= 8'b00000000;

        instMem[48] <= 8'b00110001; //-- srl	r10,r6,r2
        instMem[49] <= 8'b01000110;
        instMem[50] <= 8'b00010000;
        instMem[51] <= 8'b00000000;

        instMem[52] <= 8'b10000000; //-- Addi 	r1,r0,1024
        instMem[53] <= 8'b00100000;
        instMem[54] <= 8'b00000100;
        instMem[55] <= 8'b00000000;

        instMem[56] <= 8'b10010100; //-- st	r2,r1,0
        instMem[57] <= 8'b01000001;
        instMem[58] <= 8'b00000000;
        instMem[59] <= 8'b00000000;

        instMem[60] <= 8'b10010001; //-- ld	r11,r1,0
        instMem[61] <= 8'b01100001;
        instMem[62] <= 8'b00000000;
        instMem[63] <= 8'b00000000;

        instMem[64] <= 8'b10010100; //-- st	r3,r1,4
        instMem[65] <= 8'b01100001;
        instMem[66] <= 8'b00000000;
        instMem[67] <= 8'b00000100;

        instMem[68] <= 8'b10010100; //-- st	r4,r1,8
        instMem[69] <= 8'b10000001;
        instMem[70] <= 8'b00000000;
        instMem[71] <= 8'b00001000;

        instMem[72] <= 8'b10010100; //-- st	r5,r1,12
        instMem[73] <= 8'b10100001;
        instMem[74] <= 8'b00000000;
        instMem[75] <= 8'b00001100;

        instMem[76] <= 8'b10010100; //-- st	r6,r1,16
        instMem[77] <= 8'b11000001;
        instMem[78] <= 8'b00000000;
        instMem[79] <= 8'b00010000;

        instMem[80] <= 8'b10010100; //-- st	r7,r1,20
        instMem[81] <= 8'b11100001;
        instMem[82] <= 8'b00000000;
        instMem[83] <= 8'b00010100;

        instMem[84] <= 8'b10010101; //-- st	r8,r1,24
        instMem[85] <= 8'b00000001;
        instMem[86] <= 8'b00000000;
        instMem[87] <= 8'b00011000;

        instMem[88] <= 8'b10010101; //-- st	r9,r1,28
        instMem[89] <= 8'b00100001;
        instMem[90] <= 8'b00000000;
        instMem[91] <= 8'b00011100;

        instMem[92] <= 8'b10010101; //-- st	r10,r1,32
        instMem[93] <= 8'b01000001;
        instMem[94] <= 8'b00000000;
        instMem[95] <= 8'b00100000;

        instMem[96] <= 8'b10010101; //-- st	r11,r1,36
        instMem[97] <= 8'b01100001;
        instMem[98] <= 8'b00000000;
        instMem[99] <= 8'b00100100;

        instMem[100] <= 8'b10000000; //-- Addi 	r1,r0,3
        instMem[101] <= 8'b00100000;
        instMem[102] <= 8'b00000000;
        instMem[103] <= 8'b00000011;

        instMem[104] <= 8'b10000000; //-- Addi	r4,r0,1024
        instMem[105] <= 8'b10000000;
        instMem[106] <= 8'b00000100;
        instMem[107] <= 8'b00000000;

        instMem[108] <= 8'b10000000; //-- Addi 	r2,r0,0
        instMem[109] <= 8'b01000000;
        instMem[110] <= 8'b00000000;
        instMem[111] <= 8'b00000000;

        instMem[112] <= 8'b10000000; //-- Addi 	r3,r0,1
        instMem[113] <= 8'b01100000;
        instMem[114] <= 8'b00000000;
        instMem[115] <= 8'b00000001;

        instMem[116] <= 8'b10000001; //-- Addi 	r9,r0,2
        instMem[117] <= 8'b00100000;
        instMem[118] <= 8'b00000000;
        instMem[119] <= 8'b00000010;

        instMem[120] <= 8'b00101001; //-- sll	r8,r3,r9
        instMem[121] <= 8'b00000011;
        instMem[122] <= 8'b01001000;
        instMem[123] <= 8'b00000000;

        instMem[124] <= 8'b00000101; //-- Add 	r8,r4,r8
        instMem[125] <= 8'b00000100;
        instMem[126] <= 8'b01000000;
        instMem[127] <= 8'b00000000;

        instMem[128] <= 8'b10010000; //-- ld	r5,r8,0
        instMem[129] <= 8'b10101000;
        instMem[130] <= 8'b00000000;
        instMem[131] <= 8'b00000000;

        instMem[132] <= 8'b10010000; //-- ld	r6,r8,-4
        instMem[133] <= 8'b11001000;
        instMem[134] <= 8'b11111111;
        instMem[135] <= 8'b11111100;

        instMem[136] <= 8'b00001101; //-- sub 	r9,r5,r6
        instMem[137] <= 8'b00100101;
        instMem[138] <= 8'b00110000;
        instMem[139] <= 8'b00000000;

        instMem[140] <= 8'b10000001; //-- Addi 	r10,r0,0x8000
        instMem[141] <= 8'b01000000;
        instMem[142] <= 8'b10000000;
        instMem[143] <= 8'b00000000;

        instMem[144] <= 8'b10000001; //-- Addi	r11,r0,16
        instMem[145] <= 8'b01100000;
        instMem[146] <= 8'b00000000;
        instMem[147] <= 8'b00010000;

        instMem[148] <= 8'b00101001; //-- sll	r10,r10,r11
        instMem[149] <= 8'b01001010;
        instMem[150] <= 8'b01011000;
        instMem[151] <= 8'b00000000;

        instMem[152] <= 8'b00010101; //-- And 	r9,r9,r10
        instMem[153] <= 8'b00101001;
        instMem[154] <= 8'b01010000;
        instMem[155] <= 8'b00000000;

        instMem[156] <= 8'b10100000; //-- Bez	r9,2
        instMem[157] <= 8'b00001001;
        instMem[158] <= 8'b00000000;
        instMem[159] <= 8'b00000010;

        instMem[160] <= 8'b10010100; //-- st	r5,r8,-4
        instMem[161] <= 8'b10101000;
        instMem[162] <= 8'b11111111;
        instMem[163] <= 8'b11111100;

        instMem[164] <= 8'b10010100; //-- st	r6,r8,0
        instMem[165] <= 8'b11001000;
        instMem[166] <= 8'b00000000;
        instMem[167] <= 8'b00000000;

        instMem[168] <= 8'b10000000; //-- Addi 	r3,r3,1
        instMem[169] <= 8'b01100011;
        instMem[170] <= 8'b00000000;
        instMem[171] <= 8'b00000001;

        instMem[172] <= 8'b10100100; //-- BNE	r3,r1,-15
        instMem[173] <= 8'b01100001;
        instMem[174] <= 8'b11111111;
        instMem[175] <= 8'b11110001;

        instMem[176] <= 8'b10000000; //-- Addi 	r2,r2,1
        instMem[177] <= 8'b01000010;
        instMem[178] <= 8'b00000000;
        instMem[179] <= 8'b00000001;

        instMem[180] <= 8'b10100100; //-- BNE	r2,r1,-18
        instMem[181] <= 8'b01000001;
        instMem[182] <= 8'b11111111;
        instMem[183] <= 8'b11101110;

        instMem[184] <= 8'b10000000; //-- Addi 	r1,r0,1024
        instMem[185] <= 8'b00100000;
        instMem[186] <= 8'b00000100;
        instMem[187] <= 8'b00000000;

        instMem[188] <= 8'b10010000; //-- ld	r2,r1,0
        instMem[189] <= 8'b01000001;
        instMem[190] <= 8'b00000000;
        instMem[191] <= 8'b00000000;

        instMem[192] <= 8'b10010000; //-- ld	r3,r1,4
        instMem[193] <= 8'b01100001;
        instMem[194] <= 8'b00000000;
        instMem[195] <= 8'b00000100;

        instMem[196] <= 8'b10010000; //-- ld	r4,r1,8
        instMem[197] <= 8'b10000001;
        instMem[198] <= 8'b00000000;
        instMem[199] <= 8'b00001000;

        instMem[200] <= 8'b10010000; //-- ld	r5,r1,12
        instMem[201] <= 8'b10100001;
        instMem[202] <= 8'b00000000;
        instMem[203] <= 8'b00001100;

        instMem[204] <= 8'b10010000; //-- ld	r6,r1,16
        instMem[205] <= 8'b11000001;
        instMem[206] <= 8'b00000000;
        instMem[207] <= 8'b00010000;

        instMem[208] <= 8'b10010000; //-- ld	r7,r1,20
        instMem[209] <= 8'b11100001;
        instMem[210] <= 8'b00000000;
        instMem[211] <= 8'b00010100;

        instMem[212] <= 8'b10010001; //-- ld	r8,r1,24
        instMem[213] <= 8'b00000001;
        instMem[214] <= 8'b00000000;
        instMem[215] <= 8'b00011000;

        instMem[216] <= 8'b10010001; //-- ld	r9,r1,28
        instMem[217] <= 8'b00100001;
        instMem[218] <= 8'b00000000;
        instMem[219] <= 8'b00011100;

        instMem[220] <= 8'b10010001; //-- ld	r10,r1,32
        instMem[221] <= 8'b01000001;
        instMem[222] <= 8'b00000000;
        instMem[223] <= 8'b00100000;

        instMem[224] <= 8'b10010001; //-- ld	r11,r1,36
        instMem[225] <= 8'b01100001;
        instMem[226] <= 8'b00000000;
        instMem[227] <= 8'b00100100;

        instMem[228] <= 8'b10101000; //-- JMP 	-1
        instMem[229] <= 8'b00000000;
        instMem[230] <= 8'b11111111;
        instMem[231] <= 8'b11111111;

        instMem[232] <= 8'b00000000; //-- NOPE
        instMem[233] <= 8'b00000000;
        instMem[234] <= 8'b00000000;
        instMem[235] <= 8'b00000000;
      end
    end

  assign instruction = {instMem[address], instMem[address + 1], instMem[address + 2], instMem[address + 3]};
endmodule // insttructionMem


//Mux IF Stage

module mux #(parameter integer LENGTH =2) (in1, in2, sel, out);
  input sel;
  input [LENGTH-1:0] in1, in2;
  output [LENGTH-1:0] out;

  assign out = (sel == 0) ? in1 : in2;
endmodule // mxu

//////////////ALU///////////////////////////

module ALU (val1, val2, EXE_CMD, aluOut);
  input [`WORD_LEN-1:0] val1, val2;
  input [`EXE_CMD_LEN-1:0] EXE_CMD;
  output reg [`WORD_LEN-1:0] aluOut;

  always @ ( * ) begin
    case (EXE_CMD)
      `EXE_ADD: aluOut <= val1 + val2;
      `EXE_SUB: aluOut <= val1 - val2;
      `EXE_AND: aluOut <= val1 & val2;
      `EXE_OR: aluOut <= val1 | val2;
      `EXE_NOR: aluOut <= ~(val1 | val2);
      `EXE_XOR: aluOut <= val1 ^ val2;
      `EXE_SLA: aluOut <= val1 << val2;
      `EXE_SLL: aluOut <= val1 <<< val2;
      `EXE_SRA: aluOut <= val1 >> val2;
      `EXE_SRL: aluOut <= val1 >>> val2;
      default: aluOut <= 0;
    endcase
  end
endmodule // ALU





module mux_3input #(parameter integer LENGTH =2) (in1, in2, in3, sel, out);
  input [LENGTH-1:0] in1, in2, in3;
  input [1:0] sel;
  output [LENGTH-1:0] out;

  assign out = (sel == 2'd0) ? in1 :
               (sel == 2'd1) ? in2 : in3;
endmodule // mux





module signExtend (in, out);
  input [15:0] in;
  output [`WORD_LEN-1:0] out;

  assign out = (in[15] == 1) ? {16'b1111111111111111, in} : {16'b0000000000000000, in};
endmodule // signExtend






module dataMem (clk, rst, writeEn, readEn, address, dataIn, dataOut);
  input clk, rst, readEn, writeEn;
  input [`WORD_LEN-1:0] address, dataIn;
  output [`WORD_LEN-1:0] dataOut;

  integer i;
  reg [`MEM_CELL_SIZE-1:0] dataMem [0:`DATA_MEM_SIZE-1];
  wire [`WORD_LEN-1:0] base_address;

 assign clk_gate4 = writeEn & clk;
  always @ (posedge clk_gate4) begin
    if (rst)
      for (i = 0; i < `DATA_MEM_SIZE; i = i + 1)
        dataMem[i] <= 0;
    else 
      {dataMem[base_address], dataMem[base_address + 1], dataMem[base_address + 2], dataMem[base_address + 3]} <= dataIn;
  end

  assign base_address = ((address & 32'b11111111111111111111101111111111) >> 2) << 2;
  assign dataOut = (address < 1024) ? 0 : {dataMem[base_address], dataMem[base_address + 1], dataMem[base_address + 2], dataMem[base_address + 3]};
endmodule // dataMem





module EXE2MEM (clk, rst, WB_EN_IN, MEM_R_EN_IN, MEM_W_EN_IN, PCIn, ALUResIn, STValIn, destIn,
                          WB_EN,    MEM_R_EN,    MEM_W_EN,    PC,   ALURes,   STVal,   dest);
  input clk, rst;
  // TO BE REGISTERED FOR ID STAGE
  input WB_EN_IN, MEM_R_EN_IN, MEM_W_EN_IN;
  input [`REG_FILE_ADDR_LEN-1:0] destIn;
  input [`WORD_LEN-1:0] PCIn, ALUResIn, STValIn;
  // REGISTERED VALUES FOR ID STAGE
  output reg WB_EN, MEM_R_EN, MEM_W_EN;
  output reg [`REG_FILE_ADDR_LEN-1:0] dest;
  output reg [`WORD_LEN-1:0] PC, ALURes, STVal;

  always @ (posedge clk) begin
    if (rst) begin
      {WB_EN, MEM_R_EN, MEM_W_EN, PC, ALURes, STVal, dest} <= 0;
    end
    else begin
      WB_EN <= WB_EN_IN;
      MEM_R_EN <= MEM_R_EN_IN;
      MEM_W_EN <= MEM_W_EN_IN;
      PC <= PCIn;
      ALURes <= ALUResIn;
      STVal <= STValIn;
      dest <= destIn;
    end
  end
endmodule // EXE2MEM

module ID2EXE (clk, rst, destIn, reg2In, val1In, val2In, PCIn, EXE_CMD_IN, MEM_R_EN_IN, MEM_W_EN_IN, WB_EN_IN, brTaken_in, src1_in, src2_in,
                         dest,   ST_value,   val1,   val2,   PC,  EXE_CMD,    MEM_R_EN,    MEM_W_EN,    WB_EN, brTaken_out, src1_out, src2_out);
  input clk, rst;
  // TO BE REGISTERED FOR ID STAGE
  input MEM_R_EN_IN, MEM_W_EN_IN, WB_EN_IN, brTaken_in;
  input [`EXE_CMD_LEN-1:0] EXE_CMD_IN;
  input [`REG_FILE_ADDR_LEN-1:0] destIn, src1_in, src2_in;
  input [`WORD_LEN-1:0] reg2In, val1In, val2In, PCIn;
  // REGISTERED VALUES FOR ID STAGE
  output reg MEM_R_EN, MEM_W_EN, WB_EN, brTaken_out;
  output reg [`EXE_CMD_LEN-1:0] EXE_CMD;
  output reg [`REG_FILE_ADDR_LEN-1:0] dest, src1_out, src2_out;
  output reg [`WORD_LEN-1:0] ST_value, val1, val2, PC;

  always @ (posedge clk) begin
    if (rst) begin
      {MEM_R_EN, MEM_R_EN, WB_EN, EXE_CMD, dest, ST_value, val1, val2, PC, brTaken_out, src1_out, src2_out} <= 0;
    end
    else begin
      MEM_R_EN <= MEM_R_EN_IN;
      MEM_W_EN <= MEM_W_EN_IN;
      WB_EN <= WB_EN_IN;
      EXE_CMD <= EXE_CMD_IN;
      dest <= destIn;
      ST_value <= reg2In;
      val1 <= val1In;
      val2 <= val2In;
      PC <= PCIn;
      brTaken_out <= brTaken_in;
      src1_out <= src1_in;
      src2_out <= src2_in;
    end
  end
endmodule // ID2EXE

module IF2ID (clk, rst, flush, freeze, PCIn, instructionIn, PC, instruction);
  input clk, rst, flush, freeze;
  input [`WORD_LEN-1:0] PCIn, instructionIn;
  output reg [`WORD_LEN-1:0] PC, instruction;
  wire clk_gate2;

 assign clk_gate2 = ~freeze & clk;
always @ (posedge clk_gate2) begin
   if (rst) begin
      PC <= 0;
      instruction <= 0;
    end
    else begin
     
       if (~freeze) begin
        if (flush) begin
          instruction <= 0;
          PC <= 0;
        end
        else begin
          instruction <= instructionIn;
          PC <= PCIn;
        end
      end
    end
  end
  
endmodule // IF2ID

module MEM2WB (clk, rst, WB_EN_IN, MEM_R_EN_IN, ALUResIn, memReadValIn, destIn,
                         WB_EN,    MEM_R_EN,    ALURes,   memReadVal,   dest);
  input clk, rst;
  // TO BE REGISTERED FOR ID STAGE
  input WB_EN_IN, MEM_R_EN_IN;
  input [`REG_FILE_ADDR_LEN-1:0] destIn;
  input [`WORD_LEN-1:0] ALUResIn, memReadValIn;
  // REGISTERED VALUES FOR ID STAGE
  output reg WB_EN, MEM_R_EN;
  output reg [`REG_FILE_ADDR_LEN-1:0] dest;
  output reg [`WORD_LEN-1:0] ALURes, memReadVal;

  always @ (posedge clk) begin
    if (rst) begin
      {WB_EN, MEM_R_EN, dest, ALURes, memReadVal} <= 0;
    end
    else begin
      WB_EN <= WB_EN_IN;
      MEM_R_EN <= MEM_R_EN_IN;
      dest <= destIn;
      ALURes <= ALUResIn;
      memReadVal <= memReadValIn;
    end
  end
endmodule // MEM2WB

module regFile (clk, rst, src1, src2, dest, writeVal, writeEn, reg1, reg2);
  input clk, rst, writeEn;
  input [`REG_FILE_ADDR_LEN-1:0] src1, src2, dest;
  input [`WORD_LEN-1:0] writeVal;
  output [`WORD_LEN-1:0] reg1, reg2;

  reg [`WORD_LEN-1:0] regMem [0:`REG_FILE_SIZE-1];
  integer i;
wire clk_gate3;
  
  assign clk_gate3 = writeEn & clk;
  always @ (negedge clk_gate3) begin
    if (rst) begin
      for (i = 0; i < `WORD_LEN; i = i + 1)
        regMem[i] <= 0;
	    end

    else regMem[dest] <= writeVal;
    regMem[0] <= 0;
  end

  assign reg1 = (regMem[src1]);
  assign reg2 = (regMem[src2]);
endmodule // regFile



module forwarding_EXE (src1_EXE, src2_EXE, ST_src_EXE, dest_MEM, dest_WB, WB_EN_MEM, WB_EN_WB, val1_sel, val2_sel, ST_val_sel);
  input [`REG_FILE_ADDR_LEN-1:0] src1_EXE, src2_EXE, ST_src_EXE;
  input [`REG_FILE_ADDR_LEN-1:0] dest_MEM, dest_WB;
  input WB_EN_MEM, WB_EN_WB;
  output reg [`FORW_SEL_LEN-1:0] val1_sel, val2_sel, ST_val_sel;

  always @ ( * ) begin
    // initializing sel signals to 0
    // they will change to enable forwrding if needed.
    {val1_sel, val2_sel, ST_val_sel} <= 0;

    // determining forwarding control signal for store value (ST_val)
    if (WB_EN_MEM && ST_src_EXE == dest_MEM) ST_val_sel <= 2'd1;
    else if (WB_EN_WB && ST_src_EXE == dest_WB) ST_val_sel <= 2'd2;

    // determining forwarding control signal for ALU val1
    if (WB_EN_MEM && src1_EXE == dest_MEM) val1_sel <= 2'd1;
    else if (WB_EN_WB && src1_EXE == dest_WB) val1_sel <= 2'd2;

    // determining forwarding control signal for ALU val2
    if (WB_EN_MEM && src2_EXE == dest_MEM) val2_sel <= 2'd1;
    else if (WB_EN_WB && src2_EXE == dest_WB) val2_sel <= 2'd2;
  end
endmodule // forwarding

module hazard_detection(forward_EN, is_imm, ST_or_BNE, src1_ID, src2_ID, dest_EXE, WB_EN_EXE, dest_MEM, WB_EN_MEM, MEM_R_EN_EXE, branch_comm, hazard_detected);
  input [`REG_FILE_ADDR_LEN-1:0] src1_ID, src2_ID;
  input [`REG_FILE_ADDR_LEN-1:0] dest_EXE, dest_MEM;
  input [1:0] branch_comm;
  input forward_EN, WB_EN_EXE, WB_EN_MEM, is_imm, ST_or_BNE, MEM_R_EN_EXE;
  output hazard_detected;

  wire src2_is_valid, exe_has_hazard, mem_has_hazard, hazard, instr_is_branch;

  assign src2_is_valid =  (~is_imm) || ST_or_BNE;

  assign exe_has_hazard = WB_EN_EXE && (src1_ID == dest_EXE || (src2_is_valid && src2_ID == dest_EXE));
  assign mem_has_hazard = WB_EN_MEM && (src1_ID == dest_MEM || (src2_is_valid && src2_ID == dest_MEM));

  assign hazard = (exe_has_hazard || mem_has_hazard);
  assign instr_is_branch = (branch_comm == `COND_BEZ || branch_comm == `COND_BNE);

  assign hazard_detected = ~forward_EN ? hazard : (instr_is_branch && hazard) || (MEM_R_EN_EXE && mem_has_hazard);
endmodule // hazard_detection

module conditionChecker (reg1, reg2, cuBranchComm, brCond);
  input [`WORD_LEN-1: 0] reg1, reg2;
  input [1:0] cuBranchComm;
  output reg brCond;

  always @ ( * ) begin
    case (cuBranchComm)
      `COND_JUMP: brCond <= 1;
      `COND_BEZ: brCond <= (reg1 == 0) ? 1 : 0;
      `COND_BNE: brCond <= (reg1 != reg2) ? 1 : 0;
      default: brCond <= 0;
    endcase
  end
endmodule // conditionChecker

module controller (opCode, branchEn, EXE_CMD, Branch_command, Is_Imm, ST_or_BNE, WB_EN, MEM_R_EN, MEM_W_EN, hazard_detected);
  input hazard_detected;
  input [`OP_CODE_LEN-1:0] opCode;
  output reg branchEn;
  output reg [`EXE_CMD_LEN-1:0] EXE_CMD;
  output reg [1:0] Branch_command;
  output reg Is_Imm, ST_or_BNE, WB_EN, MEM_R_EN, MEM_W_EN;

  always @ ( * ) begin
    if (hazard_detected == 0) begin
      {branchEn, EXE_CMD, Branch_command, Is_Imm, ST_or_BNE, WB_EN, MEM_R_EN, MEM_W_EN} <= 0;
      case (opCode)
        // operations writing to the register file
        `OP_ADD: begin EXE_CMD <= `EXE_ADD; WB_EN <= 1; end
        `OP_SUB: begin EXE_CMD <= `EXE_SUB; WB_EN <= 1; end
        `OP_AND: begin EXE_CMD <= `EXE_AND; WB_EN <= 1; end
        `OP_OR:  begin EXE_CMD <= `EXE_OR;  WB_EN <= 1; end
        `OP_NOR: begin EXE_CMD <= `EXE_NOR; WB_EN <= 1; end
        `OP_XOR: begin EXE_CMD <= `EXE_XOR; WB_EN <= 1; end
        `OP_SLA: begin EXE_CMD <= `EXE_SLA; WB_EN <= 1; end
        `OP_SLL: begin EXE_CMD <= `EXE_SLL; WB_EN <= 1; end
        `OP_SRA: begin EXE_CMD <= `EXE_SRA; WB_EN <= 1; end
        `OP_SRL: begin EXE_CMD <= `EXE_SRL; WB_EN <= 1; end
        // operations using an immediate value embedded in the instruction
        `OP_ADDI: begin EXE_CMD <= `EXE_ADD; WB_EN <= 1; Is_Imm <= 1; end
        `OP_SUBI: begin EXE_CMD <= `EXE_SUB; WB_EN <= 1; Is_Imm <= 1; end
        // memory operations
        `OP_LD: begin EXE_CMD <= `EXE_ADD; WB_EN <= 1; Is_Imm <= 1; ST_or_BNE <= 1; MEM_R_EN <= 1; end
        `OP_ST: begin EXE_CMD <= `EXE_ADD; Is_Imm <= 1; MEM_W_EN <= 1; ST_or_BNE <= 1; end
        // branch operations
        `OP_BEZ: begin EXE_CMD <= `EXE_NO_OPERATION; Is_Imm <= 1; Branch_command <= `COND_BEZ; branchEn <= 1; end
        `OP_BNE: begin EXE_CMD <= `EXE_NO_OPERATION; Is_Imm <= 1; Branch_command <= `COND_BNE; branchEn <= 1; ST_or_BNE <= 1; end
        `OP_JMP: begin EXE_CMD <= `EXE_NO_OPERATION; Is_Imm <= 1; Branch_command <= `COND_JUMP; branchEn <= 1; end
        default: {branchEn, EXE_CMD, Branch_command, Is_Imm, ST_or_BNE, WB_EN, MEM_R_EN, MEM_W_EN} <= 0;
      endcase
    end

    else if (hazard_detected ==  1) begin
      // preventing any writes to the register file or the memory
      {EXE_CMD, WB_EN, MEM_W_EN} <= 0;
    end
  end
endmodule // controller
