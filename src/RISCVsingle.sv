// RISCVsingle.sv
// From Section 7.6 of Digital Design & Computer Architecture: RISC-V Edition
// Adapted MIPS to RISC-V SystemVerilog 5 April 2020
// David_Harris@hmc.edu 
// Sarah.Harris@unlv.edu

// run 210
// Expect simulator to print "Simulation succeeded"
// when the value 71 (0x47) is written to address 84 (0x54)

// Single-cycle implementation of RISC-V (RV32I)
// User-level Instruction Set Architecture V2.2 (May 7, 2017)
// Implements fraction of Base Integer Subset Instructions:
//    lw, sw
//    add, sub, and, or, slt, 
//    addi, andi, ori, slti
//    beq
//    jal
// Exceptions, traps, and interrupts not implemented
// little-endian memory


// 31 32-bit registers x1-x31, x0 hardwired to 0
// R-Type instructions
//   add, sub, and, or, slt
//   INSTR rd, rs1, rs2
//   instr[31:25] = funct7 (funct7b5 & opb5 = 1 for sub, 0 for others)
//   instr[24:20] = rs2
//   instr[19:15] = rs1
//   instr[14:12] = funct3
//   instr[11:7]  = rd
//   instr[6:0]   = opcode
// I-Type Instructions
//   lw, I-type ALU (addi, andi, ori, slti)
//   lw:         INSTR rd, imm(rs1)
//   I-type ALU: INSTR rd, rs1, imm (12-bit signed)
//   instr[31:20] = imm[11:0]
//   instr[24:20] = rs2
//   instr[19:15] = rs1
//   instr[14:12] = funct3
//   instr[11:7]  = rd
//   instr[6:0]   = opcode
// S-Type Instruction
//   sw rs2, imm(rs1) (store rs2 into address specified by rs1 + immm)
//   instr[31:25] = imm[11:5] (offset[11:5])
//   instr[24:20] = rs2 (src)
//   instr[19:15] = rs1 (base)
//   instr[14:12] = funct3
//   instr[11:7]  = imm[4:0]  (offset[4:0])
//   instr[6:0]   = opcode
// B-Type Instruction
//   beq rs1, rs2, imm (PCTarget = pc + (signed imm x 2))
//   instr[31:25] = imm[12], imm[10:5]
//   instr[24:20] = rs2
//   instr[19:15] = rs1
//   instr[14:12] = funct3
//   instr[11:7]  = imm[4:1], imm[11]
//   instr[6:0]   = opcode
// J-Type Instruction
//   jal rd, imm  (signed imm is multiplied by 2 and added to pc, rd = PC+4)
//   instr[31:12] = imm[20], imm[10:1], imm[11], imm[19:12]
//   instr[11:7]  = rd
//   instr[6:0]   = opcode

//   Instruction  opcode    funct3    funct7
//   add          0110011   000       0000000
//   sub          0110011   000       0100000
//   and          0110011   111       0000000
//   or           0110011   110       0000000
//   slt          0110011   010       0000000
//   addi         0010011   000       immediate
//   andi         0010011   111       immediate
//   ori          0010011   110       immediate
//   slti         0010011   010       immediate
//   beq          1100011   000       immediate
//   lw	          0000011   010       immediate
//   sw           0100011   010       immediate
//   jal          1101111   immediate immediate

module testbench();

  logic        clk;
  logic        reset;

  logic [31:0] WriteData, DataAdr;
  logic        MemWrite;

  // instantiate device to be tested
  top dut(clk, reset, WriteData, DataAdr, MemWrite);
  
  // initialize test
  initial
    begin
      reset <= 1; # 22; reset <= 0;
    end

  // generate clock to sequence tests
  always
    begin
      clk <= 1; # 5; clk <= 0; # 5;
    end

  // check results
  always @(negedge clk)
    begin
      if(MemWrite) begin
        if(DataAdr === 84 & WriteData === 71) begin
          $display("Simulation succeeded");
          $stop;
        end else if (DataAdr !== 80) begin
          $display("Simulation failed");
          $stop;
        end
      end
    end
endmodule


module top(input  logic        clk, reset, 
           output logic [31:0] WriteData, DataAdr, 
           output logic        MemWrite);

  logic [31:0] PC, Instr, ReadData;
  
  // instantiate processor and memories
  RISCVsingle RISCVsingle(clk, reset, PC, Instr, MemWrite, DataAdr, 
                          WriteData, ReadData);
  imem imem(PC, Instr);
  dmem dmem(clk, MemWrite, DataAdr, WriteData, ReadData);
endmodule

module dmem(input  logic        clk, WE,
            input  logic [31:0] A, WD,
            output logic [31:0] RD);

  logic [31:0] RAM[63:0];

  assign RD = RAM[A[31:2]]; // word aligned

  always_ff @(posedge clk)
    if (WE) RAM[A[31:2]] <= WD;
endmodule

module imem(input  logic [31:0] A,
            output logic [31:0] RD);

  logic [31:0] RAM[63:0];

  initial
      $readmemh("memfile.dat",RAM);

  assign RD = RAM[A[31:2]]; // word aligned
endmodule

module RISCVsingle(input  logic        clk, reset,
                   output logic [31:0] PC,
                   input  logic [31:0] Instr,
                   output logic        MemWrite,
                   output logic [31:0] ALUResult, WriteData,
                   input  logic [31:0] ReadData);

  logic       ALUSrc, RegWrite, Jump, Zero;
  logic [1:0] ResultSrc, ImmSrc;
  logic [2:0] ALUControl;

  controller c(Instr[30], Instr[14:12], Instr[6:0], Zero,
               ResultSrc, MemWrite, PCSrc,
               ALUSrc, RegWrite, Jump,
               ImmSrc, ALUControl);
  datapath dp(clk, reset, ResultSrc, PCSrc,
              ALUSrc, RegWrite,
              ImmSrc, ALUControl,
              Zero, PC, Instr,
              ALUResult, WriteData, ReadData);
endmodule

module controller(input  logic       funct7b5,
                  input  logic [2:0] funct3,
                  input  logic [6:0] op,
                  input  logic       Zero,
                  output logic [1:0] ResultSrc,
                  output logic       MemWrite,
                  output logic       PCSrc, ALUSrc,
                  output logic       RegWrite, Jump,
                  output logic [1:0] ImmSrc,
                  output logic [2:0] ALUControl);

  logic [1:0] ALUOp;
  logic       Branch;

  MainDec md(op, ResultSrc, MemWrite, Branch,
             ALUSrc, RegWrite, Jump, ImmSrc, ALUOp);
  ALUDec  ad(funct7b5, op[5], funct3, ALUOp, ALUControl);

  assign PCSrc = Branch & Zero | Jump;
endmodule

module MainDec(input  logic [6:0] op,
               output logic [1:0] ResultSrc,
               output logic       MemWrite,
               output logic       Branch, ALUSrc,
               output logic       RegWrite, Jump,
               output logic [1:0] ImmSrc,
               output logic [1:0] ALUOp);

  logic [10:0] controls;

  assign {RegWrite, ImmSrc, ALUSrc, MemWrite,
          ResultSrc, Branch, ALUOp, Jump} = controls;

  always_comb
    casez(op)
    // RegWrite_ImmSrc_ALUSrc_MemWrite_ResultSrc_Branch_ALUOp_Jump
      7'b0000011: controls = 11'b1_00_1_0_01_0_00_0; // lw
      7'b0100011: controls = 11'b0_01_1_1_00_0_00_0; // sw
      7'b0110011: controls = 11'b1_xx_0_0_00_0_10_0; // R-type 
      7'b1100011: controls = 11'b0_10_0_0_00_1_01_0; // beq
      7'b0010011: controls = 11'b1_00_1_0_00_0_10_0; // I-type ALU
      7'b1101111: controls = 11'b1_11_0_0_10_0_00_1; // jal
      default:    controls = 11'bx_xx_x_x_xx_x_xx_x; // non-implemented instruction
    endcase
endmodule

module aludec(input  logic       f7b5, op5,
              input  logic [2:0] funct3,
              input  logic [1:0] aluop,
              output logic [2:0] alucontrol);

  logic addSubType;
  assign addSubType = f7b5 & op5;
  always_comb
    case(aluop)
      2'b00: alucontrol <= 3'b000;  // add
      2'b01: alucontrol <= 3'b001;  // sub
      default: case({addSubType, funct3})        // R- or I-type
          4'b0000: alucontrol <= 3'b000; // ADD
          4'b1000: alucontrol <= 3'b001; // SUB
          4'b0111: alucontrol <= 3'b010; // AND
          4'b0110: alucontrol <= 3'b011; // OR
          4'b0010: alucontrol <= 3'b101; // SLT
          default: alucontrol <= 3'bxxx; // ???
        endcase
    endcase
endmodule

module datapath(input  logic        clk, reset,
                input  logic [1:0]  ResultSrc, 
                input  logic        PCSrc, ALUSrc,
                input  logic        RegWrite,
                input  logic [1:0]  ImmSrc,
                input  logic [2:0]  ALUControl,
                output logic        Zero,
                output logic [31:0] PC,
                input  logic [31:0] Instr,
                output logic [31:0] ALUResult, WriteData,
                input  logic [31:0] ReadData);

  logic [31:0] PCNext, PCPlus4, PCTarget;
  logic [31:0] ImmExt;
  logic [31:0] SrcA, SrcB;
  logic [31:0] Result;

  // next PC logic
  flopr #(32) pcreg(clk, reset, PCNext, PC);
  adder       pcadd4(PC, 32'd4, PCPlus4);
  adder       pcaddbranch(PC, ImmExt, PCTarget);
  mux2 #(32)  pcmux(PCPlus4, PCTarget, PCSrc, PCNext);
 
  // register file logic
  RegFile     RF(clk, RegWrite, Instr[19:15], Instr[24:20], 
                 Instr[11:7], Result, SrcA, WriteData);
  Extend      Ext(Instr[31:7], ImmSrc, ImmExt);

  // ALU logic
  mux2 #(32)  SrcBmux(WriteData, ImmExt, ALUSrc, SrcB);
  ALU         ALU(SrcA, SrcB, ALUControl, ALUResult, Zero);
  mux3 #(32)  Resultmux(ALUResult, ReadData, PCPlus4, ResultSrc, Result);
endmodule

module Extend(input  logic [31:7] Instr,
              input  logic [1:0]  ImmSrc,
              output logic [31:0] ImmExt);
 
  always_comb
    case(ImmSrc) 
               // I-type 
      2'b00:   ImmExt = {{20{Instr[31]}}, Instr[31:20]};  
               // S-type (Stores)
      2'b01:   ImmExt = {{20{Instr[31]}}, Instr[31:25], Instr[11:7]}; 
               // B-type (Branches)
      2'b10:   ImmExt = {{20{Instr[31]}}, Instr[7], Instr[30:25], Instr[11:8], 1'b0}; 
               // J-type (Jumps)
      2'b11:   ImmExt = {{12{Instr[31]}}, Instr[19:12], Instr[20], Instr[30:21], 1'b0}; 
      default: ImmExt = 32'bx; // undefined
    endcase             
endmodule

module RegFile(input  logic        clk, 
               input  logic        WE3, 
               input  logic [ 4:0] A1, A2, A3, 
               input  logic [31:0] WD3, 
               output logic [31:0] RD1, RD2);

  logic [31:0] rf[31:0];

  // three ported register file
  // read two ports combinationally (A1/RD1, A2/RD2)
  // write third port on rising edge of clock (A3/WD3/WE3)
  // register 0 hardwired to 0

  always_ff @(posedge clk)
    if (WE3) rf[A3] <= WD3;	

  assign RD1 = (A1 != 0) ? rf[A1] : 0;
  assign RD2 = (A2 != 0) ? rf[A2] : 0;
endmodule

module adder(input  [31:0] a, b,
             output [31:0] y);

  assign y = a + b;
endmodule

module flopr #(parameter WIDTH = 8)
              (input  logic             clk, reset,
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0;
    else       q <= d;
endmodule

module mux2 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, 
              input  logic             s, 
              output logic [WIDTH-1:0] y);

  assign y = s ? d1 : d0; 
endmodule

module mux3 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, d2,
              input  logic [1:0]       s, 
              output logic [WIDTH-1:0] y);

  assign y = s[1] ? d2 : (s[0] ? d1 : d0); 
endmodule

module alu(input  logic [31:0] a, b,
            input  logic [2:0]  alucontrol,
            output logic [31:0] result,
            output logic        zero); 

  logic [31:0] condinvb, sum;
  logic        sub;
  
  assign sub = (alucontrol[1:0] == 2'b01);
  assign condinvb = sub ? ~b : b; // for subtraction or slt
  assign sum = a + condinvb + sub;

  always_comb
    case (alucontrol)
      3'b000: result = sum;          // addition
      3'b001: result = sum;          // subtraction
      3'b010: result = a & b;        // and
      3'b011: result = a | b;        // or
      3'b101: result = sum[31];      // slt
      default: result = 0;
  endcase

  assign zero = (result == 32'b0);
endmodule
