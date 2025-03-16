`include "opcodes.v"
`include "instructions.v"

module ALUControlUnit (
  input [6:0] Opcode,
  input [2:0] Funct3,
  input [6:0] Funct7,
	output reg [4:0] alu_op);

  always @(*) begin
    //$display(Opcode);
    alu_op=5'b11111;

    if(Opcode == `JAL) begin
      alu_op = `INS_JUMPLINK_JAL;
	  end
	   
	  else if(Opcode == `JALR) begin
      alu_op = `INS_JUMPLINK_JALR;
	  end

    else if(Opcode == `BRANCH && Funct3 == `FUNCT3_BEQ) begin
      alu_op = `INS_BRANCH_BEQ;
    end

    else if(Opcode == `BRANCH && Funct3 == `FUNCT3_BNE) begin
      alu_op = `INS_BRANCH_BNE;
    end

    else if(Opcode == `BRANCH && Funct3 == `FUNCT3_BLT) begin
      alu_op = `INS_BRANCH_BLT;
    end

    else if(Opcode == `BRANCH && Funct3 == `FUNCT3_BGE) begin
      alu_op = `INS_BRANCH_BGE;
    end

    else if(Opcode == `LOAD) begin
      alu_op = `INS_LOAD_LW;
    end

    else if(Opcode == `STORE) begin
      alu_op = `INS_STORE_SW;
    end

    else if(Opcode == `ARITHMETIC_IMM && Funct3 == `FUNCT3_ADD) begin
      alu_op = `INS_ARITHMETIC_IMM_ADDI;
    end

    else if(Opcode == `ARITHMETIC_IMM && Funct3 == `FUNCT3_XOR) begin
      alu_op = `INS_ARITHMETIC_IMM_XORI;
    end

    else if(Opcode == `ARITHMETIC_IMM && Funct3 == `FUNCT3_OR) begin
      alu_op = `INS_ARITHMETIC_IMM_ORI;
    end

    else if(Opcode == `ARITHMETIC_IMM && Funct3 == `FUNCT3_AND) begin
      alu_op = `INS_ARITHMETIC_IMM_ANDI;
    end

    else if(Opcode == `ARITHMETIC_IMM && Funct3 == `FUNCT3_SLL) begin
      alu_op = `INS_ARITHMETIC_IMM_SLLI;
    end

    else if(Opcode == `ARITHMETIC_IMM && Funct3 == `FUNCT3_SRL) begin
      alu_op = `INS_ARITHMETIC_IMM_SRLI;
    end

    else if(Opcode == `ARITHMETIC && Funct3 == `FUNCT3_ADD) begin
      if(Funct7 == `FUNCT7_SUB) begin
        alu_op = `INS_ARITHMETIC_SUB;
      end
      else if(Funct7 == `FUNCT7_OTHERS) begin
        alu_op = `INS_ARITHMETIC_ADD;
      end
    end

    else if(Opcode == `ARITHMETIC && Funct3 == `FUNCT3_SLL) begin
      alu_op = `INS_ARITHMETIC_SLL;
    end
    
    else if(Opcode == `ARITHMETIC && Funct3 == `FUNCT3_SRL) begin
      alu_op = `INS_ARITHMETIC_SRL;
    end

    else if(Opcode == `ARITHMETIC && Funct3 == `FUNCT3_OR) begin
      alu_op = `INS_ARITHMETIC_OR;
    end

    else if(Opcode == `ARITHMETIC && Funct3 == `FUNCT3_AND) begin
      alu_op = `INS_ARITHMETIC_AND;
    end

    else if(Opcode == `ARITHMETIC && Funct3 == `FUNCT3_XOR) begin
      alu_op = `INS_ARITHMETIC_XOR;
    end

    else if(Opcode == `ECALL) begin
      alu_op = `INS_ECALL;
    end
  end

endmodule
