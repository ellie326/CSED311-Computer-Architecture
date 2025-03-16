`include "instructions.v"

module alu (
	input [4:0]  alu_op,      // input
	input [31:0] alu_in_1,    // input  
  input [31:0] alu_in_2,    // input
  output reg [31:0] alu_result,  // output
  output reg alu_bcond);     // output

	always @(*) begin
  	alu_result = 0;
    alu_bcond = 0;
        
    case(alu_op)
      `INS_JUMPLINK_JALR, `INS_LOAD_LW, `INS_STORE_SW, `INS_ARITHMETIC_IMM_ADDI, `INS_ARITHMETIC_ADD: begin
				alu_result = alu_in_1 + alu_in_2;
			end
				
			`INS_BRANCH_BEQ: begin
				alu_bcond = (alu_in_1 == alu_in_2) ? 1 : 0;
			end
				
			`INS_BRANCH_BNE: begin
				alu_bcond = (alu_in_1 != alu_in_2) ? 1 : 0;
			end
				
			`INS_BRANCH_BLT: begin
				alu_bcond = (alu_in_1 < alu_in_2) ? 1 : 0;
			end
				
			`INS_BRANCH_BGE: begin
				alu_bcond = (alu_in_1 >= alu_in_2) ? 1 : 0;
			end
				
			`INS_ARITHMETIC_IMM_XORI, `INS_ARITHMETIC_XOR: begin
				alu_result = alu_in_1 ^ alu_in_2;
			end
				
			`INS_ARITHMETIC_IMM_ORI, `INS_ARITHMETIC_OR: begin
				alu_result = alu_in_1 | alu_in_2;
			end
				
			`INS_ARITHMETIC_IMM_ANDI, `INS_ARITHMETIC_AND: begin
				alu_result = alu_in_1 & alu_in_2;
			end
				
			`INS_ARITHMETIC_IMM_SLLI, `INS_ARITHMETIC_SLL: begin
				alu_result = alu_in_1 << alu_in_2;
			end
				
			`INS_ARITHMETIC_IMM_SRLI, `INS_ARITHMETIC_SRL: begin
				alu_result = alu_in_1 >> alu_in_2;
			end
				
			`INS_ARITHMETIC_SUB: begin
				alu_result = alu_in_1 - alu_in_2;
			end
				

			default: begin

			end
		endcase
	end
endmodule
