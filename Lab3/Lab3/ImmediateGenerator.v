`include "opcodes.v"

module ImmediateGenerator(
    input [31:0] part_of_inst,  // input
    output reg [31:0] imm_gen_out  // output
  );

  reg [6:0] opcode;

  always @(*) begin 
    imm_gen_out = 32'b0;
    opcode = part_of_inst[6:0];
    case(opcode)
        /*`ARITHMETIC: begin 
            imm_gen_out = 32'b0;
        end
        */
        `ARITHMETIC_IMM: begin 
            if(part_of_inst[31] == 1) begin  // signed 
                imm_gen_out = {20'hfffff, part_of_inst[31:20]}; 
            end 
            else begin 
                imm_gen_out = {20'h0, part_of_inst[31:20]}; 
            end 
        end
        
        `LOAD: begin 
            if(part_of_inst[31] == 1) begin  // signed 
                imm_gen_out = {20'hfffff, part_of_inst[31:20]}; 
            end 
            else begin 
                imm_gen_out = {20'h0, part_of_inst[31:20]}; 
            end
        end

        `JALR: begin 
            if(part_of_inst[31] == 1) begin  // signed 
                imm_gen_out = {20'hfffff, part_of_inst[31:20]}; 
            end 
            else begin 
                imm_gen_out = {20'h0, part_of_inst[31:20]}; 
            end 
        end
        `STORE: begin 
            if(part_of_inst[31] == 1) begin  // signed 
                imm_gen_out = {20'hfffff, part_of_inst[31:25], part_of_inst[11:8], part_of_inst[7]}; 
            end 
            else begin 
                imm_gen_out = {20'h0, part_of_inst[31:25], part_of_inst[11:8], part_of_inst[7]}; 
            end 
        end
        `BRANCH: begin 
            if(part_of_inst[31] == 1) begin  // signed 
                imm_gen_out = {19'h7ffff, part_of_inst[31], part_of_inst[7], part_of_inst[30:25], part_of_inst[11:8], 1'b0}; 
            end 
            else begin 
                imm_gen_out = {19'h0, part_of_inst[31], part_of_inst[7], part_of_inst[30:25], part_of_inst[11:8], 1'b0};
            end
        end 
        `JAL: begin 
            if(part_of_inst[31] == 1) begin  // signed 
                imm_gen_out = {11'h7ff, part_of_inst[31], part_of_inst[19:12], part_of_inst[20], part_of_inst[30:21],1'b0}; 
            end 
            else begin 
                imm_gen_out = {11'h0, part_of_inst[31], part_of_inst[19:12], part_of_inst[20], part_of_inst[30:21],1'b0};  
            end 
        end 
        /*
        `ECALL: begin 
            imm_gen_out = 32'b0;
        end */

        default: begin
            imm_gen_out = 32'b0;
        end
    endcase 
  end


endmodule
