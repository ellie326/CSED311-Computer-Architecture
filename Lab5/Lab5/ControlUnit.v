`include "opcodes.v"

module ControlUnit (
  input [6:0] part_of_inst,
  //input hazard_detected,
  output reg is_jal,
  output reg is_jalr,
  output reg is_branch,
  output reg mem_read,
  output reg [1:0]mem_to_reg,
  output reg mem_write,
  //output reg reg_write,
  output reg alu_src,
  output reg write_enable,
  output reg pc_to_reg,
  output reg use_rs1,
  output reg use_rs2,
  output reg is_ecall
  //input EX_MEM_bcond, //**
  //input MEM_WB_bcond, //**
  //output reg [1:0] PC_src
  //output reg is_jump_needed
  );

  always @(*) begin
    is_jal = 0;
    is_jalr = 0;
    is_branch = 0;
    mem_read = 0;
    mem_to_reg = 2'b00;
    mem_write = 0;
    //reg_write = 0;
    alu_src = 0;
    write_enable = 0;
    pc_to_reg = 0;
    use_rs1 = 0;
    use_rs2 = 0;
    is_ecall = 0;
    //PC_src = 0; 
    //is_jump_needed = 0; 

    case(part_of_inst)
      `JAL: begin
        is_jal = 1;
        mem_to_reg = 2'b10; 
        write_enable = 1;
        pc_to_reg = 1;
        use_rs1 = 1;
        //PC_src = 2'b01; 
        //is_jump_needed = 1; 
      end

      `JALR: begin
        is_jalr = 1;
        mem_to_reg = 2'b10; 
        write_enable = 1;
        pc_to_reg = 1;
        alu_src = 1;
        use_rs1 = 1;
        //PC_src = 2'b10; 
        //is_jump_needed = 1; 
      end

      `BRANCH: begin
        is_branch = 1;
        use_rs1 = 1;
        use_rs2 = 1;
        //PC_src = 2'b01; 
        //is_jump_needed = 1; 
      end

      `LOAD: begin
        mem_read = 1;
        write_enable = 1;
        mem_to_reg = 2'b01;
        alu_src = 1;
        //reg_write = 1;
        use_rs1 = 1;
      end

      `STORE: begin
        mem_write = 1;
        alu_src = 1;
        use_rs1 = 1;
        use_rs2 = 1;
      end

      `ARITHMETIC_IMM: begin
        write_enable = 1;
        alu_src = 1;
        //reg_write = 1;
        use_rs1 = 1;
      end

      `ARITHMETIC: begin
        write_enable = 1;
        //reg_write = 1;
        use_rs1 = 1;
        use_rs2 = 1;
      end
      
      `ECALL: begin
        is_ecall = 1;
      end

      default: begin

      end
    endcase
    /*
    if(EX_MEM_bcond || MEM_WB_bcond) begin
      mem_read = 0;
      mem_to_reg = 2'b0;
      mem_write = 0;
      alu_src = 0; 
      pc_to_reg = 0;
      write_enable = 0;
      use_rs1 = 0;
      use_rs2 = 0;
      is_ecall = 0;
    end*/

  end
endmodule
