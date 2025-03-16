`include "opcodes.v"

module ControlUnit (
  input [6:0] part_of_inst,
  output reg mem_read,
  output reg mem_to_reg,
  output reg mem_write,
  output reg alu_src,
  output reg write_enable,
  output reg pc_to_reg,
  output reg use_rs1,
  output reg use_rs2,
  output reg is_ecall);

  always @(*) begin
    mem_read = 0;
    mem_to_reg = 0;
    mem_write = 0;
    alu_src = 0;
    write_enable = 0;
    pc_to_reg = 0;
    use_rs1 = 0;
    use_rs2= 0;
    is_ecall = 0;

    case(part_of_inst)
      // `JAL: begin
      //   is_jal = 1;
      //   write_enable = 1;
      //   pc_to_reg = 1;
      // end

      // `JALR: begin
      //   is_jalr = 1;
      //   write_enable = 1;
      //   pc_to_reg = 1;
      //   alu_src = 1;
      // end

      // `BRANCH: begin
      //   branch = 1;
      // end

      `LOAD: begin
        mem_read = 1;
        write_enable = 1;
        mem_to_reg = 1;
        alu_src = 1;
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
        use_rs1 = 1;
      end

      `ARITHMETIC: begin
        write_enable = 1;
        use_rs1 = 1;
        use_rs2 = 1;
      end
      
      `ECALL: begin
        is_ecall = 1;
      end

      default: begin
      end
    endcase
  end
endmodule
