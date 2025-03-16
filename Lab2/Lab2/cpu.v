// Submit this file with other files you created.
// Do not touch port declarations of the module 'cpu'.

// Guidelines
// 1. It is highly recommened to `define opcodes and something useful.
// 2. You can modify the module.
// (e.g., port declarations, remove modules, define new modules, ...)
// 3. You might need to describe combinational logics to drive them into the module (e.g., mux, and, or, ...)
// 4. `include files if required

module cpu(input reset,                     // positive reset signal
           input clk,                       // clock signal
           output is_halted,                // Whehther to finish simulation
           output [31:0] print_reg [0:31]); // TO PRINT REGISTER VALUES IN TESTBENCH (YOU SHOULD NOT USE THIS)
  
  /***** Wire declarations *****/
  /***** Register declarations *****/
  
  reg [31:0] rf17;

  // Instruction memory
  wire [31:0] inst_dout;

  // Register File
  reg [31:0] rd_din;
  reg write_enable;
  reg [31:0] rs1_dout;
  reg [31:0] rs2_dout;

  // ALU
  reg [31:0] alu_in_2;
  reg [4:0] alu_op;
  reg alu_bcond;
  reg [31:0] alu_result;

  reg [31:0] reg_PCSrc1;

  // PC
  reg [31:0] current_pc;
  reg [31:0] next_pc;
  reg [31:0] pc_4;
  reg [31:0] pc_imm;

  // control_unit
  reg is_jal;
  reg is_jalr;
  reg branch;
  reg mem_read;
  reg mem_to_reg;
  reg mem_write;
  reg alu_src;
  reg pc_to_reg;
  reg is_ecall;


  // immediate generator
  reg [31:0] imm_gen_out;

  // data memory
  reg [31:0] data_dout;
  reg [31:0] mem_to_reg_result;


  assign is_halted = is_ecall && (rf17 == 32'd10);


  // ---------- Update program counter ----------
  // PC must be updated on the rising edge (positive edge) of the clock.
  pc pc(
    .reset(reset),       // input (Use reset to initialize PC. Initial value must be 0)
    .clk(clk),         // input
    .next_pc(next_pc),     // input
    .current_pc(current_pc)   // output
  );

  adder PC_4(
    .in1(current_pc),
    .in2(4),
    .out(pc_4) 
  );

  adder PC_IMM(
    .in1(current_pc),
    .in2(imm_gen_out),
    .out(pc_imm)
  );
  
  // ---------- Instruction Memory ----------
  instruction_memory imem(
    .reset(reset),   // input
    .clk(clk),     // input
    .addr(current_pc),    // input
    .dout(inst_dout)     // output
  );



  // ---------- Register File ----------
  register_file reg_file (
    .reset (reset),        // input
    .clk (clk),          // input
    .rs1 (inst_dout[19:15]),          // input
    .rs2 (inst_dout[24:20]),          // input
    .rd (inst_dout[11:7]),           // input
    .rd_din (rd_din),       // input
    .write_enable (write_enable), // input
    .rs1_dout (rs1_dout),     // output
    .rs2_dout (rs2_dout),     // output
    .rf17(rf17),
    .print_reg (print_reg)  //DO NOT TOUCH THIS
  );


  // ---------- Control Unit ----------
  control_unit ctrl_unit (
    .part_of_inst(inst_dout[6:0]),  // input
    .is_jal(is_jal),        // output
    .is_jalr(is_jalr),       // output
    .branch(branch),        // output
    .mem_read(mem_read),      // output
    .mem_to_reg(mem_to_reg),    // output
    .mem_write(mem_write),     // output
    .alu_src(alu_src),       // output
    .write_enable(write_enable),  // output
    .pc_to_reg(pc_to_reg),     // output
    .is_ecall(is_ecall)       // output (ecall inst)
  );

  // ---------- Immediate Generator ----------
  immediate_generator imm_gen(
    .part_of_inst(inst_dout),  // input
    .imm_gen_out(imm_gen_out)    // output
  );

  mux2to1 mux_imm (
    .in1(rs2_dout),
    .in2(imm_gen_out),
    .select(alu_src),
    .out(alu_in_2)
  );


  // ---------- ALU Control Unit ----------
  alu_control_unit alu_ctrl_unit (
    .Opcode(inst_dout[6:0]),
    .Funct3(inst_dout[14:12]),
    .Funct7(inst_dout[31:25]),
    .alu_op(alu_op)         // output
  );

  // ---------- ALU ----------
  alu alu (
    .alu_op(alu_op),      // input
    .alu_in_1(rs1_dout),    // input  
    .alu_in_2(alu_in_2),    // input
    .alu_result(alu_result),  // output
    .alu_bcond(alu_bcond)    // output
  );

  mux2to1 PCSrc1 (
    .in1(pc_4),
    .in2(pc_imm),
    .select((branch & alu_bcond) | is_jal),
    .out(reg_PCSrc1)
  );

  mux2to1 PCSrc2 (
    .in1(reg_PCSrc1),
    .in2(alu_result),
    .select(is_jalr),
    .out(next_pc)
  );

  // ---------- Data Memory ----------
  data_memory dmem(
    .reset (reset),      // input
    .clk (clk),        // input
    .addr (alu_result),       // input
    .din (rs2_dout),        // input
    .mem_read (mem_read),   // input
    .mem_write (mem_write),  // input
    .dout (data_dout)        // output
  );

  mux2to1 mux_data(
    .in1(alu_result),
    .in2(data_dout),
    .select(mem_to_reg),
    .out(mem_to_reg_result)
  );

  mux2to1 mux_write_data(
    .in1(mem_to_reg_result),
    .in2(pc_4),
    .select(pc_to_reg),
    .out(rd_din)
  );


endmodule
