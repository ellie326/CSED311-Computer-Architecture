// Submit this file with other files you created.
// Do not touch port declarations of the module 'CPU'.

// Guidelines
// 1. It is highly recommened to `define opcodes and something useful.
// 2. You can modify the module.
// (e.g., port declarations, remove modules, define new modules, ...)
// 3. You might need to describe combinational logics to drive them into the module (e.g., mux, and, or, ...)
// 4. `include files if required

module cpu(input reset,       // positive reset signal
           input clk,         // clock signal
           output is_halted,
           output [31:0]print_reg[0:31]
           ); // Whehther to finish simulation
  /***** Wire declarations *****/
  wire [31:0] next_pc;
  wire [31:0] current_pc;
  wire [31:0] addr;
  wire [31:0] data;
  wire [6:0] opcodes;
  wire [4:0] rs1;
  wire [4:0] rs2;
  wire [4:0] rd;
  wire [31:0] write_data;

  wire [31:0] read_data1;
  wire [31:0] read_data2;
  wire [31:0] imm_gen_out;
  wire [31:0] ALU_in1;
  wire [31:0] ALU_in2;
  wire [31:0] ALU_result;
  wire [3:0] ALU_op;
  wire [1:0] btype;
  wire PCcond;

  wire PCWrite;
  wire IorD;
  wire MemRead;
  wire MemWrite;
  wire [1:0] MemtoReg;
  wire IRWrite;
  wire PCSource;
  wire [1:0] ALUOp;
  wire [1:0] ALUSrcB;
  wire ALUSrcA;
  wire RegWrite;
  wire is_ecall;

  wire [31:0] IR_wire;
  wire [31:0] MDR_wire;
  wire [31:0] A_wire;
  wire [31:0] B_wire;
  wire [31:0] ALUOut_wire;
  wire [10:0] alu_control;
  wire [4:0] rs1_out;

  /***** Register declarations *****/
  reg [31:0] IR; // instruction register
  reg [31:0] MDR; // memory data register
  reg [31:0] A; // Read 1 data register
  reg [31:0] B; // Read 2 data register
  reg [31:0] ALUOut; // ALU output register
  // Do not modify and use registers declared above.

  assign opcodes = IR[6:0];
  assign rs1 = IR[19:15];
  assign rs2 = IR[24:20];
  assign rd = IR[11:7];
  assign A_wire = A;
  assign B_wire = B;
  assign IR_wire = IR;
  assign MDR_wire = MDR;
  assign ALUOut_wire = ALUOut;
  assign is_halted = is_ecall && read_data1 == 10;

  assign alu_control = {IR[30], IR[14:12], IR[6:0]};

  initial begin
    IR = 32'h0;
    MDR = 32'h0;
    A = 32'h0;
    B = 32'h0;
    ALUOut = 32'h0;
  end

  always @(posedge clk) begin
    if(reset) begin
      IR <= 32'h0;
      MDR <= 32'h0;
      A <= 32'h0;
      B <= 32'h0;
      ALUOut <= 32'h0;
    end
    else begin
      if(IRWrite) begin
        IR <= data;
      end
        MDR <= data;
        A <= read_data1;
        B <= read_data2;
        ALUOut <= ALU_result;
    end
  end

  // ---------- Update program counter ----------
  // PC must be updated on the rising edge (positive edge) of the clock.
  pc pc(
    .reset(reset),       // input (Use reset to initialize PC. Initial value must be 0)
    .clk(clk),         // input
    .next_pc(next_pc),     // input
    .current_pc(current_pc),   // output
    .PCWrite(PCWrite)
  );

  MUX2x1 addr_MUX(
    .in_bit(IorD),
    .in1(ALUOut_wire),
    .in0(current_pc),
    .out(addr)
  );

  // ---------- Register File ----------
  RegisterFile reg_file(
    .reset(reset),        // input
    .clk(clk),          // input
    .rs1(rs1_out),          // input
    .rs2(rs2),          // input
    .rd(rd),           // input
    .rd_din(write_data),       // input
    .write_enable(RegWrite),    // input
    .rs1_dout(read_data1),     // output
    .rs2_dout(read_data2),      // output
    .print_reg(print_reg)     // output (TO PRINT REGISTER VALUES IN TESTBENCH)
  );

  // ---------- Memory ----------
  Memory memory(
    .reset(reset),        // input
    .clk(clk),          // input
    .addr(addr),         // input
    .din(B_wire),          // input
    .mem_read(MemRead),     // input
    .mem_write(MemWrite),    // input
    .dout(data)          // output
  );

  MUX4x1 write_data_MUX(
    .in_bit(MemtoReg),
    .in3(0),
    .in2(ALU_result),
    .in1(MDR_wire),
    .in0(ALUOut_wire),
    .out(write_data)
  );

  // ---------- Control Unit ----------
  ControlUnit ctrl_unit( //주어진 코드랑 다름 (주어진 코드는 single cycle 때 사용 되는 것 같음)
    .reset(reset),
    .clk(clk),
    .part_of_inst(opcodes), // input
    .PCcond(PCcond),
    .data(data[6:0]),
    .PCWrite(PCWrite),
    .IorD(IorD),
    .MemRead(MemRead),
    .MemWrite(MemWrite),
    .MemtoReg(MemtoReg),
    .IRWrite(IRWrite),
    .PCSource(PCSource),
    .ALUOp(ALUOp),
    .ALUSrcB(ALUSrcB),
    .ALUSrcA(ALUSrcA),
    .RegWrite(RegWrite),
    .is_ecall(is_ecall) // output (ecall inst)
  );

  MUX2X1_2bit rs1_out_MUX(
    .in_bit(is_ecall),
    .in1(17),
    .in0(rs1),
    .out(rs1_out)
  );

  // ---------- Immediate Generator ----------
  ImmediateGenerator imm_gen(
    .part_of_inst(IR_wire),  // input
    .imm_gen_out(imm_gen_out)    // output
  );

  MUX2x1 ALU_in1_MUX(
    .in_bit(ALUSrcA),
    .in1(A_wire),
    .in0(current_pc),
    .out(ALU_in1)
  );

  MUX4x1 ALU_in2_MUX(
    .in_bit(ALUSrcB),
    .in3(0),
    .in2(imm_gen_out),
    .in1(4),
    .in0(B_wire),
    .out(ALU_in2)
  );

  // ---------- ALU Control Unit ----------
  ALUControlUnit alu_ctrl_unit(
    .part_of_inst(alu_control),  // input
    .alu_op(ALU_op),         // output
    .ALUOp(ALUOp), 
    .btype(btype)
  );

  // ---------- ALU ----------
  alu alu(
    .alu_op(ALU_op),      // input
    .alu_in_1(ALU_in1),    // input  
    .alu_in_2(ALU_in2),    // input
    .alu_result(ALU_result),  // output
    .alu_bcond(PCcond),     // output
    .btype(btype)
  );

  MUX2x1 next_pc_MUX(
    .in_bit(PCSource),
    .in1(ALUOut_wire),
    .in0(ALU_result),
    .out(next_pc)
  );

endmodule
