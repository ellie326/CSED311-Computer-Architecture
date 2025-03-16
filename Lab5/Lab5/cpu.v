// Submit this file with other files you created.
// Do not touch port declarations of the module 'CPU'.

// Guidelines
// 1. It is highly recommened to `define opcodes and something useful.
// 2. You can modify modules (except InstMemory, DataMemory, and RegisterFile)
// (e.g., port declarations, remove modules, define new modules, ...)
// 3. You might need to describe combinational logics to drive them into the module (e.g., mux, and, or, ...)
// 4. `include files if required

module cpu(input reset,       // positive reset signal
           input clk,         // clock signal
           output is_halted, // Whehther to finish simulation
           output [31:0]print_reg[0:31]); // Whehther to finish simulation
  /***** Wire declarations *****/
  wire [31:0] current_pc;
  wire [31:0] next_pc;
  //wire [31:0] shifted_imm;
  wire [31:0] pc_plus_imm;
  wire [31:0] pc_plus_4;

  wire [6:0] opcode;
  wire [4:0] rs1;
  wire [4:0] rs2;
  wire [4:0] rd;

  wire [4:0] rs1_out;

  wire [1:0] forwardEcall;
  wire [31:0] ecall_data;
  wire is_halted_temp; 
  wire is_flush;

  wire is_not_cache_stall;

  /***** Register declarations *****/
  reg [31:0] rd_din;
  reg [31:0] rs1_dout;
  reg [31:0] rs2_dout;
  reg write_enable;
  reg [4:0] alu_op;
  reg mem_read;
  reg [1:0]mem_to_reg;
  reg mem_write;
  reg reg_write;
  reg alu_src;
  reg pc_to_reg;
  reg is_ecall;
  reg PCWrite;
  reg IF_IDWrite;
  reg use_rs1;
  reg use_rs2;

  reg is_branch;
  reg is_jal;
  reg is_jalr;

  reg mem_rw;

  reg [31:0] imm_gen_out;

  reg [4:0] operation;

  reg [31:0] alu_in2;

  reg [31:0] alu_result;

  reg [31:0] inst_out;
  reg [31:0] data_out;

  reg [1:0] ForwardA;
  reg [1:0] ForwardB;
  reg [31:0] forwarded_rs1;
  reg [31:0] forwarded_rs2;

  reg hazard_detected;
  
  reg is_ready;
  reg is_output_valid;
  reg is_hit;

  //assign shifted_imm = ID_EX_imm << 1;

  // You need to modify the width of registers
  // In addition, 
  // 1. You might need other pipeline registers that are not described below
  // 2. You might not need registers described below
  /***** IF/ID pipeline registers *****/
  reg [31:0] IF_ID_inst;           // will be used in ID stage
  reg [31:0] IF_ID_current_pc; // added

  /***** ID/EX pipeline registers *****/
  // From the control unit
  reg [4:0] ID_EX_alu_op;         // will be used in EX stage
  reg ID_EX_alu_src;        // will be used in EX stage
  reg ID_EX_mem_write;      // will be used in MEM stage
  reg ID_EX_mem_read;       // will be used in MEM stage
  reg [1:0]ID_EX_mem_to_reg;     // will be used in WB stage
  reg ID_EX_reg_write;      // will be used in WB stage
  reg ID_EX_is_halted;  // added
  reg ID_EX_is_branch;
  reg ID_EX_is_jal;
  reg ID_EX_is_jalr;
  // From others
  reg [31:0] ID_EX_rs1_data;
  reg [31:0] ID_EX_rs2_data;
  reg [31:0] ID_EX_imm;
  reg [4:0] ID_EX_rd;
  reg [31:0] ID_EX_current_pc; // added
  reg [31:0] ID_EX_inst;
  reg [4:0] ID_EX_rs1;
  reg [4:0] ID_EX_rs2;
  /***** EX/MEM pipeline registers *****/
  // From the control unit
  reg EX_MEM_mem_write;     // will be used in MEM stage
  reg EX_MEM_mem_read;      // will be used in MEM stage
  //reg EX_MEM_is_branch;     // will be used in MEM stage
  reg [1:0]EX_MEM_mem_to_reg;    // will be used in WB stage
  reg EX_MEM_reg_write;     // will be used in WB stage
  reg EX_MEM_is_halted; // added
  // From others
  reg [31:0] EX_MEM_alu_out;
  reg [31:0] EX_MEM_dmem_data;
  reg [4:0] EX_MEM_rd;
  reg [31:0] EX_MEM_pc_plus_imm; // added

  reg bcond;
  reg EX_MEM_bcond; 
  reg MEM_WB_bcond; 

  /***** MEM/WB pipeline registers *****/
  // From the control unit
  reg [1:0]MEM_WB_mem_to_reg;    // will be used in WB stage
  reg MEM_WB_reg_write;     // will be used in WB stage
  reg MEM_WB_is_halted; // added
  // From others
  reg [31:0] MEM_WB_mem_to_reg_src_1;
  reg [31:0] MEM_WB_mem_to_reg_src_2;
  reg [4:0] MEM_WB_rd; // added

  // assign
  assign opcode = IF_ID_inst[6:0];
  assign rs1 = IF_ID_inst[19:15];
  assign rs2 = IF_ID_inst[24:20];
  assign rd = IF_ID_inst[11:7];

  assign is_halted_temp = is_ecall & (ecall_data == 10);  // rs1_dout 대신 ecall_data 사용. ecall_data는 따로 mux에서 결정.
  assign is_halted = MEM_WB_is_halted;

  assign is_not_cache_stall = !((EX_MEM_mem_read|EX_MEM_mem_write)&!(is_ready & is_output_valid & is_hit));

  reg [31:0] IF_ID_pc_plus_4; 
  reg [31:0] ID_EX_pc_plus_4; 
  reg [31:0] EX_MEM_pc_plus_4; 
  reg [31:0] MEM_WB_pc_plus_4; 

  //reg is_jump_needed; 
  //reg ID_EX_is_jump_needed; 
  //reg PC_src_inbit; 
  //assign PC_src_inbit = bcond & ID_EX_is_jump_needed; 
  reg [1:0] pc_src;
  reg IF_ID_is_flush;

  assign is_flush = (pc_src == 2'b10) | (pc_src == 2'b01);

  // ---------- Update program counter ----------
  // PC must be updated on the rising edge (positive edge) of the clock.
  MUX4x1 mux_PCSrc (
    .in_bit(pc_src),
    .in3(0),
    .in2(alu_result),
    .in1(pc_plus_imm),
    .in0(pc_plus_4),
    .out(next_pc)
  );
  
  pc pc(
    .reset(reset),       // input (Use reset to initialize PC. Initial value must be 0)
    .clk(clk),         // input
    .PCWrite(PCWrite),
    .is_not_cache_stall(is_not_cache_stall),
    .next_pc(next_pc),     // input
    .current_pc(current_pc)   // output
  );

  adder adder_PC (
    .in1(current_pc),
    .in2(4),
    .out(pc_plus_4)
  );
  // ---------- Instruction Memory ----------
  InstMemory imem(
    .reset(reset),   // input
    .clk(clk),     // input
    .addr(current_pc),    // input
    .dout(inst_out)     // output
  );

  // Update IF/ID pipeline registers here
  always @(posedge clk) begin
    if (reset) begin
      IF_ID_inst <= 0;
      IF_ID_current_pc <= 0;
      IF_ID_pc_plus_4 <= 0; 
      IF_ID_is_flush <= 0;
    end
    else begin
      if((!hazard_detected)&is_not_cache_stall) begin
        IF_ID_inst <= inst_out;
        IF_ID_current_pc <= current_pc;
        IF_ID_pc_plus_4 <= pc_plus_4;
        IF_ID_is_flush <= is_flush;
        //$display("%h", IF_ID_inst);
      end
    end
  end

  // ---------- Register File ----------

  MUX2x1_5bit mux_handle_ecall (
    .in_bit(is_ecall),
    .in1(17),
    .in0(rs1),
    .out(rs1_out)
  );

  RegisterFile reg_file (
    .reset (reset),        // input
    .clk (clk),          // input
    .rs1 (rs1_out),          // input
    .rs2 (rs2),          // input
    .rd (MEM_WB_rd),           // input
    .rd_din (rd_din),       // input
    .write_enable (MEM_WB_reg_write),    // input
    .rs1_dout (rs1_dout),     // output
    .rs2_dout (rs2_dout),     // output
    .print_reg(print_reg)
  );

  HazardDetectUnit hazard_detect_unit (
    .rs1(rs1_out),
    .rs2(rs2),
    .ID_EX_rd(ID_EX_rd),
    .ID_EX_mem_read(ID_EX_mem_read),
    .use_rs1(use_rs1),
    .use_rs2(use_rs2),
    .PCWrite(PCWrite),
    .IF_IDWrite(IF_IDWrite),
    .hazard_detected(hazard_detected)
  );
  // ---------- Control Unit ----------
  ControlUnit ctrl_unit (
    .part_of_inst(opcode),  // input
    .is_jal(is_jal),
    .is_jalr(is_jalr),
    .is_branch(is_branch),
    //.hazard_detected(hazard_detected),
    .mem_read(mem_read),      // output
    .mem_to_reg(mem_to_reg),    // output
    .mem_write(mem_write),     // output
    .alu_src(alu_src),       // output
    .write_enable(reg_write),  // output
    .pc_to_reg(pc_to_reg),     // output
    //.alu_op(alu_op),        // output
    .use_rs1(use_rs1),
    .use_rs2(use_rs2),
    .is_ecall(is_ecall)       // output (ecall inst)
    //.is_jump_needed(is_jump_needed)
  );

  // ---------- Immediate Generator ----------
  immediate_generator imm_gen(
    .part_of_inst(IF_ID_inst),  // input
    .imm_gen_out(imm_gen_out)    // output
  );

  // Forwarding Ecall unit 작업해야함
  ForwardingEcallUnit forwarding_ecall_unit(
    .rs1(17),
    .ID_EX_rd(ID_EX_rd),
    .EX_MEM_rd(EX_MEM_rd),
    .MEM_WB_rd(MEM_WB_rd),
    .ID_EX_reg_write(ID_EX_reg_write),
    .EX_MEM_reg_write(EX_MEM_reg_write),
    .MEM_WB_reg_write(reg_write), // ?
    .forwardEcall(forwardEcall)
  );

  MUX4x1 MUX_ecall_data(
    .in_bit(forwardEcall),
    .in3(rd_din),
    .in2(EX_MEM_alu_out),
    .in1(alu_result),
    .in0(rs1_dout),
    .out(ecall_data)
  );

  // Update ID/EX pipeline registers here
  always @(posedge clk) begin
    if (reset|(hazard_detected&is_not_cache_stall)|is_flush|IF_ID_is_flush) begin
      ID_EX_alu_op <= 0;         // will be used in EX stage
      ID_EX_alu_src <= 0;        // will be used in EX stage
      ID_EX_mem_write <= 0;      // will be used in MEM stage
      ID_EX_mem_read <= 0;       // will be used in MEM stage
      ID_EX_mem_to_reg <= 0;     // will be used in WB stage
      ID_EX_reg_write <= 0;      // will be used in WB stage
      ID_EX_is_halted <= 0;
      ID_EX_pc_plus_4 <= 0; 
      //ID_EX_is_jump_needed <= 0; 
      ID_EX_is_branch <= 0;
      ID_EX_is_jal <= 0;
      ID_EX_is_jalr <= 0;
      // From others
      ID_EX_rs1_data <= 0;
      ID_EX_rs2_data <= 0;
      ID_EX_imm <= 0;
      ID_EX_rd <= 0;
      ID_EX_current_pc <= 0;
      ID_EX_inst <= 0;
      ID_EX_rs1 <= 0;
      ID_EX_rs2 <= 0;
    end
    else if(is_not_cache_stall) begin
      // From the control unit
      ID_EX_mem_write <= mem_write;      // will be used in WB stage
      ID_EX_reg_write <= reg_write;      // will be used in WB stage
      ID_EX_alu_op <= alu_op;         // will be used in EX stage
      ID_EX_alu_src <= alu_src;        // will be used in EX stage
      
      ID_EX_mem_read <= mem_read;       // will be used in MEM stage
      ID_EX_mem_to_reg <= mem_to_reg;     // will be used in WB stage
      
      ID_EX_is_halted <= is_halted_temp;
      ID_EX_pc_plus_4 <= IF_ID_pc_plus_4; 
      //ID_EX_is_jump_needed <= is_jump_needed; 
      ID_EX_is_branch <= is_branch;
      ID_EX_is_jal <= is_jal;
      ID_EX_is_jalr <= is_jalr;
      // From others
      ID_EX_rs1_data <= rs1_dout;
      ID_EX_rs2_data <= rs2_dout;
      ID_EX_imm <= imm_gen_out;
      ID_EX_rd <= rd;  // IF_ID_inst[11:7]
      ID_EX_current_pc <= IF_ID_current_pc;
      ID_EX_inst <= IF_ID_inst;
      ID_EX_rs1 <= rs1;
      ID_EX_rs2 <= rs2;
    end
  end

  // ---------- ALU Control Unit ----------
  ALUControlUnit alu_ctrl_unit (
    .Opcode(ID_EX_inst[6:0]),  // input
    .Funct3(ID_EX_inst[14:12]),
    .Funct7(ID_EX_inst[31:25]),
    .alu_op(operation)         // output
  );

  MUX2x1 mux_ALUSrc (
    .in0(forwarded_rs2),
    .in1(ID_EX_imm),
    .in_bit(ID_EX_alu_src), // 그냥 alu_src로 하니 사이클 터짐.
    .out(alu_in2)
  );

  // ---------- ALU ----------
  alu alu (
    .alu_op(operation),      // input
    .alu_in_1(forwarded_rs1),    // input  
    .alu_in_2(alu_in2),    // input
    .alu_result(alu_result),  // output
    .alu_bcond(bcond)     // output
  );

  adder adder_pc_plus_imm (
    .in1(ID_EX_current_pc),
    .in2(ID_EX_imm),
    .out(pc_plus_imm)
  );

  ForwardingUnit forwarding_unit (
    .ID_EX_rs1(ID_EX_rs1),
    .ID_EX_rs2(ID_EX_rs2),
    .EX_MEM_rd(EX_MEM_rd),
    .EX_MEM_reg_write(EX_MEM_reg_write),
    .MEM_WB_rd(MEM_WB_rd),
    .MEM_WB_reg_write(MEM_WB_reg_write),
    ._x0(0),
    .ForwardA(ForwardA),
    .ForwardB(ForwardB)
  );

  MUX4x1 mux_forward_a (  // input 순서 확인
    .in_bit(ForwardA),
    .in3(0),
    .in2(rd_din),
    .in1(EX_MEM_alu_out),
    .in0(ID_EX_rs1_data),
    .out(forwarded_rs1)
  );

  MUX4x1 mux_forward_b (   // input 순서 확인
    .in_bit(ForwardB),
    .in3(0),
    .in2(rd_din),
    .in1(EX_MEM_alu_out),
    .in0(ID_EX_rs2_data),
    .out(forwarded_rs2)
  );

  // Update EX/MEM pipeline registers here
  always @(posedge clk) begin
    if (reset) begin
      EX_MEM_mem_write <= 0;
      EX_MEM_mem_read <= 0;
      EX_MEM_mem_to_reg <= 0;
      EX_MEM_reg_write <= 0;
      EX_MEM_is_halted <= 0;
      EX_MEM_pc_plus_4 <= 0; 
      // From others
      EX_MEM_alu_out <= 0;
      EX_MEM_dmem_data <= 0;
      EX_MEM_rd <= 0;
      EX_MEM_bcond <= 0; 
    end
    else if(is_not_cache_stall)begin
      EX_MEM_mem_write <= ID_EX_mem_write;
      EX_MEM_mem_read <= ID_EX_mem_read;
      EX_MEM_mem_to_reg <= ID_EX_mem_to_reg;
      EX_MEM_reg_write <= ID_EX_reg_write;
      EX_MEM_is_halted <= ID_EX_is_halted;
      EX_MEM_pc_plus_4 <= ID_EX_pc_plus_4; 
      // From others
      EX_MEM_alu_out <= alu_result;
      //$display(alu_result);
      EX_MEM_dmem_data <= forwarded_rs2;
      EX_MEM_rd <= ID_EX_rd;
      EX_MEM_bcond <= bcond; 
      if(ID_EX_mem_read == 1) begin
        mem_rw <= 0;
      end 
      else if(ID_EX_mem_write == 1) begin
        mem_rw <= 1;
      end
      //$display("alu result: %x, bcond : ",alu_result, bcond); 
    end
  end

  // ---------- Cache -----------
  Cache cache(
    .reset(reset),
    .clk(clk),
    .is_input_valid(EX_MEM_mem_write | EX_MEM_mem_read),
    .addr(EX_MEM_alu_out),
    .mem_rw(mem_rw),
    .din(EX_MEM_dmem_data),
    .is_ready(is_ready),
    .is_output_valid(is_output_valid),
    .dout(data_out),
    .is_hit(is_hit)
  );


  /*
  // ---------- Data Memory ----------
  DataMemory dmem(
    .reset (reset),      // input
    .clk (clk),        // input
    .addr (EX_MEM_alu_out),       // input
    .din (EX_MEM_dmem_data),        // input
    .mem_read (EX_MEM_mem_read),   // input
    .mem_write (EX_MEM_mem_write),  // input
    .dout (data_out)        // output
  );
  */
  // Update MEM/WB pipeline registers here
  always @(posedge clk) begin
    if (reset) begin
      MEM_WB_mem_to_reg <= 0;
      MEM_WB_reg_write <= 0;
      MEM_WB_is_halted <= 0;
      MEM_WB_pc_plus_4 <= 0; 
      // From others
      MEM_WB_mem_to_reg_src_1 <= 0;
      MEM_WB_mem_to_reg_src_2 <= 0;
      MEM_WB_rd <= 0;
      MEM_WB_bcond <= 0; 
    end
    else if(is_not_cache_stall) begin
      MEM_WB_mem_to_reg <= EX_MEM_mem_to_reg;
      MEM_WB_reg_write <= EX_MEM_reg_write;
      MEM_WB_is_halted <= EX_MEM_is_halted;
    
      MEM_WB_pc_plus_4 <= EX_MEM_pc_plus_4; 
      // From others
      MEM_WB_mem_to_reg_src_1 <= data_out;
      MEM_WB_mem_to_reg_src_2 <= EX_MEM_alu_out;
      MEM_WB_rd <= EX_MEM_rd;
      MEM_WB_bcond <= EX_MEM_bcond; 
    end
  end

MUX4x1 mux_MemtoReg(
  .in3(0), 
  .in2(MEM_WB_pc_plus_4), 
  .in1(MEM_WB_mem_to_reg_src_1),
  .in0(MEM_WB_mem_to_reg_src_2),
  .in_bit(MEM_WB_mem_to_reg),
  .out(rd_din)
);

always @(*) begin
  if(ID_EX_is_jal|(ID_EX_is_branch & bcond)) begin
    pc_src=2'b01;
  end
  else if(ID_EX_is_jalr) begin
    pc_src=2'b10;
  end
  else begin
    pc_src=2'b00;
  end
end
  
endmodule
