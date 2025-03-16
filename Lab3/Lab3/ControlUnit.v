`include "opcodes.v"
`include "Stages.v"


module ControlUnit(
    input reset,
    input clk,
    input [6:0] part_of_inst, // input
    input PCcond,
    input [6:0] data,
    output reg PCWrite,
    output reg IorD,
    output reg MemRead,
    output reg MemWrite,
    output reg [1:0] MemtoReg,
    output reg IRWrite,
    output reg PCSource,
    output reg [1:0] ALUOp,
    output reg [1:0] ALUSrcB,
    output reg ALUSrcA,
    output reg RegWrite,
    output reg is_ecall // output (ecall inst)
);
    
    reg [2:0] state;
    wire [2:0] state_wire;
    wire [2:0] next_state;
    wire [2:0] add_state;
    wire [1:0] AddrCtrl;
    wire IRWrite_wire;
    
    assign state_wire = state;
    assign IRWrite_wire = IRWrite;

    // AddrCtrl[1], [0] 순서 바꿈.
    assign AddrCtrl[1] = (IRWrite && (data == `JAL) && (state == `IF)) || 
                         ((part_of_inst == `STORE || part_of_inst == `LOAD || (PCcond && (part_of_inst == `BRANCH))) && (state == `EX)) ||
                         ((!(PCcond) && (part_of_inst == `BRANCH)) && (state == `EX)) ||
                         ((part_of_inst == `STORE) && (state == `MEM)) ||
                         (state == `WB) ||
                         (state == `B) ||
                         (part_of_inst == `ECALL);

    assign AddrCtrl[0] = ((part_of_inst == `LOAD) && (state == `MEM)) ||
                         ((part_of_inst == `STORE) && (state == `MEM)) ||
                         ((!(PCcond) && (part_of_inst == `BRANCH)) && (state == `EX)) ||
                         (state == `WB) ||
                         (state == `B) ||
                         (part_of_inst == `ECALL);
    
    initial begin
        PCWrite = 0;
        IorD = 0;
        MemRead = 0;
        MemtoReg = 2'b00;
        MemWrite = 0;
        IRWrite = 0;
        PCSource = 0;
        ALUOp = 2'b00;
        ALUSrcB = 2'b00;
        ALUSrcA = 0;
        RegWrite = 0;
        is_ecall = 0;
        state = `IF;
    end
    
    Adder Add1(
        .A(state_wire),
        .B(1),
        .result(add_state)
    );
    
    Address_select_logic Address_select_logic(
        .opcode(part_of_inst),
        .data(data),
        .add_state(add_state),
        .AddrCtrl(AddrCtrl),
        .IRWrite(IRWrite_wire),
        .next_state(next_state)
    );

    always @(posedge clk) begin
        if(reset) begin
            state <= `IF;
        end
        else begin
            state <= next_state;
        end
    end

    always @(*) begin
        PCWrite = 0;
        IorD = 0;
        MemRead = 0;
        MemtoReg = 2'b00;
        MemWrite = 0;
        IRWrite = 0;
        PCSource = 0;
        ALUOp = 2'b00;
        ALUSrcB = 2'b00;
        ALUSrcA = 0;
        RegWrite = 0;
        is_ecall = 0;

        if(state == `IF) begin
            //PCWrite = 0;
            IorD = 0;
            MemRead = 1;
            MemtoReg = 2'b00;
            MemWrite = 0;
            IRWrite = 1;
            PCSource = 0;
            ALUOp = 2'b00;
            ALUSrcB = 2'b01;
            ALUSrcA = 0;
            RegWrite = 0;
            //is_ecall = 0;
            
            if(data == `ECALL) begin
                PCWrite = 1;
                is_ecall = 1;
            end
            else begin
                PCWrite = 0;
                is_ecall = 0;
            end
        end

        else if(state == `ID) begin
            PCWrite = 0;
            IorD = 0;
            MemRead = 1;
            MemtoReg = 2'b00;
            MemWrite = 0;
            IRWrite = 1;
            PCSource = 0;
            ALUOp = 2'b00;
            ALUSrcB = 2'b01;
            ALUSrcA = 0;
            RegWrite = 0;
            is_ecall = 0;
        end

        else if(state == `EX) begin
            PCWrite = 0;
            IorD = 0;
            MemRead = 0;
            MemtoReg = 2'b00;
            MemWrite = 0;
            IRWrite = 0;
            PCSource = 0;
            ALUOp = 2'b00;
            ALUSrcB = 2'b00;
            ALUSrcA = 0;
            RegWrite = 0;
            is_ecall = 0;

            if(part_of_inst == `BRANCH) begin
                if(!PCcond) begin
                    PCWrite = 1;
                    PCSource = 1;
                end

                ALUOp[0] = 1;
                ALUSrcA = 1;
            end

            if(part_of_inst == `ARITHMETIC) begin
                ALUOp[1] = 1;
                ALUSrcA = 1;
            end

            if(part_of_inst == `ARITHMETIC_IMM) begin
                ALUOp[1] = 1;
                ALUSrcB[1] = 1;
                ALUSrcA = 1;
            end

            if(part_of_inst == `LOAD || part_of_inst == `STORE) begin
                ALUSrcB[1] = 1;
                ALUSrcA = 1;
            end

            if(part_of_inst == `JAL) begin
                ALUSrcB[1] = 1;
            end

            if(part_of_inst == `JALR) begin
                ALUSrcB[1] = 1;
                ALUSrcA = 1;
            end
        end
        
        else if(state == `B) begin
            PCWrite = 1;
            IorD = 0;
            MemRead = 0;
            MemtoReg = 2'b00;
            MemWrite = 0;
            IRWrite = 0;
            PCSource = 0;
            ALUOp = 2'b00;
            ALUSrcB = 2'b10;
            ALUSrcA = 0;
            RegWrite = 0;
            is_ecall = 0;
        end

        else if(state == `MEM) begin
            PCWrite = 0;
            IorD = 0;
            MemRead = 0;
            MemtoReg = 2'b00;
            MemWrite = 0;
            IRWrite = 0;
            PCSource = 0;
            ALUOp = 2'b00;
            ALUSrcB = 2'b00;
            ALUSrcA = 0;
            RegWrite = 0;
            is_ecall = 0;

            if(part_of_inst == `STORE) begin
                PCWrite = 1;
                IorD = 1;
                MemWrite = 1;
                ALUSrcB[0] = 1;
            end
            if(part_of_inst == `LOAD) begin
                IorD = 1;
                MemRead = 1;
            end
        end

        else if (state == `WB) begin
            PCWrite = 1;
            IorD = 0;
            MemRead = 0;
            MemtoReg = 2'b00;
            MemWrite = 0;
            IRWrite = 0;
            PCSource = 0;
            ALUOp = 2'b00;
            ALUSrcB = 2'b01;
            ALUSrcA = 0;
            RegWrite = 1;
            is_ecall = 0;

            if(part_of_inst == `LOAD) begin
                MemtoReg[0] = 1;
            end
            if(part_of_inst == `JAL || part_of_inst == `JALR) begin
                MemtoReg[1] = 1;
                PCSource = 1;
            end
        end
    end
    
endmodule
