`include "opcodes.v"

`define MEM 2'b01
`define WB 2'b10

module ForwardingUnit(
    input [4:0] ID_EX_rs1,
    input [4:0] ID_EX_rs2,
    input [4:0] EX_MEM_rd,
    input EX_MEM_reg_write,
    input [4:0] MEM_WB_rd,
    input MEM_WB_reg_write,
    input [4:0] _x0,
    output reg [1:0] ForwardA,
    output reg [1:0] ForwardB);

    always @(*) begin
        if((ID_EX_rs1 != _x0) && (ID_EX_rs1 == EX_MEM_rd) && EX_MEM_reg_write) begin
            ForwardA = `MEM;
        end
        else if((ID_EX_rs1 != _x0) && (ID_EX_rs1 == MEM_WB_rd) && MEM_WB_reg_write) begin
            ForwardA = `WB;
        end
        else begin
            ForwardA = 2'b00;
        end

        if((ID_EX_rs2 != _x0) && (ID_EX_rs2 == EX_MEM_rd) && EX_MEM_reg_write) begin
            ForwardB = `MEM;
        end
        else if((ID_EX_rs2 != _x0) && (ID_EX_rs2 == MEM_WB_rd) && MEM_WB_reg_write) begin
            ForwardB = `WB;
        end
        else begin
            ForwardB = 2'b00;
        end
    end
    
endmodule
