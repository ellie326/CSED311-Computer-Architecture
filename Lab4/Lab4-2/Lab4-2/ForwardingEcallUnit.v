module ForwardingEcallUnit(
    input [4:0] rs1,
    input [4:0] ID_EX_rd,
    input [4:0] EX_MEM_rd,
    input [4:0] MEM_WB_rd,
    input ID_EX_reg_write,
    input EX_MEM_reg_write,
    input MEM_WB_reg_write,
    output reg [1:0] forwardEcall
    );
    
    always @(*) begin
        if((rs1 == ID_EX_rd) && ID_EX_reg_write) begin
            forwardEcall = 2'b01;
        end
        else if((rs1 == EX_MEM_rd) && EX_MEM_reg_write) begin
            forwardEcall = 2'b10;
        end
        else if((rs1 == MEM_WB_rd) && MEM_WB_reg_write) begin
            forwardEcall = 2'b11;
        end
        else begin
            forwardEcall = 2'b00;
        end
    end
    
endmodule
