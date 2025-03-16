`define ADD 4'b0010
`define SUB 4'b0110
`define SLL 4'b1010
`define XOR 4'b1001
`define OR 4'b0001
`define AND 4'b0000
`define SRL 4'b1100

module alu(
    input [3:0] alu_op,
    input [1:0] btype,
    input [31:0] alu_in_1,
    input [31:0] alu_in_2,
    output reg [31:0] alu_result,
    output reg alu_bcond
    );
    
    always @(*) begin
        if(alu_op == `ADD)begin
            alu_result = alu_in_1 + alu_in_2;
            alu_bcond = 0;
        end
        else if(alu_op == `SUB)begin
            alu_result = alu_in_1 - alu_in_2;
            case (btype)
                2'b00: begin //beq
                    if(alu_result == 32'b0) begin
                        alu_result = alu_in_1 - alu_in_2;
                        alu_bcond = 1;
                    end
                    else begin
                        alu_result = alu_in_1 - alu_in_2;
                        alu_bcond = 0;
                    end
                end
                2'b01: begin //bne
                    if(alu_result != 32'b0) begin
                        alu_result = alu_in_1 - alu_in_2;
                        alu_bcond = 1;
                    end
                    else begin
                        alu_result = alu_in_1 - alu_in_2;
                        alu_bcond = 0;
                    end
                end
                2'b10: begin //blt
                    if(alu_result[31]) begin
                        alu_result = alu_in_1 - alu_in_2;
                        alu_bcond = 1;
                    end
                    else begin
                        alu_result = alu_in_1 - alu_in_2;
                        alu_bcond = 0;
                    end
                end
                2'b11: begin //bge
                    if(!alu_result[31]) begin
                        alu_result = alu_in_1 - alu_in_2;
                        alu_bcond = 1;
                    end
                    else begin
                        alu_result = alu_in_1 - alu_in_2;
                        alu_bcond = 0;
                    end
                end
                default: begin 
                    alu_result = alu_in_1 - alu_in_2;
                    alu_bcond = 0;
                end 
            endcase
        end 
        else if(alu_op == `AND)begin 
            alu_result = alu_in_1 & alu_in_2;
            alu_bcond = 0;
        end 
        else if(alu_op == `OR)begin 
            alu_result = alu_in_1 | alu_in_2;
            alu_bcond = 0;
        end 
        else if(alu_op == `XOR)begin 
            alu_result = alu_in_1 ^ alu_in_2;
            alu_bcond = 0;
        end 
        else if(alu_op == `SLL)begin 
            alu_result = alu_in_1 << alu_in_2;
            alu_bcond = 0;
        end 
        else if(alu_op == `SRL)begin 
            alu_result = alu_in_1 >> alu_in_2;
            alu_bcond = 0;
        end 
        else begin 
            alu_result = 0;
            alu_bcond = 0;
        end 
    end
endmodule
