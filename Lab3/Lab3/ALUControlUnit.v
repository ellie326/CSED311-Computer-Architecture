`include "opcodes.v"

`define ADD 4'b0010
`define SUB 4'b0110
`define SLL 4'b1010
`define XOR 4'b1001
`define OR 4'b0001
`define AND 4'b0000
`define SRL 4'b1100

module ALUControlUnit(
    input [1:0] ALUOp,
    input [10:0] part_of_inst,
    output reg [3:0] alu_op,
    output reg [1:0] btype
    );
    wire [6:0] opcode;
    wire [2:0] funct3;
    wire funct7;
    
    assign opcode = part_of_inst[6:0];
    assign funct3 = part_of_inst[9:7];
    assign funct7 = part_of_inst[10];
    
    always @(*) begin
        case (ALUOp)
            2'b00: begin //load or store -> add 
                alu_op = `ADD;   
                btype = 2'b00;
            end
            2'b01: begin //branch instruction -> subtract 
                alu_op = `SUB;
                btype = 2'b00; 
                case(funct3)
                    `FUNCT3_BEQ: begin
                        alu_op = `SUB;  
                        btype = 2'b00;
                    end
                    `FUNCT3_BNE: begin
                        alu_op = `SUB;  
                        btype = 2'b01;
                    end
                    `FUNCT3_BLT: begin
                        alu_op = `SUB;  
                        btype = 2'b10;
                    end
                    `FUNCT3_BGE: begin
                        alu_op = `SUB;  
                        btype = 2'b11;
                    end
                    default: begin
                        alu_op = `SUB; 
                        btype = 2'b00; 
                     end
                endcase  
            end
            2'b10: begin //Rtype 
                if(opcode == `ARITHMETIC && funct7) begin //funct7 != 0 -> subtract 
                    alu_op = `SUB;
                    btype = 2'b00; 
                end
                else begin
                    alu_op = 4'b0000;
                    btype = 2'b00; 
                    case(funct3)
                        `FUNCT3_ADD: begin
                            alu_op = `ADD;
                            btype = 2'b00; 
                        end
                        `FUNCT3_SLL: begin
                            alu_op = `SLL;
                            btype = 2'b00; 
                        end
                        `FUNCT3_XOR: begin
                            alu_op = `XOR;
                            btype = 2'b00; 
                        end
                        `FUNCT3_OR: begin
                            alu_op = `OR;
                            btype = 2'b00; 
                        end
                        `FUNCT3_AND: begin
                            alu_op = `AND;
                            btype = 2'b00; 
                        end
                        `FUNCT3_SRL: begin
                            alu_op = `SRL;
                            btype = 2'b00; 
                        end
                        default: begin 
                            alu_op = 4'b0000;
                            btype = 2'b00; 
                        end 
                    endcase
                end
            end
            default:begin 
                alu_op = 4'b0000;
                btype = 2'b00; 
            end
        endcase
    end
    
endmodule
