`include "opcodes.v"
`include "Stages.v"

module Address_select_logic(
    input [6:0] opcode,
    input [6:0] data,
    input [2:0] add_state,
    input [1:0] AddrCtrl,
    input IRWrite,
    output [2:0] next_state
    );
    
    reg [2:0] ROM1;
    reg [2:0] ROM2;
    wire [2:0] ROM1_wire;
    wire [2:0] ROM2_wire;
    
    assign ROM1_wire[2:0] = ROM1;
    assign ROM2_wire[2:0] = ROM2;
    
    initial begin
        ROM1 = 0;
        ROM2 = 0;
    end
    
    always @(*) begin
        if(IRWrite && data == `JAL) begin // JAL 명령어에 대한 예외 처리: 프로그램 진행을 바꾸므로, 예외적으로 다뤄야 함.
            ROM1 = `EX;
            ROM2 = `IF;
        end
        else if(opcode == `ARITHMETIC) begin // R-type 연산 처리
            ROM1 = `EX;
            ROM2 = `IF;
        end
        else if(opcode == `BRANCH) begin
            ROM1 = `B;
            ROM2 = `IF;
        end
        else if(opcode == `STORE) begin
            ROM1 = `MEM;
            ROM2 = `IF; 
        end
        else if(opcode == `LOAD) begin
            ROM1 = `MEM;
            ROM2 = `WB;
        end
        else begin 
            ROM1 = `IF;
            ROM2 = `IF;
        end 
    end

    MUX4x1_2bit next_state_MUX(
        .in_bit(AddrCtrl),
        .in3(0),
        .in2(ROM1_wire),    // ROM1, ROM2 순서 바꿈.
        .in1(ROM2_wire),
        .in0(add_state),
        .out(next_state)
    );

endmodule
