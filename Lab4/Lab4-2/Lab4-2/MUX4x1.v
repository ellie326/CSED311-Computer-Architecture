module MUX4x1(input [1:0] in_bit,
    input [31:0] in3,
    input [31:0] in2,
    input [31:0] in1,
    input [31:0] in0,
    output [31:0] out);

    assign out = in_bit[1] ? (in_bit[0] ? in3 : in2) : (in_bit[0] ? in1 : in0);
endmodule
