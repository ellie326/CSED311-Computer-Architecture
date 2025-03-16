module MUX2x1(input in_bit,
input [31:0] in1,
input [31:0] in0,
output [31:0] out);

    assign out = in_bit ? in1 : in0;

endmodule
