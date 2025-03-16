module MUX2X1_2bit (input in_bit,
input [4:0] in1,
input [4:0] in0,
output [4:0] out);

    assign out = in_bit ? in1 : in0;

endmodule
