module pc(input reset,
input clk,
input PCWrite,
input is_not_cache_stall,
input [31:0] next_pc,
output reg [31:0] current_pc);

    always @(posedge clk) begin
        if(reset) begin
            current_pc <= 0;
        end
        else begin
            if(PCWrite & is_not_cache_stall) begin
                current_pc <= next_pc;
            end
        end
  end
    
endmodule
