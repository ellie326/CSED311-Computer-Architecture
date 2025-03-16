`include "vending_machine_def.v"

   

module check_time_and_coin(clk,reset_n,wait_time,o_return_coin,return_total,current_total_nxt,i_trigger_return,coin_value);
   input clk;
   input reset_n;
   output  [`kNumCoins-1:0] o_return_coin;
   output reg [31:0] wait_time;
   input i_trigger_return;
   input [`kTotalBits-1:0] coin_value [`kNumCoins-1:0];
   input [`kTotalBits-1:0] return_total;
   output reg [`kTotalBits-1:0] current_total_nxt;

   reg [`kNumCoins-1:0] next_return_coin;
   reg [`kTotalBits-1:0] current_total_nxt_seq; 
   reg [`kTotalBits-1:0] reg_return_total;

   assign o_return_coin = next_return_coin;
   assign current_total_nxt = current_total_nxt_seq;
   

   // initiate values
   initial begin
      // TODO: initiate values
      wait_time = `kWaitTime + 1;
      next_return_coin = 0;
      reg_return_total = return_total;

   end


   always @(posedge clk ) begin
      reg_return_total <= return_total;

      if (!reset_n) begin
      // TODO: reset all states.
         wait_time <= `kWaitTime;        
      end
      else begin
      // TODO: update all states.
         if(i_trigger_return) begin // return trigger  
            wait_time <= 3;
            current_total_nxt_seq <= 0;
         end
         else begin
            wait_time <= wait_time - 1;
         end


         if(reg_return_total >= coin_value[2]) begin
            next_return_coin <= `kNumCoins'b100;
            reg_return_total <= reg_return_total - coin_value[2];
         end
         else if(reg_return_total >= coin_value[1]) begin
            next_return_coin <= `kNumCoins'b010;
            reg_return_total <= reg_return_total - coin_value[1];
         end
         else if(reg_return_total >= coin_value[0]) begin
            next_return_coin <= `kNumCoins'b001;
            reg_return_total <= reg_return_total - coin_value[0];
         end
         else begin
            next_return_coin <= `kNumCoins'b000;
         end
      end
   end
endmodule 
