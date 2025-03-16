
`include "vending_machine_def.v"
	

module calculate_current_state(clk, reset_n, i_input_coin,i_select_item,item_price,coin_value,current_total,
return_total,current_total_nxt,wait_time,o_available_item,o_output_item);

	input clk;
  input reset_n;
	
	input [`kNumCoins-1:0] i_input_coin;
	input [`kNumItems-1:0] i_select_item;			
	input [31:0] item_price [`kNumItems-1:0];
	input [31:0] coin_value [`kNumCoins-1:0];	
	input [`kTotalBits-1:0] current_total;
	input [31:0] wait_time;
	output reg [`kTotalBits-1:0] return_total;
	output [`kNumItems-1:0] o_available_item,o_output_item;
	output reg [`kTotalBits-1:0] current_total_nxt;
	integer i;	

	reg [`kTotalBits-1:0] input_total, output_total;
	reg [`kNumItems-1:0] reg_available_item, reg_output_item;
	reg [`kTotalBits-1:0] balance;
	reg [`kTotalBits-1:0] balance_nxt;

	assign o_available_item = reg_available_item;
	assign o_output_item = reg_output_item;

	initial begin
		current_total_nxt = 0;
		reg_available_item = 0;
		reg_output_item = 0;
		input_total = 0;
		output_total = 0;
		return_total = 0;
		balance = 0;
		balance_nxt = 0;
	end

	// Combinational logic for the next states
	always @(*) begin
		// TODO: current_total_nxt
		// You don't have to worry about concurrent activations in each input vector (or array).
		// Calculate the next current_total state.
		current_total_nxt = 0;

		if(current_total == 0) begin	// idle state and money input
			if(wait_time == 0) begin
				current_total_nxt = 2;
			end

			if(i_select_item != 0) begin
				current_total_nxt = 1;	
			end
		end
		else if(current_total == 1) begin	// item input
			if(wait_time == 0) begin
				current_total_nxt = 2;
			end
			
			if(i_input_coin != 0) begin
				current_total_nxt = 0;
			end
			
		end
		else if(current_total == 2) begin	// return change and exit
		end
  end


	always @(posedge clk) begin 
    if(!reset_n) begin
        input_total <= 0;
        output_total <= 0;
        return_total <= 0;
				balance_nxt <= 0; 
    end
    else begin
			if(current_total == 0 && i_input_coin != 0) begin
				for(i = 0; i < `kNumItems; i = i + 1) begin
					if(i_input_coin[i] == 1) begin
						input_total <= input_total + coin_value[i];
						balance_nxt <= balance + coin_value[i];
					end
				end
			end
			else if(i_select_item != 0) begin 
				for(i = 0; i < `kNumItems; i = i + 1) begin
					if(o_output_item[i] == 1) begin
						output_total <= output_total + item_price[i];
						balance_nxt <= balance - item_price[i];
					end
				end
			end
			else begin 
					return_total <= input_total - output_total;
			end
    end
	end


	always @(*)begin 
		balance = balance_nxt;
	end


	// Combinational logic for the outputs
	always @(*) begin
		// TODO: o_available_item
		// TODO: o_output_item
		case(current_total)	
			0,2: begin	// idle_state
				if(balance >= item_price[3]) begin
					reg_available_item = 4'b1111;
				end

				else if(balance >= item_price[2]) begin
					reg_available_item = 4'b0111;	
				end

				else if(balance >= item_price[1]) begin
					reg_available_item = 4'b0011;	
				end

				else if(balance >= item_price[0]) begin
					reg_available_item = 4'b0001;	
				end
		
				else begin
					reg_available_item = 4'b0000;
				end
				
				reg_output_item = reg_available_item & i_select_item;
			end

			default: begin
				reg_available_item = 4'b0000;
				reg_output_item = 4'b0000;
			end
		endcase
	end


endmodule 
