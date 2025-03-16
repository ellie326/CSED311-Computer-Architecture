`include "CLOG2.v"

`define IDLE 2'b00
`define COMPARE_TAG 2'b01
`define ALLOCATE 2'b10
`define WRITE_BACK 2'b11


module Cache #(parameter LINE_SIZE = 16,
               parameter NUM_SETS = 16
               //parameter NUM_WAYS = 1,
               )                (
    input reset,
    input clk,

    input is_input_valid,
    input [31:0] addr,
    input mem_rw, // mem_rw == 0이면 read, 1이면 write
    input [31:0] din,

    output is_ready,
    output is_output_valid,
    output [31:0] dout,
    output is_hit);

  // Wire declarations
  wire [23:0] input_tag;
  wire [3:0] input_set_index;
  wire [1:0] input_block_offset;
  
  wire [127:0] cache_data_read;
  wire [23:0] cache_tag_read;
  wire cache_valid;
  wire cache_dirty;

  wire is_data_mem_ready;
  wire [31:0] clog2;
  wire [127:0] wire_dmem_dout;
  wire wire_dmem_is_output_valid;

  // Reg declarations
  reg [127:0] data_bank [0:NUM_SETS-1];
  reg [23:0] tag_bank [0:NUM_SETS-1];
  reg valid_table [0:NUM_SETS-1];
  reg dirty_table [0:NUM_SETS-1];
  
  reg [1:0] current_state;
  reg [1:0] next_state;

  reg [127:0] cache_write_data; // data에 덮어씌울 값
  reg [23:0] cache_write_tag;   // tag에 덮어씌울 값
  reg cache_valid_bit;          // 현재 access된 cache unit의 valid bit
  reg cache_dirty_bit;          // 현재 access된 cache unit의 dirty bit     
  reg cache_data_write_enable;  // cache data를 덮어쓸지 여부를 나타내는 시그널
  reg cache_tag_write_enable;   // tag data를 덮어쓸지 여부를 나타내는 시그널

  reg [31:0] dmem_addr; // Data memory의 input address. 
  reg [127:0] dmem_din; // Write Back 시 Data memory에 저장될 값.
  reg reg_dmem_is_input_valid;  // Data memory의 input이 valid한지 여부를 나타내는 시그널.
  reg dmem_read;  // Data memory read 시그널
  reg dmem_write; // Data memory write 시그널
  
  reg [31:0] cache_dout;  // 4개의 블록에 맞게 적절히 배치. cache hit & read 일 때 사용

  reg cache_is_output_valid;  // Cache의 작업이 끝났음을 나타내는 시그널. Cache hit 시 1로 활성화.

  integer i;
  
  // Hit rate computing
  reg [127:0] miss_count;
  reg [127:0] total_access;
  // You might need registers to keep the status.
  
  assign input_tag = addr[31:8];
  assign input_set_index = addr[7:4];
  assign input_block_offset = addr[3:2];
  assign clog2 = `CLOG2(LINE_SIZE);
  
  assign is_ready = is_data_mem_ready;
  assign is_output_valid = cache_is_output_valid;
  assign dout = cache_dout;
  assign is_hit = (input_tag == cache_tag_read) & cache_valid;
  
  always @(posedge clk) begin
    if(reset) begin
      for(i = 0; i < 16; i = i + 1) begin
        data_bank[i] <= 0;
        tag_bank[i] <= 0;
        valid_table[i] <= 0;
        dirty_table[i] <= 0;
      end
    end
  end
  
  assign cache_data_read = data_bank[input_set_index];
  assign cache_tag_read = tag_bank[input_set_index];
  assign cache_valid = valid_table[input_set_index];
  assign cache_dirty = dirty_table[input_set_index];
  
  always @(posedge clk) begin
      if(cache_data_write_enable) begin
        data_bank[input_set_index] <= cache_write_data;
      end
      if(cache_tag_write_enable) begin
        tag_bank[input_set_index] <= cache_write_tag;
        valid_table[input_set_index] <= cache_valid_bit;
        dirty_table[input_set_index] <= cache_dirty_bit;
      end
  end
  
  always @(posedge clk) begin
    if(reset) begin
      current_state <= `IDLE;
    end
    else begin
      current_state <= next_state;
    end
  end

  always @(posedge clk) begin
    if(reset) begin
      miss_count <= 0;
      total_access <= 0;
    end
    else begin
      case(current_state)
        `IDLE: begin end
        `COMPARE_TAG: begin
          if(is_hit) begin
            total_access <= total_access + 1;
          end
          else begin
            miss_count <= miss_count + 1;
          end
        end
        `ALLOCATE: begin end
        `WRITE_BACK: begin end
      endcase
      $display("total_access = %0d", total_access);
      $display("miss_count = %0d", miss_count);
      if (total_access > 0) begin
        real hit_ratio = (real'(total_access - miss_count)) * 100 / total_access;
        $display("hit ratio: %.2f", hit_ratio);
      end
      else begin
        $display("hit ratio: undefined (total_access is zero)");
      end
    end
  end
  
  always @(*) begin
    cache_write_tag = 0;
    cache_valid_bit = 0;
    cache_dirty_bit = 0;
    cache_tag_write_enable = 0;
    cache_data_write_enable = 0;
    reg_dmem_is_input_valid = 0; 
    dmem_read = 0;
    dmem_write = 0;
    cache_dout = 0;
    cache_is_output_valid = 0;
    dmem_addr = 0;
    cache_write_data = cache_data_read;
    dmem_din = cache_data_read;

    case(current_state)
      `IDLE: begin
        if(is_input_valid)
          next_state = `COMPARE_TAG;
        else begin
          next_state = `IDLE;
        end
      end

      `COMPARE_TAG: begin
        // cache hit
        if(is_hit) begin
          cache_is_output_valid = 1;
          next_state = `IDLE;
          
          if(mem_rw == 1) begin
            cache_tag_write_enable = 1; 
            cache_write_tag = cache_tag_read; 
            cache_valid_bit = 1;
            cache_dirty_bit = 1;

            cache_data_write_enable = 1;
            cache_write_data = cache_data_read;

            case(input_block_offset)
              2'b00: cache_write_data[31:0] = din;
              2'b01: cache_write_data[63:32] = din;
              2'b10: cache_write_data[95:64] = din;
              2'b11: cache_write_data[127:96] = din;
            endcase
          end
          case(input_block_offset) 
            2'b00: cache_dout = cache_data_read[31:0];
            2'b01: cache_dout = cache_data_read[63:32];
            2'b10: cache_dout = cache_data_read[95:64];
            2'b11: cache_dout = cache_data_read[127:96];
          endcase
        end
        // cache miss
        else begin
          reg_dmem_is_input_valid = 1; // Cache miss 시, Data memory로부터 데이터를 가져와야 하므로, 1로 설정
          if(cache_dirty == 0)begin
            dmem_read = 1;  // Cache miss 이고 dirty bit = 0 일 때, write-back을 할 필요가 없음.
            dmem_write = 0;
            dmem_addr = addr;
            next_state = `ALLOCATE;
          end
          else begin
            dmem_write = 1; // dirty bit = 1 이면 write-back 필요.
            dmem_addr = {cache_tag_read, addr[7:0]};
            dmem_din = cache_data_read;
            next_state = `WRITE_BACK;
          end
        end
      end

      `ALLOCATE: begin
        cache_tag_write_enable = 1;
        cache_valid_bit = 1;
        cache_write_tag = input_tag;
        cache_dirty_bit = mem_rw; 
       
        if(is_data_mem_ready) begin 
          cache_data_write_enable = 1;
          cache_write_data = wire_dmem_dout;
          next_state = `COMPARE_TAG;
        end
        else begin
          next_state = `ALLOCATE; // state 유지
        end
      end

       // write-back state 
      `WRITE_BACK: begin
        if(is_data_mem_ready) begin 
          reg_dmem_is_input_valid = 1;
          dmem_read = 1;
          dmem_addr = addr;
          next_state = `ALLOCATE; 
        end
        else begin
          next_state = `WRITE_BACK; // state 유지
        end
      end
    endcase
  end

  // Instantiate data memory
  DataMemory #(.BLOCK_SIZE(LINE_SIZE)) data_mem(
    .reset(reset),
    .clk(clk),
    
    .is_input_valid(reg_dmem_is_input_valid),
    .addr(dmem_addr >> clog2), // NOTE: address must be shifted by CLOG2(LINE_SIZE)
    .mem_read(dmem_read),
    .mem_write(dmem_write),
    .din(dmem_din),
    
    // is output from the data memory valid?
    .is_output_valid(wire_dmem_is_output_valid),
    .dout(wire_dmem_dout),
    // is data memory ready to accept request?
    .mem_ready(is_data_mem_ready)
  );

endmodule
