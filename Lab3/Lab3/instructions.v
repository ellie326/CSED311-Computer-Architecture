// Need to implement total 22 instructions - can express in 5 bits
`define INS_JUMPLINK_JAL 5'b00000
`define INS_JUMPLINK_JALR 5'b00001
`define INS_BRANCH_BEQ 5'b00010
`define INS_BRANCH_BNE 5'b00011
`define INS_BRANCH_BLT 5'b00100
`define INS_BRANCH_BGE 5'b00101
`define INS_LOAD_LW 5'b00110
`define INS_STORE_SW 5'b00111
`define INS_ARITHMETIC_IMM_ADDI 5'b01000
`define INS_ARITHMETIC_IMM_XORI 5'b01001
`define INS_ARITHMETIC_IMM_ORI 5'b01010
`define INS_ARITHMETIC_IMM_ANDI 5'b01011
`define INS_ARITHMETIC_IMM_SLLI 5'b01100
`define INS_ARITHMETIC_IMM_SRLI 5'b01101
`define INS_ARITHMETIC_ADD 5'b01110
`define INS_ARITHMETIC_SUB 5'b01111
`define INS_ARITHMETIC_SLL 5'b10000
`define INS_ARITHMETIC_XOR 5'b10001
`define INS_ARITHMETIC_SRL  5'b10010
`define INS_ARITHMETIC_OR 5'b10011
`define INS_ARITHMETIC_AND 5'b10100
`define INS_ECALL 5'b10101

`define INS_ALU_ADDER 5'b10110
