module HazardDetectUnit(
    input [4:0] rs1,
    input [4:0] rs2,
    input [4:0] ID_EX_rd,
    input ID_EX_mem_read,
    input use_rs1,
    input use_rs2,
    output reg PCWrite,
    output reg IF_IDWrite,
    output reg hazard_detected);


    always @(*) begin
        if(((((rs1 == ID_EX_rd) && use_rs1)) || (((rs2 == ID_EX_rd) && use_rs2))) && ID_EX_mem_read) begin
            hazard_detected = 1;
            IF_IDWrite = 0;
            PCWrite = 0;
        end
        else begin
            hazard_detected = 0;
            IF_IDWrite = 1;
            PCWrite = 1;
        end
    end
    
endmodule
