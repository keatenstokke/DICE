`timescale 1ns / 1ps

module Read_Write(
    input clock,
    input grad_busy,
    input grad_wea_ints,
    input [31:0] new_frame,
    input [16:0] grad_addr_ints,
    input [16:0] gamma_addr_ints_ref,
    input [16:0] gamma_addr_ints_def,
    output reg [3:0] out_grad_wea_ints,
    output reg [31:0] out_grad_gamma_addr_ints_ref,
    output reg [31:0] out_gamma_addr_ints_def
    );
    
    reg waiting = 1'b0;
    
    always @(posedge clock)
    begin
        if(new_frame == 32'b1)
        begin
            waiting = 1'b0;
            if(grad_wea_ints == 1'b1)
            begin
                out_grad_wea_ints = 4'b1111;
            end
            else
            begin
                out_grad_wea_ints = 4'b0000;
            end            
            if(grad_busy == 1'b1)
            begin
                out_grad_gamma_addr_ints_ref = grad_addr_ints * 4;
            end
            else
            begin
                out_grad_gamma_addr_ints_ref = gamma_addr_ints_ref * 4;
                out_gamma_addr_ints_def = gamma_addr_ints_def * 4;
            end
        end
        else
        begin
            waiting = 1'b1;
        end
    end
endmodule
