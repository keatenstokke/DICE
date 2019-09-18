`timescale 1ns / 1ps

module Save_Results(
    input clock,
    input gamma_done,
    input [31:0] dis_X,
    input [31:0] dis_Y,
    input [31:0] dis_Z,
    output reg [31:0] addr,
    output reg [3:0] we,
    output reg ea,
    output reg [31:0] result_done,
    output reg [31:0] din,
    output reg [31:0] Save_Done
    );
    
    reg [5:0] state = 6'b000000;
    
    always @(posedge clock)
    begin              
        case(state)
        6'b000000:
        begin
            we = 4'b1111;
            ea = 1'b1; 
            addr = 32'b0; //0
            Save_Done = 32'b0;
            result_done = 32'b0;
            if (gamma_done == 1'b1)
            begin             
                state = 6'b000001;
            end
            else
            begin
                state = 6'b000000;  
            end
        end
        6'b000001:
        begin
            addr = 32'b0; //0
            din = dis_X;             
            state = 6'b000010;             
        end
        6'b000010:
        begin
            state = 6'b000011; 
        end
        6'b000011:
        begin
            state = 6'b000100;
        end
        6'b000100:
        begin
            addr = 32'b0100; //4
            din = dis_Y; 
            state = 6'b000101; 
        end
        6'b000101:
        begin
            state = 6'b000110; 
        end
        6'b000110:
        begin
            state = 6'b000111; 
        end
        6'b000111:
        begin
            state = 6'b001000; 
        end            
        6'b001000:
        begin
            addr = 32'b1000; //8
            din = dis_Z; 
            state = 6'b001001; 
        end
        6'b001001:
        begin
            state = 6'b001010;
        end
        6'b001010:
        begin
            state = 6'b001011; 
        end
        6'b001011:
        begin
            state = 6'b001100;
        end
        6'b001100:
        begin
            result_done = 32'b1;
            Save_Done = 32'b1;
            state = 6'b000001; 
        end
        endcase
    end
endmodule
