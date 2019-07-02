`timescale 1ns / 1ps

module params_subs(
    input clk,
    input [31:0] data_in,
    output reg [3:0] we = 4'b0,
    output reg ea = 1'b1,
    output reg [31:0] addr = 32'b0,
    output reg [31:0] num_of_bits = 32'b0,
    output reg [31:0] num_of_pxl = 32'b0,
    output reg [31:0] num_of_subsets = 32'b0,
    output reg [31:0] width_ = 32'b0,
    output reg [31:0] height_ = 32'b0,
    output reg [31:0] param_bram_dout = 32'b0,
    output reg [31:0] optimization_method = 32'b0,
    output reg [31:0] correlation_routine = 32'b0,
    output reg param_done = 1'b0,
    input [31:0] param_start
);
    
    reg [5:0] state = 6'b000000;
    integer subset_counter = 0;
    reg [31:0] subset_range_selection;
    
    always @(posedge clk)
    begin
        case(state)
            6'b000000:
            begin     
                if(param_start == 32'b1)
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
                state = 6'b000010; 
            end
            6'b000010:
            begin
                state = 6'b000011; 
            end
            6'b000011:
            begin
                height_ = data_in;
                addr = addr + 4;
                state = 6'b000100; 
            end
            6'b000100:
            begin
                state = 6'b000101; 
            end
            6'b000101:
            begin
                state = 6'b000110; 
            end
            6'b000110:
            begin
                width_ = data_in;
                addr = addr + 4;
                state = 6'b000111; 
            end
            6'b000111:
            begin
                state = 6'b001000; 
            end            
            6'b001000:
            begin
                state = 6'b001001; 
            end
            6'b001001:
            begin
                num_of_pxl = data_in;
                addr = addr + 4;
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
                num_of_bits = data_in;
                addr = addr + 4;
                state = 6'b001101;
            end 
            6'b001101:
            begin
                state = 6'b001110; 
            end
            6'b001110:
            begin
                state = 6'b001111;
            end
            6'b001111:
            begin
                num_of_subsets = data_in;
                addr = addr + 4;
                state = 6'b010000;
            end
            6'b010000:
            begin
                state = 6'b010001;
            end
            6'b010001:
            begin
                state = 6'b010010;
            end
            6'b010010:
            begin
                optimization_method = data_in;
                addr = addr + 4;
                state = 6'b010011;
            end                       
            6'b010011:
            begin
                state = 6'b010100;
            end   
            6'b010100:
            begin
                state = 6'b010101;
            end
            6'b010101:
            begin
                correlation_routine = data_in;
                state = 6'b101100;
            end                      
            6'b101100:
            begin
                if(num_of_pxl != 32'b0)
                begin
                    param_done = 1'b1;
                end
            end         
         endcase
    end
endmodule
