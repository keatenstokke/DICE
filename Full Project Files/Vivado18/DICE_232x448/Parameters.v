`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/27/2018 03:35:00 PM
// Design Name: 
// Module Name: params
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
module param(
		clk,
		data_in,
		we,
		ea,
		addr,
		num_of_bits,
		num_of_pxl,
		num_of_subsets,
		width_,
		height_,
		subset_size,
		half_subset_size,
		subset_centerpoint_x,
		subset_centerpoint_y,
		param_bram_dout,
		optimization_method,
		correlation_routine,
		param_done
    );
    input clk;
    input [31:0] data_in;
    output reg we = 1'b0;
    output reg ea = 1'b1;
    output reg [3:0] addr = 4'b0;
    output reg [31:0] num_of_bits = 32'b1;
    output reg [31:0] num_of_pxl = 32'b1;
    output reg [31:0] num_of_subsets = 32'b0;
    output reg [31:0] width_ = 32'b1;
    output reg [31:0] height_ = 32'b1;
    output reg [31:0] subset_size = 32'b0;
    output reg [31:0] half_subset_size = 32'b1;
    output reg [31:0] subset_centerpoint_x = 32'b1;
    output reg [31:0] subset_centerpoint_y = 32'b1;
    output reg [31:0] param_bram_dout = 32'b0;
    output reg [31:0] optimization_method = 32'b10;
    output reg [31:0] correlation_routine = 32'b10;
    output reg param_done = 1'b0;
    
    reg [5:0] state = 6'b000000;
    
    always @(posedge clk)
    begin
        case(state)
            6'b000000:
            begin             
                state = 6'b000001; 
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
                addr = 4'b0001;
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
                addr = 4'b0010;
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
                addr = 4'b0011;
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
                addr = 4'b0100;
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
                addr = 4'b0101;
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
                subset_size = data_in;
                addr = 4'b0110;
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
                half_subset_size = data_in;
                addr = 4'b0111;
                state = 6'b010110;
            end
            6'b010110:
            begin
                state = 6'b010111;
            end 
            6'b010111:
            begin
                state = 6'b011000;
            end    
            6'b011000:
            begin
                subset_centerpoint_x = data_in;
                addr = 4'b1000;
                state = 6'b011001;
            end  
            6'b011001:
            begin
                state = 6'b011010;
            end              
            6'b011010:
            begin
                state = 6'b011011;
            end                      
            6'b011011:
            begin
                subset_centerpoint_y = data_in;
                addr = 4'b1001;
                state = 6'b011100;
            end     
            6'b011100:
            begin
                state = 6'b011101;
            end              
            6'b011101:
            begin
                state = 6'b011110;
            end              
            6'b011110:
            begin
                optimization_method = data_in;
                addr = 4'b1010;
                state = 6'b011111;
            end                       
            6'b011111:
            begin
                state = 6'b100000;
            end   
            6'b100000:
            begin
                state = 6'b100001;
            end
            6'b100001:
            begin
                correlation_routine = data_in;
                state = 6'b100010;
            end  
            6'b100010:
            begin
                state = 6'b100011;
            end  
            6'b100011:
            begin
                state = 6'b100100;
            end                          
            6'b100100:
            begin
                if(num_of_subsets != 32'b0 && num_of_pxl != 32'b1 && correlation_routine != 32'b10 && optimization_method != 32'b10 && num_of_bits != 32'b1 && subset_size != 32'b0 && width_ != 32'b1 && height_ != 32'b1 && subset_centerpoint_x != 32'b1 && subset_centerpoint_y != 32'b1 && half_subset_size != 32'b1)
                begin
                    param_done = 1'b1;
                end
            end         
         endcase
    end
endmodule