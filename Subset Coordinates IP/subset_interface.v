`timescale 1ns / 1ps

module Coords_Intr(
    input clock,
    input coord_done,
    input parameters_done,
    input [31:0] param_dout,
    input [31:0] coord_subset_number,
    input coord_new_subset,
    output reg param_ea,
    output reg [3:0] param_wea,
    output reg [31:0] param_addr,
    output reg [31:0] coord_cx,
    output reg [31:0] coord_cy,
    output reg [31:0] subset_size,
    output reg [31:0] half_subset_size,
    output reg [31:0] subset_shape,
    output reg coord_interface_done = 1'b0
);
    
    reg [5:0] state = 6'b000000;  
    
    always @(posedge clock) 
    begin   
        case(state)
        'b000000:
        begin
            if(parameters_done == 1'b1)
            begin
                param_ea = 1'b1;
                param_wea = 'b0;
                coord_interface_done = 1'b0;
                if(coord_new_subset == 1'b1)
                begin
                    coord_interface_done = 1'b0; 
                    state = 'b000001;
                end
                else
                begin
                    coord_interface_done = 1'b0;
                    state = 'b000000; //wait for a new_subset
                end
            end
            else
            begin
                state = 'b000000;  
            end
        end
        'b000001:
        begin 
            param_addr = ((((coord_subset_number + 1) * 5) + 3) * 4); //to read cx
            state = 'b000010;             
        end
        'b000010:
        begin
            state = 'b000011; 
        end
        'b000011:
        begin
            state = 'b000100;
        end
        'b000100:
        begin
            coord_cx = param_dout;
            param_addr = ((((coord_subset_number + 1) * 5) + 4) * 4); //to read cy
            state = 'b000101; 
        end
        'b000101:
        begin
            state = 'b000110; 
        end
        'b000110:
        begin
            state = 'b000111; 
        end
        'b000111:
        begin
            coord_cy = param_dout;
            param_addr = ((((coord_subset_number + 2) * 5)) * 4); //to read size
            state = 'b001000; 
        end            
        'b001000:
        begin
            state = 'b001001; 
        end
        'b001001:
        begin
            state = 'b001010;
        end
        'b001010:
        begin
            subset_size = param_dout;
            param_addr = ((((coord_subset_number + 2) * 5) + 1) * 4); //to read haf_size
            state = 'b001011; 
        end
        'b001011:
        begin
            state = 'b001100;
        end
        'b001100:
        begin
            state = 'b001101; 
        end 
        'b001101:
        begin
            half_subset_size = param_dout;
            param_addr = ((((coord_subset_number + 1) * 5) + 2) * 4); //to read shape
            state = 'b001110; 
        end
        'b001110:
        begin
            state = 'b001111;
        end
        'b001111:
        begin
            state = 'b010000;
        end
        'b010000:
        begin
            subset_shape = param_dout;
            coord_interface_done = 1'b1; 
            state = 'b010001;
        end
        'b010001:
        begin
            state = 'b010010;
        end
        'b010010:
        begin
            state = 'b000000;
        end
        endcase
    end
endmodule
