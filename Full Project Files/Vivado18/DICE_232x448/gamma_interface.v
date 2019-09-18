`timescale 1ns / 1ps

module gamma_interface(
    input clock,
    input gam_new_subset,
    input subset_done,
    input [31:0] num_of_subsets,
    input [31:0] subset_counter,
    input [31:0] gam_subset_number,
    input parameters_done,
    input [31:0] param_dout,
    input [31:0] base_address,
    output reg param_ea,
    output reg [3:0] param_wea,
    output reg [31:0] param_addr,
    output reg [31:0] gam_cx,
    output reg [31:0] gam_cy,
    output reg gam_interface_done = 1'b0,
    output reg [31:0] base_addr_out = 32'b0
);
    
    reg [5:0] state = 6'b000000;  
    reg [447:0] base_addr_reg = 448'b0;
    reg [31:0] subset_range = 32'b0;
    
    always @(posedge clock) 
    begin  
        case(state)
        'b000000:
        begin
            if(subset_done == 1'b0)
            begin
                // Read in and save the base address
                if(subset_counter < num_of_subsets)
                begin
                    subset_range = subset_counter * 32 + 31;
                    base_addr_reg[subset_range-:32] = base_address;
                    state = 6'b000000;
                end
                else
                begin
                    // Go to start state
                    state = 6'b001010;
                end
            end
            else
            begin
                // Subsets is done procesing, so we can start
                state = 6'b001010;
            end
        end
        'b001010:
        begin
            
        
        
        end    
        'b000001:
        begin
            if(parameters_done == 1'b1)
            begin
                param_ea = 1'b1;
                param_wea = 'b0;
                if(gam_new_subset == 1'b1)
                begin
                    gam_interface_done = 1'b0; 
                    subset_range = gam_subset_number * 32 + 31;
                    base_addr_out = base_addr_reg[subset_range-:32];
                    state = 'b000010;
                end
                else
                begin
                    state = 'b000001; //wait for a new_subset
                end
            end
            else
            begin
                state = 'b000001;  
            end
        end
        'b000010:
        begin 
            param_addr = ((((gam_subset_number + 1) * 5) + 3) * 4); //to read cx
            state = 'b000011;             
        end
        'b000011:
        begin
            state = 'b000100; 
        end
        'b000100:
        begin
            state = 'b000101;
        end
        'b000101:
        begin
            gam_cx = param_dout;
            param_addr = ((((gam_subset_number + 1) * 5) + 4) * 4); //to read cy
            state = 'b000110; 
        end
        'b000110:
        begin
            state = 'b000111; 
        end
        'b000111:
        begin
            state = 'b001000; 
        end
        'b001000:
        begin
            gam_cy = param_dout;
            state = 'b001001; 
        end            
        'b001001:
        begin
            gam_interface_done = 1'b1;
            state = 'b000001;
        end
        endcase
    end   
endmodule
