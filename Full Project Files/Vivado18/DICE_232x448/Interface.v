`timescale 1ns / 1ps

module Read_Write(
    input clock,
    input grad_done,
    input grad_wea_ints,
    input [31:0] BRAM_Full,
    input [11:0] grad_addr_ints,
    input [11:0] gamma_addr_ints_ref,
    input [11:0] gamma_addr_ints_def,
    output reg [3:0] out_grad_wea_ints,
    output reg [31:0] out_grad_gamma_addr_ints_ref,
    output reg [31:0] out_gamma_addr_ints_def,
    output reg in_if,
    output reg in_else,
    output reg in_always
    );
    
    //reg [5:0] state = 6'b000000;
    
    always @(posedge clock)
    begin
        in_if = 1'b0;
        in_else = 1'b0;
        in_always = 1'b0;
        if(BRAM_Full == 32'b1)
        begin
            in_if = 1'b1;
            if(grad_wea_ints == 1'b1)
            begin
                out_grad_wea_ints = 4'b1111;
            end
            else
            begin
                in_else = 1'b1;
                out_grad_wea_ints = 4'b0000;
            end            
            if(grad_done == 1'b0)
            begin
                out_grad_gamma_addr_ints_ref = grad_addr_ints * 4;
            end
            else
            begin
                out_grad_gamma_addr_ints_ref = gamma_addr_ints_ref * 4;
                out_gamma_addr_ints_def = gamma_addr_ints_def * 4;
            end
        end
        in_always = 1'b1;       
        /*if(BRAM_Full == 32'b1)
        begin
            out_grad_ints_ea_ref_def = grad_ints_ea_ref_def;
            if(grad_ints_wea_ref_def == 1'b1)
            begin
                out_grad_ints_wea_ref_def = 4'b1111;
            end
            else
            begin
                out_grad_ints_wea_ref_def = 4'b0000;
            end
            if(grad_done == 1'b0)
            begin
                out_grad_gamma_ints_addr_ref = grad_ints_addr_ref * 4;
                out_grad_gamma_ints_dout_ref = grad_ints_dout_ref;
            end
            else
            begin
                out_grad_gamma_ints_addr_ref = gamma_ints_addr_ref * 4;
                out_grad_gamma_ints_dout_ref = gamma_ints_dout_ref;
            end
            out_grad_ints_din_ref_def = grad_ints_din_ref_def;
            out_gamma_ints_addr_def = gamma_ints_addr_def * 4;
            out_gamma_ints_dout_def = gamma_ints_dout_def;
        end*/
        /*case(state)
        6'b000000:
        begin
            we_ref = 4'b0000;
            we_def = 4'b0000;
            ea_ref = 1'b1; 
            ea_def = 1'b1; 
            flag = 1'b0;
            addr_ref = 32'b000100000100; //104
            addr_def = 32'b000100000100; //104
            if (bram_data_in_def != 32'b0)
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
            addr_ref = 32'b000100000100; //104
            addr_def = 32'b000100000100; //104              
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
            bram_data_out_1_ref = bram_data_in_ref;
            bram_data_out_1_def = bram_data_in_def;
            addr_ref = 32'b00000000000000000000000011111100; //0FC
            addr_def = 32'b00000000000000000000000011111100; //0FC
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
            bram_data_out_2_ref = bram_data_in_ref;
            bram_data_out_2_def = bram_data_in_def;
            addr_ref = 32'b00000000000000000010111111111000; //2FF8
            addr_def = 32'b00000000000000000010111111111000; //2FF8
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
            bram_data_out_3_ref = bram_data_in_ref;
            bram_data_out_3_def = bram_data_in_def;
            addr_ref = 32'b01000111111000; //11F8
            addr_def = 32'b01000111111000; //11F8
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
            state = 6'b010000;
        end
        6'b010000:
        begin
            bram_data_out_4_ref = bram_data_in_ref;
            bram_data_out_4_def = bram_data_in_def;
            addr_ref = 32'b11111111111100; // 3FFC max
            addr_def = 32'b11111111111100; // 3FFC max
            state = 6'b010001;
        end
        6'b010001:
        begin
            state = 6'b010010;
        end
        6'b010010:
        begin
            state = 6'b010011;
        end
        6'b010011:
        begin
            state = 6'b010100;
        end
        6'b010100:
        begin
            bram_data_out_5_ref = bram_data_in_ref;
            bram_data_out_5_def = bram_data_in_def;
            state = 6'b010101;
        end
        6'b010101:
        begin
            flag = 1'b1;
            state = 6'b000001; 
        end
        endcase*/
    end
endmodule
