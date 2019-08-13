`timescale 1ns / 1ps

module Read_Write(
    input clock,
    input grad_busy,
    input grad_wea_ints,
    input [31:0] new_frame,
    input [16:0] grad_addr_ints,
    input [16:0] gamma_addr_ints_ref,
    input [16:0] gamma_addr_ints_def,
    input [31:0] frame_counter,
    input [31:0] img_in_0,
    input [31:0] img_in_1,
    output reg [31:0] ref_img_out,
    output reg [31:0] def_img_out,
    output reg [3:0] out_grad_wea_ints,
    output reg [31:0] out_grad_gamma_addr_ints_0,
    output reg [31:0] out_grad_gamma_addr_ints_1
);
    
    reg waiting = 1'b0;
    reg state = 6'b0;
    
    always @(posedge clock)
    begin
    
        case(state)
        
        6'b000000:
        begin
            if(new_frame == 32'b1)
            begin
                state = 6'b000001;
                waiting = 1'b0;
            end
            else
            begin
                state = 6'b000000;
                waiting = 1'b1;
            end
        end
        
        6'b000001:
        begin
            if(grad_wea_ints == 1'b1)
            begin
                out_grad_wea_ints = 4'b1111;
            end
            else
            begin
                out_grad_wea_ints = 4'b0000;
            end
            state = 6'b000010;
        end
        
        6'b000010:
        begin
            // Read reference image to Gradients
            if(grad_busy == 1'b1)
            begin
                // BRAM_0 = Ref; BRAM_1 = Def
                if(frame_counter == 32'b1 || frame_counter == 32'b10)
                begin
                    out_grad_gamma_addr_ints_0 = grad_addr_ints * 4;
                end
                // BRAM_0 = Ref; BRAM_1 = Def
                else if(frame_counter > 32'b10 && frame_counter % 32'b10 == 32'b0)
                begin
                    out_grad_gamma_addr_ints_0 = grad_addr_ints * 4;
                end
                // BRAM_0 = Def; BRAM_1 = Ref
                else if(frame_counter > 32'b10 && frame_counter % 32'b10 == 32'b1)
                begin
                    out_grad_gamma_addr_ints_1 = grad_addr_ints * 4;
                end
            end
            
            // Read reference and deformed images to Gamma
            else
            begin
                // BRAM_0 = Ref; BRAM_1 = Def
                if(frame_counter == 32'b1 || frame_counter == 32'b10)
                begin
                    out_grad_gamma_addr_ints_0 = gamma_addr_ints_ref * 4;
                    out_grad_gamma_addr_ints_1 = gamma_addr_ints_def * 4;
                end
                // BRAM_0 = Ref; BRAM_1 = Def
                else if(frame_counter > 32'b10 && frame_counter % 32'b10 == 32'b0)
                begin
                    out_grad_gamma_addr_ints_0 = gamma_addr_ints_ref * 4;
                    out_grad_gamma_addr_ints_1 = gamma_addr_ints_def * 4;
                end
                // BRAM_0 = Def; BRAM_1 = Ref
                else if(frame_counter > 32'b10 && frame_counter % 32'b10 == 32'b1)
                begin
                    out_grad_gamma_addr_ints_0 = gamma_addr_ints_def * 4;
                    out_grad_gamma_addr_ints_1 = gamma_addr_ints_ref * 4;
                end
            end
        
            state = 6'b000011;
        end
        
        6'b000011:
        begin
            state = 6'b000100;
        end
        
        6'b000100:
        begin
            state = 6'b000101;
        end
        
        6'b000101:
        begin
            // Flip image data (original MUX)
            // BRAM_0 = Ref; BRAM_1 = Def
            if(frame_counter == 32'b1 || frame_counter == 32'b10)
            begin             
                ref_img_out[31:0] = img_in_0[31:0];
                def_img_out[31:0] = img_in_1[31:0];
            end
            // BRAM_0 = Ref; BRAM_1 = Def
            else if(frame_counter > 32'b10 && frame_counter % 32'b10 == 32'b0)
            begin
                ref_img_out[31:0] = img_in_0[31:0];
                def_img_out[31:0] = img_in_1[31:0];
            end
            // BRAM_0 = Def; BRAM_1 = Ref
            else if(frame_counter > 32'b10 && frame_counter % 32'b10 == 32'b1)
            begin
                ref_img_out[31:0] = img_in_1[31:0];
                def_img_out[31:0] = img_in_0[31:0];
            end
            state = 6'b000010;
        end
        
        endcase

    end
endmodule
