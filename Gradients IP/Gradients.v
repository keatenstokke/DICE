`timescale 1ns / 1ps

module Gradients_Multi_Subs_Multi_shapes_232_448(
    input clk,
    input param_ready,
    input [31:0] width_,
    input [31:0] height_,
    input [31:0] dout_ints,
    output reg grad_ea,
    output reg ints_ea,
    output reg grad_wea,
    output reg ints_wea,
    output reg [16:0] addr_ints,
    output reg [16:0] addr_grad_x_,
    output reg [16:0] addr_grad_y_,
    output reg [31:0] din_grad_x_,
    output reg [31:0] din_grad_y_, 
    output [31:0] din_ints, 
    output reg grad_done,
    input [31:0] new_frame,
    input [31:0] in_frame_counter,
    output reg [31:0] out_frame_counter,
    output reg grad_busy = 1'b0,
    output reg [127:0] grad_idle_counter = 128'b0
);


parameter image_size = 3325951; // 8 pixels, 256 bits, 255 with -1 already subtracted
                                // 103,936 pixels, 3,325,952 bits, 3,325,951 with -1 subtracted
reg [5:0] state =  6'b0; 
reg [5:0] temp_state;
reg [31:0] width_2 = 32'b110111110; //446
reg [31:0] height_2 = 32'b11100110; //230
reg [31:0] grad_c1_ = 32'b00111101101010011111101111100111; //0.083
reg [31:0] grad_c2_ = 32'b10111111010000000000000000000000; //-0.75
reg [31:0] t1, t2, t3, t4, t5, t6;

reg [31:0] Multiplier_Float;
reg [7:0]e_sum,exponent;
reg [47:0]prod;
reg [23:0] product;
reg multiplier_done;

reg done;
reg [31:0] a, b, Subtractor_Float, Adder_Float;
reg [7:0]e1,e2,exy;
reg s1,s2,sr,sign;
reg [23:0]m1,m2,mx,my;
reg [24:0]mxy,mxy2;
reg [7:0] diff,x;

//for debug
integer debug_in_b001 = 0;
reg debug_in_b010_else;
integer debug_in_b001_if = 0;
reg [31:0] debug_a_in_b010, debug_a_in_b100000, debug_a_in_b100101, debug_a_i_b1001;
reg [31:0] debug_grad_out, debug_grad_inside;
reg [31:0] debug_addr_grad_x_;
   
integer s, t, i; // loop index counters
integer counter = 0;
integer factor = 32;
integer counter_g = 0;
reg [1:0] clk_counter_a = 2'b00;
reg [1:0] clk_counter_b = 2'b00;


always @(posedge clk)
begin
    out_frame_counter = in_frame_counter;
        case(state)
        // State 0
        6'b000:
        begin
            t = 0;
            if(param_ready == 1'b1 && new_frame == 32'b1)
            begin
                state = 6'b001;
                grad_ea = 1'b1;
                ints_ea = 1'b1;
                grad_wea = 1'b1;
                ints_wea = 1'b0;
                grad_busy = 1'b1;
            end
            else
            begin
                grad_idle_counter = grad_idle_counter + 128'b1;
                grad_busy = 1'b0;
                grad_done = 1'b0;
                state = 6'b000;
            end
        end
            
       // State 1
       6'b001:
       begin
            debug_in_b001 = debug_in_b001 + 1;
            if(t < height_) 
            begin
                debug_in_b001_if = debug_in_b001_if + 1;
                s = 0;
                state = 6'b010;
            end
            else
            begin
                state = 6'b10001; //State 5
            end
        end

        // State 2
        6'b010: 
        begin
            if(s < width_)
            begin
                if(s < 2)
                begin
                    if(clk_counter_a == 2'b10)
                    begin
                        a = dout_ints;
                        clk_counter_a =  2'b00;
                        clk_counter_b = 2'b01;
                        state = 6'b010;                       
                    end
                    else if(clk_counter_b == 2'b00)
                    begin
                        addr_ints =  t*width_+s+1;
                        clk_counter_a = clk_counter_a + 1;
                        state = 6'b010;
                    end
                    else if(clk_counter_b == 2'b11)
                    begin
                        b = dout_ints;
                        clk_counter_b =  2'b00;
                        state = 6'b100101; //subtractor
                        temp_state = 6'b1001; // State 3                      
                    end
                    else
                    begin
                        addr_ints =  t*width_+s;
                        clk_counter_b = clk_counter_b + 1;
                        state = 6'b010;
                    end                   
                end
                else if(s >= width_2)
                begin
                    if(clk_counter_a == 2'b10)
                    begin
                        a = dout_ints;
                        clk_counter_a =  2'b00;
                        clk_counter_b = 2'b01;
                        state = 6'b010;                       
                    end
                    else if(clk_counter_b == 2'b00)
                    begin
                        addr_ints =  t*width_+s;
                        clk_counter_a = clk_counter_a + 1;
                        state = 6'b010;
                    end
                    else if(clk_counter_b == 2'b11)
                    begin
                        b = dout_ints;
                        clk_counter_b =  2'b00;
                        state = 6'b100101; //subtractor
                        temp_state = 6'b1001; // State                      
                    end
                    else
                    begin
                        addr_ints =  t*width_+s-1;
                        clk_counter_b = clk_counter_b + 1;
                        state = 6'b010;
                    end  
                end
                else
                begin
                    a = grad_c1_;
                    if(clk_counter_b == 2'b10)
                    begin
                        b = dout_ints;
                        clk_counter_b =  2'b00;
                        state = 6'b100110; //multiplier
                        temp_state = 6'b11; //*                     
                    end
                    else
                    begin
                        addr_ints =  t*width_+s-2;
                        clk_counter_b = clk_counter_b + 1;
                        state = 6'b010;
                    end 
                end
            end
            else
            begin
                t = t + 1;
                state = 6'b001;
            end
        end
        6'b11:
        begin
            t1 = Multiplier_Float;
            a = grad_c2_;            
            if(clk_counter_b == 2'b10)
            begin
                b = dout_ints;
                clk_counter_b =  2'b00;
                state = 6'b100110; //multiplier 
                temp_state = 6'b100;                     
            end
            else
            begin
                addr_ints =  t*width_+s-1;
                clk_counter_b = clk_counter_b + 1;
                state = 6'b11;
            end 
        end
        6'b100:
        begin
            t2 = Multiplier_Float;
            a = grad_c2_;            
            if(clk_counter_b == 2'b10)
            begin
                b = dout_ints;
                clk_counter_b =  2'b00;
                state = 6'b100110; //multiplier 
                temp_state = 6'b101;                     
            end
            else
            begin
                addr_ints =  t*width_+s+1;
                clk_counter_b = clk_counter_b + 1;
                state = 6'b100;
            end 
        end
        6'b101:
        begin
            t3 = Multiplier_Float;
            a = grad_c1_;
            if(clk_counter_b == 2'b10)
            begin
                b = dout_ints;
                clk_counter_b =  2'b00;
                state = 6'b100110; //multiplier 
                temp_state = 6'b110;                    
            end
            else
            begin
                addr_ints =  t*width_+s+2;
                clk_counter_b = clk_counter_b + 1;
                state = 6'b101;
            end 
        end
        6'b110: //******************
        begin
            t4 = Multiplier_Float;
            //t5 = Adder_Float(t1, t2);
            a = t1;
            b = t2;
            state = 6'b100110; //multiplier 
            temp_state = 6'b111;
        end
        5'b0111:
        begin
            t5 = Adder_Float;
            //t6 = Subtractor_Float(t5, t3);
            a = t5;
            b = t3;
            state = 6'b100101; //subtractor
            temp_state = 6'b1000;
        end
        6'b1000:
        begin
            t6 = Subtractor_Float;
            //grad_x_[((t*width_+s)*32+31)-:32] = Subtractor_Float(t6, t4);
            a = t6;
            b = t4;
            state = 6'b100101; //subtractor
            temp_state = 6'b1001;
        end
        // State 3
        6'b1001:
        begin
            //grad_x_[((t*width_+s)*32+31)-:32] = Subtractor_Float; //?????????????????????????
            addr_grad_x_ = t*width_+s;
            debug_addr_grad_x_ = addr_grad_x_; 
            din_grad_x_ = Subtractor_Float;
            if(t < 2)
            begin
                if(clk_counter_a == 2'b10)
                begin
                    a = dout_ints;
                    clk_counter_a =  2'b00;
                    clk_counter_b = 2'b01;
                    state = 6'b1001;                       
                end
                else if(clk_counter_b == 2'b00)
                begin
                    addr_ints =  (t+1)*width_+s;
                    clk_counter_a = clk_counter_a + 1;
                    state = 6'b1001;
                end
                else if(clk_counter_b == 2'b11)
                begin
                    b = dout_ints;
                    clk_counter_b =  2'b00;
                    state = 6'b100101; //subtractor
                    temp_state = 6'b10000; // State 4                      
                end
                else
                begin
                    addr_ints =  t*width_+s;
                    clk_counter_b = clk_counter_b + 1;
                    state = 6'b1001;
                end                
            end
            else if(t >= height_2)
            begin
                if(clk_counter_a == 2'b10)
                begin
                    a = dout_ints;
                    clk_counter_a =  2'b00;
                    clk_counter_b = 2'b01;
                    state = 6'b1001;                       
                end
                else if(clk_counter_b == 2'b00)
                begin
                    addr_ints = t*width_+s;
                    clk_counter_a = clk_counter_a + 1;
                    state = 6'b1001;
                end
                else if(clk_counter_b == 2'b11)
                begin
                    b = dout_ints;
                    clk_counter_b =  2'b00;
                    state = 6'b100101; //subtractor
                    temp_state = 6'b10000; // State 4                     
                end
                else
                begin
                    addr_ints = (t-1)*width_+s;
                    clk_counter_b = clk_counter_b + 1;
                    state = 6'b1001;
                end
            end
            else
            begin 
                a = grad_c1_;
                if(clk_counter_b == 2'b10)
                begin
                    b = dout_ints;
                    clk_counter_b =  2'b00;
                    state = 6'b100110; //multiplier 
                    temp_state = 6'b1010;                      
                end
                else
                begin
                    addr_ints = (t-2)*width_+s;
                    clk_counter_b = clk_counter_b + 1;
                    state = 6'b1001;
                end                      
            end                        
        end
        6'b1010:
        begin
            t1 = Multiplier_Float;
            a = grad_c2_;
            if(clk_counter_b == 2'b10)
            begin
                b = dout_ints;
                clk_counter_b =  2'b00;
                state = 6'b100110; //multiplier  
                temp_state = 6'b1011;                    
            end
            else
            begin
                addr_ints = (t-1)*width_+s;
                clk_counter_b = clk_counter_b + 1;
                state = 6'b1010;
            end
        end
        6'b1011:
        begin
            t2 = Multiplier_Float;
            a = grad_c2_;            
            if(clk_counter_b == 2'b10)
            begin
                b = dout_ints;
                clk_counter_b =  2'b00;
                state = 6'b100110; //multiplier  
                temp_state = 6'b1100;                    
            end
            else
            begin
                addr_ints = (t+1)*width_+s;
                clk_counter_b = clk_counter_b + 1;
                state = 6'b1011;
            end
        end
        6'b1100:
        begin
            t3 = Multiplier_Float;
            a = grad_c1_;
            if(clk_counter_b == 2'b10)
            begin
                b = dout_ints;
                clk_counter_b =  2'b00;
                state = 6'b100110; //multiplier 
                temp_state = 6'b1101;                   
            end
            else
            begin
                addr_ints = (t+2)*width_+s;
                clk_counter_b = clk_counter_b + 1;
                state = 6'b1100;
            end
        end
        6'b1101: //**
        begin
            t4 = Multiplier_Float;
            //t5 = Adder_Float(t1, t2);
            a = t1;
            b = t2;
            state = 6'b100000; //adder 
            temp_state = 6'b1110;
        end
        6'b1110:
        begin
            t5 = Adder_Float;
            //t6 = Subtractor_Float(t5, t3);
            a = t5;
            b = t3;
            state = 6'b100101; //subtractor
            temp_state = 6'b1111;
        end
        6'b1111:
        begin
            t6 = Subtractor_Float;
            a = t6;
            b = t4;
            state = 6'b100101; //subtractor
            temp_state = 6'b10000;
        end
        // State 4
        6'b10000:
        begin
            //grad_y_[((t*width_+s)*32+31)-:32] = Subtractor_Float;
            addr_grad_y_ = t*width_+s;
            din_grad_y_ = Subtractor_Float;
            s = s + 1;
            state = 6'b010;
        end

        // State 5
        6'b10001: //1011
        begin
            //grad_x_[((t*width_+s)*32+31)-:32] = Subtractor_Float; //from 01  
            //addr_grad_x_ = t*width_+s; 
            //din_grad_x_ = Subtractor_Float;            
            grad_ea = 1'b1;
            ints_ea = 1'b1;
            grad_wea = 1'b0;
            ints_wea = 1'b0;        
            grad_done = 1'b1;
            grad_busy = 1'b0;
            state = 6'b000000;
        end

        ////////////////////////////ADDER //////////////////////////// 
        6'b100000: //3'b000:
        begin
            debug_a_in_b100000 = a;
            s1=a[31];
            s2=b[31];
            e1=a[30:23];
            e2=b[30:23];
            m1[23]=1'b1;
            m2[23]=1'b1;
            m1[22:0]=a[22:0];
            m2[22:0]=b[22:0];
            
             if(e1==e2)
              begin
              mx=m1;
              my=m2;
              exy=e1+1'b1;
              sign=s1;
              end
              else if(e1>e2)
              begin
              diff=e1-e2;
              mx=m1;
              my=m2>>diff;
              exy=e1+1'b1;
              sign=s1;
              end
              else
              begin
              diff=e2-e1;
              
              mx=m2;
              my=m1>>diff;
              exy=e2+1'b1;
              sign=s2;
              end
              sr=s1^s2;
              state = 6'b100001;
        end
        6'b100001: //3'b001:
        begin
            if(sr==0)
            begin
            mxy=mx+my;
            sign=s1;
            end
            else
            begin
            if(mx >= my)
            mxy=mx-my;
            else
            mxy=my-mx;
            end
            mxy2=mxy;
            
            if(s1==0 && s2==0)
            sign=1'b0;
            else if (s1==1 && s2==1)
            sign=1'b1;
            else if (s1==0 && s2==1)
            begin
            if(e1<e2 || ((e1==e2) && (m1<m2)))
            sign=1'b1;
            else
            sign=1'b0;
            end 
            else
            begin
            if(e1<e2 || ((e1==e2) && (m1<m2)))
            sign=1'b0;
            else
            sign=1'b1;
            end
            state = 6'b100010;
        end
        6'b100010: //3'b010:
        begin
            for(i=0;i<12;i=i+1)
            if (mxy[24]==0)
            begin
            mxy = mxy << 1;
            exy = exy - 1;
            end
            state = 6'b100011;
        end
        6'b100011: //3'b011:
        begin
            for(i=12;i<24;i=i+1)
            if (mxy[24]==0)
            begin
            mxy = mxy << 1;
            exy = exy - 1;
            end
            state = 6'b100100;                        
        end
        6'b100100: //3'b100:
        begin
            if(a[30:0] == 31'b0000000000000000000000000000000)
            Adder_Float = b[31:0];
            else if (b[30:0] == 31'b0000000000000000000000000000000)
            Adder_Float = a[31:0];
            else
            Adder_Float= {sign,exy,mxy[23:1]};
            done = 1'b1;
            Subtractor_Float = Adder_Float;
            state = temp_state;
        end
        ////////////////////////////SUBTRACTOR //////////////////////////// 
        6'b100101: //subtractor
        begin
            debug_a_in_b100101 = a;
            if (b[30:0] == 31'b0)
            begin
                Subtractor_Float = a[31:0];
                state = temp_state;
            end
            else if(a[31:0] == b[31:0])
            begin
                Subtractor_Float = 32'b0;
                state = temp_state;
            end
            else
            begin
                b[31:0] ={{ !b[31]}, {b[30:0]}};
                //Subtractor_Float = Adder_Float(a,b);
                state = 6'b100000; //adder
            end
        end
        ////////////////////////////MULTIPLIER ////////////////////////////    
        6'b100110:
        begin
                s1 = a[31]; //Sign bit
                s2 = b[31]; 
                e1 = a[30:23]; // Exponent bits
                e2 = b[30:23];
                m1[23] = 1'b1; //Mantissa 24th bit should be 1
                m2[23] = 1'b1;
                m1[22:0] = a[22:0]; // Mantissa bits
                m2[22:0] = b[22:0];
                state = 6'b100111;
        end
        6'b100111: //4'b0001:
        begin
            e_sum = e1 + e2;  //Exponent addition +1'b1
            exponent= e_sum - 8'b01111110; //01111111
            state = 6'b101000;
        end
        6'b101000: //4'b0010:
        begin
            if ( a!=0 || b!=0 )
            begin
                prod = m1*m2;    // mantissa product
                product = prod[47:24];
            end
            state = 6'b101001;
        end
        6'b101001: //4'b0011:
        begin
            if(product == 0)
            begin
                Multiplier_Float=32'b0;
                state = 6'b101011;
            end
            else
            begin
                //Normalization
                for(i = 0;i < 12;i = i + 1)
                    if (product[23]== 0)
                    begin
                        product = product << 1;
                        exponent = exponent - 1;
                    end
                state = 6'b101100;
           end //else
        end
        6'b101100:
        begin
            //Normalization
            for(i = 12;i < 23;i = i + 1)
                if (product[23]== 0)
                begin
                    product = product << 1;
                    exponent = exponent - 1;
                end
            state = 6'b101010;
        end
        6'b101010: //4'b0100:
        begin
            sign= s1 ^ s2; // Sign Calculation
            if(a[30:0] == 31'b0 || b[30:0] == 31'b0) // if any input is 0, output is 0
                Multiplier_Float = 32'b0;
            else
                Multiplier_Float = {sign,exponent,product[22:0]}; // Output
            state = 6'b101011;         
        end
        6'b101011: //4'b0101:
        begin
            multiplier_done = 1'b1;
            //state = 4'b0110;
            state = temp_state;
        end
            
        endcase // end of case  
end // end of always block
  
endmodule
