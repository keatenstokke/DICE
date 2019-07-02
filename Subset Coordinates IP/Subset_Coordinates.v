`timescale 1ns / 1ps

module SubCoord_Multi_Subs_Multi_Shapes_232_448(
    input clock,
    input [31:0] num_of_subsets,
    input [31:0] subset_centerpoint_x,
    input [31:0] subset_centerpoint_y,
    input [31:0] subset_size,
    input [31:0] half_subset_size,
    input [31:0] subset_shape,
    input param_ready,
    input coord_interface_done,
    output reg ea_x,
    output reg ea_y,
    output reg wea_x,
    output reg wea_y,
    output reg [31:0] din_x,
    output reg [31:0] din_y,
    output reg [15:0] addr_x,
    output reg [15:0] addr_y,
    output reg [31:0] base_address,
    output reg [31:0] num_pxl_Int,
    output reg [31:0] num_pxl_FP,
    output reg sub_done = 1'b0,
    output reg coord_new_subset,
    output reg [31:0] subset_counter = 32'b0
);

reg [6:0] state = 'b0000;
reg [6:0] temp_state;
reg [31:0] last_x_k, last_y_k;
reg [31:0] loop_counter = 32'b0;
reg r_done;
reg [31:0] a, b, Subtractor_Float, Adder_Float, Multiplier_Float;
reg [7:0]e1,e2,exy;
reg s1,s2,sr,sign;
reg [23:0]m1,m2,mx,my;
reg [24:0]mxy,mxy2;
reg [7:0] diff,i;
reg [7:0]e_sum,exponent;
reg [47:0]prod;
reg [23:0] product;
reg multiplier_done;

reg [31:0] k = 32'b0; //addressing, nener resets
reg [31:0] lc = 32'b0; //loop_counter_resets for each subset
reg [31:0] temp_reminder;
reg [31:0] k1;
reg [31:0] pxl_cnt_Int = 32'b0;
reg [31:0] pxl_cnt_FP = 32'b0;

reg [5:0] debug_addr_x;
reg in_if_debug, in_else_debug;

//for FP
reg [31:0] min_x, max_x, min_y, max_y, x_i, y_i, y_i_2, x_i_2;
reg [31:0] ii, jj;

//for circle
reg all_subs_done;

reg [31:0] sub_pxl_counter = 32'b0;
reg mod = 1'b1;

always @(posedge clock)    
begin    
     case(state)
     'b0000:
     begin
        if(param_ready == 1)
        begin
             //x[31:0] = Subtractor_Float(subset_centerpoint_x, half_subset_size);
             //last_x_k = x[31:0];
            ea_x = 1'b1;
            ea_y = 1'b1;
            wea_x = 1'b1;
            wea_y = 1'b1;
            if(subset_counter < num_of_subsets)
            begin
                coord_new_subset = 1'b1;
                if(coord_interface_done == 1'b0)
                begin
                    state = 'b0;
                end
                else //ready to start the subset
                begin
                    coord_new_subset = 1'b0;
                    base_address = k;
                    num_pxl_Int = pxl_cnt_Int;
                    num_pxl_FP = pxl_cnt_FP;
                    state = 'b100001;
                end               
            end
            else
            begin
                num_pxl_Int = pxl_cnt_Int;
                num_pxl_FP = pxl_cnt_FP;
                state = 'b100000;
            end          
        end
        else
        begin
            state = 'b0000;
        end
        
     end
     'b100001:
     begin
        if(subset_shape == 32'b1)
        begin
            pxl_cnt_Int = 32'b0;
            pxl_cnt_FP = 32'b0;
            //k = 0;
            lc = 32'b0;
            sub_pxl_counter = subset_size;
            a = subset_centerpoint_x;
            b = half_subset_size;
            loop_counter = subset_size * subset_size; 
            state = 'b1111; //Call Subtractor
            temp_state = 'b0001;
        end
        else if(subset_shape == 32'b0) //circle
        begin
            pxl_cnt_Int = 32'b0;
            pxl_cnt_FP = 32'b0;
            state = 'b10000;
        end      
     end 
     ///////////////////////////////// Square //////////////////////////////  
     'b0001:
     begin
         //x[k * 32 + 31 -: 32] = Subtractor_Float;
         addr_x = k;
         debug_addr_x = k;
         din_x = Subtractor_Float;
         last_x_k = din_x;
         //y[31:0] = Subtractor_Float(subset_centerpoint_y, half_subset_size);
         //last_y_k = y[31:0];
         a = subset_centerpoint_y;
         b = half_subset_size;
         state = 'b1111; //Call Subtractor
         temp_state = 'b0010;
     end
     'b0010:
     begin
         //y[k * 32 + 31 -: 32] = Subtractor_Float;
         addr_y = k;
         din_y = Subtractor_Float;
         last_y_k = din_y;
         k = k + 1;
         //state = 4'b0011;
         state = 'b0011;
     end
     'b0011:
     begin
         if(lc < loop_counter)
         begin          
             pxl_cnt_Int = pxl_cnt_Int + 1;
            //call floating_point adder
            a = pxl_cnt_FP;
            b = 'b00111111100000000000000000000000;
            state = 'b1010; //Call Adder 
            temp_state = 'b100101;           
         end
         else
         begin
             state = 'b1001;
         end
     end
     'b100101:
     begin
        pxl_cnt_FP = Adder_Float;       
        state = 'b100010;
     end
     'b100010:
     begin
        //temp_reminder = lc % subset_size;
        //temp_reminder = lc % 32'b11;
        if(sub_pxl_counter == subset_size)
        begin
            mod = 1'b0;
            sub_pxl_counter = 32'b0;
        end
        else
        begin
            mod = 1'b1;
        end
        state = 'b100011;
     end
     'b100011:
     begin
        state = 'b101101;
     end
     'b101101:
     begin
        state = 'b100100;
     end
     'b100100:
     begin
         //if(lc % subset_size == 32'b0)
         if(mod == 1'b0)
         begin
             k1 = k;
             state = 'b0100;
         end
         else
         begin
             k1 = k;
             state = 'b0111;
         end
     end
     'b0100:
     begin
         //x[k1-:32] = Adder_Float(last_x_k, 32'b00111111100000000000000000000000); //+1
         a = last_x_k;
         b = 32'b00111111100000000000000000000000;
         state = 'b1010; //Call Adder
         temp_state = 'b0101;
     end
     'b0101:
     begin
         //x[k1-:32] = Adder_Float;
         addr_x = k;
         din_x = Adder_Float;
         //y[k1-:32] = Adder_Float(last_y_k, 32'b01000011111000000000000000000000); //+448
         a = last_y_k;
         b = 32'b01000011111000000000000000000000;
         state = 'b1010; //Call Adder 
         temp_state = 'b0110;
     end
     'b0110:
     begin
         //y[k1-:32] = Adder_Float;
         addr_y = k;
         din_y = Adder_Float;
         //y[k1-:32] = Subtractor_Float(y[k1-:32], 32'b11000010111011000000000000000000); //-232/2 + 2 = -118
         a = din_y;
         b = 32'b11000010111011000000000000000000;
         state = 'b1010; //Call Adder 
         temp_state = 'b1000;
     end
     'b0111:
     begin
         //x[k1-:32] = last_x_k;
         din_x = last_x_k;
         addr_x = k;
         //y[k1-:32] = Adder_Float(last_y_k, 32'b00111111100000000000000000000000); //+1
         a = last_y_k;
         b = 32'b00111111100000000000000000000000;
         state = 'b1010; //Call Adder 
         temp_state = 'b1000;
     end
     'b1000:
     begin
         din_y = Adder_Float;
         addr_y = k;
         last_x_k = din_x;
         last_y_k = din_y;
         k = k + 1;
         lc = lc + 32'b1;
         sub_pxl_counter = sub_pxl_counter + 32'b1;
         state = 'b0011;
     end
     'b1001:
     begin
        subset_counter = subset_counter + 1;
        state = 'b0;
    end
    'b100000:
    begin
        ea_x = 1'b1;
        ea_y = 1'b1;
        wea_x = 1'b0;
        wea_y = 1'b0;
        sub_done = 1'b1;
     end     
     ///////////////////////////////// Circle //////////////////////////////     
       'b10000:
       begin
          a = subset_centerpoint_x;
          b = subset_size;
          state = 'b1111; //Call Subtractor
          temp_state = 'b10001;
       end
       'b10001:
       begin
          min_x = Subtractor_Float;
          a = subset_centerpoint_x;
          b = subset_size;
          state = 'b1010; //Call Adder 
          temp_state = 'b10010;
       end
       'b10010:
       begin
          max_x = Adder_Float;
          a = subset_centerpoint_y;
          b = subset_size;
          state = 'b1111; //Call Subtractor
          temp_state = 'b11101;
       end
       'b11101:
       begin
          min_y = Subtractor_Float;
          a = subset_centerpoint_y;
          b = subset_size;
          state = 'b1010; //Call Adder 
          temp_state = 'b10011;
       end
       'b10011:
       begin
          max_y = Adder_Float;
          ii = min_x;
          state = 'b10100;
       end
       'b10100:
       begin
          if(less_than_or_equal(ii, max_x))
          begin
              jj = min_y;
              state = 'b10101;
          end
          else
          begin
              state = 'b11100;//end for i
          end
       end
       'b10101:
       begin
          if(less_than_or_equal(jj, max_y))
           begin
               //call (x-i)^2 + (y-j)^2 < r_2
               a = subset_centerpoint_x;
               b = ii;
               state = 'b1111; //Call Subtractor
               temp_state = 'b10110;
           end
           else
           begin
              //call add i
              a = ii;
              b = 32'b00111111100000000000000000000000; //1
              state = 'b1010; //Call Adder 
              temp_state = 'b11110; //end for j
           end
       end
       'b11110:
       begin
          ii = Adder_Float;
          state = 'b10100; //end for j
       end
       'b10110:
       begin
          x_i = Subtractor_Float;
          a = subset_centerpoint_y;
          b = jj;
          state = 'b1111; //Call Subtractor
          temp_state = 'b10111;    
       end
       'b10111:
       begin
          y_i = Subtractor_Float;
          a = x_i;
          b = x_i;
          state = 'b100110; //Call Multiplier
          temp_state = 'b11000;
       end
       'b11000:
       begin
          x_i_2 = Multiplier_Float;
          a = y_i;
          b = y_i;
          state = 'b100110; //Call Multiplier
          temp_state = 'b11001;
       end
       'b11001:
       begin
          y_i_2 = Multiplier_Float;
          a = x_i_2;
          b = y_i_2;
          state = 'b1010; //Call Adder 
          temp_state = 'b11010;
       end
       'b11010:
       begin
          if(less_than_or_equal(Adder_Float, half_subset_size))
          begin         
            pxl_cnt_Int = pxl_cnt_Int + 1;
            //call floating_point adder
            a = pxl_cnt_FP;
            b = 'b00111111100000000000000000000000;
            state = 'b1010; //Call Adder 
            temp_state = 'b101110;
          end
          else
          begin
              //call add j
              a = jj;
              b = 32'b00111111100000000000000000000000; //1
              state = 'b1010; //Call Adder 
              temp_state = 'b11011;
          end
       end
       'b101110:
       begin
           pxl_cnt_FP = Adder_Float;
           k1 = (k-1)*32+31;
           //x[k1-:32] = ii;
           //y[k1-:32] = jj;
           addr_x = k;
           addr_y = k;
           din_x = ii;
           din_y = jj;
           k = k + 1;
           //call add j
           a = jj;
           b = 32'b00111111100000000000000000000000; //1
           state = 'b1010; //Call Adder 
           temp_state = 'b11011;
       end       
       'b11011:
       begin
          jj = Adder_Float;
          state = 'b10101;
       end
       'b11100:
       begin
          subset_counter = subset_counter + 1;
          state = 'b0000;
       end 
     
     ////////////////////////// ADDER ///////////////////////////
     'b1010: //3'b000:
     begin
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
        state = 'b1011;
     end
     'b1011: //3'b001:
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
        state = 'b1100;
     end
     'b1100: //3'b010:
     begin
         for(i=0;i<12;i=i+1)
            if (mxy[24]==0)
            begin
                 mxy = mxy << 1;
                 exy = exy - 1;
            end
         state = 'b1101;
     end
     'b1101: //3'b011:
     begin
         for(i=12;i<24;i=i+1)
             if (mxy[24]==0)
             begin
                 mxy = mxy << 1;
                 exy = exy - 1;
             end
         state = 'b1110;                        
     end
     'b1110: //3'b100:
     begin
         if(a[30:0] == 31'b0000000000000000000000000000000)
            Adder_Float = b[31:0];
         else if (b[30:0] == 31'b0000000000000000000000000000000)
            Adder_Float = a[31:0];
         else
            Adder_Float= {sign,exy,mxy[23:1]};
         r_done = 1'b1;
         Subtractor_Float = Adder_Float;
         state = temp_state;
     end
     'b1111: //subtractor
     begin
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
             state = 'b1010;
         end
     end
     'b100110: //Multiplier_Floar
      begin
              s1 = a[31]; //Sign bit
              s2 = b[31]; 
              e1 = a[30:23]; // Exponent bits
              e2 = b[30:23];
              m1[23] = 1'b1; //Mantissa 24th bit should be 1
              m2[23] = 1'b1;
              m1[22:0] = a[22:0]; // Mantissa bits
              m2[22:0] = b[22:0];
              state = 'b100111;
      end
      'b100111: //4'b0001:
      begin
          e_sum = e1 + e2;  //Exponent addition +1'b1
          exponent= e_sum - 8'b01111110; //01111111
          state = 'b101000;
      end
      'b101000: //4'b0010:
      begin
          if ( a!=0 || b!=0 )
          begin
              prod = m1*m2;    // mantissa product
              product = prod[47:24];
          end
          state = 'b101001;
      end
      'b101001: //4'b0011:
      begin
          if(product == 0)
          begin
              Multiplier_Float=32'b0;
              state = 'b101011;
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
              state = 'b101100;
         end //else
      end
      'b101100:
      begin
          //Normalization
          for(i = 12;i < 23;i = i + 1)
              if (product[23]== 0)
              begin
                  product = product << 1;
                  exponent = exponent - 1;
              end
          state = 'b101010;
      end
      'b101010: //4'b0100:
      begin
          sign= s1 ^ s2; // Sign Calculation
          if(a[30:0] == 31'b0 || b[30:0] == 31'b0) // if any input is 0, output is 0
              Multiplier_Float = 32'b0;
          else
              Multiplier_Float = {sign,exponent,product[22:0]}; // Output
          state = 'b101011;         
      end
      'b101011: //4'b0101:
      begin
          multiplier_done = 1'b1;
          //state = 4'b0110;
          state = temp_state;
      end
     endcase
     
end //always

    function less_than_or_equal;
        input [31:0] a;
        input [31:0] b;
        begin
            if(a[31] == 1'b1 && b[31] == 1'b0)
            begin
                less_than_or_equal = 1'b1;
            end
            else if(a[31] == 1'b0 && b[31] == 1'b1)
            begin
                less_than_or_equal = 1'b0;
            end
            else
            begin
                if(a[30:0] <= b[30:0])
                begin
                    less_than_or_equal = 1'b1;
                end
                else
                begin
                    less_than_or_equal = 1'b0;
                end
            end
        end
    endfunction 
endmodule
