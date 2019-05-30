`timescale 1ns / 1ps

// Module instatiation
module Subset_Coordinates(
	// Required clock input signal
    input clock,
	
	// User defined parameters that come from the parameters IP
    input [31:0] subset_centerpoint_x,
    input [31:0] subset_centerpoint_y,
    input [31:0] subset_size,
    input [31:0] half_subset_size,
	
	// Start signal for this IP to begin operations - All values from paramters IP have been received
    input param_ready,
	
	// This IPs outputs to be sent to the gamma IP
    output reg [287:0] x,
    output reg [287:0] y,

	// Done signal for this IP
    output reg sub_done
);

// State transistion registers
reg [3:0] state = 4'b0000;
reg [3:0] temp_state;

// Registers for various IP functions (loops)
reg [31:0] last_x_k, last_y_k;
reg [31:0] loop_counter = 32'b0;
reg [31:0] k, k1;
reg r_done;

// Registers for floating point computations
reg [31:0] a, b, Subtractor_Float, Adder_Float;
reg [7:0]e1,e2,exy;
reg s1,s2,sr,sign;
reg [23:0]m1,m2,mx,my;
reg [24:0]mxy,mxy2;
reg [7:0] diff,i;


always @(posedge clock)
begin    
     case(state)
     4'b0000:
     begin
		// Handles a square subset; computes the indexes for the image/frame
        if(param_ready == 1)
        begin
         //x[31:0] = Subtractor_Float(subset_centerpoint_x, half_subset_size);
         //last_x_k = x[31:0];
         a = subset_centerpoint_x;
         b = half_subset_size;
         state = 4'b1111; //Call Subtractor
         temp_state = 4'b0001;
        end // if ready
        else
        begin
            state = 4'b0000;
        end
     end
     4'b0001:
     begin
         x[31:0] = Subtractor_Float;
         last_x_k = x[31:0];
         //y[31:0] = Subtractor_Float(subset_centerpoint_y, half_subset_size);
         //last_y_k = y[31:0];
         a = subset_centerpoint_y;
         b = half_subset_size;
         state = 4'b1111; //Call Subtractor
         temp_state = 4'b0010;
     end
     4'b0010:
     begin
         y[31:0] = Subtractor_Float;
         last_y_k = y[31:0];
         k = 1;
         loop_counter = subset_size * subset_size; 
         //state = 4'b0011;
         state = 4'b0011;
     end
     4'b0011:
     begin
         if(k < loop_counter)
         begin
             //if(k % subset_size == 32'b0)
			 // These valeus should be parameters to handle the unique subset size that the user sets; right now the value is a default subset size of 3
             if(k % 32'b10 == 32'b0)
             begin
                 k1 = k*32+31;
                 state = 4'b0100;
             end
             else
             begin
                 k1 = k*32+31;
                 state = 4'b0111;
             end
         end
         else
         begin
             state = 4'b1001;
         end
     end
     4'b0100:
     begin
         //x[k1-:32] = Adder_Float(last_x_k, 32'b00111111100000000000000000000000); //+1
         a = last_x_k;
         b = 32'b00111111100000000000000000000000;
         state = 4'b1010; //Call Adder
         temp_state = 4'b0101;
     end
     4'b0101:
     begin
         x[k1-:32] = Adder_Float;
         //y[k1-:32] = Adder_Float(last_y_k, 32'b01000100001000000000000000000000); //+640
         a = last_y_k;
         b = 32'b01000100001000000000000000000000;
         state = 4'b1010; //Call Adder 
         temp_state = 4'b0110;
     end
     4'b0110:
     begin
         y[k1-:32] = Adder_Float;
         //y[k1-:32] = Subtractor_Float(y[k1-:32], 32'b01000001110100000000000000000000); //-26
         a = y[k1-:32];
         b = 32'b11000001110100000000000000000000;
         state = 4'b1010; //Call Adder 
         temp_state = 4'b1000;
     end
     4'b0111:
     begin
         x[k1-:32] = last_x_k;
         //y[k1-:32] = Adder_Float(last_y_k, 32'b00111111100000000000000000000000); //+1
         a = last_y_k;
         b = 32'b00111111100000000000000000000000;
         state = 4'b1010; //Call Adder 
         temp_state = 4'b1000;
     end
     4'b1000:
     begin
         y[k1-:32] = Adder_Float;
         last_x_k = x[k1-:32];
         last_y_k = y[k1-:32];
         k = k + 1;
         state = 4'b0011;
     end
     4'b1001:
     begin
        sub_done = 1'b1;
     end
	 
	 
	 
	 
	 
	 // Floating point functions (updates contuine for floating point operations)
     ////////////////////////// ADDER ///////////////////////////
     4'b1010: //3'b000:
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
           state = 4'b1011;
     end
     4'b1011: //3'b001:
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
         state = 4'b1100;
     end
     4'b1100: //3'b010:
     begin
         for(i=0;i<12;i=i+1)
         if (mxy[24]==0)
         begin
         mxy = mxy << 1;
         exy = exy - 1;
         end
         state = 4'b1101;
     end
     4'b1101: //3'b011:
     begin
         for(i=12;i<24;i=i+1)
         if (mxy[24]==0)
         begin
         mxy = mxy << 1;
         exy = exy - 1;
         end
         state = 4'b1110;                        
     end
     4'b1110: //3'b100:
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
	 
	 
	////////////////////////// SUBTRACTOR ///////////////////////////
     4'b1111:
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
             state = 4'b1010;
         end
     end
     endcase
     
end //always

endmodule