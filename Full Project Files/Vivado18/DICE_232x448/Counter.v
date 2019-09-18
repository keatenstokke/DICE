`timescale 1ns / 1ps

module Counter(
    input clock,
    input gam_busy,
	input grad_busy,
	output reg [127:0] gam_busy_counter,
	output reg [127:0] grad_busy_counter
    );
    
    
    always @(posedge clock)
    begin
        in_if = 1'b0;
        in_else = 1'b0;
        in_always = 1'b0;
        if(gam_busy == 1'b1)
        begin
            gam_busy_counter = gam_busy_counter + 128'b1;
        end
		if(grad_busy == 1'b1)
		begin
			grad_busy_counter = grad_busy_counter + 128'b1;
		end
    end
endmodule
