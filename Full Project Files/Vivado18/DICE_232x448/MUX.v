// MUX
`timescale 1ns / 1ps

module MUX(
    input clk,
	input [31:0] frame_counter,
    input [31:0] ref_img_in,
    input [31:0] def_img_in,
    output reg [31:0] ref_img_out,
    output reg [31:0] def_img_out
);


always @(posedge clk)
begin
	if(frame_counter == 32'b1 || frame_counter == 32'b10)
	begin
		ref_img_out[31:0] = ref_img_in[31:0];
		def_img_out[31:0] = def_img_in[31:0];
	end
	else if(frame_counter > 32'b10 && frame_counter % 32'b10 == 32'b0)
	begin
		ref_img_out[31:0] = ref_img_in[31:0];
		def_img_out[31:0] = def_img_in[31:0];
	end
	else if(frame_counter > 32'b10 && frame_counter % 32'b10 == 32'b1)
	begin
		ref_img_out[31:0] = def_img_in[31:0];
		def_img_out[31:0]  = ref_img_in[31:0];
	end
end	// End always

endmodule