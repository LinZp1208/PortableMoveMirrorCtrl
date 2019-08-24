module CaptureT(
						input clk,
						input reset_n,
						input SamplePlusIn1,
						output reg[31:0]T1,
						output samplein_sycronous,
						output INT
					);
assign INT = SamplePlusIn1;
assign	samplein_sycronous=e;
reg[31:0]Cap1,Cap2;
reg[31:0]NUM;
always@(posedge clk)
begin
	NUM<=NUM+32'd1;
end
 
reg a,b,c,d,e,f,g,h;

always@(posedge clk)
begin
	a<=SamplePlusIn1;
	b<=a;
	c<=b;
	d<=c;
	e<=d;
	g<=(b)&(c)&(~d)&(~e);
end

always@(posedge clk)
begin
if(g)begin
	Cap1<=Cap2;
	Cap2<=NUM;	
	T1<=Cap2-Cap1;
end
else begin
	Cap1<=Cap1;
	Cap2<=Cap2;	
	T1<=T1;
end
end	
		
endmodule
					