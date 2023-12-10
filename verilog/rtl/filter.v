module top(data_in, clk, mavg_en, reset, data_out, data_ack, fir_en);

localparam DW = 16;
localparam MEM = 3;
localparam [MEM-1:0] INITIAL_NAVG = 7;

input signed [15:0] data_in;
input clk;
input mavg_en;
input reset;
input fir_en;

output signed [23:0] data_out;
output data_ack;

reg signed [DW+MEM:0] mavg_out;

assign data_out = (mavg_en == 1) ? mavg_out: (fir_en==1)? sum_2[34:11] : 24'b0;

reg [MEM-1:0] wraddr, rdaddr;
reg [DW-1:0] data_reg [0:((1<<MEM)-1)];

reg signed [DW-1:0] memval, preval;
reg signed [DW:0] sub;
reg [(DW+MEM)-1:0] acc;

wire [(MEM-1):0]	w_requested_navg;

assign w_requested_navg = INITIAL_NAVG;
reg full;

assign data_ack = full;

always @(posedge clk) begin
	if (reset)
		wraddr <= 0;
	else if (mavg_en)
		wraddr <= wraddr + 1'b1;
end

initial rdaddr = ~INITIAL_NAVG;
always @(posedge clk) begin
    if (reset)
        rdaddr <= -w_requested_navg;
    else if (mavg_en)
        rdaddr <= rdaddr + 1'b1;
end

initial preval = 0;
always @(posedge clk) begin
    if (reset)
        preval <= 0;
    else if (mavg_en)
        preval <= data_in;
end

always @(posedge clk) begin
    if (mavg_en)
        data_reg[wraddr] <= data_in;
end

initial memval = 0;
always @(posedge clk) begin
    if (mavg_en)
        memval <= data_reg[rdaddr];
end


always @(posedge clk) begin
    if (reset)
        full <= 0;
    else if (mavg_en)
        full <= (full) || (rdaddr == 0);
end

always @(posedge clk) begin
    if (reset)
        sub <= 0;
    else if (mavg_en) begin
        if (full)
            sub <= (preval - memval)>>3;
        else
            sub <= preval>>3;
    end
end

always @(posedge clk) begin
    if (reset)
        acc <= 0;
    else if (mavg_en) begin
        acc <= acc + sub;
    end
end

always @(posedge clk) begin
    if (reset)
        mavg_out <= 0;
    else if (mavg_en) begin
        mavg_out <= acc;
    end
end


integer i,j;

// Coefficients for 8-tap FIR. 1.1.14

// 1000 KHz sampling clock

// Noisy signal to be filtered, 1.1.14

// Filtered output signal. 1.1.14

//-16MHz cutoff frequency at 108MHz sampling rate 
wire signed [15:0] coeff [0:8];

assign coeff[0] = 16'hF9A0;
assign coeff[1] = 16'hF888; 
assign coeff[2] = 16'h0DBE;
assign coeff[3] = 16'h3DDF;
assign coeff[4] = 16'h3DDF;
assign coeff[5] = 16'h0DBE;
assign coeff[6] = 16'hF888;
assign coeff[7] = 16'hF9A0;

reg signed [15:0] delayed_signal [0:8];
reg signed [31:0] prod [0:8];
reg signed [32:0] sum_0 [0:4];
reg signed [33:0] sum_1 [0:2];
reg signed [34:0] sum_2 ;


// Feed the noisy signal input signal into a 8-tap delayed line to prep for convolution 
always @(posedge clk)   
if (reset) begin
    delayed_signal[0] <= 0;
     for (i=1; i<8; i=i+1) begin
        delayed_signal[i] <= delayed_signal[i-1];
    end
end
else if (fir_en) begin
    delayed_signal[0] <= data_in;
    for (i=1; i<8; i=i+1) begin
        delayed_signal[i] <= delayed_signal[i-1];
    end
end 

always @(posedge clk)
begin
if (fir_en) begin
    for (j=0; j<8; j=j+1) begin
        prod[j] <= delayed_signal[j]*coeff[j];
    end
end
end

always @(posedge clk)
begin
if (fir_en) begin
    sum_0[0] <= prod[0] + prod[1]; 
    sum_0[1] <= prod[2] + prod[3]; 
    sum_0[2] <= prod[4] + prod[5]; 
    sum_0[3] <= prod[6] + prod[7]; end
end

always @(posedge clk)
begin
if (fir_en) begin
    sum_1[0] <= sum_0[0] + sum_0[1]; 
    sum_1[1] <= sum_0[2] + sum_0[3]; end
end

always @(posedge clk)
begin
if (fir_en) begin
    sum_2 <= sum_1[0] + sum_1[1]; end
end
 
endmodule
