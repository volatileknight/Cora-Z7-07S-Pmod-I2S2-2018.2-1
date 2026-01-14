`timescale 1ns / 1ps
/*
Follows AXI-Stream interface conventions. That is all the master and slave signals
*/
module FIR
#(
    parameter INPUT_WIDTH = 24,
    parameter COEFF_WIDTH = 24,
    parameter NUM_TAPS = 15,
    localparam ACC_WIDTH = INPUT_WIDTH + COEFF_WIDTH, // width of one multiply output
    localparam OUTPUT_WIDTH = ACC_WIDTH + $clog2(NUM_TAPS) // width of final accumulation output
)
    (
    input clk,
    input reset_n,
    input [NUM_TAPS][COEFF_WIDTH-1:0] coeffs, // FIR coefficients
    input signed [INPUT_WIDTH-1:0] s_axis_fir_tdata,
    input [5:0] s_axis_fir_tkeep, // number of valid bytes: 48 / 8 = 6
    input s_axis_fir_tlast, // end of frame signal from upstream module
    input s_axis_fir_tvalid, // indicates valid data from upstream module
    input m_axis_fir_tready, // indicates downstream module is ready to accept data
    output reg m_axis_fir_tvalid, // indicates valid data to downstream module
    output reg s_axis_fir_tready, // indicates ready to accept data from upstream module
    output reg m_axis_fir_tlast, // end of frame signal to downstream module
    output reg [5:0] m_axis_fir_tkeep, // number of valid bytes: 48 / 8 = 6
    output reg signed [OUTPUT_WIDTH-1:0] m_axis_fir_tdata
    );


    always @ (posedge clk)
        begin
            m_axis_fir_tkeep <= s_axis_fir_tkeep; // pass through tkeep from input to output
            // m_axis_fir_tkeep <= '1; // all bytes are valid
        end

    always @ (posedge clk)
        begin
            if (s_axis_fir_tlast == 1'b1)
                begin
                    m_axis_fir_tlast <= 1'b1;
                end
            else
                begin
                    m_axis_fir_tlast <= 1'b0;
                end
        end

    // 15-tap FIR
    reg enable_fir, enable_buff;
    reg [NUM_TAPS-1:0] buff_cnt; // definitely more than enough bits to count to number of taps
    reg signed [INPUT_WIDTH-1:0] in_sample;
    reg signed [INPUT_WIDTH-1:0] buff0, buff1, buff2, buff3, buff4, buff5, buff6, buff7, buff8, buff9, buff10, buff11, buff12, buff13, buff14;
    wire signed [COEFF_WIDTH-1:0] tap0, tap1, tap2, tap3, tap4, tap5, tap6, tap7, tap8, tap9, tap10, tap11, tap12, tap13, tap14;
    reg signed [ACC_WIDTH-1:0] acc0, acc1, acc2, acc3, acc4, acc5, acc6, acc7, acc8, acc9, acc10, acc11, acc12, acc13, acc14;


    // /* Taps for LPF running @ 1MSps with a cutoff freq of 400kHz*/
    // assign tap0 = 16'hFC9C;  // twos(-0.0265 * 32768) = 0xFC9C
    // assign tap1 = 16'h0000;  // 0
    // assign tap2 = 16'h05A5;  // 0.0441 * 32768 = 1445.0688 = 1445 = 0x05A5
    // assign tap3 = 16'h0000;  // 0
    // assign tap4 = 16'hF40C;  // twos(-0.0934 * 32768) = 0xF40C
    // assign tap5 = 16'h0000;  // 0
    // assign tap6 = 16'h282D;  // 0.3139 * 32768 = 10285.8752 = 10285 = 0x282D
    // assign tap7 = 16'h4000;  // 0.5000 * 32768 = 16384 = 0x4000
    // assign tap8 = 16'h282D;  // 0.3139 * 32768 = 10285.8752 = 10285 = 0x282D
    // assign tap9 = 16'h0000;  // 0
    // assign tap10 = 16'hF40C; // twos(-0.0934 * 32768) = 0xF40C
    // assign tap11 = 16'h0000; // 0
    // assign tap12 = 16'h05A5; // 0.0441 * 32768 = 1445.0688 = 1445 = 0x05A5
    // assign tap13 = 16'h0000; // 0
    // assign tap14 = 16'hFC9C; // twos(-0.0265 * 32768) = 0xFC9C
    assign tap0 = coeffs[0];
    assign tap1 = coeffs[1];
    assign tap2 = coeffs[2];
    assign tap3 = coeffs[3];
    assign tap4 = coeffs[4];
    assign tap5 = coeffs[5];
    assign tap6 = coeffs[6];
    assign tap7 = coeffs[7];
    assign tap8 = coeffs[8];
    assign tap9 = coeffs[9];
    assign tap10 = coeffs[10];
    assign tap11 = coeffs[11];
    assign tap12 = coeffs[12];
    assign tap13 = coeffs[13];
    assign tap14 = coeffs[14];

    /* This loop sets the tvalid flag on the output of the FIR high once
     * the circular buffer has been filled with input samples for the
     * first time after a reset condition. */
    always @ (posedge clk or negedge reset_n)
        begin
            if (reset_n == 1'b0) //if (reset == 1'b0 || tvalid_in == 1'b0)
                begin
                    buff_cnt <= 4'd0;
                    enable_fir <= 1'b0;
                    in_sample <= 8'd0;
                end
            // the downstream module is not ready or no valid input data
            else if (m_axis_fir_tready == 1'b0 || s_axis_fir_tvalid == 1'b0)
                begin
                    enable_fir <= 1'b0;
                    buff_cnt <= NUM_TAPS-1;
                    in_sample <= in_sample;
                end
            else if (buff_cnt == NUM_TAPS-1) // i don't know if this makes it 15 or 16 taps
                begin
                    buff_cnt <= 4'd0;
                    enable_fir <= 1'b1;
                    in_sample <= s_axis_fir_tdata;
                end
            else
                begin
                    buff_cnt <= buff_cnt + 1;
                    in_sample <= s_axis_fir_tdata;
                end
        end

    always @ (posedge clk)
        begin
            if(reset_n == 1'b0 || m_axis_fir_tready == 1'b0 || s_axis_fir_tvalid == 1'b0)
                begin
                    s_axis_fir_tready <= 1'b0;
                    m_axis_fir_tvalid <= 1'b0;
                    enable_buff <= 1'b0;
                end
            else
                begin
                    s_axis_fir_tready <= 1'b1;
                    m_axis_fir_tvalid <= 1'b1;
                    enable_buff <= 1'b1;
                end
        end

    /* Circular buffer bring in a serial input sample stream that
     * creates an array of 15 input samples for the 15 taps of the filter. */
    always @ (posedge clk)
        begin
            if(enable_buff == 1'b1)
                begin
                    buff0 <= in_sample;
                    buff1 <= buff0;
                    buff2 <= buff1;
                    buff3 <= buff2;
                    buff4 <= buff3;
                    buff5 <= buff4;
                    buff6 <= buff5;
                    buff7 <= buff6;
                    buff8 <= buff7;
                    buff9 <= buff8;
                    buff10 <= buff9;
                    buff11 <= buff10;
                    buff12 <= buff11;
                    buff13 <= buff12;
                    buff14 <= buff13;
                end
            else
                begin
                    buff0 <= buff0;
                    buff1 <= buff1;
                    buff2 <= buff2;
                    buff3 <= buff3;
                    buff4 <= buff4;
                    buff5 <= buff5;
                    buff6 <= buff6;
                    buff7 <= buff7;
                    buff8 <= buff8;
                    buff9 <= buff9;
                    buff10 <= buff10;
                    buff11 <= buff11;
                    buff12 <= buff12;
                    buff13 <= buff13;
                    buff14 <= buff14;
                end
        end

    /* Multiply stage of FIR */
    always @ (posedge clk)
        begin
            if (enable_fir == 1'b1)
                begin
                    acc0 <= tap0 * buff0;
                    acc1 <= tap1 * buff1;
                    acc2 <= tap2 * buff2;
                    acc3 <= tap3 * buff3;
                    acc4 <= tap4 * buff4;
                    acc5 <= tap5 * buff5;
                    acc6 <= tap6 * buff6;
                    acc7 <= tap7 * buff7;
                    acc8 <= tap8 * buff8;
                    acc9 <= tap9 * buff9;
                    acc10 <= tap10 * buff10;
                    acc11 <= tap11 * buff11;
                    acc12 <= tap12 * buff12;
                    acc13 <= tap13 * buff13;
                    acc14 <= tap14 * buff14;
                end
        end

     /* Accumulate stage of FIR */
    always @ (posedge clk)
        begin
            if (enable_fir == 1'b1)
                begin
                    m_axis_fir_tdata <= acc0 + acc1 + acc2 + acc3 + acc4 + acc5 + acc6 + acc7 + acc8 + acc9 + acc10 + acc11 + acc12 + acc13 + acc14;
                end
        end



endmodule
