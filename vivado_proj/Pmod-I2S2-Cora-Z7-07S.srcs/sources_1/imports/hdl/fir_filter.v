`timescale 1ns / 1ps
/*
Follows AXI-Stream interface conventions. That is all the master and slave signals
*/
module FIR
#(
    parameter INPUT_WIDTH = 24,
    parameter COEFF_WIDTH = 24,
    parameter NUM_TAPS = 15,
    parameter ACC_WIDTH = INPUT_WIDTH + COEFF_WIDTH,
    parameter OUTPUT_WIDTH = ACC_WIDTH + 4  // log2(15) ~= 4
)
(
    input wire clk,
    input wire reset_n,
    input wire signed [COEFF_WIDTH*NUM_TAPS-1:0] coeffs, // flattened
    input wire signed [INPUT_WIDTH-1:0] s_axis_fir_tdata,
    input wire [5:0] s_axis_fir_tkeep,
    input wire s_axis_fir_tlast,
    input wire s_axis_fir_tvalid,
    input wire m_axis_fir_tready,
    output reg m_axis_fir_tvalid,
    output wire s_axis_fir_tready,
    output reg m_axis_fir_tlast,
    output reg [5:0] m_axis_fir_tkeep,
    output reg signed [OUTPUT_WIDTH-1:0] m_axis_fir_tdata,
    output reg signed [INPUT_WIDTH*NUM_TAPS-1:0] buff_array // flattened input buffer
);

    // Circular buffer registers
    reg signed [INPUT_WIDTH-1:0] buff0, buff1, buff2, buff3, buff4, buff5, buff6, buff7,
                                 buff8, buff9, buff10, buff11, buff12, buff13, buff14;

    // Extract taps from flattened coeffs
    wire signed [COEFF_WIDTH-1:0] tap0  = coeffs[0*COEFF_WIDTH +: COEFF_WIDTH];
    wire signed [COEFF_WIDTH-1:0] tap1  = coeffs[1*COEFF_WIDTH +: COEFF_WIDTH];
    wire signed [COEFF_WIDTH-1:0] tap2  = coeffs[2*COEFF_WIDTH +: COEFF_WIDTH];
    wire signed [COEFF_WIDTH-1:0] tap3  = coeffs[3*COEFF_WIDTH +: COEFF_WIDTH];
    wire signed [COEFF_WIDTH-1:0] tap4  = coeffs[4*COEFF_WIDTH +: COEFF_WIDTH];
    wire signed [COEFF_WIDTH-1:0] tap5  = coeffs[5*COEFF_WIDTH +: COEFF_WIDTH];
    wire signed [COEFF_WIDTH-1:0] tap6  = coeffs[6*COEFF_WIDTH +: COEFF_WIDTH];
    wire signed [COEFF_WIDTH-1:0] tap7  = coeffs[7*COEFF_WIDTH +: COEFF_WIDTH];
    wire signed [COEFF_WIDTH-1:0] tap8  = coeffs[8*COEFF_WIDTH +: COEFF_WIDTH];
    wire signed [COEFF_WIDTH-1:0] tap9  = coeffs[9*COEFF_WIDTH +: COEFF_WIDTH];
    wire signed [COEFF_WIDTH-1:0] tap10 = coeffs[10*COEFF_WIDTH +: COEFF_WIDTH];
    wire signed [COEFF_WIDTH-1:0] tap11 = coeffs[11*COEFF_WIDTH +: COEFF_WIDTH];
    wire signed [COEFF_WIDTH-1:0] tap12 = coeffs[12*COEFF_WIDTH +: COEFF_WIDTH];
    wire signed [COEFF_WIDTH-1:0] tap13 = coeffs[13*COEFF_WIDTH +: COEFF_WIDTH];
    wire signed [COEFF_WIDTH-1:0] tap14 = coeffs[14*COEFF_WIDTH +: COEFF_WIDTH];

    // Handshake for a single-stage output buffer (1-cycle latency).
    wire in_fire = s_axis_fir_tvalid && s_axis_fir_tready;
    assign s_axis_fir_tready = ~m_axis_fir_tvalid || m_axis_fir_tready;

    // Circular buffer shift
    always @(posedge clk) begin
        if (in_fire) begin
            buff0  <= s_axis_fir_tdata;
            buff1  <= buff0;
            buff2  <= buff1;
            buff3  <= buff2;
            buff4  <= buff3;
            buff5  <= buff4;
            buff6  <= buff5;
            buff7  <= buff6;
            buff8  <= buff7;
            buff9  <= buff8;
            buff10 <= buff9;
            buff11 <= buff10;
            buff12 <= buff11;
            buff13 <= buff12;
            buff14 <= buff13;
        end
    end

    // Update flattened buffer output for LMS
    always @(posedge clk) begin
        buff_array <= {buff14, buff13, buff12, buff11, buff10, buff9, buff8,
                       buff7, buff6, buff5, buff4, buff3, buff2, buff1, buff0};
    end

    // Multiply stage
    reg signed [ACC_WIDTH-1:0] acc0, acc1, acc2, acc3, acc4, acc5, acc6, acc7,
                                acc8, acc9, acc10, acc11, acc12, acc13, acc14;

    always @(posedge clk) begin
        if (in_fire) begin
            acc0  <= tap0  * buff0;
            acc1  <= tap1  * buff1;
            acc2  <= tap2  * buff2;
            acc3  <= tap3  * buff3;
            acc4  <= tap4  * buff4;
            acc5  <= tap5  * buff5;
            acc6  <= tap6  * buff6;
            acc7  <= tap7  * buff7;
            acc8  <= tap8  * buff8;
            acc9  <= tap9  * buff9;
            acc10 <= tap10 * buff10;
            acc11 <= tap11 * buff11;
            acc12 <= tap12 * buff12;
            acc13 <= tap13 * buff13;
            acc14 <= tap14 * buff14;
        end
    end

    wire signed [OUTPUT_WIDTH-1:0] acc_sum = acc0 + acc1 + acc2 + acc3 + acc4 + acc5 + acc6 + acc7 +
                                             acc8 + acc9 + acc10 + acc11 + acc12 + acc13 + acc14;

    reg fire_d = 1'b0;
    reg [5:0] tkeep_d = 6'b0;
    reg tlast_d = 1'b0;

    always @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            fire_d  <= 1'b0;
            tkeep_d <= 6'b0;
            tlast_d <= 1'b0;
        end else begin
            fire_d <= in_fire;
            if (in_fire) begin
                tkeep_d <= s_axis_fir_tkeep;
                tlast_d <= s_axis_fir_tlast;
            end
        end
    end

    // Output stage (aligned with acc_sum from previous cycle)
    always @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            m_axis_fir_tvalid <= 1'b0;
            m_axis_fir_tdata  <= {OUTPUT_WIDTH{1'b0}};
            m_axis_fir_tkeep  <= 6'b0;
            m_axis_fir_tlast  <= 1'b0;
        end else if (fire_d) begin
            m_axis_fir_tvalid <= 1'b1;
            m_axis_fir_tdata  <= acc_sum;
            m_axis_fir_tkeep  <= tkeep_d;
            m_axis_fir_tlast  <= tlast_d;
        end else if (m_axis_fir_tvalid && m_axis_fir_tready) begin
            m_axis_fir_tvalid <= 1'b0;
        end
    end

endmodule
