`default_nettype none

/*
The SPH0645LM4H microphone operates as an I2S 
slave. The master must provide the BCLK and WS 
signals. The Over Sampling Rate is fixed at 64 therefore 
the WS signal must be BCLK/64 and synchronized 
to the BCLK. Clock frequencies from 2.048Mhz to 
4.096MHz are supported so sampling rates from 32KHz 
to 64KHz can be had by changing the clock frequency. 
The Data Format is I2S, 24 bit, 2’s compliment, MSB 
first. The Data Precision is 18 bits, unused bits are zeros.

WS must be synchronized(changing) on the falling edge of BCLK
• WS must be BCLK/64
• The Hold Time (see Figure 2) must be greater than the Hold Time of the Receiver
• The mode must be I2S with MSB delayed 1 BCLK cycle after LRCLK changes
*/


module mems_clk_gen #(
    parameter integer F_CLK   = 125_000_000,
    parameter integer F_BCLK  = 3_072_000,
    parameter integer BITS    = 32   // bits per channel
)(
    input  wire clk,
    input  wire rst,
    output reg  bclk,
    output reg  lrclk
);

    // -------------------------
    // BCLK DDS
    // -------------------------
    localparam integer ACC_W = 32;
    reg [ACC_W-1:0] acc = 0;

    localparam integer PHASE_INC =
        (2 * F_BCLK << ACC_W) / F_CLK;

    // BCLK edge detect
    reg bclk_d;

    // LRCLK bit counter
    localparam integer LR_DIV = 2 * BITS;
    reg [$clog2(LR_DIV)-1:0] bit_cnt = 0;

    always @(posedge clk) begin
        if (rst) begin
            acc     <= 0;
            bclk    <= 0;
            bclk_d  <= 0;
            lrclk   <= 0;
            bit_cnt <= 0;
        end else begin
            // --- BCLK generation ---
            acc <= acc + PHASE_INC;
            if (acc[ACC_W-1]) begin
                acc  <= acc & ~(1 << (ACC_W-1));
                bclk <= ~bclk;
            end

            // --- Edge detect ---
            bclk_d <= bclk;

            // --- LRCLK generation ---
            if (bclk & ~bclk_d) begin  // rising edge of BCLK
                if (bit_cnt == LR_DIV-1) begin
                    bit_cnt <= 0;
                    lrclk   <= ~lrclk;
                end else begin
                    bit_cnt <= bit_cnt + 1;
                end
            end
        end
    end
endmodule

