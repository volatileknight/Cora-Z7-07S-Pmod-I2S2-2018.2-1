`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// Company: Digilent
// Engineer: Arthur Brown
// 
// Create Date: 03/23/2018 01:23:15 PM
// Module Name: axis_volume_controller
// Description: AXI-Stream volume controller intended for use with AXI Stream Pmod I2S2 controller.
//              Whenever a 2-word packet is received on the slave interface, it is multiplied by 
//              the value of the switches, taken to represent the range 0.0:1.0, then sent over the
//              master interface. Reception of data on the slave interface is halted while processing and
//              transfer is taking place.
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module axis_volume_controller #(
    parameter SWITCH_WIDTH = 4, // WARNING: this module has not been tested with other values of SWITCH_WIDTH, it will likely need some changes
    parameter DATA_WIDTH = 24
) (
    input wire clk,
    input wire reset_n,
    input wire [SWITCH_WIDTH-1:0] sw,
    
    //AXIS SLAVE INTERFACE
    input  wire [DATA_WIDTH-1:0] s_axis_data,
    input  wire s_axis_valid,
    output wire s_axis_ready,
    input  wire s_axis_last,
    
    // AXIS MASTER INTERFACE
    output reg [DATA_WIDTH-1:0] m_axis_data = 1'b0,
    output reg m_axis_valid = 1'b0,
    input  wire m_axis_ready,
    output reg m_axis_last = 1'b0,

    // MIC INPUT
    input wire lmem_data,
    input wire lmem_lrclk,
    input wire lmem_bclk,
    input wire rmem_data
);
    localparam MULTIPLIER_WIDTH = 24;
    reg [MULTIPLIER_WIDTH+DATA_WIDTH-1:0] data [1:0];
        
    reg [SWITCH_WIDTH-1:0] sw_sync_r [2:0];
    wire [SWITCH_WIDTH-1:0] sw_sync = sw_sync_r[2];
//    wire [SWITCH_WIDTH:0] m = {1'b0, sw_sync} + 1;
    reg [MULTIPLIER_WIDTH:0] multiplier = 'b0; // range of 0x00:0x10 for width=4
    
    wire m_select = m_axis_last;
    wire m_new_word = (m_axis_valid == 1'b1 && m_axis_ready == 1'b1) ? 1'b1 : 1'b0;
    wire m_new_packet = (m_new_word == 1'b1 && m_axis_last == 1'b1) ? 1'b1 : 1'b0;
    
    reg s_new_packet_r = 1'b0;
    reg s_axis_ready_int = 1'b1;


    /************ MEM INPUT PROCESSING ************/
    //mem_lrclk falls, then on the posedge of mem_bclk, lmem_data is valid
    reg [23:0] shift_lmem_data = 24'd0; //shift lmem_data into this
    reg [23:0] axis_lmem_data = 24'd0; //register shift_lmem_data into this
    reg [23:0] shift_rmem_data = 24'd0; //shift lmem_data into this
    reg [23:0] axis_rmem_data = 24'd0; //register shift_rmem_data into this
    // counts from 0 to 63 for each sample, then resets on the next lrclk falling edge
    reg [6:0] lmem_data_count = 7'd0; 
    reg old_lmem_lrclk = 1'b0;
    
    always@(posedge lmem_bclk)
        old_lmem_lrclk <= lmem_lrclk;

    always@(posedge lmem_bclk) begin //this happens slightly after negedge lrclk
        if (old_lmem_lrclk == 1'b1 && lmem_lrclk == 1'b0) //detect lrclk falling edge
            lmem_data_count <= 1'b0; //reset count on lrclk falling edge
        else
            lmem_data_count <= lmem_data_count + 1'b1; //increment count

        if (lmem_data_count < 7'd24)
            shift_lmem_data <= (shift_lmem_data << 1) | lmem_data;
        else if (lmem_data_count == 7'd24)
            axis_lmem_data <= shift_lmem_data; //register the full sample

        if (lmem_data_count < 7'd24)
            shift_rmem_data <= (shift_rmem_data << 1) | rmem_data;
        else if (lmem_data_count == 7'd24)
            axis_rmem_data <= shift_rmem_data; //register the full sample
    end

    localparam FIR_COEFF_WIDTH = 24;
    localparam FIR_OUTPUT_WIDTH = DATA_WIDTH + FIR_COEFF_WIDTH + 4;
    localparam FIR_FRAC_BITS = 23;
    wire signed [359:0] buff_array;
    wire signed [359:0] coeffs;
    wire signed [FIR_OUTPUT_WIDTH-1:0] fir_m_data;
    wire [5:0] fir_m_keep;
    wire fir_m_valid;
    wire fir_m_ready;
    wire fir_m_last;
    wire fir_s_ready;
    function automatic signed [DATA_WIDTH-1:0] sat_fir;
        input signed [FIR_OUTPUT_WIDTH-1:0] x;
        reg signed [FIR_OUTPUT_WIDTH-1:0] max_ext;
        reg signed [FIR_OUTPUT_WIDTH-1:0] min_ext;
    begin
        max_ext = {{(FIR_OUTPUT_WIDTH-DATA_WIDTH){1'b0}}, {1'b0, {(DATA_WIDTH-1){1'b1}}}};
        min_ext = {{(FIR_OUTPUT_WIDTH-DATA_WIDTH){1'b1}}, {1'b1, {(DATA_WIDTH-1){1'b0}}}};
        if (x > max_ext)
            sat_fir = max_ext[DATA_WIDTH-1:0];
        else if (x < min_ext)
            sat_fir = min_ext[DATA_WIDTH-1:0];
        else
            sat_fir = x[DATA_WIDTH-1:0];
    end
    endfunction

    localparam signed [FIR_OUTPUT_WIDTH-1:0] FIR_ROUND =
        ({{(FIR_OUTPUT_WIDTH-1){1'b0}}, 1'b1} <<< (FIR_FRAC_BITS-1));
    wire signed [FIR_OUTPUT_WIDTH-1:0] fir_round =
        fir_m_data + (fir_m_data[FIR_OUTPUT_WIDTH-1] ? -FIR_ROUND : FIR_ROUND);
    wire signed [FIR_OUTPUT_WIDTH-1:0] fir_shift = fir_round >>> FIR_FRAC_BITS;
    wire signed [DATA_WIDTH-1:0] fir_sample = sat_fir(fir_shift);
    wire s_select = fir_m_last;
    wire s_new_word = (fir_m_valid == 1'b1 && fir_m_ready == 1'b1) ? 1'b1 : 1'b0;
    wire s_new_packet = (s_new_word == 1'b1 && fir_m_last == 1'b1) ? 1'b1 : 1'b0;

    /************ FIR, LMS INTEGRATION ************/
    FIR #(
        .INPUT_WIDTH(DATA_WIDTH),
        .COEFF_WIDTH(FIR_COEFF_WIDTH)
    ) fir (
        .clk(clk),
        .reset_n(reset_n),
        .s_axis_fir_tdata(s_axis_data),
        .s_axis_fir_tkeep(6'b111111), // all bytes are valid since we're using 24-bit samples
        .s_axis_fir_tlast(s_axis_last),
        .s_axis_fir_tvalid(s_axis_valid && s_axis_ready_int),
        .s_axis_fir_tready(fir_s_ready),
        .m_axis_fir_tdata(fir_m_data),
        .m_axis_fir_tkeep(fir_m_keep),
        .m_axis_fir_tlast(fir_m_last),
        .m_axis_fir_tvalid(fir_m_valid),
        .m_axis_fir_tready(fir_m_ready),
        .buff_array(buff_array),
        .coeffs(coeffs)
    );

    LMS lms (
        .clk(clk),
        .reset_n(reset_n),
        .buff_array(buff_array),
        .err(axis_lmem_data), // use left mic as error signal for testing
        .wn_u(coeffs)
     );

    assign s_axis_ready = s_axis_ready_int && fir_s_ready;
    assign fir_m_ready = s_axis_ready_int;


    reg [SWITCH_WIDTH-1:0] vol;

    always@(posedge clk)
        vol <= 4'b0001;
    
    always@(posedge clk) begin
        sw_sync_r[2] <= sw_sync_r[1];
        sw_sync_r[1] <= sw_sync_r[0];
        sw_sync_r[0] <= sw; 
        
        
//        if (&sw_sync == 1'b1)
//            multiplier <= {1'b1, {MULTIPLIER_WIDTH{1'b0}}};
//        else
            // multiplier <= {1'b0, sw, {MULTIPLIER_WIDTH-SWITCH_WIDTH{1'b0}}} + 1;
            multiplier <= {vol,{MULTIPLIER_WIDTH{1'b0}}} / {SWITCH_WIDTH{1'b1}};
            
        s_new_packet_r <= s_new_packet;
    end
    
    always@(posedge clk)
        if (s_new_word == 1'b1) // sign extend and register FIR output
            data[s_select] <= {{MULTIPLIER_WIDTH{fir_sample[DATA_WIDTH-1]}}, fir_sample};
        else if (s_new_packet_r == 1'b1) begin
            data[0] <= $signed(data[0]) * multiplier;
            data[1] <= $signed(data[1]) * multiplier;
        end
        
    always@(posedge clk)
        if (s_new_packet_r == 1'b1)
            m_axis_valid <= 1'b1;
        else if (m_new_packet == 1'b1)
            m_axis_valid <= 1'b0;
            
    always@(posedge clk)
        if (m_new_packet == 1'b1)
            m_axis_last <= 1'b0;
        else if (m_new_word == 1'b1)
            m_axis_last <= 1'b1;
            
    always@(m_axis_valid, data[0], data[1], m_select)
        if (m_axis_valid == 1'b1)
            m_axis_data = data[m_select][MULTIPLIER_WIDTH+DATA_WIDTH-1:MULTIPLIER_WIDTH];
        else
            m_axis_data = 'b0;
            
    always@(posedge clk)
        if (s_new_packet == 1'b1)
            s_axis_ready_int <= 1'b0;
        else if (m_new_packet == 1'b1)
            s_axis_ready_int <= 1'b1;
endmodule
