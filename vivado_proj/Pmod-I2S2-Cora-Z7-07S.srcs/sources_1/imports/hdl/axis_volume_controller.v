`timescale 1ns / 1ps
`default_nettype none

module axis_volume_controller #(
    parameter SWITCH_WIDTH = 4,
    parameter DATA_WIDTH = 24
) (
    input wire clk,
    input wire lrclk,
    input wire reset_n,
    input wire [SWITCH_WIDTH-1:0] sw,
    
    //AXIS SLAVE INTERFACE
    input  wire [DATA_WIDTH-1:0] s_axis_data,
    input  wire s_axis_valid,
    output reg  s_axis_ready = 1'b1,
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
    reg [MULTIPLIER_WIDTH:0] multiplier = 'b0;
    
    wire m_select = m_axis_last;
    wire m_new_word = (m_axis_valid == 1'b1 && m_axis_ready == 1'b1) ? 1'b1 : 1'b0;
    wire m_new_packet = (m_new_word == 1'b1 && m_axis_last == 1'b1) ? 1'b1 : 1'b0;
    
    wire s_select = s_axis_last;
    wire s_new_word = (s_axis_valid == 1'b1 && s_axis_ready == 1'b1) ? 1'b1 : 1'b0;
    wire s_new_packet = (s_new_word == 1'b1 && s_axis_last == 1'b1) ? 1'b1 : 1'b0;
    reg s_new_packet_r = 1'b0;

    /************ MEM INPUT PROCESSING ************/
    reg [23:0] shift_lmem_data = 24'd0;
    reg [23:0] axis_lmem_data = 24'd0;
    reg [23:0] shift_rmem_data = 24'd0;
    reg [23:0] axis_rmem_data = 24'd0;
    reg [6:0] lmem_data_count = 7'd0;
    reg old_lmem_lrclk = 1'b0;
    
    always@(posedge lmem_bclk)
        old_lmem_lrclk <= lmem_lrclk;

    always@(posedge lmem_bclk) begin
        if (old_lmem_lrclk == 1'b1 && lmem_lrclk == 1'b0)
            lmem_data_count <= 1'b0;
        else
            lmem_data_count <= lmem_data_count + 1'b1;

        if (lmem_data_count < 7'd24)
            shift_lmem_data <= (shift_lmem_data << 1) | lmem_data;
        else if (lmem_data_count == 7'd24)
            axis_lmem_data <= shift_lmem_data;

        if (lmem_data_count < 7'd24)
            shift_rmem_data <= (shift_rmem_data << 1) | rmem_data;
        else if (lmem_data_count == 7'd24)
            axis_rmem_data <= shift_rmem_data;
    end

    /************ FIR INPUT GENERATION (lrclk domain) ************/
    reg fir_input_valid = 1'b0;
    reg old_lmem_lrclk_clk = 1'b0;
    reg [1:0] lmem_lrclk_sync = 2'b0;
    
    // Synchronize lmem_lrclk to lrclk domain
    always@(posedge lrclk) begin
        lmem_lrclk_sync <= {lmem_lrclk_sync[0], lmem_lrclk};
        old_lmem_lrclk_clk <= lmem_lrclk_sync[1];
        
        // Generate valid pulse on falling edge of lmem_lrclk (new sample ready)
        if (old_lmem_lrclk_clk == 1'b1 && lmem_lrclk_sync[1] == 1'b0)
            fir_input_valid <= 1'b1;
        else if (fir_output_ready)
            fir_input_valid <= 1'b0;
    end

    /************ FIR, LMS INTEGRATION ************/
    wire signed [359:0] buff_array;
    wire signed [359:0] coeffs;
    wire [23:0] fir_output;
    wire fir_output_valid;
    reg fir_output_ready = 1'b0;

    FIR fir (
        .clk(lrclk),
        .reset_n(reset_n),
        .s_axis_fir_tdata(axis_rmem_data),
        .s_axis_fir_tkeep(6'b111111),
        .s_axis_fir_tlast(1'b0),
        .s_axis_fir_tvalid(fir_input_valid),
        .m_axis_fir_tready(fir_output_ready),
        .m_axis_fir_tdata(fir_output),
        .m_axis_fir_tvalid(fir_output_valid),
        .buff_array(buff_array),
        .coeffs(coeffs)
    );

    LMS lms (
        .clk(lrclk),
        .reset_n(reset_n),
        .buff_array(buff_array),
        .err(axis_lmem_data),
        .wn_u(coeffs)
    );

    /************ CLOCK DOMAIN CROSSING (lrclk -> clk) ************/
    reg [23:0] fir_output_sync [2:0];
    reg fir_valid_sync [2:0];
    reg fir_valid_pulse = 1'b0;
    reg fir_valid_prev = 1'b0;
    
    // Multi-stage synchronizer for data
    always@(posedge clk) begin
        fir_output_sync[0] <= fir_output;
        fir_output_sync[1] <= fir_output_sync[0];
        fir_output_sync[2] <= fir_output_sync[1];
        
        fir_valid_sync[0] <= fir_output_valid;
        fir_valid_sync[1] <= fir_valid_sync[0];
        fir_valid_sync[2] <= fir_valid_sync[1];
        
        fir_valid_prev <= fir_valid_sync[2];
        fir_valid_pulse <= fir_valid_sync[2] & ~fir_valid_prev; // edge detect
    end

    wire [23:0] fir_output_amp;
    assign fir_output_amp = fir_output_sync[2] <<< 2; //+12dB scaling
    
    // Acknowledge in lrclk domain
    reg ready_toggle = 1'b0;
    reg [2:0] ready_sync = 3'b0;
    reg ready_prev = 1'b0;
    
    always@(posedge clk) begin
        if (fir_valid_pulse)
            ready_toggle <= ~ready_toggle;
    end
    
    always@(posedge lrclk) begin
        ready_sync <= {ready_sync[1:0], ready_toggle};
        ready_prev <= ready_sync[2];
        fir_output_ready <= (ready_sync[2] != ready_prev);
    end

    /************ VOLUME CONTROL ************/
    reg [SWITCH_WIDTH-1:0] vol;
    wire [23:0] comb_mem_data = axis_lmem_data + axis_rmem_data;
    
    always@(posedge clk)
        vol <= 4'b0001;
    
    always@(posedge clk) begin
        sw_sync_r[2] <= sw_sync_r[1];
        sw_sync_r[1] <= sw_sync_r[0];
        sw_sync_r[0] <= sw; 
        
        multiplier <= {vol,{MULTIPLIER_WIDTH{1'b0}}} / {SWITCH_WIDTH{1'b1}};
        s_new_packet_r <= s_new_packet;
    end
    
    always@(posedge clk) begin
        if (s_new_word == 1'b1) begin
            if (sw_sync == 4'b0001)
                data[s_select] <= {{MULTIPLIER_WIDTH{s_axis_data[DATA_WIDTH-1]}}, s_axis_data};
            else
                data[s_select] <= {{MULTIPLIER_WIDTH{fir_output_amp[2][DATA_WIDTH-1]}}, fir_output_amp[2]};
        end else if (s_new_packet_r == 1'b1) begin
            data[0] <= $signed(data[0]) * multiplier;
            data[1] <= $signed(data[1]) * multiplier;
        end
    end
        
    always@(posedge clk) begin
        if (s_new_packet_r == 1'b1)
            m_axis_valid <= 1'b1;
        else if (m_new_packet == 1'b1)
            m_axis_valid <= 1'b0;
    end
            
    always@(posedge clk) begin
        if (m_new_packet == 1'b1)
            m_axis_last <= 1'b0;
        else if (m_new_word == 1'b1)
            m_axis_last <= 1'b1;
    end
            
    always@(*) begin
        if (m_axis_valid == 1'b1)
            m_axis_data = data[m_select][MULTIPLIER_WIDTH+DATA_WIDTH-1:MULTIPLIER_WIDTH];
        else
            m_axis_data = 'b0;
    end
            
    always@(posedge clk) begin
        if (s_new_packet == 1'b1)
            s_axis_ready <= 1'b0;
        else if (m_new_packet == 1'b1)
            s_axis_ready <= 1'b1;
    end
endmodule