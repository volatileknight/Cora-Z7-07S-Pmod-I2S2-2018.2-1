module LMS #(
    parameter INPUT_WIDTH = 24,
    parameter COEFF_WIDTH = 24,
    parameter NUM_TAPS = 15
)(
    input  wire clk,
    input  wire reset_n,
    input  wire signed [INPUT_WIDTH*NUM_TAPS-1:0] buff_array, // flattened buffer
    input  wire signed [INPUT_WIDTH-1:0] err,                     // Error mic sample
    output reg  signed [COEFF_WIDTH*NUM_TAPS-1:0] wn_u       // flattened coefficients
);

    parameter signed [23:0] gamma = 24'sh000CCD; // ≈ 0.05

    // Internal registers (flattened)
    reg signed [COEFF_WIDTH-1:0] wn [0:NUM_TAPS-1];
    integer i;

    // LMS update
    always @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            for (i = 0; i < NUM_TAPS; i = i + 1) begin
                wn[i] <= 0;
            end
            wn_u <= 0;
        end else begin
            for (i = 0; i < NUM_TAPS; i = i + 1) begin
                // Extract input sample from flattened vector
                // Note: [msb +: width] works in Verilog-2001
                // Flattened slice: bits i*INPUT_WIDTH to (i+1)*INPUT_WIDTH-1
                // Extend to wider bit width for multiply
                wn[i] <= wn[i] + ((gamma * (err * buff_array[i*INPUT_WIDTH +: INPUT_WIDTH])) >>> INPUT_WIDTH);
                
                // Update flattened output
                wn_u[i*COEFF_WIDTH +: COEFF_WIDTH] <= wn[i];
            end
        end
    end

endmodule
