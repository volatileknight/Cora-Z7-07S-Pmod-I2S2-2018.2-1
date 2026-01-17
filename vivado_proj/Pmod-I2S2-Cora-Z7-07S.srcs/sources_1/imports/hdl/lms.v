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

    parameter signed [23:0] gamma = 24'sh000CCD; // Q0.23 ~= 0.00039

    // Internal registers (flattened)
    reg signed [COEFF_WIDTH-1:0] wn [0:NUM_TAPS-1];
    integer i;

    function automatic signed [COEFF_WIDTH-1:0] sat_coeff;
        input signed [COEFF_WIDTH:0] x;
        reg signed [COEFF_WIDTH-1:0] maxv;
        reg signed [COEFF_WIDTH-1:0] minv;
    begin
        maxv = {1'b0, {(COEFF_WIDTH-1){1'b1}}};
        minv = {1'b1, {(COEFF_WIDTH-1){1'b0}}};
        if (x > maxv)
            sat_coeff = maxv;
        else if (x < minv)
            sat_coeff = minv;
        else
            sat_coeff = x[COEFF_WIDTH-1:0];
    end
    endfunction

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
                reg signed [2*INPUT_WIDTH-1:0] err_x;
                reg signed [2*INPUT_WIDTH+COEFF_WIDTH-1:0] mult;
                reg signed [2*INPUT_WIDTH+COEFF_WIDTH-1:0] mult_round;
                reg signed [COEFF_WIDTH-1:0] delta;
                reg signed [COEFF_WIDTH:0] wn_sum;

                err_x = err * buff_array[i*INPUT_WIDTH +: INPUT_WIDTH];
                mult = gamma * err_x;
                mult_round = mult + (mult[2*INPUT_WIDTH+COEFF_WIDTH-1] ? -(1'sb1 <<< (INPUT_WIDTH-1))
                                                                      :  (1'sb1 <<< (INPUT_WIDTH-1)));
                delta = mult_round >>> INPUT_WIDTH;
                wn_sum = wn[i] + delta;
                wn[i] <= sat_coeff(wn_sum);

                // Update flattened output with the new coefficient
                wn_u[i*COEFF_WIDTH +: COEFF_WIDTH] <= sat_coeff(wn_sum);
            end
        end
    end

endmodule
