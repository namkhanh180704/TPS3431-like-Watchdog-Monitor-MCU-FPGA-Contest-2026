// =============================================================================
// Module: sync_debounce
// Description: 2-FF synchronizer + counter-based debounce for active-low buttons
//              Outputs active-high debounced signal + falling/rising edge pulses
// Platform: Kiwi 1P5 (Gowin GW1N-UV1P5), CLK = 27 MHz
// =============================================================================

module sync_debounce #(
    parameter CLK_FREQ   = 27_000_000,  // 27 MHz
    parameter DEBOUNCE_MS = 10           // 10 ms debounce window
)(
    input  wire clk,
    input  wire rst_n,       // active-low async reset
    input  wire btn_n,       // raw button input (active-low)
    output reg  btn_out,     // debounced output (active-high: 1 = pressed)
    output wire btn_fall,    // single-cycle pulse on falling edge of btn_out
    output wire btn_rise     // single-cycle pulse on rising edge of btn_out
);

    // -------------------------------------------------------------------------
    // Debounce counter width calculation
    // 27 MHz * 10 ms = 270,000 -> need 19 bits
    // -------------------------------------------------------------------------
    localparam DEBOUNCE_COUNT = (CLK_FREQ / 1000) * DEBOUNCE_MS;

    // Pure Verilog clog2 function (replaces SystemVerilog $clog2)
    function integer clog2;
        input integer value;
        integer i;
        begin
            clog2 = 0;
            for (i = value - 1; i > 0; i = i >> 1)
                clog2 = clog2 + 1;
        end
    endfunction

    localparam CNT_W = clog2(DEBOUNCE_COUNT + 1);

    // -------------------------------------------------------------------------
    // 2-FF synchronizer (metastability protection)
    // -------------------------------------------------------------------------
    reg sync_ff1, sync_ff2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync_ff1 <= 1'b1;  // button not pressed (active-low)
            sync_ff2 <= 1'b1;
        end else begin
            sync_ff1 <= btn_n;
            sync_ff2 <= sync_ff1;
        end
    end

    // Invert: active-low button -> active-high internal signal
    wire btn_sync = ~sync_ff2;

    // -------------------------------------------------------------------------
    // Counter-based debounce
    // Signal must be stable for DEBOUNCE_COUNT cycles to change output
    // -------------------------------------------------------------------------
    reg [CNT_W-1:0] cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt     <= {CNT_W{1'b0}};
            btn_out <= 1'b0;
        end else begin
            if (btn_sync != btn_out) begin
                // Input differs from current output — count up
                if (cnt == DEBOUNCE_COUNT - 1) begin
                    btn_out <= btn_sync;
                    cnt     <= {CNT_W{1'b0}};
                end else begin
                    cnt <= cnt + 1'b1;
                end
            end else begin
                // Input matches output — reset counter
                cnt <= {CNT_W{1'b0}};
            end
        end
    end

    // -------------------------------------------------------------------------
    // Edge detection (1-cycle delayed)
    // -------------------------------------------------------------------------
    reg btn_out_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            btn_out_d <= 1'b0;
        else
            btn_out_d <= btn_out;
    end

    assign btn_fall = btn_out_d & ~btn_out;  // was 1, now 0 (button released)
    assign btn_rise = ~btn_out_d & btn_out;  // was 0, now 1 (button pressed)

endmodule
