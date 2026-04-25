// =============================================================================
// Module: uart_rx
// Description: UART receiver, 8N1, parameterizable baud rate
//              Oversamples at 16x baud rate, samples at mid-bit
// =============================================================================

module uart_rx #(
    parameter CLK_FREQ = 27_000_000,
    parameter BAUD     = 115_200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rxd,         // UART RX pin (active-high idle)
    output reg  [7:0] rx_data,     // received byte
    output reg        rx_valid     // single-cycle pulse when byte ready
);

    // =========================================================================
    // Baud rate timing
    // =========================================================================
    localparam CLKS_PER_BIT  = CLK_FREQ / BAUD;          // ~234 for 115200@27MHz
    localparam HALF_BIT      = CLKS_PER_BIT / 2;         // sample at mid-bit

    function integer clog2;
        input integer value;
        integer i;
        begin
            clog2 = 0;
            for (i = value - 1; i > 0; i = i >> 1)
                clog2 = clog2 + 1;
        end
    endfunction

    localparam CNT_W         = clog2(CLKS_PER_BIT + 1);

    // =========================================================================
    // 2-FF synchronizer for RXD
    // =========================================================================
    reg rxd_ff1, rxd_ff2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rxd_ff1 <= 1'b1;
            rxd_ff2 <= 1'b1;
        end else begin
            rxd_ff1 <= rxd;
            rxd_ff2 <= rxd_ff1;
        end
    end

    wire rxd_sync = rxd_ff2;

    // =========================================================================
    // FSM
    // =========================================================================
    localparam [1:0] S_IDLE  = 2'd0,
                     S_START = 2'd1,
                     S_DATA  = 2'd2,
                     S_STOP  = 2'd3;

    reg [1:0]       state;
    reg [CNT_W-1:0] clk_cnt;
    reg [2:0]       bit_idx;
    reg [7:0]       rx_shift;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            clk_cnt  <= {CNT_W{1'b0}};
            bit_idx  <= 3'd0;
            rx_shift <= 8'd0;
            rx_data  <= 8'd0;
            rx_valid <= 1'b0;
        end else begin
            rx_valid <= 1'b0;

            case (state)
                S_IDLE: begin
                    clk_cnt <= {CNT_W{1'b0}};
                    bit_idx <= 3'd0;
                    if (rxd_sync == 1'b0) begin
                        // Start bit detected
                        state <= S_START;
                    end
                end

                S_START: begin
                    // Wait to mid-point of start bit to confirm
                    if (clk_cnt == HALF_BIT - 1) begin
                        clk_cnt <= {CNT_W{1'b0}};
                        if (rxd_sync == 1'b0) begin
                            // Valid start bit
                            state <= S_DATA;
                        end else begin
                            // False start — glitch
                            state <= S_IDLE;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                S_DATA: begin
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= {CNT_W{1'b0}};
                        // Sample bit (LSB first)
                        rx_shift[bit_idx] <= rxd_sync;
                        if (bit_idx == 3'd7) begin
                            state <= S_STOP;
                        end else begin
                            bit_idx <= bit_idx + 1'b1;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                S_STOP: begin
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt  <= {CNT_W{1'b0}};
                        rx_data  <= rx_shift;
                        rx_valid <= 1'b1;
                        state    <= S_IDLE;
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
