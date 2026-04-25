// =============================================================================
// Module: uart_tx
// Description: UART transmitter, 8N1, parameterizable baud rate
// =============================================================================

module uart_tx #(
    parameter CLK_FREQ = 27_000_000,
    parameter BAUD     = 115_200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] tx_data,     // byte to send
    input  wire       tx_start,    // single-cycle pulse to start TX
    output reg        txd,         // UART TX pin
    output wire       tx_busy      // high while transmitting
);

    // =========================================================================
    // Baud rate timing
    // =========================================================================
    localparam CLKS_PER_BIT = CLK_FREQ / BAUD;

    function integer clog2;
        input integer value;
        integer i;
        begin
            clog2 = 0;
            for (i = value - 1; i > 0; i = i >> 1)
                clog2 = clog2 + 1;
        end
    endfunction

    localparam CNT_W        = clog2(CLKS_PER_BIT + 1);

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
    reg [7:0]       tx_shift;

    assign tx_busy = (state != S_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            clk_cnt  <= {CNT_W{1'b0}};
            bit_idx  <= 3'd0;
            tx_shift <= 8'd0;
            txd      <= 1'b1;  // idle high
        end else begin
            case (state)
                S_IDLE: begin
                    txd     <= 1'b1;
                    clk_cnt <= {CNT_W{1'b0}};
                    bit_idx <= 3'd0;
                    if (tx_start) begin
                        tx_shift <= tx_data;
                        state    <= S_START;
                    end
                end

                S_START: begin
                    txd <= 1'b0;  // start bit
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= {CNT_W{1'b0}};
                        state   <= S_DATA;
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                S_DATA: begin
                    txd <= tx_shift[bit_idx];  // LSB first
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= {CNT_W{1'b0}};
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
                    txd <= 1'b1;  // stop bit
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= {CNT_W{1'b0}};
                        state   <= S_IDLE;
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
