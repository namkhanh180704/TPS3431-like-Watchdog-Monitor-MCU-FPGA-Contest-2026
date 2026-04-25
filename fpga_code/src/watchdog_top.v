// =============================================================================
// Module: watchdog_top
// Description: Top-level for TPS3431-like watchdog on Kiwi 1P5 board
//              Gowin GW1N-UV1P5QN48XF
//
// Clock source: On-board 27 MHz crystal X3 -> Pin 4 (IOL6A/GCLKT_7)
//               (Verified from official KiCad schematic)
//
// Pin mapping (from schematic):
//   CLK 27MHz   -> Pin 4  IOL6A  (SYS_CLK from X3 crystal)
//   S1 (WDI)    -> Pin 35 IOR1B  (KEY1, active-low)
//   S2 (EN)     -> Pin 36 IOR1A  (KEY2, active-low)
//   LED D3 WDO  -> Pin 27 IOR17A (active-HIGH)
//   LED D4 ENOUT-> Pin 28 IOR15B (active-HIGH)
//   UART RXD    -> Pin 34 IOR11A (U2U_TXD: GWU2U TX -> FPGA RX)
//   UART TXD    -> Pin 33 IOR11B (U2U_RXD: FPGA TX -> GWU2U RX)
// =============================================================================

module watchdog_top #(
    parameter CLK_FREQ    = 27_000_000,  // 27 MHz crystal (precise)
    parameter BAUD        = 115_200,
    parameter DEBOUNCE_MS = 15
)(
    input  wire clk_27m,     // 27 MHz crystal on Pin 4
    input  wire btn_s1_n,    // Button S1 - WDI kick (active-low)
    input  wire btn_s2_n,    // Button S2 - EN (active-low)
    input  wire uart_rxd,    // UART RX from PC (Pin 34)
    output wire uart_txd,    // UART TX to PC (Pin 33)
    output wire led_d3,      // LED D3 - WDO fault indicator
    output wire led_d4       // LED D4 - ENOUT indicator
);

    // =========================================================================
    // Internal reset (simple power-on reset)
    // =========================================================================
    reg [3:0] por_cnt = 4'd0;
    reg       rst_n_int = 1'b0;

    always @(posedge clk_27m) begin
        if (por_cnt < 4'd15) begin
            por_cnt   <= por_cnt + 1'b1;
            rst_n_int <= 1'b0;
        end else begin
            rst_n_int <= 1'b1;
        end
    end

    // =========================================================================
    // Debounce: Button S1 (WDI)
    // =========================================================================
    wire wdi_debounced;
    wire wdi_fall;
    wire wdi_rise;

    sync_debounce #(
        .CLK_FREQ    (CLK_FREQ),
        .DEBOUNCE_MS (DEBOUNCE_MS)
    ) u_debounce_wdi (
        .clk      (clk_27m),
        .rst_n    (rst_n_int),
        .btn_n    (btn_s1_n),
        .btn_out  (wdi_debounced),
        .btn_fall (wdi_fall),
        .btn_rise (wdi_rise)
    );

    // =========================================================================
    // Debounce: Button S2 (EN)
    // =========================================================================
    wire en_debounced;
    wire en_fall;
    wire en_rise;

    sync_debounce #(
        .CLK_FREQ    (CLK_FREQ),
        .DEBOUNCE_MS (DEBOUNCE_MS)
    ) u_debounce_en (
        .clk      (clk_27m),
        .rst_n    (rst_n_int),
        .btn_n    (btn_s2_n),
        .btn_out  (en_debounced),
        .btn_fall (en_fall),
        .btn_rise (en_rise)
    );

    // =========================================================================
    // UART RX / TX
    // =========================================================================
    wire [7:0] rx_data;
    wire       rx_valid;
    wire [7:0] tx_data;
    wire       tx_start;
    wire       tx_busy;

    uart_rx #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD     (BAUD)
    ) u_uart_rx (
        .clk      (clk_27m),
        .rst_n    (rst_n_int),
        .rxd      (uart_rxd),
        .rx_data  (rx_data),
        .rx_valid (rx_valid)
    );

    uart_tx #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD     (BAUD)
    ) u_uart_tx (
        .clk      (clk_27m),
        .rst_n    (rst_n_int),
        .tx_data  (tx_data),
        .tx_start (tx_start),
        .txd      (uart_txd),
        .tx_busy  (tx_busy)
    );

    // =========================================================================
    // UART Frame Parser
    // =========================================================================
    wire        reg_wr_en;
    wire [7:0]  reg_wr_addr;
    wire [31:0] reg_wr_data;
    wire        reg_rd_en;
    wire [7:0]  reg_rd_addr;
    wire [31:0] reg_rd_data;
    wire        reg_rd_valid;
    wire        uart_kick;

    uart_frame_parser u_parser (
        .clk         (clk_27m),
        .rst_n       (rst_n_int),
        .rx_data     (rx_data),
        .rx_valid    (rx_valid),
        .tx_data     (tx_data),
        .tx_start    (tx_start),
        .tx_busy     (tx_busy),
        .reg_wr_en   (reg_wr_en),
        .reg_wr_addr (reg_wr_addr),
        .reg_wr_data (reg_wr_data),
        .reg_rd_en   (reg_rd_en),
        .reg_rd_addr (reg_rd_addr),
        .reg_rd_data (reg_rd_data),
        .reg_rd_valid(reg_rd_valid),
        .uart_kick   (uart_kick)
    );

    // =========================================================================
    // Kick source mux + last kick source tracking
    // =========================================================================
    wire en_combined = en_debounced | en_sw;
    wire kick_combined = wdi_rise | uart_kick;

    reg last_kick_src;

    always @(posedge clk_27m or negedge rst_n_int) begin
        if (!rst_n_int)
            last_kick_src <= 1'b0;
        else if (uart_kick)
            last_kick_src <= 1'b1;
        else if (wdi_rise)
            last_kick_src <= 1'b0;
    end

    // =========================================================================
    // Register File
    // =========================================================================
    wire        en_sw;
    wire        wdi_src;
    wire        clr_fault;
    wire [31:0] tWD_ms;
    wire [31:0] tRST_ms;
    wire [15:0] arm_delay_us;
    wire        wdo_n;
    wire        enout;
    wire        fault_active;
    wire        en_effective;

    regfile u_regfile (
        .clk           (clk_27m),
        .rst_n         (rst_n_int),
        .wr_en         (reg_wr_en),
        .wr_addr       (reg_wr_addr),
        .wr_data       (reg_wr_data),
        .rd_en         (reg_rd_en),
        .rd_addr       (reg_rd_addr),
        .rd_data       (reg_rd_data),
        .rd_valid      (reg_rd_valid),
        .en_effective  (en_effective),
        .fault_active  (fault_active),
        .enout_status  (enout),
        .wdo_status    (~wdo_n),
        .last_kick_src (last_kick_src),
        .en_sw         (en_sw),
        .wdi_src       (wdi_src),
        .clr_fault     (clr_fault),
        .tWD_ms        (tWD_ms),
        .tRST_ms       (tRST_ms),
        .arm_delay_us  (arm_delay_us)
    );

    // =========================================================================
    // Watchdog Core
    // =========================================================================
    watchdog_core #(
        .CLK_FREQ (CLK_FREQ)
    ) u_wdt (
        .clk          (clk_27m),
        .rst_n        (rst_n_int),
        .en           (en_combined),
        .kick         (kick_combined),
        .clr_fault    (clr_fault),
        .tWD_ms       (tWD_ms),
        .tRST_ms      (tRST_ms),
        .arm_delay_us (arm_delay_us),
        .wdo_n        (wdo_n),
        .enout        (enout),
        .fault_active (fault_active),
        .en_effective (en_effective)
    );

    // =========================================================================
    // LED outputs (active-HIGH LEDs on Kiwi 1P5)
    // =========================================================================
    assign led_d3 = ~wdo_n;      // fault -> LED ON
    assign led_d4 = enout;       // armed -> LED ON

endmodule