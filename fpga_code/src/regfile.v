// =============================================================================
// Module: regfile
// Description: Configuration register file for watchdog parameters
//              Supports read/write from UART frame parser
//              Addr 0x00: CTRL    (R/W, 32-bit)
//              Addr 0x04: tWD_ms  (R/W, 32-bit)
//              Addr 0x08: tRST_ms (R/W, 32-bit)
//              Addr 0x0C: arm_delay_us (R/W, 16-bit stored in 32-bit reg)
//              Addr 0x10: STATUS  (R, 32-bit)
// =============================================================================

module regfile #(
    parameter DEFAULT_TWD_MS       = 32'd1600,
    parameter DEFAULT_TRST_MS      = 32'd200,
    parameter DEFAULT_ARM_DELAY_US = 16'd150
)(
    input  wire        clk,
    input  wire        rst_n,

    // --- Write interface (from UART frame parser) ---
    input  wire        wr_en,
    input  wire [7:0]  wr_addr,
    input  wire [31:0] wr_data,

    // --- Read interface (from UART frame parser) ---
    input  wire        rd_en,
    input  wire [7:0]  rd_addr,
    output reg  [31:0] rd_data,
    output reg         rd_valid,

    // --- Status inputs (from watchdog_core) ---
    input  wire        en_effective,
    input  wire        fault_active,
    input  wire        enout_status,
    input  wire        wdo_status,
    input  wire        last_kick_src,  // 0=button, 1=UART

    // --- Outputs to watchdog_core / top ---
    output wire        en_sw,          // CTRL bit0: software enable
    output wire        wdi_src,        // CTRL bit1: WDI source select
    output reg         clr_fault,      // CTRL bit2: write-1-to-clear (pulse)
    output wire [31:0] tWD_ms,
    output wire [31:0] tRST_ms,
    output wire [15:0] arm_delay_us
);

    // =========================================================================
    // Register storage
    // =========================================================================
    reg [31:0] reg_ctrl;
    reg [31:0] reg_tWD_ms;
    reg [31:0] reg_tRST_ms;
    reg [31:0] reg_arm_delay_us;  // only lower 16 bits used

    // =========================================================================
    // Output assignments
    // =========================================================================
    assign en_sw        = reg_ctrl[0];
    assign wdi_src      = reg_ctrl[1];
    assign tWD_ms       = reg_tWD_ms;
    assign tRST_ms      = reg_tRST_ms;
    assign arm_delay_us = reg_arm_delay_us[15:0];

    // =========================================================================
    // STATUS register (read-only, assembled from live signals)
    // =========================================================================
    wire [31:0] status_reg = {27'd0,
                              last_kick_src,   // bit4
                              wdo_status,      // bit3
                              enout_status,    // bit2
                              fault_active,    // bit1
                              en_effective};   // bit0

    // =========================================================================
    // Write logic
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_ctrl         <= 32'd0;
            reg_tWD_ms       <= DEFAULT_TWD_MS;
            reg_tRST_ms      <= DEFAULT_TRST_MS;
            reg_arm_delay_us <= {16'd0, DEFAULT_ARM_DELAY_US};
            clr_fault        <= 1'b0;
        end else begin
            // CLR_FAULT is auto-clearing (write-1-to-clear, pulse for 1 cycle)
            clr_fault <= 1'b0;

            if (wr_en) begin
                case (wr_addr)
                    8'h00: begin
                        // CTRL register
                        reg_ctrl[1:0] <= wr_data[1:0];  // EN_SW, WDI_SRC
                        // bit2 CLR_FAULT: write-1-to-clear
                        if (wr_data[2])
                            clr_fault <= 1'b1;
                    end
                    8'h04: reg_tWD_ms       <= wr_data;
                    8'h08: reg_tRST_ms      <= wr_data;
                    8'h0C: reg_arm_delay_us <= {16'd0, wr_data[15:0]};
                    default: ; // ignore invalid addresses
                endcase
            end
        end
    end

    // =========================================================================
    // Read logic
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_data  <= 32'd0;
            rd_valid <= 1'b0;
        end else begin
            rd_valid <= 1'b0;
            if (rd_en) begin
                rd_valid <= 1'b1;
                case (rd_addr)
                    8'h00:   rd_data <= reg_ctrl;
                    8'h04:   rd_data <= reg_tWD_ms;
                    8'h08:   rd_data <= reg_tRST_ms;
                    8'h0C:   rd_data <= reg_arm_delay_us;
                    8'h10:   rd_data <= status_reg;
                    default: rd_data <= 32'hDEAD_BEEF;  // invalid address
                endcase
            end
        end
    end

endmodule
