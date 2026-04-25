// =============================================================================
// Module: uart_frame_parser
// Description: Parses UART frame protocol and generates register R/W signals
//
// Frame format (RX): [0x55][CMD][ADDR][LEN][DATA_0]...[DATA_N-1][CHK]
//   CMD: 0x01=WRITE_REG, 0x02=READ_REG, 0x03=KICK, 0x04=GET_STATUS
//   CHK: XOR of all bytes from CMD through last DATA byte
//   LEN: number of DATA bytes (0..4)
//
// Response (TX): [0x55][CMD|0x80][ADDR][LEN][DATA...][CHK]
//   ACK for WRITE: [0x55][0x81][ADDR][0x00][CHK]
//   RESP for READ:  [0x55][0x82][ADDR][0x04][D0][D1][D2][D3][CHK]
//   ACK for KICK:   [0x55][0x83][0x00][0x00][CHK]
//   RESP STATUS:    [0x55][0x84][0x10][0x04][D0][D1][D2][D3][CHK]
// =============================================================================

module uart_frame_parser (
    input  wire        clk,
    input  wire        rst_n,

    // --- UART RX byte stream ---
    input  wire [7:0]  rx_data,
    input  wire        rx_valid,

    // --- UART TX byte stream ---
    output reg  [7:0]  tx_data,
    output reg         tx_start,
    input  wire        tx_busy,

    // --- Register file interface ---
    output reg         reg_wr_en,
    output reg  [7:0]  reg_wr_addr,
    output reg  [31:0] reg_wr_data,
    output reg         reg_rd_en,
    output reg  [7:0]  reg_rd_addr,
    input  wire [31:0] reg_rd_data,
    input  wire        reg_rd_valid,

    // --- Kick output ---
    output reg         uart_kick
);

    // =========================================================================
    // RX frame parser FSM
    // =========================================================================
    localparam [3:0] RX_IDLE     = 4'd0,
                     RX_CMD      = 4'd1,
                     RX_ADDR     = 4'd2,
                     RX_LEN      = 4'd3,
                     RX_DATA     = 4'd4,
                     RX_CHK      = 4'd5,
                     RX_EXECUTE  = 4'd6;

    reg [3:0]  rx_state;
    reg [7:0]  cmd;
    reg [7:0]  addr;
    reg [7:0]  len;
    reg [7:0]  data_buf [0:3];   // max 4 data bytes
    reg [2:0]  data_cnt;
    reg [7:0]  chk_calc;         // running XOR checksum

    // =========================================================================
    // TX response FSM
    // =========================================================================
    localparam [3:0] TX_IDLE     = 4'd0,
                     TX_SYNC     = 4'd1,
                     TX_CMD      = 4'd2,
                     TX_ADDR     = 4'd3,
                     TX_LEN      = 4'd4,
                     TX_DATA     = 4'd5,
                     TX_CHK      = 4'd6,
                     TX_WAIT     = 4'd7,
                     TX_FINISH   = 4'd8;

    reg [3:0]  tx_state;
    reg [7:0]  resp_cmd;
    reg [7:0]  resp_addr;
    reg [7:0]  resp_len;
    reg [7:0]  resp_data [0:3];
    reg [2:0]  tx_data_cnt;
    reg [7:0]  tx_chk_calc;
    reg        tx_pending;       // a response needs to be sent
    reg        tx_done;          // TX FSM signals completion

    // =========================================================================
    // RX frame parser
    // =========================================================================
    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state   <= RX_IDLE;
            cmd        <= 8'd0;
            addr       <= 8'd0;
            len        <= 8'd0;
            data_cnt   <= 3'd0;
            chk_calc   <= 8'd0;
            reg_wr_en  <= 1'b0;
            reg_wr_addr<= 8'd0;
            reg_wr_data<= 32'd0;
            reg_rd_en  <= 1'b0;
            reg_rd_addr<= 8'd0;
            uart_kick  <= 1'b0;
            tx_pending <= 1'b0;
            resp_cmd   <= 8'd0;
            resp_addr  <= 8'd0;
            resp_len   <= 8'd0;
            for (i = 0; i < 4; i = i + 1)
                data_buf[i] <= 8'd0;
        end else begin
            // Default: clear single-cycle pulses
            reg_wr_en <= 1'b0;
            reg_rd_en <= 1'b0;
            uart_kick <= 1'b0;

            // Clear tx_pending when TX FSM acknowledges
            if (tx_done)
                tx_pending <= 1'b0;

            case (rx_state)
                RX_IDLE: begin
                    if (rx_valid && rx_data == 8'h55) begin
                        rx_state <= RX_CMD;
                        chk_calc <= 8'd0;
                    end
                end

                RX_CMD: begin
                    if (rx_valid) begin
                        cmd      <= rx_data;
                        chk_calc <= rx_data;
                        rx_state <= RX_ADDR;
                    end
                end

                RX_ADDR: begin
                    if (rx_valid) begin
                        addr     <= rx_data;
                        chk_calc <= chk_calc ^ rx_data;
                        rx_state <= RX_LEN;
                    end
                end

                RX_LEN: begin
                    if (rx_valid) begin
                        len      <= rx_data;
                        chk_calc <= chk_calc ^ rx_data;
                        data_cnt <= 3'd0;
                        if (rx_data == 8'd0) begin
                            // No data bytes, go directly to checksum
                            rx_state <= RX_CHK;
                        end else begin
                            rx_state <= RX_DATA;
                        end
                    end
                end

                RX_DATA: begin
                    if (rx_valid) begin
                        if (data_cnt < 3'd4)
                            data_buf[data_cnt] <= rx_data;
                        chk_calc <= chk_calc ^ rx_data;
                        if (data_cnt == len[2:0] - 1) begin
                            rx_state <= RX_CHK;
                        end else begin
                            data_cnt <= data_cnt + 1'b1;
                        end
                    end
                end

                RX_CHK: begin
                    if (rx_valid) begin
                        if (rx_data == chk_calc) begin
                            // Checksum valid -> execute command
                            rx_state <= RX_EXECUTE;
                        end else begin
                            // Checksum error -> discard, back to idle
                            rx_state <= RX_IDLE;
                        end
                    end
                end

                RX_EXECUTE: begin
                    case (cmd)
                        8'h01: begin  // WRITE_REG
                            reg_wr_en   <= 1'b1;
                            reg_wr_addr <= addr;
                            // Assemble 32-bit data (little-endian)
                            reg_wr_data <= {data_buf[3], data_buf[2],
                                            data_buf[1], data_buf[0]};
                            // Prepare ACK response
                            resp_cmd  <= 8'h81;
                            resp_addr <= addr;
                            resp_len  <= 8'd0;
                            tx_pending <= 1'b1;
                        end

                        8'h02: begin  // READ_REG
                            reg_rd_en   <= 1'b1;
                            reg_rd_addr <= addr;
                            // Response will be sent after rd_valid
                            resp_cmd  <= 8'h82;
                            resp_addr <= addr;
                            resp_len  <= 8'd4;
                            // tx_pending set after rd_valid
                        end

                        8'h03: begin  // KICK
                            uart_kick <= 1'b1;
                            resp_cmd  <= 8'h83;
                            resp_addr <= 8'd0;
                            resp_len  <= 8'd0;
                            tx_pending <= 1'b1;
                        end

                        8'h04: begin  // GET_STATUS
                            reg_rd_en   <= 1'b1;
                            reg_rd_addr <= 8'h10;  // STATUS register
                            resp_cmd  <= 8'h84;
                            resp_addr <= 8'h10;
                            resp_len  <= 8'd4;
                        end

                        default: ; // unknown command, ignore
                    endcase
                    rx_state <= RX_IDLE;
                end

                default: rx_state <= RX_IDLE;
            endcase

            // --- Handle READ_REG / GET_STATUS response data ---
            if (reg_rd_valid && !tx_pending) begin
                resp_data[0] <= reg_rd_data[7:0];
                resp_data[1] <= reg_rd_data[15:8];
                resp_data[2] <= reg_rd_data[23:16];
                resp_data[3] <= reg_rd_data[31:24];
                tx_pending   <= 1'b1;
            end
        end
    end

    // =========================================================================
    // TX response FSM
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state    <= TX_IDLE;
            tx_data     <= 8'd0;
            tx_start    <= 1'b0;
            tx_data_cnt <= 3'd0;
            tx_chk_calc <= 8'd0;
            tx_done     <= 1'b0;
        end else begin
            tx_start <= 1'b0;
            tx_done  <= 1'b0;

            case (tx_state)
                TX_IDLE: begin
                    if (tx_pending && !tx_busy) begin
                        // Send sync byte 0x55
                        tx_data     <= 8'h55;
                        tx_start    <= 1'b1;
                        tx_chk_calc <= 8'd0;
                        tx_state    <= TX_WAIT;
                        tx_data_cnt <= 3'd0;
                        // Next state after TX_WAIT
                    end
                end

                TX_WAIT: begin
                    // Wait for current byte to finish transmitting
                    if (!tx_busy && !tx_start) begin
                        // Determine what to send next based on tx_data_cnt progression
                        // We use a simple sequence: SYNC(done) -> CMD -> ADDR -> LEN -> DATA... -> CHK
                        tx_state <= TX_CMD;
                    end
                end

                TX_CMD: begin
                    if (!tx_busy && !tx_start) begin
                        tx_data     <= resp_cmd;
                        tx_start    <= 1'b1;
                        tx_chk_calc <= resp_cmd;
                        tx_state    <= TX_ADDR;
                    end
                end

                TX_ADDR: begin
                    if (!tx_busy && !tx_start) begin
                        tx_data     <= resp_addr;
                        tx_start    <= 1'b1;
                        tx_chk_calc <= tx_chk_calc ^ resp_addr;
                        tx_state    <= TX_LEN;
                    end
                end

                TX_LEN: begin
                    if (!tx_busy && !tx_start) begin
                        tx_data     <= resp_len;
                        tx_start    <= 1'b1;
                        tx_chk_calc <= tx_chk_calc ^ resp_len;
                        tx_data_cnt <= 3'd0;
                        if (resp_len == 8'd0) begin
                            tx_state <= TX_CHK;
                        end else begin
                            tx_state <= TX_DATA;
                        end
                    end
                end

                TX_DATA: begin
                    if (!tx_busy && !tx_start) begin
                        tx_data     <= resp_data[tx_data_cnt];
                        tx_start    <= 1'b1;
                        tx_chk_calc <= tx_chk_calc ^ resp_data[tx_data_cnt];
                        if (tx_data_cnt == resp_len[2:0] - 1) begin
                            tx_state <= TX_CHK;
                        end else begin
                            tx_data_cnt <= tx_data_cnt + 1'b1;
                        end
                    end
                end

                TX_CHK: begin
                    if (!tx_busy && !tx_start) begin
                        tx_data    <= tx_chk_calc;
                        tx_start   <= 1'b1;
                        tx_done    <= 1'b1;
                        tx_state   <= TX_FINISH;
                    end
                end

                TX_FINISH: begin
                    // Wait for last byte to finish transmitting
                    // This also ensures tx_pending is cleared by RX FSM
                    if (!tx_busy && !tx_start) begin
                        tx_state <= TX_IDLE;
                    end
                end

                default: tx_state <= TX_IDLE;
            endcase
        end
    end

endmodule
