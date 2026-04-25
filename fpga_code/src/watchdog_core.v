// =============================================================================
// Module: watchdog_core
// Description: TPS3431-like watchdog FSM with configurable timers
//              - Monitors kick signal (WDI falling edge)
//              - Asserts WDO (active-low) on timeout for tRST duration
//              - Supports enable/disable with arm_delay window
// Platform: Kiwi 1P5 (Gowin GW1N-UV1P5), CLK = 27 MHz
// =============================================================================

module watchdog_core #(
    parameter CLK_FREQ = 27_000_000  // 27 MHz system clock
)(
    input  wire        clk,
    input  wire        rst_n,

    // --- Control inputs ---
    input  wire        en,              // watchdog enable (active-high, debounced)
    input  wire        kick,            // 1-cycle pulse: valid kick event
    input  wire        clr_fault,       // 1-cycle pulse: clear fault immediately

    // --- Configuration (from register file) ---
    input  wire [31:0] tWD_ms,          // watchdog timeout in ms (default 1600)
    input  wire [31:0] tRST_ms,         // WDO hold time in ms (default 200)
    input  wire [15:0] arm_delay_us,    // arm delay in us (default 150)

    // --- Outputs ---
    output reg         wdo_n,           // watchdog output, active-low (0 = fault)
    output reg         enout,           // enable output (1 = watchdog armed & running)
    output reg         fault_active,    // status: currently in FAULT state
    output reg         en_effective     // status: watchdog is effectively enabled
);

    // =========================================================================
    // Tick generators
    // =========================================================================

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

    // --- 1 us tick ---
    localparam US_DIV = CLK_FREQ / 1_000_000;  // 27 for 27 MHz
    localparam US_W   = clog2(US_DIV);

    reg [US_W-1:0] us_prescaler;
    reg            us_tick;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            us_prescaler <= {US_W{1'b0}};
            us_tick      <= 1'b0;
        end else begin
            if (us_prescaler == US_DIV - 1) begin
                us_prescaler <= {US_W{1'b0}};
                us_tick      <= 1'b1;
            end else begin
                us_prescaler <= us_prescaler + 1'b1;
                us_tick      <= 1'b0;
            end
        end
    end

    // --- 1 ms tick (derived from us_tick) ---
    reg [9:0] ms_prescaler;  // counts 0..999
    reg       ms_tick;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ms_prescaler <= 10'd0;
            ms_tick      <= 1'b0;
        end else begin
            ms_tick <= 1'b0;
            if (us_tick) begin
                if (ms_prescaler == 10'd999) begin
                    ms_prescaler <= 10'd0;
                    ms_tick      <= 1'b1;
                end else begin
                    ms_prescaler <= ms_prescaler + 1'b1;
                end
            end
        end
    end

    // =========================================================================
    // FSM
    // =========================================================================
    localparam [2:0] S_DISABLED = 3'd0,
                     S_ARMING   = 3'd1,
                     S_ACTIVE   = 3'd2,
                     S_FAULT    = 3'd3;

    reg [2:0]  state, state_next;
    reg [15:0] arm_cnt;     // arm delay counter (us)
    reg [31:0] wd_cnt;      // watchdog timeout counter (ms)
    reg [31:0] rst_cnt;     // reset hold counter (ms)

    // Previous EN for edge detection (disabled → enabled)
    reg en_prev;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            en_prev <= 1'b0;
        else
            en_prev <= en;
    end

    wire en_rising = en & ~en_prev;

    // =========================================================================
    // FSM — sequential
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_DISABLED;
            arm_cnt      <= 16'd0;
            wd_cnt       <= 32'd0;
            rst_cnt      <= 32'd0;
            wdo_n        <= 1'b1;    // released (no fault)
            enout        <= 1'b0;
            fault_active <= 1'b0;
            en_effective <= 1'b0;
        end else begin
            case (state)

                // =============================================================
                // DISABLED: EN=0, everything idle
                // =============================================================
                S_DISABLED: begin
                    wdo_n        <= 1'b1;  // released
                    enout        <= 1'b0;
                    fault_active <= 1'b0;
                    en_effective <= 1'b0;
                    arm_cnt      <= 16'd0;
                    wd_cnt       <= 32'd0;
                    rst_cnt      <= 32'd0;

                    if (en) begin
                        // EN just went high -> start arming
                        state   <= S_ARMING;
                        arm_cnt <= 16'd0;
                    end
                end

                // =============================================================
                // ARMING: ignore WDI kicks during arm_delay_us window
                // =============================================================
                S_ARMING: begin
                    en_effective <= 1'b1;
                    enout        <= 1'b0;  // ENOUT stays low during arming
                    wdo_n        <= 1'b1;  // no fault yet
                    fault_active <= 1'b0;

                    if (!en) begin
                        // EN went low during arming -> back to disabled
                        state <= S_DISABLED;
                    end else if (us_tick) begin
                        if (arm_cnt >= arm_delay_us - 1) begin
                            // Arm delay elapsed -> go active
                            state  <= S_ACTIVE;
                            wd_cnt <= 32'd0;
                            enout  <= 1'b1;
                        end else begin
                            arm_cnt <= arm_cnt + 1'b1;
                        end
                    end
                end

                // =============================================================
                // ACTIVE: watchdog running, waiting for kicks
                // =============================================================
                S_ACTIVE: begin
                    en_effective <= 1'b1;
                    enout        <= 1'b1;
                    wdo_n        <= 1'b1;  // no fault
                    fault_active <= 1'b0;

                    if (!en) begin
                        state <= S_DISABLED;
                    end else if (kick) begin
                        // Valid kick received -> reset watchdog counter
                        wd_cnt <= 32'd0;
                    end else if (ms_tick) begin
                        if (wd_cnt >= tWD_ms - 1) begin
                            // Timeout! -> enter FAULT state
                            state        <= S_FAULT;
                            rst_cnt      <= 32'd0;
                            wdo_n        <= 1'b0;  // assert WDO (pull low)
                            fault_active <= 1'b1;
                        end else begin
                            wd_cnt <= wd_cnt + 1'b1;
                        end
                    end
                end

                // =============================================================
                // FAULT: WDO asserted for tRST_ms, then release and restart
                // =============================================================
                S_FAULT: begin
                    en_effective <= 1'b1;
                    enout        <= 1'b1;
                    wdo_n        <= 1'b0;   // WDO held low
                    fault_active <= 1'b1;

                    if (!en) begin
                        // Disable during fault -> go to disabled
                        state <= S_DISABLED;
                    end else if (clr_fault) begin
                        // CLR_FAULT: release WDO immediately, start new cycle
                        state        <= S_ACTIVE;
                        wd_cnt       <= 32'd0;
                        wdo_n        <= 1'b1;
                        fault_active <= 1'b0;
                    end else if (ms_tick) begin
                        if (rst_cnt >= tRST_ms - 1) begin
                            // tRST elapsed -> release WDO, start new cycle
                            state        <= S_ACTIVE;
                            wd_cnt       <= 32'd0;
                            wdo_n        <= 1'b1;
                            fault_active <= 1'b0;
                        end else begin
                            rst_cnt <= rst_cnt + 1'b1;
                        end
                    end
                end

                default: begin
                    state <= S_DISABLED;
                end
            endcase
        end
    end

endmodule
