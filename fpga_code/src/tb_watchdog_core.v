// =============================================================================
// Testbench: tb_watchdog_core
// Covers: normal kick, timeout, disable, disable->enable (arm_delay),
//         CLR_FAULT, and basic sync_debounce verification
// Note: Uses short timer values for fast simulation
// =============================================================================

`timescale 1ns / 1ps

module tb_watchdog_core;

    // =========================================================================
    // Parameters — use SMALL values for fast simulation
    // Real board: 27 MHz.  Testbench: 27 MHz but short ms/us values
    // =========================================================================
    localparam CLK_FREQ     = 27_000_000;
    localparam CLK_PERIOD   = 37;  // ~27 MHz (37.037 ns)

    // Short timer values for simulation (not real defaults)
    localparam SIM_TWD_MS       = 10;   // 10 ms watchdog timeout
    localparam SIM_TRST_MS      = 3;    // 3 ms reset hold
    localparam SIM_ARM_DELAY_US = 5000;  // 5 ms arm delay (large for sim to observe arming)
    localparam SIM_DEBOUNCE_MS  = 1;    // 1 ms debounce (for faster sim)

    // =========================================================================
    // Signals
    // =========================================================================
    reg        clk;
    reg        rst_n;
    reg        btn_wdi_n;   // raw button S1 (active-low)
    reg        btn_en_n;    // raw button S2 (active-low)
    reg        uart_kick;   // UART kick pulse
    reg        clr_fault;

    // Debounced outputs
    wire       wdi_debounced;
    wire       wdi_fall;
    wire       wdi_rise;
    wire       en_debounced;
    wire       en_fall;
    wire       en_rise;

    // Watchdog outputs
    wire       wdo_n;
    wire       enout;
    wire       fault_active;
    wire       en_effective;

    // Combined kick: button press (physical WDI falling edge = btn_out rising)
    // OR UART kick pulse
    wire       kick = wdi_rise | uart_kick;

    // =========================================================================
    // Clock generation
    // =========================================================================
    initial clk = 0;
    always #(CLK_PERIOD / 2) clk = ~clk;

    // =========================================================================
    // DUT: sync_debounce for WDI (Button S1)
    // =========================================================================
    sync_debounce #(
        .CLK_FREQ    (CLK_FREQ),
        .DEBOUNCE_MS (SIM_DEBOUNCE_MS)
    ) u_debounce_wdi (
        .clk      (clk),
        .rst_n    (rst_n),
        .btn_n    (btn_wdi_n),
        .btn_out  (wdi_debounced),
        .btn_fall (wdi_fall),
        .btn_rise (wdi_rise)
    );

    // =========================================================================
    // DUT: sync_debounce for EN (Button S2)
    // =========================================================================
    sync_debounce #(
        .CLK_FREQ    (CLK_FREQ),
        .DEBOUNCE_MS (SIM_DEBOUNCE_MS)
    ) u_debounce_en (
        .clk      (clk),
        .rst_n    (rst_n),
        .btn_n    (btn_en_n),
        .btn_out  (en_debounced),
        .btn_fall (en_fall),
        .btn_rise (en_rise)
    );

    // =========================================================================
    // DUT: watchdog_core
    // =========================================================================
    watchdog_core #(
        .CLK_FREQ (CLK_FREQ)
    ) u_wdt (
        .clk          (clk),
        .rst_n        (rst_n),
        .en           (en_debounced),
        .kick         (kick),
        .clr_fault    (clr_fault),
        .tWD_ms       (SIM_TWD_MS),
        .tRST_ms      (SIM_TRST_MS),
        .arm_delay_us (SIM_ARM_DELAY_US),
        .wdo_n        (wdo_n),
        .enout        (enout),
        .fault_active (fault_active),
        .en_effective (en_effective)
    );

    // =========================================================================
    // Helper tasks
    // =========================================================================

    // Wait N milliseconds (simulation time)
    task wait_ms(input integer n);
        begin
            #(n * 1_000_000);  // 1 ms = 1,000,000 ns
        end
    endtask

    // Wait N microseconds
    task wait_us(input integer n);
        begin
            #(n * 1_000);
        end
    endtask

    // Press button (active-low): press = drive low, hold, release = drive high
    task press_button_wdi;
        begin
            btn_wdi_n = 1'b0;       // press (active-low)
            wait_ms(2);              // hold > debounce time
            btn_wdi_n = 1'b1;       // release
            wait_ms(2);              // wait for debounce to settle
        end
    endtask

    // Enable watchdog: press and HOLD S2 (toggle-style: hold = enabled)
    task enable_watchdog;
        begin
            btn_en_n = 1'b0;        // press EN button (active-low → en=1)
            wait_ms(2);              // wait for debounce
        end
    endtask

    // Disable watchdog: release S2
    task disable_watchdog;
        begin
            btn_en_n = 1'b1;        // release EN button
            wait_ms(2);             // wait for debounce
        end
    endtask

    // Send single-cycle UART kick
    task send_uart_kick;
        begin
            @(posedge clk);
            uart_kick = 1'b1;
            @(posedge clk);
            uart_kick = 1'b0;
        end
    endtask

    // Send single-cycle CLR_FAULT
    task send_clr_fault;
        begin
            @(posedge clk);
            clr_fault = 1'b1;
            @(posedge clk);
            clr_fault = 1'b0;
        end
    endtask

    // =========================================================================
    // Test stimulus
    // =========================================================================
    integer test_num;

    initial begin
        $dumpfile("tb_watchdog_core.vcd");
        $dumpvars(0, tb_watchdog_core);

        // --- Initialize ---
        rst_n     = 1'b0;
        btn_wdi_n = 1'b1;  // not pressed
        btn_en_n  = 1'b1;  // not pressed (disabled)
        uart_kick = 1'b0;
        clr_fault = 1'b0;

        // --- Reset ---
        #200;
        rst_n = 1'b1;
        #200;

        // =================================================================
        // TEST 1: After reset — watchdog disabled, WDO released
        // =================================================================
        test_num = 1;
        $display("=== TEST %0d: Reset state ===", test_num);
        assert_check("WDO released",    wdo_n,        1'b1);
        assert_check("ENOUT low",       enout,        1'b0);
        assert_check("No fault",        fault_active, 1'b0);
        assert_check("Not enabled",     en_effective, 1'b0);
        $display("TEST %0d PASSED", test_num);
        #1000;

        // =================================================================
        // TEST 2: Disable → Enable (arm_delay verification)
        // =================================================================
        test_num = 2;
        $display("\n=== TEST %0d: Enable with arm_delay ===", test_num);
        enable_watchdog;  // waits 2ms for debounce

        // During arm delay (5ms): ENOUT should still be 0
        // Already ~2ms into arm_delay after debounce settled
        wait_ms(1);  // ~3ms total, still within 5ms arm_delay
        assert_check("ENOUT low during arming", enout, 1'b0);
        assert_check("en_effective high",       en_effective, 1'b1);

        // Wait for remaining arm delay to complete (~2ms + margin)
        wait_ms(3);
        wait_us(500);  // extra margin

        assert_check("ENOUT high after arm_delay", enout, 1'b1);
        assert_check("WDO still released",         wdo_n, 1'b1);
        $display("TEST %0d PASSED", test_num);

        // =================================================================
        // TEST 3: Normal kick — WDI button press resets timer
        // =================================================================
        test_num = 3;
        $display("\n=== TEST %0d: Normal kick resets watchdog ===", test_num);

        // Wait 5 ms (less than tWD=10ms), then kick
        wait_ms(5);
        assert_check("No timeout yet", wdo_n, 1'b1);

        press_button_wdi;  // kick via button (resets wd_cnt)

        // Wait 5 ms — should NOT timeout (timer was reset by kick)
        wait_ms(5);
        assert_check("No timeout after kick", wdo_n, 1'b1);

        // Kick again via UART
        send_uart_kick;
        wait_ms(5);
        assert_check("No timeout after UART kick", wdo_n, 1'b1);

        $display("TEST %0d PASSED", test_num);

        // =================================================================
        // TEST 4: Timeout — no kick within tWD → WDO asserted for tRST
        // =================================================================
        test_num = 4;
        $display("\n=== TEST %0d: Timeout -> FAULT ===", test_num);

        // Last UART kick was ~5ms ago. Wait for tWD=10ms to expire
        // Need ~6ms more + margin
        wait_ms(6);
        wait_us(500);

        assert_check("WDO asserted (fault)", wdo_n,        1'b0);
        assert_check("fault_active high",    fault_active, 1'b1);
        assert_check("ENOUT still high",     enout,        1'b1);

        // Wait for tRST to expire (3 ms)
        wait_ms(4);

        assert_check("WDO released after tRST", wdo_n,        1'b1);
        assert_check("fault cleared",           fault_active, 1'b0);

        $display("TEST %0d PASSED", test_num);

        // =================================================================
        // TEST 5: Disable while active
        // =================================================================
        test_num = 5;
        $display("\n=== TEST %0d: Disable watchdog ===", test_num);

        // Kick to be in clean ACTIVE state
        send_uart_kick;
        wait_us(100);

        disable_watchdog;

        assert_check("WDO released on disable", wdo_n,        1'b1);
        assert_check("ENOUT low on disable",    enout,        1'b0);
        assert_check("en_effective low",         en_effective, 1'b0);

        // Verify WDI kicks are ignored while disabled
        wait_ms(15);  // way past tWD (10ms)
        assert_check("No timeout while disabled", wdo_n, 1'b1);

        $display("TEST %0d PASSED", test_num);

        // =================================================================
        // TEST 6 (Bonus): CLR_FAULT — clear fault immediately
        // =================================================================
        test_num = 6;
        $display("\n=== TEST %0d: CLR_FAULT ===", test_num);

        // Re-enable watchdog
        enable_watchdog;           // 2ms debounce, en goes high ~1ms in
        wait_ms(6);                // wait past arm_delay (5ms from en high)

        // Now in ACTIVE state, wd_cnt started ~0ms ago
        // Wait for tWD=10ms + small margin to enter FAULT
        wait_ms(10);
        wait_us(500);

        assert_check("In fault state", fault_active, 1'b1);
        assert_check("WDO asserted",   wdo_n,        1'b0);

        // Clear fault immediately
        send_clr_fault;
        @(posedge clk); @(posedge clk); @(posedge clk);

        assert_check("Fault cleared by CLR_FAULT", fault_active, 1'b0);
        assert_check("WDO released by CLR_FAULT",  wdo_n,        1'b1);

        $display("TEST %0d PASSED", test_num);

        // =================================================================
        // Done
        // =================================================================
        #10000;
        $display("\n========================================");
        $display("  ALL TESTS PASSED");
        $display("========================================");
        $finish;
    end

    // =========================================================================
    // Assertion helper
    // =========================================================================
    task assert_check(
        input [255:0] msg,  // message (packed string)
        input actual,
        input expected
    );
        begin
            if (actual !== expected) begin
                $display("  FAIL: %0s — expected %b, got %b at time %0t",
                         msg, expected, actual, $time);
                $display("  TEST FAILED — stopping simulation");
                $finish;
            end else begin
                $display("  OK:   %0s = %b", msg, actual);
            end
        end
    endtask

    // =========================================================================
    // Timeout watchdog (prevent infinite sim)
    // =========================================================================
    initial begin
        #500_000_000;  // 500 ms max
        $display("ERROR: Simulation timeout!");
        $finish;
    end

endmodule
