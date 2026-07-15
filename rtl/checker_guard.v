// =============================================================================
// checker_guard.v  —  "who watches the watchman" online self-test supervisor
//
// The residue self-check (mac_err) is the safety mechanism. If its comparator
// or syndrome bit suffers a permanent stuck-at-0, a REAL fault would pass
// silently — the alarm is dead and nobody knows. In a safety-critical drone
// that is the worst failure mode: undetected wrong actuator command.
//
// This guard closes that hole with a periodic functional self-test that costs
// ZERO change to the PE. Because operand residues are CARRIED IN (res_a_in),
// the sequencer injects a deliberately WRONG residue for one known op during a
// "self-test window" (selftest_active=1). A LIVE checker MUST raise mac_err in
// response. The guard watches the window:
//     - mac_err pulsed during the window  -> checker is alive    (no action)
//     - window closed with NO mac_err      -> checker is DEAD     -> checker_fault
//
// Cost: one capture flop + one verdict flop per monitored checker (or one shared
// guard time-multiplexed across PEs). The dangerous mode (alarm stuck silent) is
// covered; stuck-at-1 is self-evident (it false-alarms continuously).
//
// Lineage: totally-self-checking checkers (Carter/Schneider, Anderson-Metze,
// 1970s). Delta here: the test stimulus rides the existing carried-residue path,
// so the watchman is checked with no added datapath — same "test epoch" idea the
// aging monitor reuses.
// =============================================================================
`default_nettype none

module checker_guard (
    input  wire clk,
    input  wire rst_n,
    input  wire selftest_active,  // high while a known-detectable fault is injected
    input  wire mac_err,          // the checker output under observation
    output reg  checker_fault     // sticky: set if a self-test failed to raise mac_err
);
    reg saw_err;   // did mac_err fire anywhere inside the current window?
    reg st_d;      // selftest_active delayed one cycle (to find the window edges)

    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            saw_err       <= 1'b0;
            st_d          <= 1'b0;
            checker_fault <= 1'b0;
        end else begin
            st_d <= selftest_active;

            if (selftest_active & ~st_d)            // window opens: arm
                saw_err <= 1'b0;
            else if (selftest_active & mac_err)     // inside window: capture the pulse
                saw_err <= 1'b1;

            if (st_d & ~selftest_active & ~saw_err) // window closed with no pulse: dead
                checker_fault <= 1'b1;
        end
endmodule

`default_nettype wire
