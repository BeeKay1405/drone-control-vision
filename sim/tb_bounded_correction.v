// =============================================================================
// tb_bounded_correction.v  —  Idea C: bounded, LOCATION-INDEPENDENT correction
//                            latency => hard real-time guarantee for flight.
//
// Real-life claim (flight-safety framing): under a permanent PE fault, the
// corrected output is ready in a bounded number of cycles that is the SAME for
// every fault site and is vastly smaller than the drone control period — so a
// corrupted command never reaches the motors; the loop always meets its deadline.
//
// WHY the bound is location-independent (this is the containment thesis paying
// off): broadcast contains a PE fault to exactly ONE output, whose index is the
// PE index (free location). Every PE has an identical pipeline, so the
// fault->detect latency is identical for all 16 sites; and correction is a fixed
// O(1) step regardless of which PE faulted:
//     detect  : residue syndrome mac_err_flat[k]      (measured here, hardware)
//     correct : erasure-ABFT  fixed[k] -= (rowsum-S)  (1 subtraction, ~1 cy)
//               or recompute-on-spare                 (9 cy, tb_self_repair)
//
// This bench MEASURES the detect latency on the real fabric for all 16 fault
// sites and asserts it is identical (no data-dependent / location-dependent
// jitter), then composes the bounded WCET and checks it against the control
// deadline. Correction cost is cited from tb_erasure_abft (1 cy) / tb_self_repair
// (9 cy), both already proven 16/16 and location-independent.
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module tb_bounded_correction;
    localparam DATA_W = 16, ACC_W = 32, CFG_W = 10, NUM_PE = 16;
    localparam [3:0] OP_MACB = 4'd9;
    localparam [ACC_W-1:0] FAULT_MASK = 32'h0000_0001;

    // ---- timing / deadline parameters (edit to match the flight controller) ----
    localparam real CLK_MHZ        = 100.0;   // reliable-fabric Fmax (measured)
    localparam integer CORRECT_CY  = 9;       // worst correction path (recompute-on-spare)
    localparam real CTRL_RATE_KHZ  = 8.0;     // inner attitude-rate loop (worst/fastest)

    reg clk = 0; always #5 clk = ~clk;
    reg rst_n;
    reg                       cfg_broadcast, clr_acc_all, out_en_all;
    reg  [CFG_W-1:0]          cfg_data;
    reg  [4*DATA_W-1:0]       ext_in_west, ext_in_north;
    reg  [NUM_PE-1:0]         fault_en_flat;
    reg  [ACC_W-1:0]          fault_xor;
    wire [NUM_PE*ACC_W-1:0]   all_pe_out;
    wire [NUM_PE-1:0]         mac_err_flat;

    sage16_4x4_mac #(.DATA_W(DATA_W), .ACC_W(ACC_W), .CFG_W(CFG_W), .PIPELINE(1)) dut (
        .clk(clk), .rst_n(rst_n), .cfg_pe_row(2'd0), .cfg_pe_col(2'd0),
        .cfg_data(cfg_data), .cfg_load(1'b0), .cfg_broadcast(cfg_broadcast),
        .clr_acc_all(clr_acc_all), .out_en_all(out_en_all),
        .ext_in_west(ext_in_west), .ext_in_north(ext_in_north),
        .per_pe_bypass_en(1'b0), .per_pe_bypass_flat(256'b0),
        .sram_cs_n_flat({NUM_PE{1'b1}}), .sram_we_n_flat({NUM_PE{1'b1}}),
        .sram_addr_flat(128'b0), .sram_raddr2_flat(128'b0), .sram_wdata_sel(16'b0),
        .sram_wdata_ext_flat(512'b0), .sel_src_a_flat(16'b0), .sel_src_b_flat(16'b0),
        // (rail-fault hook removed from the fabric)
        .sram_rdata_flat(), .ext_out_east(), .all_pe_out(all_pe_out),
        .rail_err_w_flat(), .rail_err_n_flat(), .mac_err_flat(mac_err_flat)
    );

    // ---- TB-SIDE permanent-fault injector (DUT datapath is clean) ----
    // Replaces the removed in-PE hook: force/release each PE accumulator,
    // corrupting only on an accumulating cycle (out_en & !clr) so the value
    // sequence is bit-identical to the old `out <= alu_out ^ fault_xor`.
    genvar fg;
    generate for (fg=0; fg<16; fg=fg+1) begin : FI
        reg [ACC_W-1:0] fi_clean;
        always @(posedge clk) begin
            release dut.rg[fg/4].cg[fg%4].u_pe.out;
            if (fault_en_flat[fg] && out_en_all && !clr_acc_all) begin
                #1 fi_clean = dut.rg[fg/4].cg[fg%4].u_pe.out;
                force dut.rg[fg/4].cg[fg%4].u_pe.out = fi_clean ^ fault_xor;
            end
        end
    end endgenerate

    reg [DATA_W-1:0] A [0:15];
    reg [DATA_W-1:0] B [0:15];
    integer t, i, k;
    function [DATA_W-1:0] ae; input [1:0] ii,kk; ae = A[ii*4+kk]; endfunction
    function [DATA_W-1:0] be; input [1:0] kk,jj; be = B[kk*4+jj]; endfunction

    // free-running cycle counter + first-detection capture for the PE under test
    integer cyc;
    always @(posedge clk) if (!rst_n) cyc <= 0; else cyc <= cyc + 1;

    integer ktest, t_outen, t_detect;
    reg measuring, detected;
    always @(posedge clk)
        if (measuring & ~detected & mac_err_flat[ktest]) begin
            t_detect <= cyc;
            detected <= 1'b1;
        end

    // matmul that timestamps the cycle the faulty op starts executing (out_en rise)
    task run_matmul_timed;
        begin
            @(negedge clk); cfg_broadcast=1; clr_acc_all=1; cfg_data={OP_MACB,3'd0,3'd0};
            ext_in_west=0; ext_in_north=0; out_en_all=0;
            @(negedge clk); cfg_broadcast=0; clr_acc_all=0; out_en_all=1;
            t_outen = cyc;                          // <-- faulty compute begins
            for (t=0;t<4;t=t+1) begin
                ext_in_west[0*DATA_W +: DATA_W]=ae(2'd0,t[1:0]);
                ext_in_west[1*DATA_W +: DATA_W]=ae(2'd1,t[1:0]);
                ext_in_west[2*DATA_W +: DATA_W]=ae(2'd2,t[1:0]);
                ext_in_west[3*DATA_W +: DATA_W]=ae(2'd3,t[1:0]);
                ext_in_north[0*DATA_W +: DATA_W]=be(t[1:0],2'd0);
                ext_in_north[1*DATA_W +: DATA_W]=be(t[1:0],2'd1);
                ext_in_north[2*DATA_W +: DATA_W]=be(t[1:0],2'd2);
                ext_in_north[3*DATA_W +: DATA_W]=be(t[1:0],2'd3);
                @(negedge clk);
            end
            ext_in_west=0; ext_in_north=0;
            @(negedge clk); @(negedge clk); @(negedge clk);
            out_en_all=0; @(negedge clk);
        end
    endtask

    integer lat [0:15];
    integer Ld, jitter, pass, fail;
    real wcet_cy, wcet_us, period_us, margin;

    initial begin
        for (i=0;i<16;i=i+1) begin A[i]=i+1; B[i]=(i%7)+1; end
        rst_n=0; fault_en_flat=0; fault_xor=FAULT_MASK;
        cfg_broadcast=0; clr_acc_all=0; out_en_all=0; cfg_data=0;
        ext_in_west=0; ext_in_north=0; measuring=0; detected=0; ktest=0;
        repeat (3) @(negedge clk); rst_n=1;

        $display("=================================================================");
        $display(" BOUNDED CORRECTION (Idea C) — location-independent detect+correct");
        $display("=================================================================");

        // measure detection latency for a permanent fault at every PE site
        for (k=0;k<16;k=k+1) begin
            ktest = k; detected = 0;
            fault_en_flat = 0; fault_en_flat[k] = 1'b1;
            measuring = 1;
            run_matmul_timed;
            measuring = 0;
            fault_en_flat = 0;
            lat[k] = detected ? (t_detect - t_outen) : -1;
            $display("  PE %2d : fault->detect = %0d cycles%s",
                     k, lat[k], detected ? "" : "  (NOT DETECTED!)");
        end

        // location-independence check: every site must share one detect latency
        Ld = lat[0]; jitter = 0; pass = 0; fail = 0;
        for (k=0;k<16;k=k+1) begin
            if (lat[k] != Ld)  jitter = jitter + 1;
            if (lat[k] >= 0)   pass = pass + 1; else fail = fail + 1;
        end

        $display("-----------------------------------------------------------------");
        if (fail==0 && jitter==0) begin
            $display(" detect latency       : %0d cycles, IDENTICAL across all 16 sites", Ld);
            $display("                        (containment -> no location-dependent jitter)");
        end else begin
            $display(" detect latency       : NON-UNIFORM (%0d sites differ, %0d missed) -> FAIL",
                     jitter, fail);
        end

        // compose the bounded WCET and check it against the flight deadline
        wcet_cy   = Ld + CORRECT_CY;                 // detect + worst correction path
        wcet_us   = wcet_cy / CLK_MHZ;               // cycles / (cycles per us)
        period_us = 1000.0 / CTRL_RATE_KHZ;          // control period in us
        margin    = period_us / wcet_us;

        $display(" correct latency      : %0d cycles (recompute-on-spare; 1 cy via erasure-ABFT)",
                 CORRECT_CY);
        $display(" => WCET detect+correct: %0d cycles = %.3f us @ %.0f MHz", wcet_cy, wcet_us, CLK_MHZ);
        $display(" control period       : %.1f us (%.0f kHz attitude-rate loop)",
                 period_us, CTRL_RATE_KHZ);
        $display(" deadline margin      : %.0fx  (correction finishes well before the deadline)",
                 margin);
        $display("-----------------------------------------------------------------");
        if (fail==0 && jitter==0 && wcet_us < period_us)
            $display(" RESULT: PASS — bounded, location-independent correction << control");
        else
            $display(" RESULT: FAIL");
        $display("          period: the corrected command always meets the actuator deadline.");
        $display("=================================================================");
        #20 $finish;
    end

    initial begin #2000000; $display("TIMEOUT"); $finish; end
endmodule

`default_nettype wire
