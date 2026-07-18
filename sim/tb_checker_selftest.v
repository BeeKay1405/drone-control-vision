// =============================================================================
// tb_checker_selftest.v  —  proves the guard catches a DEAD checker
//
// "Harden the checker": the residue self-check is the safety net, but what if
// the net itself breaks? This bench drives the REAL pe.v plus checker_guard and
// shows the online self-test (inject a wrong CARRIED residue, expect mac_err):
//
//   Phase 1  healthy checker : self-test fires mac_err  -> checker_fault stays 0
//   Phase 2  DEAD checker     : mac_err stuck-at-0       -> guard raises checker_fault
//
// A dead checker is modelled the way it fails on silicon: mac_err masked to 0
// (mac_err_obs). The injection rides the existing carried-residue path, so the
// PE itself is untouched.
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module tb_checker_selftest;
    localparam DATA_W = 16, ACC_W = 32, CFG_W = 10;
    localparam [3:0] OP_MACB = 4'd9;

    reg clk = 0; always #5 clk = ~clk;
    reg rst_n;
    reg cfg_load; reg [CFG_W-1:0] cfg_in;
    reg out_en, clr_acc;
    reg [DATA_W-1:0] in_bypass, in_b_col;
    reg [1:0] res_a_in, res_b_in;

    wire [ACC_W-1:0]  out;
    wire [DATA_W-1:0] out_mesh;
    wire              mac_err;
    wire [ACC_W-1:0]  in_self = out;

    pe #(.DATA_W(DATA_W), .ACC_W(ACC_W), .CFG_W(CFG_W),
         .PIPELINE(1), .RESIDUE_MOD7(0)) dut (
        .clk(clk), .rst_n(rst_n),
        .cfg_load(cfg_load), .cfg_in(cfg_in),
        .out_en(out_en), .clr_acc(clr_acc),
        .in_north(16'd0), .in_south(16'd0), .in_east(16'd0), .in_west(16'd0),
        .in_self(in_self),
        .in_bypass(in_bypass), .in_b_col(in_b_col),
        .sram_rdata(32'd0), .sel_src_a(1'b0), .sel_src_b(1'b0),
        .res_a_in(res_a_in), .res_b_in(res_b_in), .sram_res(2'd0),
        .out_mesh(out_mesh), .out(out), .mac_err(mac_err)
    );

    // model a possibly-dead checker as the guard sees it (stuck-at-0 when killed)
    reg  kill_checker;
    wire mac_err_obs = mac_err & ~kill_checker;

    reg  selftest_active;
    wire checker_fault;
    checker_guard guard (
        .clk(clk), .rst_n(rst_n),
        .selftest_active(selftest_active),
        .mac_err(mac_err_obs),
        .checker_fault(checker_fault)
    );

    integer pass = 0, fail = 0;

    task cfg_macb; begin
        @(negedge clk); cfg_in = {OP_MACB, 3'd0, 3'd0}; cfg_load = 1;
        @(negedge clk); cfg_load = 0;
    end endtask
    task resync; begin
        @(negedge clk); clr_acc = 1; out_en = 1;
        @(negedge clk); clr_acc = 0;
    end endtask
    // clean carried residues for operands 7,5
    task drive_clean;  begin in_bypass=16'd7; in_b_col=16'd5; res_a_in=7%3; res_b_in=5%3; end endtask
    // POISONED residue: deliberately off by one -> a live checker must flag it
    task drive_poison; begin in_bypass=16'd7; in_b_col=16'd5; res_a_in=(7%3+1)%3; res_b_in=5%3; end endtask

    initial begin
        rst_n=0; cfg_load=0; cfg_in=0; out_en=0; clr_acc=0;
        in_bypass=0; in_b_col=0; res_a_in=0; res_b_in=0;
        selftest_active=0; kill_checker=0;
        repeat (3) @(negedge clk); rst_n = 1;

        $display("===========================================================");
        $display(" CHECKER SELF-TEST — who watches the watchman?");
        $display("===========================================================");

        cfg_macb; resync; drive_clean; out_en=1;
        repeat (6) @(negedge clk);          // pipeline fill, mac_err must be 0

        // ---------- Phase 1: HEALTHY checker passes the self-test ----------
        kill_checker = 0;
        selftest_active = 1; drive_poison;  // inject wrong carried residue
        repeat (8) @(negedge clk);
        selftest_active = 0; drive_clean;
        @(negedge clk);                     // let guard render its verdict
        @(negedge clk);
        if (!checker_fault) begin
            $display(" Phase 1 (alive)  : self-test raised mac_err, guard OK   -> OK");
            pass=pass+1;
        end else begin
            $display(" Phase 1 (alive)  : guard falsely cried checker_fault    -> FAIL");
            fail=fail+1;
        end

        // ---------- Phase 2: DEAD checker is caught ----------
        resync; drive_clean; out_en=1;
        repeat (4) @(negedge clk);
        kill_checker = 1;                   // checker comparator stuck-at-0
        selftest_active = 1; drive_poison;  // same known-bad injection
        repeat (8) @(negedge clk);
        selftest_active = 0; drive_clean;
        @(negedge clk);
        @(negedge clk);
        if (checker_fault) begin
            $display(" Phase 2 (dead)   : alarm silent, guard raised checker_fault -> OK");
            pass=pass+1;
        end else begin
            $display(" Phase 2 (dead)   : dead checker went UNNOTICED            -> FAIL");
            fail=fail+1;
        end

        $display("-----------------------------------------------------------");
        $display(" SUMMARY: pass=%0d fail=%0d / %0d", pass, fail, pass+fail);
        if (fail==0) $display(" RESULT: PASS — the watchman is watched; a dead checker self-reports.");
        else         $display(" RESULT: FAIL");
        $display("===========================================================");
        $finish;
    end

    initial begin #100000; $display("TIMEOUT"); $finish; end
endmodule

`default_nettype wire
