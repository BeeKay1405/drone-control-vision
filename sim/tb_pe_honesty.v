// =============================================================================
// tb_pe_honesty.v  —  "the PE must be honest about its own faults"
//
// Unit test of the REAL pe.v: injects a permanent PE fault and proves the PE's
// built-in mod-3 residue self-check (mac_err) flags it. The fault injector is
// NOT in the silicon: pe.v has a clean datapath (out <= alu_out). The permanent
// fault is modelled the verification-correct way — force/release on the PE's own
// accumulator register from this testbench:
//
//     posedge: release out  -> PE latches its clean next value
//     +1ns   : sample it, then force out = clean ^ fi_xor  (re-corrupt)
//
// This reproduces exactly the old in-datapath model (out <= alu_out ^ fault_xor,
// corruption feeding forward through in_self) without any hook in the DUT.
//
// Honesty contract proven here:
//   (1) NO FALSE ALARM  : a healthy PE streaming clean MACBs never raises mac_err.
//   (2) CATCHES ITS OWN  : every single-bit corruption of the accumulator is
//                          flagged (2^p mod 3 in {1,2}, never 0 -> always changes
//                          the residue), regardless of operands.
//   (3) PERMANENT = STICKY: a permanent fault keeps flagging every cycle it is
//                          used (not a one-shot), matching the permanent-fault scope.
//
// Multi-bit fast-path coverage (the ~66% mod-3 / checksum-backstop story) is
// characterised separately in tb_residue_coverage.v / tb_fault_campaign.v.
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module tb_pe_honesty;
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

    // Output-stationary fabric: the PE's accumulate addend IS its own register.
    wire [ACC_W-1:0] in_self = out;

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

    // ---- TB-SIDE permanent-fault injector (DUT datapath is clean) ----
    reg              fi_en;
    reg [ACC_W-1:0]  fi_xor;
    reg [ACC_W-1:0]  fi_clean;
    always @(posedge clk) begin
        release dut.out;                          // free the register every cycle
        if (fi_en) begin
            #1 fi_clean = dut.out;                // sample the clean latched value
            force dut.out = fi_clean ^ fi_xor;    // re-corrupt -> permanent fault
        end
    end

    integer pass = 0, fail = 0;

    // sticky error monitor (posedge so it never races the negedge control flow)
    reg watch, err_seen; integer err_cycles;
    always @(posedge clk) if (watch && mac_err) begin
        err_seen   <= 1'b1;
        err_cycles <= err_cycles + 1;
    end

    // drive one clean MACB operand pair + its carried-in residues
    task drive; input [DATA_W-1:0] a, b; begin
        in_bypass = a; in_b_col = b;
        res_a_in  = a % 3; res_b_in = b % 3;
    end endtask

    task cfg_macb; begin
        @(negedge clk); cfg_in = {OP_MACB, 3'd0, 3'd0}; cfg_load = 1;
        @(negedge clk); cfg_load = 0;
    end endtask

    task resync; begin           // clear accumulator, start fresh
        @(negedge clk); clr_acc = 1; out_en = 1;
        @(negedge clk); clr_acc = 0;
    end endtask

    integer i;
    reg [ACC_W-1:0] pat [0:5];

    initial begin
        // single-bit corruptions across the 32-bit accumulator: LSB, bit1,
        // bit8, bit15, bit16, MSB. Each is guaranteed to change mod-3.
        pat[0]=32'h0000_0001; pat[1]=32'h0000_0002; pat[2]=32'h0000_0100;
        pat[3]=32'h0000_8000; pat[4]=32'h0001_0000; pat[5]=32'h8000_0000;

        rst_n=0; cfg_load=0; cfg_in=0; out_en=0; clr_acc=0;
        in_bypass=0; in_b_col=0; res_a_in=0; res_b_in=0;
        fi_en=0; fi_xor=0; watch=0; err_seen=0; err_cycles=0;
        repeat (3) @(negedge clk); rst_n = 1;

        $display("===========================================================");
        $display(" PE HONESTY TEST — does the PE flag its OWN injected fault?");
        $display(" (fault injected from the TESTBENCH; pe.v datapath is clean)");
        $display("===========================================================");

        cfg_macb;
        resync;
        drive(16'd7, 16'd5);     // clean operands: 35 / cycle, never overflows here

        // ---------- (1) NO FALSE ALARM ----------
        out_en=1; fi_en=0;
        repeat (5) @(negedge clk);          // let the pipeline fill
        err_seen=0; err_cycles=0; watch=1;
        repeat (16) @(negedge clk);         // 16 clean accumulates
        watch=0;
        if (!err_seen) begin
            $display(" (1) clean stream, 16 cycles : mac_err never fired -> OK");
            pass=pass+1;
        end else begin
            $display(" (1) clean stream            : FALSE ALARM -> FAIL");
            fail=fail+1;
        end

        // ---------- (2) CATCHES EACH SINGLE-BIT FAULT ----------
        for (i=0; i<6; i=i+1) begin
            resync; drive(16'd7, 16'd5);
            out_en=1; fi_en=0;
            repeat (4) @(negedge clk);      // refill clean
            err_seen=0; err_cycles=0; watch=1;
            fi_en=1; fi_xor=pat[i];         // permanent fault asserted now
            repeat (8) @(negedge clk);
            watch=0; fi_en=0;
            if (err_seen) begin
                $display(" (2) fi_xor=%08h           : flagged (%0d cyc) -> OK",
                         pat[i], err_cycles);
                pass=pass+1;
            end else begin
                $display(" (2) fi_xor=%08h           : MISSED -> FAIL", pat[i]);
                fail=fail+1;
            end
        end

        // ---------- (3) PERMANENT FAULT IS STICKY (flags repeatedly) ----------
        resync; drive(16'd7, 16'd5);
        out_en=1; fi_en=0;
        repeat (4) @(negedge clk);
        err_seen=0; err_cycles=0; watch=1;
        fi_en=1; fi_xor=32'h0000_0001;
        repeat (10) @(negedge clk);
        watch=0; fi_en=0;
        if (err_cycles >= 5) begin
            $display(" (3) permanent fault, 10 cyc : flagged %0d cycles -> OK", err_cycles);
            pass=pass+1;
        end else begin
            $display(" (3) permanent fault         : only %0d cycles (expected >=5) -> FAIL",
                     err_cycles);
            fail=fail+1;
        end

        $display("-----------------------------------------------------------");
        $display(" SUMMARY: pass=%0d fail=%0d / %0d", pass, fail, pass+fail);
        if (fail==0) $display(" RESULT: PASS — the PE is honest: never lies, always owns its fault.");
        else         $display(" RESULT: FAIL");
        $display("===========================================================");
        $finish;
    end

    initial begin #100000; $display("TIMEOUT"); $finish; end
endmodule

`default_nettype wire
