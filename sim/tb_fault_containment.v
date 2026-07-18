// =============================================================================
// tb_fault_containment.v  —  PHASE 3a: containment measurement (broadcast fabric)
//
// Claim under test (Pillar 1): in a broadcast / output-stationary fabric, a single
// faulty PE corrupts exactly ONE output element — it cannot propagate to other PEs.
// (Contrast: systolic arrays produce "line errors" — Libano et al., IEEE TC 2023,
//  report ~70% of single-PE faults corrupt a whole row/column.)
//
// Method:
//   1. Run a fixed 4x4 matmul on the bare sage16_4x4_mac fabric, NO fault.
//      Capture all 16 outputs = GOLDEN.
//   2. For each PE k = 0..15:
//        re-run the identical matmul with fault_en[k]=1 (PE k corrupted).
//        count how many of the 16 outputs differ from GOLDEN.
//   3. Containment holds iff every fault corrupts exactly 1 output.
//
// Output: per-PE wrong-count + summary (max / mean). Expected: all 1.
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module tb_fault_containment;

    localparam DATA_W  = 16;
    localparam ACC_W   = 32;
    localparam CFG_W   = 10;
    localparam NUM_PE  = 16;
    localparam SRAM_AW = 8;
    localparam SRAM_DW = 32;

    localparam [3:0] OP_MACB = 4'd9;          // unsigned broadcast MAC (matmul)
    localparam [ACC_W-1:0] FAULT_MASK = 32'h0000_0001; // flip LSB of faulted PE

    reg clk = 0; always #5 clk = ~clk;
    reg rst_n;

    // fabric control
    reg                       cfg_broadcast, clr_acc_all, out_en_all;
    reg  [CFG_W-1:0]          cfg_data;
    reg  [4*DATA_W-1:0]       ext_in_west, ext_in_north;
    reg  [NUM_PE-1:0]         fault_en_flat;
    reg  [ACC_W-1:0]          fault_xor;
    wire [NUM_PE*ACC_W-1:0]   all_pe_out;

    sage16_4x4_mac #(
        .DATA_W(DATA_W), .ACC_W(ACC_W), .CFG_W(CFG_W), .PIPELINE(1)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .cfg_pe_row(2'd0), .cfg_pe_col(2'd0),
        .cfg_data(cfg_data), .cfg_load(1'b0),
        .cfg_broadcast(cfg_broadcast),
        .clr_acc_all(clr_acc_all), .out_en_all(out_en_all),
        .ext_in_west(ext_in_west), .ext_in_north(ext_in_north),
        .per_pe_bypass_en(1'b0), .per_pe_bypass_flat(256'b0),
        .sram_cs_n_flat({NUM_PE{1'b1}}), .sram_we_n_flat({NUM_PE{1'b1}}),
        .sram_addr_flat(128'b0), .sram_raddr2_flat(128'b0), .sram_wdata_sel(16'b0),
        .sram_wdata_ext_flat(512'b0),
        .sel_src_a_flat(16'b0), .sel_src_b_flat(16'b0),
        // PE faults injected from the TB (force/release) - no DUT hook
        // (rail-fault hook removed from the fabric)
        .sram_rdata_flat(),
        .ext_out_east(),
        .all_pe_out(all_pe_out)
    );

    // ---- fixed test matrices A (row-major) and B (row-major), 4x4 int ----
    // A[i][k], B[k][j]; result C[i][j] lands in PE (i,j) = output i*4+j.
    reg [DATA_W-1:0] A [0:15];
    reg [DATA_W-1:0] B [0:15];
    integer i, k, j, t;

    function [DATA_W-1:0] ae; input [1:0] ii, kk; ae = A[ii*4+kk]; endfunction
    function [DATA_W-1:0] be; input [1:0] kk, jj; be = B[kk*4+jj]; endfunction

    // ---- run one matmul; returns nothing, leaves result in all_pe_out ----
    task run_matmul;
        begin
            // reset accumulators + config
            @(negedge clk);
            cfg_broadcast = 1; clr_acc_all = 1; cfg_data = {OP_MACB,3'd0,3'd0};
            ext_in_west = 0; ext_in_north = 0; out_en_all = 0;
            @(negedge clk);
            cfg_broadcast = 0; clr_acc_all = 0; out_en_all = 1;
            // 4 inner-product taps: broadcast A[:,k] on west, B[k,:] on north
            for (t = 0; t < 4; t = t+1) begin
                ext_in_west[0*DATA_W +: DATA_W] = ae(2'd0, t[1:0]);
                ext_in_west[1*DATA_W +: DATA_W] = ae(2'd1, t[1:0]);
                ext_in_west[2*DATA_W +: DATA_W] = ae(2'd2, t[1:0]);
                ext_in_west[3*DATA_W +: DATA_W] = ae(2'd3, t[1:0]);
                ext_in_north[0*DATA_W +: DATA_W] = be(t[1:0], 2'd0);
                ext_in_north[1*DATA_W +: DATA_W] = be(t[1:0], 2'd1);
                ext_in_north[2*DATA_W +: DATA_W] = be(t[1:0], 2'd2);
                ext_in_north[3*DATA_W +: DATA_W] = be(t[1:0], 2'd3);
                @(negedge clk);
            end
            // drive zeros + extra cycles to drain DSP pipeline (PIPELINE=1)
            ext_in_west = 0; ext_in_north = 0;
            @(negedge clk); @(negedge clk); @(negedge clk);
            out_en_all = 0;
            @(negedge clk);
        end
    endtask

    // ---- storage for golden + per-run capture ----
    reg [ACC_W-1:0] golden [0:15];
    integer wrong, max_wrong, sum_wrong, faulty_idx_seen;

    initial begin
        // init matrices: A = i*4+k+1 ; B = (k*4+j)+1  (small, nonzero, distinct)
        for (i = 0; i < 16; i = i+1) begin
            A[i] = i + 1;
            B[i] = (i % 7) + 1;       // keep small to avoid overflow noise
        end

        rst_n = 0; fault_en_flat = 0; fault_xor = FAULT_MASK;
        cfg_broadcast=0; clr_acc_all=0; out_en_all=0; cfg_data=0;
        ext_in_west=0; ext_in_north=0;
        repeat (3) @(negedge clk); rst_n = 1;

        // -------- golden run (no fault) --------
        fault_en_flat = 0;
        run_matmul;
        for (i = 0; i < 16; i = i+1) golden[i] = all_pe_out[i*ACC_W +: ACC_W];

        $display("===========================================================");
        $display(" CONTAINMENT TEST — broadcast fabric, single-PE fault");
        $display(" fault model: faulted PE output XOR 0x%08x", FAULT_MASK);
        $display("===========================================================");
        $display(" golden C (PE outputs):");
        for (i = 0; i < 4; i = i+1)
            $display("   [%0d..%0d] = %0d %0d %0d %0d", i*4, i*4+3,
                     golden[i*4+0], golden[i*4+1], golden[i*4+2], golden[i*4+3]);
        $display("-----------------------------------------------------------");

        // -------- fault sweep: one PE faulty at a time --------
        max_wrong = 0; sum_wrong = 0;
        for (k = 0; k < 16; k = k+1) begin
            fault_en_flat = 0;
            fault_en_flat[k] = 1'b1;
            run_matmul;
            wrong = 0; faulty_idx_seen = -1;
            for (i = 0; i < 16; i = i+1) begin
                if (all_pe_out[i*ACC_W +: ACC_W] !== golden[i]) begin
                    wrong = wrong + 1;
                    faulty_idx_seen = i;
                end
            end
            $display(" fault PE %2d -> %0d output(s) wrong %s",
                     k, wrong,
                     (wrong==1 && faulty_idx_seen==k) ? "(only its own - CONTAINED)" :
                     (wrong==0) ? "(masked)" : "(SPREAD!)");
            if (wrong > max_wrong) max_wrong = wrong;
            sum_wrong = sum_wrong + wrong;
        end
        fault_en_flat = 0;

        $display("-----------------------------------------------------------");
        $display(" SUMMARY: max wrong outputs from a single PE fault = %0d", max_wrong);
        $display("          mean wrong outputs over 16 faults        = %0d.%02d",
                 sum_wrong/16, (sum_wrong%16)*100/16);
        if (max_wrong <= 1)
            $display(" RESULT: CONTAINED — every single-PE fault stays in 1 output.");
        else
            $display(" RESULT: NOT contained — a fault spread to %0d outputs.", max_wrong);
        $display("===========================================================");
        #20 $finish;
    end

    initial begin #200000; $display("TIMEOUT"); $finish; end
    // ---- TB-SIDE permanent PE-fault injector (DUT datapath is clean) ----
    // Replaces the removed in-PE hook: force/release each PE accumulator,
    // corrupting only on an accumulating cycle (out_en & !clr) so the value
    // sequence is bit-identical to the old `out <= alu_out ^ fault_xor`.
    genvar fipe;
    generate for (fipe=0; fipe<16; fipe=fipe+1) begin : FINJ
        reg [ACC_W-1:0] fi_clean;
        always @(posedge clk) begin
            release dut.rg[fipe/4].cg[fipe%4].u_pe.out;
            if (fault_en_flat[fipe] && out_en_all && !clr_acc_all) begin
                #1 fi_clean = dut.rg[fipe/4].cg[fipe%4].u_pe.out;
                force dut.rg[fipe/4].cg[fipe%4].u_pe.out = fi_clean ^ fault_xor;
            end
        end
    end endgenerate

endmodule

`default_nettype wire
