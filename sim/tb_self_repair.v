// =============================================================================
// tb_self_repair.v  —  PHASE 3c: containment-enabled cheap self-repair
//
// Builds on the containment result (Pillar 1). Because a faulty PE corrupts
// exactly ONE output AND we know which one for free (output index = PE index),
// repair is trivial: recompute that single output's inner product on a HEALTHY
// PE and substitute it.
//
//   For each faulty PE k = (i,j):
//     1. run full matmul with PE k faulted   -> faulted result (output k wrong).
//     2. REPAIR: recompute C[i][j] = sum_k A[i,k]*B[k,j] on a spare healthy PE p
//        (drive only that PE's row/col rails for 4 taps) -> corrected value.
//     3. substitute corrected value at index k -> repaired result.
//     4. check repaired == golden (the no-fault answer).
//   Report repair cost (cycles to fix one output).
//
// Cost story: broadcast repair = ONE inner product (N MACs) to fix 1 output.
// A systolic array must recompute the whole corrupted column (N outputs) or
// spend a spare row. So broadcast repair is ~1/N the recompute work.
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module tb_self_repair;
    localparam DATA_W  = 16;
    localparam ACC_W   = 32;
    localparam CFG_W   = 10;
    localparam NUM_PE  = 16;
    localparam [3:0] OP_MACB = 4'd9;
    localparam [ACC_W-1:0] FAULT_MASK = 32'h0000_0001;

    reg clk = 0; always #5 clk = ~clk;
    reg rst_n;
    reg                       cfg_broadcast, clr_acc_all, out_en_all;
    reg  [CFG_W-1:0]          cfg_data;
    reg  [4*DATA_W-1:0]       ext_in_west, ext_in_north;
    reg  [NUM_PE-1:0]         fault_en_flat;
    reg  [ACC_W-1:0]          fault_xor;
    wire [NUM_PE*ACC_W-1:0]   all_pe_out;

    sage16_4x4_mac #(.DATA_W(DATA_W), .ACC_W(ACC_W), .CFG_W(CFG_W), .PIPELINE(1)) dut (
        .clk(clk), .rst_n(rst_n), .cfg_pe_row(2'd0), .cfg_pe_col(2'd0),
        .cfg_data(cfg_data), .cfg_load(1'b0), .cfg_broadcast(cfg_broadcast),
        .clr_acc_all(clr_acc_all), .out_en_all(out_en_all),
        .ext_in_west(ext_in_west), .ext_in_north(ext_in_north),
        .per_pe_bypass_en(1'b0), .per_pe_bypass_flat(256'b0),
        .sram_cs_n_flat({NUM_PE{1'b1}}), .sram_we_n_flat({NUM_PE{1'b1}}),
        .sram_addr_flat(128'b0), .sram_raddr2_flat(128'b0), .sram_wdata_sel(16'b0),
        .sram_wdata_ext_flat(512'b0), .sel_src_a_flat(16'b0), .sel_src_b_flat(16'b0),
        // PE faults injected from the TB (force/release) — no DUT hook
        // (rail-fault hook removed from the fabric)
        .sram_rdata_flat(), .ext_out_east(), .all_pe_out(all_pe_out)
    );

    reg [DATA_W-1:0] A [0:15];
    reg [DATA_W-1:0] B [0:15];
    integer t, i, k, p, repair_cycles;
    function [DATA_W-1:0] ae; input [1:0] ii,kk; ae = A[ii*4+kk]; endfunction
    function [DATA_W-1:0] be; input [1:0] kk,jj; be = B[kk*4+jj]; endfunction

    // ---- full 4x4 matmul (all PEs) ----
    task run_matmul;
        begin
            @(negedge clk); cfg_broadcast=1; clr_acc_all=1; cfg_data={OP_MACB,3'd0,3'd0};
            ext_in_west=0; ext_in_north=0; out_en_all=0;
            @(negedge clk); cfg_broadcast=0; clr_acc_all=0; out_en_all=1;
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

    // ---- REPAIR: recompute C[i][j] on healthy spare PE p; return its output ----
    // drives only PE p's row (west[p_row]) and col (north[p_col]) rails.
    // counts cycles spent (repair cost for ONE output).
    reg [ACC_W-1:0] repaired_val;
    task repair_one;
        input [3:0] bad;     // faulty output index (i*4+j)
        input [3:0] spare;   // healthy PE to recompute on
        reg [1:0] bi, bj, pr, pc;
        begin
            bi = bad[3:2]; bj = bad[1:0];     // (i,j) of the bad output
            pr = spare[3:2]; pc = spare[1:0]; // (row,col) of the spare PE
            repair_cycles = 0;
            @(negedge clk); cfg_broadcast=1; clr_acc_all=1; cfg_data={OP_MACB,3'd0,3'd0};
            ext_in_west=0; ext_in_north=0; out_en_all=0;
            repair_cycles = repair_cycles + 1;
            @(negedge clk); cfg_broadcast=0; clr_acc_all=0; out_en_all=1;
            repair_cycles = repair_cycles + 1;
            for (t=0;t<4;t=t+1) begin
                ext_in_west=0; ext_in_north=0;
                ext_in_west [pr*DATA_W +: DATA_W] = ae(bi, t[1:0]);  // A[i,t]
                ext_in_north[pc*DATA_W +: DATA_W] = be(t[1:0], bj);  // B[t,j]
                @(negedge clk);
                repair_cycles = repair_cycles + 1;
            end
            ext_in_west=0; ext_in_north=0;
            @(negedge clk); @(negedge clk); @(negedge clk);   // DSP drain
            repair_cycles = repair_cycles + 3;
            out_en_all=0; @(negedge clk);
            repaired_val = all_pe_out[spare*ACC_W +: ACC_W];
        end
    endtask

    reg [ACC_W-1:0] golden [0:15];
    reg [ACC_W-1:0] repaired [0:15];
    integer pass_cnt, fail_cnt;

    initial begin
        for (i=0;i<16;i=i+1) begin A[i]=i+1; B[i]=(i%7)+1; end
        rst_n=0; fault_en_flat=0; fault_xor=FAULT_MASK;
        cfg_broadcast=0; clr_acc_all=0; out_en_all=0; cfg_data=0;
        ext_in_west=0; ext_in_north=0;
        repeat (3) @(negedge clk); rst_n=1;

        // golden
        fault_en_flat=0; run_matmul;
        for (i=0;i<16;i=i+1) golden[i]=all_pe_out[i*ACC_W +: ACC_W];

        $display("===========================================================");
        $display(" SELF-REPAIR — fault one PE, recompute its 1 output on a spare");
        $display("===========================================================");

        pass_cnt=0; fail_cnt=0;
        for (k=0;k<16;k=k+1) begin
            // 1. faulted full matmul
            fault_en_flat=0; fault_en_flat[k]=1'b1; run_matmul;
            for (i=0;i<16;i=i+1) repaired[i]=all_pe_out[i*ACC_W +: ACC_W];
            // 2. repair output k on a healthy spare (k+1 mod 16, guaranteed != k)
            p = (k+1) % 16;
            repair_one(k[3:0], p[3:0]);
            repaired[k] = repaired_val;          // substitute corrected value
            // 3. verify whole result now matches golden
            fail_cnt = 0;
            for (i=0;i<16;i=i+1) if (repaired[i] !== golden[i]) fail_cnt = fail_cnt + 1;
            if (fail_cnt==0) begin
                pass_cnt = pass_cnt + 1;
                $display(" fault PE %2d -> recompute on PE %2d -> REPAIRED (got %0d) [%0d cy]",
                         k, p, repaired_val, repair_cycles);
            end else begin
                $display(" fault PE %2d -> repair FAILED (%0d still wrong)", k, fail_cnt);
            end
        end
        fault_en_flat=0;

        $display("-----------------------------------------------------------");
        $display(" SELF-REPAIR SUMMARY: %0d/16 faults fully repaired", pass_cnt);
        $display("   repair cost: %0d cycles to fix ONE output (1 inner product)", repair_cycles);
        $display("   contrast: systolic must recompute the whole corrupted column");
        $display("             (N outputs) or burn a spare row.");
        if (pass_cnt==16) $display(" RESULT: PASS — every single-PE fault self-repaired.");
        else              $display(" RESULT: FAIL — %0d not repaired.", 16-pass_cnt);
        $display("===========================================================");
        #20 $finish;
    end

    initial begin #300000; $display("TIMEOUT"); $finish; end
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
