// =============================================================================
// tb_fault_campaign.v — large randomized PERMANENT-fault campaign, 3 fault models
//
// Sweeps the campaign over THREE permanent-fault models (the standard spectrum):
//   * XOR  : random 32-bit bit-flip pattern (out ^ mask) — always corrupts
//   * SA0  : stuck-at-0 on masked bits (out & ~mask)      — may be DATA-MASKED
//   * SA1  : stuck-at-1 on masked bits (out | mask)       — may be DATA-MASKED
// Each model: NTRIALS trials, random A,B, 1..MAXK simultaneous faulted PEs.
//
// Stuck-at faults can be silent when the stuck bits already equal the correct
// value, so "every fault corrupts" is FALSE for SA0/SA1. We therefore measure
// the TRUE containment invariant and an honest detection denominator:
//   CONTAINMENT : a corrupted output never appears at a NON-faulted PE
//                 (spread == 0). This is the broadcast property, at scale.
//   EFFECTIVENESS: how many faulted PEs actually produced a wrong output
//                 (the rest were data-masked — reported, not a failure).
//   DETECTION   : among ACTUALLY-corrupted outputs, did residue (mac_err) or the
//                 row checksum (D0 != 0) fire? ESCAPE = corrupted, both blind.
//
// PASS = zero spread violations AND zero escapes, across all three models.
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module tb_fault_campaign;
    localparam DATA_W = 16, ACC_W = 32, CFG_W = 10, NUM_PE = 16;
    localparam [3:0] OP_MACB = 4'd9;
    localparam integer NTRIALS = 17000;    // per model; avg ~2 faults -> ~34k inj/model (~100k total)
    localparam integer MAXK    = 3;        // up to 3 simultaneous faults
    localparam [1:0]   M_XOR = 2'd0, M_SA0 = 2'd1, M_SA1 = 2'd2;

    reg clk = 0; always #5 clk = ~clk;
    reg rst_n;
    reg                      cfg_broadcast, clr_acc_all, out_en_all;
    reg  [CFG_W-1:0]         cfg_data;
    reg  [4*DATA_W-1:0]      ext_in_west, ext_in_north;
    reg  [NUM_PE-1:0]        fault_en_flat;
    reg  [ACC_W-1:0]         fault_mask;
    reg  [1:0]               fault_model;
    wire [NUM_PE*ACC_W-1:0]  all_pe_out;
    wire [NUM_PE-1:0]        mac_err_flat;
    wire [4*ACC_W-1:0]       cksum_flat, cksum_w_flat;

    sage16_4x4_mac #(.DATA_W(DATA_W), .ACC_W(ACC_W), .CFG_W(CFG_W), .PIPELINE(1)) dut (
        .clk(clk), .rst_n(rst_n), .cfg_pe_row(2'd0), .cfg_pe_col(2'd0),
        .cfg_data(cfg_data), .cfg_load(1'b0), .cfg_broadcast(cfg_broadcast),
        .clr_acc_all(clr_acc_all), .out_en_all(out_en_all),
        .ext_in_west(ext_in_west), .ext_in_north(ext_in_north),
        .per_pe_bypass_en(1'b0), .per_pe_bypass_flat(256'b0),
        .sram_cs_n_flat({NUM_PE{1'b1}}), .sram_we_n_flat({NUM_PE{1'b1}}),
        .sram_addr_flat(128'b0), .sram_raddr2_flat(128'b0), .sram_wdata_sel(16'b0),
        .sram_wdata_ext_flat(512'b0), .sel_src_a_flat(16'b0), .sel_src_b_flat(16'b0),
        .sram_rdata_flat(), .ext_out_east(), .all_pe_out(all_pe_out),
        .rail_err_w_flat(), .rail_err_n_flat(), .mac_err_flat(mac_err_flat)
    );

    abft_checksum #(.ROWS(4), .COLS(4), .DATA_W(DATA_W), .ACC_W(ACC_W)) u_ck (
        .clk(clk), .rst_n(rst_n), .clr_acc(clr_acc_all), .out_en(out_en_all),
        .ext_in_west(ext_in_west), .ext_in_north(ext_in_north),
        .cksum_flat(cksum_flat), .cksum_w_flat(cksum_w_flat)
    );

    // sticky mac_err over the run
    reg [NUM_PE-1:0] st_mac; reg capture;
    always @(posedge clk) if (capture) st_mac <= st_mac | mac_err_flat;

    reg [DATA_W-1:0] A [0:15];
    reg [DATA_W-1:0] B [0:15];
    reg [ACC_W-1:0]  golden [0:15];
    integer t, i, j, k;
    function [DATA_W-1:0] ae; input [1:0] ii,kk; ae = A[ii*4+kk]; endfunction
    function [DATA_W-1:0] be; input [1:0] kk,jj; be = B[kk*4+jj]; endfunction

    task compute_golden;
        reg [ACC_W-1:0] s;
        begin
            for (i=0;i<4;i=i+1) for (j=0;j<4;j=j+1) begin
                s = 0;
                for (k=0;k<4;k=k+1) s = s + A[i*4+k]*B[k*4+j];
                golden[i*4+j] = s;
            end
        end
    endtask

    task run_matmul;
        begin
            st_mac = 0; capture = 1;
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
            out_en_all=0; @(negedge clk); @(negedge clk);
            capture = 0;
        end
    endtask

    integer seed, trial, fc, picked, p, r;
    integer tot_inj, tot_wrong, corr_f, spread_viol, max_spread, wnf;
    integer det_tot, det_res, det_ck, escapes;
    integer g_spread, g_esc;
    reg [NUM_PE-1:0] fmask;
    reg [ACC_W-1:0]  d0_row;
    reg              badv;
    integer row_has_fault [0:3];
    reg [23:0] mname;

    task run_model;
        input [1:0] mdl;
        begin
            fault_model = mdl;
            case (mdl) M_SA0: mname="SA0"; M_SA1: mname="SA1"; default: mname="XOR"; endcase
            tot_inj=0; tot_wrong=0; corr_f=0; spread_viol=0; max_spread=0;
            det_tot=0; det_res=0; det_ck=0; escapes=0;

            for (trial=0; trial<NTRIALS; trial=trial+1) begin
                for (i=0;i<16;i=i+1) begin A[i]=$random(seed); B[i]=$random(seed); end
                compute_golden;

                fc = ({$random(seed)} % MAXK) + 1;     // 1..MAXK faults
                fmask = 0; picked = 0;
                while (picked < fc) begin
                    p = {$random(seed)} % 16;
                    if (!fmask[p]) begin fmask[p]=1'b1; picked=picked+1; end
                end
                fault_mask = $random(seed); if (fault_mask==0) fault_mask = 32'hABCD_1234;

                fault_en_flat = fmask; run_matmul; fault_en_flat = 0;

                for (r=0;r<4;r=r+1) row_has_fault[r]=0;
                for (i=0;i<16;i=i+1) if (fmask[i]) row_has_fault[i/4]=row_has_fault[i/4]+1;
                tot_inj = tot_inj + fc;

                // ---- containment: corrupted output must stay at a FAULTED PE ----
                wnf = 0;
                for (i=0;i<16;i=i+1) begin
                    badv = (all_pe_out[i*ACC_W +: ACC_W] !== golden[i]);
                    if (badv) begin
                        tot_wrong = tot_wrong + 1;
                        if (fmask[i]) corr_f = corr_f + 1;
                        else          wnf    = wnf + 1;   // spread to a clean PE = violation
                    end
                end
                if (wnf > 0)          spread_viol = spread_viol + 1;
                if (wnf > max_spread) max_spread  = wnf;

                // ---- detection over ACTUALLY-corrupted faulted PEs ----
                for (i=0;i<16;i=i+1) if (fmask[i]) begin
                    badv = (all_pe_out[i*ACC_W +: ACC_W] !== golden[i]);
                    if (badv) begin
                        det_tot = det_tot + 1;
                        if (st_mac[i]) det_res = det_res + 1;
                        r = i/4;
                        d0_row = (all_pe_out[(r*4+0)*ACC_W +: ACC_W]
                                + all_pe_out[(r*4+1)*ACC_W +: ACC_W]
                                + all_pe_out[(r*4+2)*ACC_W +: ACC_W]
                                + all_pe_out[(r*4+3)*ACC_W +: ACC_W])
                                - cksum_flat[r*ACC_W +: ACC_W];
                        if (d0_row !== 0) det_ck = det_ck + 1;
                        if (!st_mac[i] && d0_row === 0) escapes = escapes + 1;
                    end
                end
            end

            g_spread = g_spread + spread_viol;
            g_esc    = g_esc + escapes;

            $display("-----------------------------------------------------------------");
            $display(" MODEL %0s — %0d trials, %0d injections", mname, NTRIALS, tot_inj);
            $display("   effectiveness (faults that corrupted): %0d / %0d", corr_f, tot_inj);
            $display("   CONTAINMENT spread to clean PEs       : %0d trials (max %0d)  [target 0]",
                     spread_viol, max_spread);
            if (det_tot>0) begin
                $display("   residue (mac_err) coverage           : %0d / %0d  (%0d.%01d%%)",
                         det_res, det_tot, (det_res*100)/det_tot, ((det_res*1000)/det_tot)%10);
                $display("   checksum (D0!=0) coverage            : %0d / %0d  (%0d.%01d%%)",
                         det_ck, det_tot, (det_ck*100)/det_tot, ((det_ck*1000)/det_tot)%10);
            end
            $display("   ESCAPES (corrupted, both blind)       : %0d  [target 0]", escapes);
        end
    endtask

    initial begin
        seed = 32'h5A6E16;
        rst_n=0; fault_en_flat=0; fault_mask=32'h1; fault_model=M_XOR;
        cfg_broadcast=0; clr_acc_all=0; out_en_all=0; cfg_data=0;
        ext_in_west=0; ext_in_north=0; capture=0; st_mac=0;
        repeat (3) @(negedge clk); rst_n=1;
        g_spread=0; g_esc=0;

        $display("=================================================================");
        $display(" PERMANENT-FAULT CAMPAIGN — 3 models x %0d trials, 1..%0d faults", NTRIALS, MAXK);
        $display("   random matrices, random masks/sites; broadcast 4x4 fabric");
        $display("=================================================================");

        run_model(M_XOR);
        run_model(M_SA0);
        run_model(M_SA1);

        $display("=================================================================");
        if (g_spread==0 && g_esc==0)
            $display(" RESULT: PASS — across all 3 models: 0 spread (every fault contained");
        else
            $display(" RESULT: FAIL — spread_viol=%0d escapes=%0d", g_spread, g_esc);
        $display("         to its own output), 0 escapes (every corruption flagged).");
        $display("=================================================================");
        #20 $finish;
    end

    initial begin #2000000000; $display("TIMEOUT"); $finish; end

    // ---- TB-SIDE permanent PE-fault injector (DUT datapath is clean) ----
    // force/release each PE accumulator on accumulating cycles. The corruption
    // law depends on fault_model: XOR flip, stuck-at-0, or stuck-at-1.
    genvar fipe;
    generate for (fipe=0; fipe<16; fipe=fipe+1) begin : FINJ
        reg [ACC_W-1:0] fi_clean;
        always @(posedge clk) begin
            release dut.rg[fipe/4].cg[fipe%4].u_pe.out;
            if (fault_en_flat[fipe] && out_en_all && !clr_acc_all) begin
                #1 fi_clean = dut.rg[fipe/4].cg[fipe%4].u_pe.out;
                case (fault_model)
                    M_SA0:   force dut.rg[fipe/4].cg[fipe%4].u_pe.out = fi_clean & ~fault_mask;
                    M_SA1:   force dut.rg[fipe/4].cg[fipe%4].u_pe.out = fi_clean |  fault_mask;
                    default: force dut.rg[fipe/4].cg[fipe%4].u_pe.out = fi_clean ^  fault_mask;
                endcase
            end
        end
    end endgenerate

endmodule

`default_nettype wire
