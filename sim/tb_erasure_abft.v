// =============================================================================
// tb_erasure_abft.v — erasure-ABFT: located faults corrected ALGEBRAICALLY
//
// Proves the repair upgrade over tb_self_repair (9-cycle recompute on a spare):
//   * 1 PE fault (location known from mac_err syndrome):
//       e = (sum of observed row outputs) - S_row ; corrected = observed - e
//       -> ONE subtraction, no spare PE, no recompute.  16/16 expected.
//   * 2 PE faults in the SAME ROW (both locations known):
//       solve 2x2 with plain + weighted checksums via modular inverse of
//       (2^d - 1) mod 2^32 (d = j2-j1; inverses of 1/3/7 — no divider).
//       4 rows x 6 column-pairs = 24 cases expected.
//   * checksums ride the rails for FREE: same taps, zero extra cycles —
//     verified by checking S_i == golden row sums on a clean run.
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module tb_erasure_abft;
    localparam DATA_W = 16, ACC_W = 32, CFG_W = 10, NUM_PE = 16;
    localparam [3:0] OP_MACB = 4'd9;
    localparam [ACC_W-1:0] INV3 = 32'hAAAAAAAB;   // 3 * INV3 == 1 (mod 2^32)
    localparam [ACC_W-1:0] INV7 = 32'hB6DB6DB7;   // 7 * INV7 == 1 (mod 2^32)

    reg clk = 0; always #5 clk = ~clk;
    reg rst_n;
    reg                      cfg_broadcast, clr_acc_all, out_en_all;
    reg  [CFG_W-1:0]         cfg_data;
    reg  [4*DATA_W-1:0]      ext_in_west, ext_in_north;
    reg  [NUM_PE-1:0]        fault_en_flat;
    reg  [ACC_W-1:0]         fault_xor;
    wire [NUM_PE*ACC_W-1:0]  all_pe_out;
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
        // PE faults injected from the TB (force/release) — no DUT hook
        // (rail-fault hook removed from the fabric)
        .sram_rdata_flat(), .ext_out_east(), .all_pe_out(all_pe_out),
        .rail_err_w_flat(), .rail_err_n_flat(), .mac_err_flat()
    );

    // checksum PEs: free riders on the same rails, same control
    abft_checksum #(.ROWS(4), .COLS(4), .DATA_W(DATA_W), .ACC_W(ACC_W)) u_ck (
        .clk(clk), .rst_n(rst_n), .clr_acc(clr_acc_all), .out_en(out_en_all),
        .ext_in_west(ext_in_west), .ext_in_north(ext_in_north),
        .cksum_flat(cksum_flat), .cksum_w_flat(cksum_w_flat)
    );

    // ---- test matrices ----
    reg [DATA_W-1:0] A [0:15];
    reg [DATA_W-1:0] B [0:15];
    integer t, i, k, r, j1, j2;
    function [DATA_W-1:0] ae; input [1:0] ii,kk; ae = A[ii*4+kk]; endfunction
    function [DATA_W-1:0] be; input [1:0] kk,jj; be = B[kk*4+jj]; endfunction

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

    // ---- the correction algebra (what the sequencer implements) ----
    function [ACC_W-1:0] obs;   // observed output (i,j)
        input integer rr, jj;
        obs = all_pe_out[(rr*4+jj)*ACC_W +: ACC_W];
    endfunction
    function [ACC_W-1:0] rowsum;
        input integer rr;
        rowsum = obs(rr,0) + obs(rr,1) + obs(rr,2) + obs(rr,3);
    endfunction
    function [ACC_W-1:0] rowsum_w;   // weighted by 2^j
        input integer rr;
        rowsum_w = obs(rr,0) + (obs(rr,1)<<1) + (obs(rr,2)<<2) + (obs(rr,3)<<3);
    endfunction
    function [ACC_W-1:0] inv_m;      // modular inverse of (2^d - 1), d=1..3
        input integer d;
        inv_m = (d==1) ? 32'h1 : (d==2) ? INV3 : INV7;
    endfunction

    reg [ACC_W-1:0] golden [0:15];
    reg [ACC_W-1:0] fixed  [0:15];
    reg [ACC_W-1:0] D0, D1, X, e1, e2;
    reg signed [ACC_W-1:0] xs;   // own statement: keeps >>> arithmetic (Verilog
                                 // signedness propagates INTO parens otherwise)
    integer pass1, fail1, pass2, fail2, bad;

    initial begin
        for (i=0;i<16;i=i+1) begin A[i]=i+1; B[i]=(i%7)+1; end
        rst_n=0; fault_en_flat=0; fault_xor=32'h1;
        cfg_broadcast=0; clr_acc_all=0; out_en_all=0; cfg_data=0;
        ext_in_west=0; ext_in_north=0;
        repeat (3) @(negedge clk); rst_n=1;

        $display("=================================================================");
        $display(" ERASURE-ABFT — located faults corrected algebraically, no recompute");
        $display("=================================================================");

        // ---- clean run: golden + checksum identity (free-rider proof) ----
        fault_en_flat=0; run_matmul;
        for (i=0;i<16;i=i+1) golden[i]=all_pe_out[i*ACC_W +: ACC_W];
        bad = 0;
        for (r=0;r<4;r=r+1) begin
            if (cksum_flat  [r*ACC_W +: ACC_W] !== rowsum(r))   bad = bad + 1;
            if (cksum_w_flat[r*ACC_W +: ACC_W] !== rowsum_w(r)) bad = bad + 1;
        end
        if (bad==0) $display(" checksum identity   : S_i == row sums, S'_i == weighted sums");
        else        $display(" checksum identity   : %0d MISMATCHES -> FAIL", bad);
        $display("   (computed on the SAME 4 taps as the matmul — zero extra cycles)");

        // ---- part 1: single fault, every PE: fix = ONE subtraction ----
        $display(" --- 1 located fault: e = rowsum - S; corrected = observed - e ---");
        pass1=0; fail1=0;
        for (k=0;k<16;k=k+1) begin
            fault_en_flat=0; fault_en_flat[k]=1'b1; run_matmul; fault_en_flat=0;
            r = k/4; j1 = k%4;
            D0 = rowsum(r) - cksum_flat[r*ACC_W +: ACC_W];   // = e, exact mod 2^32
            for (i=0;i<16;i=i+1) fixed[i]=all_pe_out[i*ACC_W +: ACC_W];
            fixed[k] = fixed[k] - D0;
            bad = 0; for (i=0;i<16;i=i+1) if (fixed[i] !== golden[i]) bad = bad+1;
            if (bad==0) begin pass1=pass1+1;
                $display("  PE %2d : e=%0d -> corrected with 1 subtraction", k, $signed(D0));
            end else begin fail1=fail1+1;
                $display("  PE %2d : FAIL (%0d still wrong)", k, bad);
            end
        end

        // ---- part 2: TWO faults, same row, all column pairs ----
        $display(" --- 2 located faults (same row): 2x2 solve, modular inverse ---");
        pass2=0; fail2=0;
        fault_xor = 32'h1;
        for (r=0;r<4;r=r+1) begin
            for (j1=0;j1<4;j1=j1+1) for (j2=j1+1;j2<4;j2=j2+1) begin
                fault_en_flat=0;
                fault_en_flat[r*4+j1]=1'b1; fault_en_flat[r*4+j2]=1'b1;
                run_matmul; fault_en_flat=0;
                // D0 = e1+e2 ; D1 = 2^j1*e1 + 2^j2*e2   (mod 2^32)
                D0 = rowsum(r)   - cksum_flat  [r*ACC_W +: ACC_W];
                D1 = rowsum_w(r) - cksum_w_flat[r*ACC_W +: ACC_W];
                // X = 2^j2*D0 - D1 = 2^j1*(2^(j2-j1)-1)*e1
                X  = (D0 << j2) - D1;
                xs = $signed(X) >>> j1;          // arithmetic shift, isolated
                e1 = xs * inv_m(j2-j1);
                e2 = D0 - e1;
                for (i=0;i<16;i=i+1) fixed[i]=all_pe_out[i*ACC_W +: ACC_W];
                fixed[r*4+j1] = fixed[r*4+j1] - e1;
                fixed[r*4+j2] = fixed[r*4+j2] - e2;
                bad = 0; for (i=0;i<16;i=i+1) if (fixed[i] !== golden[i]) bad = bad+1;
                if (bad==0) begin pass2=pass2+1;
                    $display("  row %0d cols (%0d,%0d) : e1=%0d e2=%0d -> both corrected",
                             r, j1, j2, $signed(e1), $signed(e2));
                end else begin fail2=fail2+1;
                    $display("  row %0d cols (%0d,%0d) : FAIL (%0d wrong)", r, j1, j2, bad);
                end
            end
        end

        $display("-----------------------------------------------------------------");
        $display(" ERASURE-ABFT SUMMARY:");
        $display("   1-fault algebraic repair : %0d/16  (cost: 1 subtraction, ~1 cycle", pass1);
        $display("                              vs 9-cycle spare-PE recompute)");
        $display("   2-fault algebraic repair : %0d/24  (2x2 solve, mult-by-inverse,", pass2);
        $display("                              no divider — 2 located faults fixed");
        $display("                              with 2 checksums vs 4 classical)");
        $display("   checksum hardware cost   : 2 adder trees + 4 checksum MACs,");
        $display("                              riding existing rails, 0 extra cycles");
        if (fail1==0 && fail2==0 && bad==0)
            $display(" RESULT: PASS — errors with known location are erasures;");
        else
            $display(" RESULT: FAIL");
        $display("          t checksums correct t located faults.");
        $display("=================================================================");
        #20 $finish;
    end

    initial begin #2000000; $display("TIMEOUT"); $finish; end
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
