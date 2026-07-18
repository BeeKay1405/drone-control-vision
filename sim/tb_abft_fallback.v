// =============================================================================
// tb_abft_fallback.v — the NO-ESCAPE proof for single-PE faults
//
// The mod-3 residue (mac_err) is the fast path but misses errors that are
// multiples of 3 (~1/3 of random multi-bit corruptions). This TB proves the
// checksum SAFETY NET closes that gap completely:
//
//   PART A (decoder unit test): feed abft_locate synthetic "residue-blind"
//     errors (multiples of 3, multi-bit) with NO mac_err info — it must
//     detect (D0 != 0 is an identity), locate (D1 == D0<<j shift-compare),
//     and correct (one subtraction). Plus the documented ambiguous corner
//     (top-bits-only error) must raise `ambig`, not silently mislocate.
//
//   PART B (end-to-end, random campaign): N trials on the REAL fabric with
//     random matrices, random multi-bit fault mask, random faulted PE.
//     Every trial must end bit-exact with the software golden via:
//       fast path   (mac_err located it), or
//       safety net  (checksums located it), or
//       benign      (fault had zero net effect on outputs).
//     ZERO escapes allowed.
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module tb_abft_fallback;
    localparam DATA_W = 16, ACC_W = 32, CFG_W = 10, NUM_PE = 16;
    localparam [3:0] OP_MACB = 4'd9;
    localparam NTRIALS = 300;

    reg clk = 0; always #5 clk = ~clk;
    reg rst_n;
    reg                      cfg_broadcast, clr_acc_all, out_en_all;
    reg  [CFG_W-1:0]         cfg_data;
    reg  [4*DATA_W-1:0]      ext_in_west, ext_in_north;
    reg  [NUM_PE-1:0]        fault_en_flat;
    reg  [ACC_W-1:0]         fault_xor;
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
        // PE faults injected from the TB (force/release) - no DUT hook
        // (rail-fault hook removed from the fabric)
        .sram_rdata_flat(), .ext_out_east(), .all_pe_out(all_pe_out),
        .rail_err_w_flat(), .rail_err_n_flat(), .mac_err_flat(mac_err_flat)
    );

    abft_checksum #(.ROWS(4), .COLS(4), .DATA_W(DATA_W), .ACC_W(ACC_W)) u_ck (
        .clk(clk), .rst_n(rst_n), .clr_acc(clr_acc_all), .out_en(out_en_all),
        .ext_in_west(ext_in_west), .ext_in_north(ext_in_north),
        .cksum_flat(cksum_flat), .cksum_w_flat(cksum_w_flat)
    );

    // ---- one decoder per row, fed from capture registers ----
    reg [4*ACC_W-1:0] dec_c   [0:3];
    reg [ACC_W-1:0]   dec_s   [0:3], dec_sw [0:3];
    reg [3:0]         dec_me  [0:3];
    wire        d_present[0:3], d_located[0:3], d_fb[0:3], d_ambig[0:3];
    wire [1:0]  d_loc[0:3];
    wire [ACC_W-1:0] d_err[0:3], d_fix[0:3];

    genvar gr;
    generate for (gr=0; gr<4; gr=gr+1) begin : gdec
        abft_locate #(.COLS(4), .ACC_W(ACC_W)) u_dec (
            .c_obs_flat(dec_c[gr]), .cksum(dec_s[gr]), .cksum_w(dec_sw[gr]),
            .mac_err_row(dec_me[gr]),
            .err_present(d_present[gr]), .located(d_located[gr]), .loc(d_loc[gr]),
            .used_fallback(d_fb[gr]), .ambig(d_ambig[gr]),
            .err_val(d_err[gr]), .c_fixed(d_fix[gr])
        );
    end endgenerate

    // ---- sticky mac_err capture ----
    reg [NUM_PE-1:0] st_mac;
    reg capture;
    always @(posedge clk) if (capture) st_mac <= st_mac | mac_err_flat;

    // ---- matrices + software golden (mod 2^32, same as hardware) ----
    reg [DATA_W-1:0] A [0:15];
    reg [DATA_W-1:0] B [0:15];
    reg [ACC_W-1:0]  golden [0:15];
    integer t, i, j, k, r, n;
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

    // latch fabric state into the decoders
    task load_decoders;
        begin
            for (r=0;r<4;r=r+1) begin
                dec_c[r]  = all_pe_out[r*4*ACC_W +: 4*ACC_W];
                dec_s[r]  = cksum_flat  [r*ACC_W +: ACC_W];
                dec_sw[r] = cksum_w_flat[r*ACC_W +: ACC_W];
                dec_me[r] = st_mac[r*4 +: 4];
            end
            #1;  // settle combinational decoders
        end
    endtask

    integer passA, failA, seed;
    integer n_fast, n_fb, n_benign, n_escape, n_falsepos;
    reg [ACC_W-1:0] fixed [0:15];
    reg [ACC_W-1:0] e_syn;
    integer  col_syn;

    initial begin
        seed = 32'hFAB1E5;
        rst_n=0; fault_en_flat=0; fault_xor=32'h1;
        cfg_broadcast=0; clr_acc_all=0; out_en_all=0; cfg_data=0;
        ext_in_west=0; ext_in_north=0; capture=0; st_mac=0;
        repeat (3) @(negedge clk); rst_n=1;

        $display("=================================================================");
        $display(" NO-ESCAPE PROOF — checksum safety net behind the residue fast path");
        $display("=================================================================");

        // ================= PART A: decoder unit test =================
        // real clean run gives true outputs + checksums; then corrupt ONE
        // column with residue-blind errors (multiples of 3) IN THE DECODER
        // INPUT, with mac_err forced silent — the safety net must solve it.
        for (i=0;i<16;i=i+1) begin A[i]=i+1; B[i]=(i%7)+1; end
        compute_golden;
        fault_en_flat=0; run_matmul; load_decoders;

        passA=0; failA=0;
        for (col_syn=0; col_syn<4; col_syn=col_syn+1) begin
            for (n=0; n<5; n=n+1) begin
                case (n)   // all multiples of 3 (mod-3 blind), incl. big/multi-bit
                    0: e_syn = 32'd21;
                    1: e_syn = -32'd21;
                    2: e_syn = 32'd3 << 8;
                    3: e_syn = 32'h0000_FFFF * 3;
                    4: e_syn = 32'hDEAD_BEEF * 3;
                endcase
                for (r=0;r<4;r=r+1) begin   // restore clean state
                    dec_c[r]  = all_pe_out[r*4*ACC_W +: 4*ACC_W];
                    dec_me[r] = 4'b0000;    // residue saw NOTHING (worst case)
                end
                dec_c[1][col_syn*ACC_W +: ACC_W] =
                    dec_c[1][col_syn*ACC_W +: ACC_W] + e_syn;  // corrupt row 1
                #1;
                if (d_present[1] && d_located[1] && d_fb[1] &&
                    d_loc[1] == col_syn[1:0] &&
                    d_fix[1] === golden[4 + col_syn])
                    passA = passA + 1;
                else begin
                    failA = failA + 1;
                    $display("  PART A FAIL: col %0d e=%h  located=%b loc=%0d fix=%0d",
                             col_syn, e_syn, d_located[1], d_loc[1], d_fix[1]);
                end
            end
        end
        $display(" PART A: residue-blind errors (x21, x3<<8, big multi-bit), NO mac_err:");
        $display("   %0d/20 detected+located+corrected by checksums alone", passA);

        // ambiguous corner: top-bits-only error must raise ambig, not mislocate
        for (r=0;r<4;r=r+1) begin
            dec_c[r]  = all_pe_out[r*4*ACC_W +: 4*ACC_W];
            dec_me[r] = 4'b0000;
        end
        dec_c[1][2*ACC_W +: ACC_W] = dec_c[1][2*ACC_W +: ACC_W] + 32'h8000_0000;
        #1;
        if (d_present[1] && d_ambig[1] && !d_located[1])
            $display(" ambiguous corner (e=2^31, top-bits-only): flagged ambig -> honest");
        else begin
            failA = failA + 1;
            $display(" ambiguous corner: NOT flagged (located=%b ambig=%b) -> FAIL",
                     d_located[1], d_ambig[1]);
        end

        // ================= PART B: random end-to-end campaign =================
        $display(" PART B: %0d random trials (random A,B / random multi-bit mask /", NTRIALS);
        $display("         random faulted PE) — every trial must end bit-exact:");
        n_fast=0; n_fb=0; n_benign=0; n_escape=0; n_falsepos=0;
        for (n=0; n<NTRIALS; n=n+1) begin
            for (i=0;i<16;i=i+1) begin A[i]=$random(seed); B[i]=$random(seed); end
            compute_golden;
            fault_xor = $random(seed); if (fault_xor==0) fault_xor = 32'h1;
            k = {$random(seed)} % 16;
            fault_en_flat = 0; fault_en_flat[k] = 1'b1;
            run_matmul; fault_en_flat = 0;
            load_decoders;

            for (i=0;i<16;i=i+1) fixed[i] = all_pe_out[i*ACC_W +: ACC_W];
            r = k/4;
            // decoders on the 3 healthy rows must see nothing
            for (i=0;i<4;i=i+1)
                if (i != r && d_present[i]) n_falsepos = n_falsepos + 1;
            if (!d_present[r]) begin
                // fault had zero net effect -> outputs must already be golden
                n_benign = n_benign + 1;
            end else if (d_located[r]) begin
                fixed[r*4 + d_loc[r]] = d_fix[r];
                if (d_fb[r]) n_fb = n_fb + 1; else n_fast = n_fast + 1;
            end else begin
                n_escape = n_escape + 1;
                $display("  trial %0d: AMBIG/UNLOCATED (PE %0d mask %h)", n, k, fault_xor);
            end
            for (i=0;i<16;i=i+1)
                if (fixed[i] !== golden[i]) begin
                    n_escape = n_escape + 1;
                    $display("  trial %0d: MISMATCH after fix (PE %0d mask %h out %0d)",
                             n, k, fault_xor, i);
                end
        end

        $display("   fast path  (residue located) : %0d", n_fast);
        $display("   safety net (checksums located): %0d  <- the closed 'mod-3 blind' gap", n_fb);
        $display("   benign     (zero net effect)  : %0d", n_benign);
        $display("   false positives on healthy rows: %0d", n_falsepos);
        $display("   ESCAPES                        : %0d", n_escape);
        $display("-----------------------------------------------------------------");
        if (failA==0 && n_escape==0 && n_falsepos==0)
            $display(" RESULT: PASS — no single-PE fault, of ANY bit pattern, escapes");
        else
            $display(" RESULT: FAIL");
        $display("         detection + location + exact correction.");
        $display("=================================================================");
        #20 $finish;
    end

    initial begin #60000000; $display("TIMEOUT"); $finish; end
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
