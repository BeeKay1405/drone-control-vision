`timescale 1ns/1ps
`default_nettype none
// =============================================================================
// tb_checksum_reduction.v — DEMO of the "checksum-reduction identity" IN HARDWARE
//
// Now wires the real selfval_reduce.v module (the "one adder does both"): it
// computes R[i] = sum_j C[i][j], which is SIMULTANEOUSLY
//   (1) the FUNCTIONAL reduction (per-row mean numerator; LayerNorm-mean over
//       features; column analog = global-average-pool), and
//   (2) the ACTUAL ABFT checksum compared to the free input-derived prediction
//       S_i from abft_checksum.
// => one adder tree does BOTH the reduction AND detection, and the reduction is
//    SELF-VALIDATING (reduce_valid[i] from the module flags a corrupted output).
//
// Phase 1 (fault-free): module reduction == prediction == golden, reduce_valid=1.
// Phase 2 (permanent PE fault): faulted row's reduce_valid=0 (self-flagged) and its
//          reduction is wrong; other rows clean+valid (containment).
//
// Run: iverilog -g2012 -o build/tb_csr sim/tb_checksum_reduction.v rtl/sage16_4x4_mac.v \
//      rtl/pe.v rtl/sram_1rw_256x32.v rtl/mod3_reduce.v rtl/mod7_reduce.v \
//      rtl/abft_checksum.v rtl/selfval_reduce.v ; vvp build/tb_csr
// =============================================================================
module tb_checksum_reduction;
    localparam DATA_W=16, ACC_W=32, CFG_W=10, NUM_PE=16;
    localparam [3:0] OP_MACB = 4'd9;

    reg clk=0; always #5 clk=~clk;
    reg rst_n;
    reg                     cfg_broadcast, clr_acc_all, out_en_all;
    reg  [CFG_W-1:0]        cfg_data;
    reg  [4*DATA_W-1:0]     ext_in_west, ext_in_north;
    reg  [NUM_PE-1:0]       fault_en_flat;
    reg  [ACC_W-1:0]        fault_mask;
    wire [NUM_PE*ACC_W-1:0] all_pe_out;
    wire [NUM_PE-1:0]       mac_err_flat;
    wire [4*ACC_W-1:0]      cksum_flat, cksum_w_flat;

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
        .rail_err_w_flat(), .rail_err_n_flat(), .mac_err_flat(mac_err_flat));

    // free rail-listener checksum predictor (input-derived S_i)
    abft_checksum #(.ROWS(4), .COLS(4), .DATA_W(DATA_W), .ACC_W(ACC_W)) u_ck (
        .clk(clk), .rst_n(rst_n), .clr_acc(clr_acc_all), .out_en(out_en_all),
        .ext_in_west(ext_in_west), .ext_in_north(ext_in_north),
        .cksum_flat(cksum_flat), .cksum_w_flat(cksum_w_flat));

    // THE NOVELTY IN HW: one adder = reduction + self-validation
    wire [4*ACC_W-1:0] row_reduce_flat;
    wire [3:0]         reduce_valid;
    wire               any_invalid;
    selfval_reduce #(.ROWS(4), .COLS(4), .ACC_W(ACC_W), .GEN_CHECK(1)) u_red (
        .all_pe_out(all_pe_out), .pred_flat(cksum_flat),
        .row_reduce_flat(row_reduce_flat), .reduce_valid(reduce_valid),
        .any_invalid(any_invalid));

    reg [DATA_W-1:0] A [0:15];
    reg [DATA_W-1:0] B [0:15];
    reg [ACC_W-1:0]  Cg [0:15];
    reg [ACC_W-1:0]  Rg [0:3];
    integer t,i,j,k,pass,fail;
    function [DATA_W-1:0] ae; input [1:0] ii,kk; ae=A[ii*4+kk]; endfunction
    function [DATA_W-1:0] be; input [1:0] kk,jj; be=B[kk*4+jj]; endfunction
    function [ACC_W-1:0]  hwred; input integer r; hwred=row_reduce_flat[r*ACC_W +: ACC_W]; endfunction
    function [ACC_W-1:0]  pred;  input integer r; pred =cksum_flat[r*ACC_W +: ACC_W];     endfunction

    task compute_golden;
        reg [ACC_W-1:0] s;
        begin
            for (i=0;i<4;i=i+1) for (j=0;j<4;j=j+1) begin
                s=0; for (k=0;k<4;k=k+1) s=s+A[i*4+k]*B[k*4+j];
                Cg[i*4+j]=s;
            end
            for (i=0;i<4;i=i+1) begin
                s=0; for (j=0;j<4;j=j+1) s=s+Cg[i*4+j];
                Rg[i]=s;
            end
        end
    endtask

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
            out_en_all=0; @(negedge clk); @(negedge clk);
        end
    endtask

    initial begin
        rst_n=0; fault_en_flat=0; fault_mask=32'h0000_0040;
        cfg_broadcast=0; clr_acc_all=0; out_en_all=0; cfg_data=0;
        ext_in_west=0; ext_in_north=0; pass=0; fail=0;
        repeat(3) @(negedge clk); rst_n=1; @(negedge clk);

        for (i=0;i<16;i=i+1) begin A[i]=(i%7)+1; B[i]=(i%5)+2; end
        compute_golden;

        // ---------------- PHASE 1: the identity, fault-free (via HW module) ------
        $display("=================================================================");
        $display(" CHECKSUM-REDUCTION IDENTITY (HW: selfval_reduce)  4x4, fault-free");
        $display("   row_reduce_flat = sum_j C[i][j] : per-row mean numerator AND");
        $display("   ABFT checksum; reduce_valid = self-validation (one adder, both).");
        $display("=================================================================");
        fault_en_flat=0; run_matmul;
        for (i=0;i<4;i=i+1) begin
            $display(" row %0d: HW-reduction=%0d  ABFT-pred=%0d  golden=%0d  mean=%0d  valid=%0d",
                     i, hwred(i), pred(i), Rg[i], hwred(i)/4, reduce_valid[i]);
            if (hwred(i)===Rg[i] && hwred(i)===pred(i) && reduce_valid[i]===1'b1) begin
                pass=pass+1; $display("        PASS: reduction == checksum == golden, self-validated");
            end else begin fail=fail+1; $display("        FAIL"); end
        end
        if (any_invalid===1'b0) begin pass=pass+1; $display(" any_invalid=0 (whole layer validated)  PASS"); end
        else begin fail=fail+1; $display(" any_invalid FAIL"); end

        // ------- PHASE 2: self-validating reduction under a permanent fault -------
        $display("-----------------------------------------------------------------");
        $display(" Inject PERMANENT fault at PE(2,1). Row 2 reduce_valid must go 0");
        $display(" (self-flagged) and its reduction be wrong; rows 0,1,3 valid (contained).");
        $display("-----------------------------------------------------------------");
        fault_en_flat=0; fault_en_flat[2*4+1]=1'b1;
        run_matmul; fault_en_flat=0;
        for (i=0;i<4;i=i+1) begin
            if (i==2) begin
                if (reduce_valid[i]===1'b0 && hwred(i)!==Rg[i]) begin
                    pass=pass+1;
                    $display(" row %0d: HW-reduction=%0d pred=%0d golden=%0d valid=0 -> SELF-FLAGGED  (PASS)",
                             i, hwred(i), pred(i), Rg[i]);
                end else begin fail=fail+1; $display(" row %0d: FAIL (fault not self-flagged)", i); end
            end else begin
                if (reduce_valid[i]===1'b1 && hwred(i)===Rg[i]) begin
                    pass=pass+1;
                    $display(" row %0d: HW-reduction=%0d valid=1 -> clean + validated (PASS, contained)", i, hwred(i));
                end else begin fail=fail+1; $display(" row %0d: FAIL (clean row disturbed)", i); end
            end
        end
        if (any_invalid===1'b1) begin pass=pass+1; $display(" any_invalid=1 (layer-level alarm raised)  PASS"); end
        else begin fail=fail+1; $display(" any_invalid FAIL"); end

        $display("=================================================================");
        $display(" RESULT: %0d passed, %0d failed", pass, fail);
        if (fail==0) $display(" ALL PASS - one HW adder = reduction + detection; reduction self-validates.");
        else         $display(" FAIL");
        $display("=================================================================");
        #20 $finish;
    end

    initial begin #20000000; $display("TIMEOUT"); $finish; end

    genvar fipe;
    generate for (fipe=0; fipe<16; fipe=fipe+1) begin : FINJ
        reg [ACC_W-1:0] fi_clean;
        always @(posedge clk) begin
            release dut.rg[fipe/4].cg[fipe%4].u_pe.out;
            if (fault_en_flat[fipe] && out_en_all && !clr_acc_all) begin
                #1 fi_clean = dut.rg[fipe/4].cg[fipe%4].u_pe.out;
                force dut.rg[fipe/4].cg[fipe%4].u_pe.out = fi_clean ^ fault_mask;
            end
        end
    end endgenerate
endmodule
`default_nettype wire
