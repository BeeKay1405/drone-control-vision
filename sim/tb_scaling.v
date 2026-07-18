// =============================================================================
// tb_scaling.v — reliability mechanism SCALING test (8x8, 16x16, ...)
//
// Shows the broadcast reliability properties are NOT a 4x4 fluke: at N x N the
// fabric still (a) computes a clean matmul with zero false alarms, (b) CONTAINS a
// permanent PE fault to that PE's single output (no spread), and (c) DETECTS it
// via the residue self-check. Single-bit fault masks are used so residue
// detection is deterministic (a 1-bit error is never a multiple of 3).
//
// Size is set at compile time:  iverilog -D NVAL=8   (or 16, 4, ...)
//
// NOTE: this is a FUNCTIONAL test — it proves the mechanism generalizes. It does
// NOT model rail parasitics; the PPA cost of large-N broadcast (the fan-out
// cliff) is a separate physical experiment (ROADMAP.md M2, ASIC P&R).
// =============================================================================
`timescale 1ns/1ps
`default_nettype none
`ifndef NVAL
 `define NVAL 8
`endif

module tb_scaling;
    localparam integer N = `NVAL;
    localparam DATA_W = 16, ACC_W = 32, CFG_W = 10;
    localparam integer NPE = N*N;
    localparam [3:0] OP_MACB = 4'd9;

    reg clk = 0; always #5 clk = ~clk;
    reg rst_n;
    reg                      cfg_broadcast, clr_acc_all, out_en_all;
    reg  [CFG_W-1:0]         cfg_data;
    reg  [$clog2(N)-1:0]     cfg_row_z, cfg_col_z;
    reg  [N*DATA_W-1:0]      ext_in_west, ext_in_north;
    reg  [NPE-1:0]           fault_en_flat;
    reg  [ACC_W-1:0]         fault_mask;
    wire [NPE*ACC_W-1:0]     all_pe_out;
    wire [NPE-1:0]           mac_err_flat;

    sage16_4x4_mac #(.ROWS(N), .COLS(N), .DATA_W(DATA_W), .ACC_W(ACC_W),
                     .CFG_W(CFG_W), .PIPELINE(1)) dut (
        .clk(clk), .rst_n(rst_n), .cfg_pe_row(cfg_row_z), .cfg_pe_col(cfg_col_z),
        .cfg_data(cfg_data), .cfg_load(1'b0), .cfg_broadcast(cfg_broadcast),
        .clr_acc_all(clr_acc_all), .out_en_all(out_en_all),
        .ext_in_west(ext_in_west), .ext_in_north(ext_in_north),
        .per_pe_bypass_en(1'b0), .per_pe_bypass_flat({(NPE*DATA_W){1'b0}}),
        .sram_cs_n_flat({NPE{1'b1}}), .sram_we_n_flat({NPE{1'b1}}),
        .sram_addr_flat({(NPE*8){1'b0}}), .sram_raddr2_flat({(NPE*8){1'b0}}),
        .sram_wdata_sel({NPE{1'b0}}), .sram_wdata_ext_flat({(NPE*32){1'b0}}),
        .sel_src_a_flat({NPE{1'b0}}), .sel_src_b_flat({NPE{1'b0}}),
        .sram_rdata_flat(), .ext_out_east(), .all_pe_out(all_pe_out),
        .rail_err_w_flat(), .rail_err_n_flat(), .mac_err_flat(mac_err_flat)
    );

    reg [NPE-1:0] st_mac; reg capture;
    always @(posedge clk) if (capture) st_mac <= st_mac | mac_err_flat;

    reg [DATA_W-1:0] A [0:NPE-1];
    reg [DATA_W-1:0] B [0:NPE-1];
    reg [ACC_W-1:0]  golden [0:NPE-1];
    integer t, i, j, k, seed, pass, fail, wrong, wnf;
    reg badv;

    task compute_golden;
        reg [ACC_W-1:0] s;
        begin
            for (i=0;i<N;i=i+1) for (j=0;j<N;j=j+1) begin
                s = 0;
                for (k=0;k<N;k=k+1) s = s + A[i*N+k]*B[k*N+j];
                golden[i*N+j] = s;
            end
        end
    endtask

    task run_matmul;
        begin
            st_mac = 0; capture = 1;
            @(negedge clk); cfg_broadcast=1; clr_acc_all=1; cfg_data={OP_MACB,3'd0,3'd0};
            ext_in_west=0; ext_in_north=0; out_en_all=0;
            @(negedge clk); cfg_broadcast=0; clr_acc_all=0; out_en_all=1;
            for (t=0;t<N;t=t+1) begin
                for (i=0;i<N;i=i+1) begin
                    ext_in_west [i*DATA_W +: DATA_W] = A[i*N + t];   // A[i][t] on west rail i
                    ext_in_north[i*DATA_W +: DATA_W] = B[t*N + i];   // B[t][i] on north rail i
                end
                @(negedge clk);
            end
            ext_in_west=0; ext_in_north=0;
            @(negedge clk); @(negedge clk); @(negedge clk);
            out_en_all=0; @(negedge clk); @(negedge clk);
            capture = 0;
        end
    endtask

    task randomize_AB;
        begin for (i=0;i<NPE;i=i+1) begin A[i]=$random(seed); B[i]=$random(seed); end end
    endtask

    task check_clean;
        begin
            fault_en_flat = 0;
            randomize_AB; compute_golden; run_matmul;
            wrong = 0;
            for (i=0;i<NPE;i=i+1)
                if (all_pe_out[i*ACC_W +: ACC_W] !== golden[i]) wrong = wrong + 1;
            if (wrong==0 && st_mac=={NPE{1'b0}}) begin
                pass=pass+1; $display("  PASS  clean matmul : 0 wrong, 0 false mac_err");
            end else begin
                fail=fail+1; $display("  FAIL  clean matmul : wrong=%0d  st_mac!=0", wrong);
            end
        end
    endtask

    // single permanent fault at PE 'pe', single-bit mask -> residue must fire
    task check_single;
        input integer pe;
        begin
            randomize_AB; compute_golden;
            fault_mask = 32'b1 << ({$random(seed)} % 32);
            fault_en_flat = {NPE{1'b0}}; fault_en_flat[pe] = 1'b1;
            run_matmul; fault_en_flat = 0;
            wrong = 0; wnf = 0;
            for (i=0;i<NPE;i=i+1) begin
                badv = (all_pe_out[i*ACC_W +: ACC_W] !== golden[i]);
                if (badv) begin wrong=wrong+1; if (i!=pe) wnf=wnf+1; end
            end
            if (wrong==1 && wnf==0
                && all_pe_out[pe*ACC_W +: ACC_W]!==golden[pe] && st_mac[pe]) begin
                pass=pass+1; $display("  PASS  fault@PE%0d   : contained to 1 output, residue fired", pe);
            end else begin
                fail=fail+1; $display("  FAIL  fault@PE%0d   : wrong=%0d spread=%0d mac=%b",
                                      pe, wrong, wnf, st_mac[pe]);
            end
        end
    endtask

    // two simultaneous permanent faults -> both contained, both detected
    task check_multi;
        input integer p1, p2;
        begin
            randomize_AB; compute_golden;
            fault_mask = 32'b1 << ({$random(seed)} % 32);
            fault_en_flat = {NPE{1'b0}}; fault_en_flat[p1]=1'b1; fault_en_flat[p2]=1'b1;
            run_matmul; fault_en_flat = 0;
            wrong = 0; wnf = 0;
            for (i=0;i<NPE;i=i+1) begin
                badv = (all_pe_out[i*ACC_W +: ACC_W] !== golden[i]);
                if (badv) begin wrong=wrong+1; if (i!=p1 && i!=p2) wnf=wnf+1; end
            end
            if (wrong==2 && wnf==0 && st_mac[p1] && st_mac[p2]) begin
                pass=pass+1; $display("  PASS  fault@PE%0d,%0d: both contained, both detected", p1, p2);
            end else begin
                fail=fail+1; $display("  FAIL  fault@PE%0d,%0d: wrong=%0d spread=%0d mac=%b%b",
                                      p1, p2, wrong, wnf, st_mac[p1], st_mac[p2]);
            end
        end
    endtask

    initial begin
        seed = 32'h5A6E16 + N;
        rst_n=0; fault_en_flat=0; fault_mask=32'h1; cfg_row_z=0; cfg_col_z=0;
        cfg_broadcast=0; clr_acc_all=0; out_en_all=0; cfg_data=0;
        ext_in_west=0; ext_in_north=0; capture=0; st_mac=0;
        repeat (3) @(negedge clk); rst_n=1;
        pass=0; fail=0;

        $display("=================================================================");
        $display(" SCALING TEST — %0d x %0d fabric (%0d PEs)", N, N, NPE);
        $display("=================================================================");

        check_clean;
        check_single(0);            // top-left corner
        check_single(NPE-1);        // bottom-right corner
        check_single(NPE/2 + N/2);  // ~center
        check_single(N);            // start of row 1
        check_multi(1, NPE-2);      // two distant PEs
        check_multi(0, N+1);        // two in different rows/cols

        $display("-----------------------------------------------------------------");
        $display(" %0dx%0d RESULT: %0d passed, %0d failed", N, N, pass, fail);
        if (fail==0) $display(" ALL PASS — containment + detection hold at %0dx%0d", N, N);
        else         $display(" FAIL");
        $display("=================================================================");
        #20 $finish;
    end

    initial begin #50000000; $display("TIMEOUT"); $finish; end

    // TB-side permanent PE-fault injector (single-bit XOR; DUT datapath clean)
    genvar fipe;
    generate for (fipe=0; fipe<NPE; fipe=fipe+1) begin : FINJ
        reg [ACC_W-1:0] fi_clean;
        always @(posedge clk) begin
            release dut.rg[fipe/N].cg[fipe%N].u_pe.out;
            if (fault_en_flat[fipe] && out_en_all && !clr_acc_all) begin
                #1 fi_clean = dut.rg[fipe/N].cg[fipe%N].u_pe.out;
                force dut.rg[fipe/N].cg[fipe%N].u_pe.out = fi_clean ^ fault_mask;
            end
        end
    end endgenerate

endmodule

`default_nettype wire
