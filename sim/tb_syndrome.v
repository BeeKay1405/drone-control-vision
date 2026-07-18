// =============================================================================
// tb_syndrome.v — end-to-end mod-3 protection: fault SYNDROME classification
//
// The fabric now carries residues with the broadcast rails (transport check at
// every PE tap) and residue-verifies every MAC inside the PE (compute check).
// A fault therefore produces a SYNDROME that both CLASSIFIES and LOCATES it,
// with no diagnosis logic:
//
//   syndrome observed                  -> classification
//   ------------------------------------------------------------------
//   mac_err at exactly one PE k        -> PE-internal fault at PE k
//   rail_err_w at all taps of row r    -> west (A-operand) rail fault, row r
//   rail_err_n at all taps of col c    -> north (B-operand) rail fault, col c
//   no flags                           -> healthy
//
// Sweep: 1 clean + 16 PE faults + 4 west-rail faults + 4 north-rail faults
// = 25 runs. Each must classify AND locate correctly.
//
// Also verifies two industrial details:
//   * a rail fault must NOT raise mac_err (the PE computes correctly on bad
//     data — transport and compute checks are disjoint by construction);
//   * accumulator wraparound (big values) must NOT false-alarm — proves the
//     end-around carry correction (2^32 == 1 mod 3) is right.
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module tb_syndrome;
    localparam DATA_W = 16, ACC_W = 32, CFG_W = 10, NUM_PE = 16;
    localparam [3:0] OP_MACB = 4'd9;

    reg clk = 0; always #5 clk = ~clk;
    reg rst_n;
    reg                      cfg_broadcast, clr_acc_all, out_en_all;
    reg  [CFG_W-1:0]         cfg_data;
    reg  [4*DATA_W-1:0]      ext_in_west, ext_in_north;
    reg  [NUM_PE-1:0]        fault_en_flat;
    reg  [ACC_W-1:0]         fault_xor;
    reg  [3:0]               rail_fault_w_en, rail_fault_n_en;
    reg  [DATA_W-1:0]        rail_fault_xor;
    wire [NUM_PE*ACC_W-1:0]  all_pe_out;
    wire [NUM_PE-1:0]        rail_err_w_flat, rail_err_n_flat, mac_err_flat;

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
        // rail faults injected from the TB (force/release) - no DUT hook
        .sram_rdata_flat(), .ext_out_east(), .all_pe_out(all_pe_out),
        .rail_err_w_flat(rail_err_w_flat), .rail_err_n_flat(rail_err_n_flat),
        .mac_err_flat(mac_err_flat)
    );

    // ---- test matrices ----
    reg [DATA_W-1:0] A [0:15];
    reg [DATA_W-1:0] B [0:15];
    integer t, i, k;
    function [DATA_W-1:0] ae; input [1:0] ii,kk; ae = A[ii*4+kk]; endfunction
    function [DATA_W-1:0] be; input [1:0] kk,jj; be = B[kk*4+jj]; endfunction

    // ---- sticky syndrome capture (OR of flags over the whole run) ----
    reg [NUM_PE-1:0] st_rail_w, st_rail_n, st_mac;
    reg capture;
    always @(posedge clk) if (capture) begin
        st_rail_w <= st_rail_w | rail_err_w_flat;
        st_rail_n <= st_rail_n | rail_err_n_flat;
        st_mac    <= st_mac    | mac_err_flat;
    end

    task run_matmul;
        begin
            st_rail_w = 0; st_rail_n = 0; st_mac = 0; capture = 1;
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

    // ---- syndrome decoder (what a sequencer/host-free FSM would implement) ----
    // returns: {2'b00,4'dx}=healthy  {2'b01,PE k}  {2'b10,row r}  {2'b11,col c}
    function [5:0] decode;
        input [NUM_PE-1:0] sw, sn, sm;
        integer ii, cnt, loc;
        begin
            decode = 6'b00_0000;
            // PE fault: mac_err set, rail flags clean
            cnt = 0; loc = 0;
            for (ii=0; ii<NUM_PE; ii=ii+1) if (sm[ii]) begin cnt=cnt+1; loc=ii; end
            if (cnt == 1 && sw == 0 && sn == 0) decode = {2'b01, loc[3:0]};
            // west rail fault: rail_err_w across one full row
            for (ii=0; ii<4; ii=ii+1)
                if (sw[ii*4 +: 4] == 4'hF && sm == 0) decode = {2'b10, ii[3:0]};
            // north rail fault: rail_err_n at one tap per row, same column
            for (ii=0; ii<4; ii=ii+1)
                if (sn[0+ii] && sn[4+ii] && sn[8+ii] && sn[12+ii] && sm == 0)
                    decode = {2'b11, ii[3:0]};
        end
    endfunction

    reg [ACC_W-1:0] golden [0:15];
    integer wrong, pass_cnt, fail_cnt;
    reg [5:0] syn;

    task check_case;
        input [5:0] expect_syn;
        input [143:0] name;
        input integer idx;
        begin
            syn = decode(st_rail_w, st_rail_n, st_mac);
            if (syn === expect_syn) begin
                pass_cnt = pass_cnt + 1;
                $display("  %0s %2d : syndrome %b_%0d -> correct",
                         name, idx, syn[5:4], syn[3:0]);
            end else begin
                fail_cnt = fail_cnt + 1;
                $display("  %0s %2d : syndrome %b_%0d EXPECTED %b_%0d  (w=%h n=%h m=%h) -> FAIL",
                         name, idx, syn[5:4], syn[3:0], expect_syn[5:4], expect_syn[3:0],
                         st_rail_w, st_rail_n, st_mac);
            end
        end
    endtask

    initial begin
        for (i=0;i<16;i=i+1) begin A[i]=i+1; B[i]=(i%7)+1; end
        rst_n=0; fault_en_flat=0; fault_xor=32'h1;
        rail_fault_w_en=0; rail_fault_n_en=0; rail_fault_xor=16'h0001;
        cfg_broadcast=0; clr_acc_all=0; out_en_all=0; cfg_data=0;
        ext_in_west=0; ext_in_north=0; capture=0;
        st_rail_w=0; st_rail_n=0; st_mac=0;
        pass_cnt=0; fail_cnt=0;
        repeat (3) @(negedge clk); rst_n=1;

        $display("=================================================================");
        $display(" FAULT SYNDROME — end-to-end mod-3: classify AND locate, no logic");
        $display("=================================================================");

        // ---- clean run (also captures golden) ----
        run_matmul;
        for (i=0;i<16;i=i+1) golden[i]=all_pe_out[i*ACC_W +: ACC_W];
        check_case(6'b00_0000, "clean", 0);

        // ---- wraparound stress: big values, still must be silent ----
        for (i=0;i<16;i=i+1) begin A[i]=16'hFFFF; B[i]=16'hFFFF; end
        run_matmul;
        check_case(6'b00_0000, "clean wraparound", 0);
        for (i=0;i<16;i=i+1) begin A[i]=i+1; B[i]=(i%7)+1; end

        // ---- 16 PE-internal faults ----
        $display(" --- PE-internal faults (expect: single mac_err, no rail flags) ---");
        for (k=0;k<16;k=k+1) begin
            fault_en_flat=0; fault_en_flat[k]=1'b1;
            run_matmul;
            fault_en_flat=0;
            check_case({2'b01, k[3:0]}, "PE fault", k);
        end

        // ---- 4 west (A-operand) rail faults ----
        $display(" --- west rail faults (expect: rail_err_w across row, NO mac_err) ---");
        for (k=0;k<4;k=k+1) begin
            rail_fault_w_en=0; rail_fault_w_en[k]=1'b1;
            run_matmul;
            rail_fault_w_en=0;
            // outputs of that row must actually be wrong (the fault is real)
            wrong = 0;
            for (i=0;i<16;i=i+1)
                if (all_pe_out[i*ACC_W +: ACC_W] !== golden[i]) wrong = wrong + 1;
            if (wrong == 0) begin
                fail_cnt = fail_cnt + 1;
                $display("  west rail %0d : fault had NO effect on outputs -> FAIL", k);
            end
            check_case({2'b10, k[3:0]}, "west rail", k);
        end

        // ---- 4 north (B-operand) rail faults ----
        $display(" --- north rail faults (expect: rail_err_n down column, NO mac_err) ---");
        for (k=0;k<4;k=k+1) begin
            rail_fault_n_en=0; rail_fault_n_en[k]=1'b1;
            run_matmul;
            rail_fault_n_en=0;
            check_case({2'b11, k[3:0]}, "north rail", k);
        end

        $display("-----------------------------------------------------------------");
        $display(" SYNDROME SUMMARY: %0d pass / %0d fail (of %0d cases)",
                 pass_cnt, fail_cnt, pass_cnt+fail_cnt);
        if (fail_cnt==0) begin
            $display(" RESULT: PASS — every fault classified AND located by its syndrome:");
            $display("   single mac_err  -> PE fault, index = which PE");
            $display("   rail_err row    -> A-rail fault, pattern = which row");
            $display("   rail_err column -> B-rail fault, pattern = which column");
            $display("   transport vs compute checks are disjoint (no cross-trips)");
            $display("   wraparound-safe (end-around carry correction verified)");
        end else
            $display(" RESULT: FAIL");
        $display("=================================================================");
        #20 $finish;
    end

    initial begin #500000; $display("TIMEOUT"); $finish; end
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

    // ---- TB-SIDE rail (wire-defect) injector: force/release the rail wires ----
    // Source residue is computed from the clean input, so corrupting the rail
    // wire makes the tap residue disagree -> rail_err fires (same as old hook).
    // Updated at negedge (+1) so the corrupted value is stable at the posedge
    // the fabric samples its tap residue.
    genvar rwi;
    generate for (rwi=0; rwi<4; rwi=rwi+1) begin : RWFI
        always @(negedge clk) begin
            #1;
            if (rail_fault_w_en[rwi])
                force dut.west_rail[rwi] = ext_in_west[rwi*DATA_W +: DATA_W] ^ rail_fault_xor;
            else
                release dut.west_rail[rwi];
        end
    end endgenerate
    genvar rni;
    generate for (rni=0; rni<4; rni=rni+1) begin : RNFI
        always @(negedge clk) begin
            #1;
            if (rail_fault_n_en[rni])
                force dut.north_rail[rni] = ext_in_north[rni*DATA_W +: DATA_W] ^ rail_fault_xor;
            else
                release dut.north_rail[rni];
        end
    end endgenerate

endmodule

`default_nettype wire
