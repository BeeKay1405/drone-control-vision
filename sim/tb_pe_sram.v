// =============================================================================
// tb_pe_sram.v  —  PE + paired SRAM integration test
//
// Mounts a full sage16_4x4_mac instance and exercises the SRAM lifecycle for
// PE[0,0]:
//   T1. Write 0x0007 to SRAM[0] via external wdata path.
//   T2. Read SRAM[0] (cycle N), assert sel_src_a in cycle N+1, multiply by
//       in_b_col=3 → expect 7*3 = 21 in the accumulator.
//   T3. Write the accumulator (21) back to SRAM[1] via sram_wdata_sel=0
//       (PE accumulator source), read it back, verify.
//
// This is the smallest end-to-end test of the v2 SRAM datapath:
//   external → SRAM → PE operand → MAC → accumulator → SRAM → readback.
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module tb_pe_sram;

    localparam DATA_W  = 16;
    localparam ACC_W   = 32;
    localparam CFG_W   = 10;
    localparam NUM_PE  = 16;
    localparam SRAM_AW = 8;
    localparam SRAM_DW = 32;

    reg clk = 0;
    always #5 clk = ~clk;
    reg rst_n = 0;

    // fabric pins
    reg [1:0]                       cfg_pe_row = 0, cfg_pe_col = 0;
    reg [CFG_W-1:0]                 cfg_data   = 0;
    reg                             cfg_load = 0, cfg_broadcast = 0;
    reg                             clr_acc_all = 0, out_en_all = 0;
    reg [4*DATA_W-1:0]              ext_in_west = 0, ext_in_north = 0;
    reg                             per_pe_bypass_en = 0;
    reg [NUM_PE*DATA_W-1:0]         per_pe_bypass_flat = 0;

    reg [NUM_PE-1:0]                sram_cs_n    = {NUM_PE{1'b1}};
    reg [NUM_PE-1:0]                sram_we_n    = {NUM_PE{1'b1}};
    reg [NUM_PE*SRAM_AW-1:0]        sram_addr    = 0;
    reg [NUM_PE*SRAM_AW-1:0]        sram_raddr2  = 0;   // port B read addr
    reg [NUM_PE-1:0]                sram_wdata_sel = 0;
    reg [NUM_PE*SRAM_DW-1:0]        sram_wdata_ext = 0;
    reg [NUM_PE-1:0]                sel_src_a    = 0;
    reg [NUM_PE-1:0]                sel_src_b    = 0;

    wire [NUM_PE*SRAM_DW-1:0]       sram_rdata;
    wire [4*ACC_W-1:0]              ext_out_east;
    wire [NUM_PE*ACC_W-1:0]         all_pe_out;

    sage16_4x4_mac dut (
        .clk(clk), .rst_n(rst_n),
        .cfg_pe_row(cfg_pe_row), .cfg_pe_col(cfg_pe_col),
        .cfg_data(cfg_data), .cfg_load(cfg_load),
        .cfg_broadcast(cfg_broadcast),
        .clr_acc_all(clr_acc_all), .out_en_all(out_en_all),
        .ext_in_west(ext_in_west), .ext_in_north(ext_in_north),
        .per_pe_bypass_en(per_pe_bypass_en),
        .per_pe_bypass_flat(per_pe_bypass_flat),
        .sram_cs_n_flat(sram_cs_n),
        .sram_we_n_flat(sram_we_n),
        .sram_addr_flat(sram_addr),
        .sram_wdata_sel(sram_wdata_sel),
        .sram_wdata_ext_flat(sram_wdata_ext),
        .sel_src_a_flat(sel_src_a),
        .sel_src_b_flat(sel_src_b),
        .sram_raddr2_flat(sram_raddr2),
        // (rail-fault hook removed from the fabric)
        .sram_rdata_flat(sram_rdata),
        .ext_out_east(ext_out_east),
        .all_pe_out(all_pe_out)
    );

    integer pass_cnt = 0, fail_cnt = 0;
    wire [ACC_W-1:0] pe00_out = all_pe_out[0*ACC_W +: ACC_W];
    wire [SRAM_DW-1:0] pe00_sram_rd = sram_rdata[0*SRAM_DW +: SRAM_DW];

    task check;
        input [31:0] got, exp;
        input [63:0] tag;
        begin
            if (got === exp) begin
                $display("  PASS T%0d: 0x%0x", tag, got);
                pass_cnt = pass_cnt + 1;
            end else begin
                $display("  FAIL T%0d: 0x%0x exp=0x%0x", tag, got, exp);
                fail_cnt = fail_cnt + 1;
            end
        end
    endtask

    initial begin
        rst_n = 0;
        repeat (3) @(negedge clk); rst_n = 1;
        @(negedge clk);

        // Configure PE[0,0] with OP_MACB (opcode 9), sel_a=0 sel_b=0
        cfg_broadcast = 1;
        cfg_data      = {4'd9, 3'd0, 3'd0};
        clr_acc_all   = 1;
        @(negedge clk);
        cfg_broadcast = 0;
        clr_acc_all   = 0;

        // ---------- T1: write 7 to SRAM[0] of PE[0,0] ----------
        $display("\n--- T1: write SRAM[0] = 7 via external wdata ---");
        sram_cs_n[0]                          = 1'b0;
        sram_we_n[0]                          = 1'b0;
        sram_addr[0*SRAM_AW +: SRAM_AW]       = 8'd0;
        sram_wdata_sel[0]                     = 1'b1;            // external
        sram_wdata_ext[0*SRAM_DW +: SRAM_DW]  = 32'd7;
        @(negedge clk);
        sram_cs_n[0]   = 1'b1;
        sram_we_n[0]   = 1'b1;
        sram_wdata_sel = 0;

        // Read SRAM[0] to verify
        sram_cs_n[0]                          = 1'b0;
        sram_we_n[0]                          = 1'b1;
        sram_addr[0*SRAM_AW +: SRAM_AW]       = 8'd0;
        @(negedge clk);
        sram_cs_n[0]   = 1'b1;
        #1; check(pe00_sram_rd, 32'd7, 1);

        // ---------- T2: SRAM[0] → operand A, multiply by in_b_col=3 ----------
        $display("\n--- T2: SRAM[0] -> mul_a (port B), in_b_col=3 -> expect 21 ---");
        // Cycle 1: issue read of SRAM[0] on the dedicated read port (port B)
        sram_raddr2[0*SRAM_AW +: SRAM_AW]     = 8'd0;
        @(negedge clk);
        // Cycle 2: rdata2 (port B) valid; assert sel_src_a, drive in_b_col, latch MAC
        sel_src_a[0]                          = 1'b1;
        ext_in_north[0*DATA_W +: DATA_W]      = 16'd3;    // B-broadcast col 0
        out_en_all                            = 1'b1;
        @(negedge clk);
        // PIPELINE=1 adds 2 drain cycles before the product appears in out
        @(negedge clk);
        @(negedge clk);
        sel_src_a[0] = 0;
        ext_in_north = 0;
        out_en_all   = 0;
        #1; check(pe00_out, 32'd21, 2);

        // ---------- T3: write PE accumulator (21) to SRAM[1], read back ----------
        $display("\n--- T3: write PE acc → SRAM[1], read back ---");
        sram_cs_n[0]                          = 1'b0;
        sram_we_n[0]                          = 1'b0;
        sram_addr[0*SRAM_AW +: SRAM_AW]       = 8'd1;
        sram_wdata_sel[0]                     = 1'b0;     // use PE accumulator
        @(negedge clk);
        sram_cs_n[0]   = 1'b1;
        sram_we_n[0]   = 1'b1;
        sram_wdata_sel = 0;

        // Read SRAM[1] back
        sram_cs_n[0]                          = 1'b0;
        sram_we_n[0]                          = 1'b1;
        sram_addr[0*SRAM_AW +: SRAM_AW]       = 8'd1;
        @(negedge clk);
        sram_cs_n[0] = 1'b1;
        #1; check(pe00_sram_rd, 32'd21, 3);

        $display("\n=== PE+SRAM SUMMARY: pass=%0d fail=%0d / %0d ===",
                 pass_cnt, fail_cnt, pass_cnt + fail_cnt);
        if (fail_cnt == 0) $display("ALL PASS"); else $display("FAILURES");
        #20 $finish;
    end

    initial begin #100000; $display("TIMEOUT"); $finish; end
endmodule

`default_nettype wire
