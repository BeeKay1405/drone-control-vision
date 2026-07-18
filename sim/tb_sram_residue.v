// =============================================================================
// tb_sram_residue.v — one-cycle detection of a STORED-WORD fault via the SRAM
//                     residue tag (the "residue carried in memory" extension).
//
// Each SRAM word now carries a 2-bit mod-3 tag, written from mod3(wdata). When
// the word is consumed as a MAC operand, the PE predicts from this trusted tag
// while the multiplier sees the actual (possibly corrupted) value -> a stored
// bit-flip shows up as a residue mismatch (mac_err) the cycle it is used, with
// no separate checker and no extra reducer.
//
// Test: write 10 -> SRAM[0]; clean MACB 10*5 = 50 (mac_err must stay 0); then
// backdoor-corrupt the DATA cell 10 -> 11 (tag still mod3(10)=1) and repeat —
// mac_err must fire (the stored fault is caught in-cycle).
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module tb_sram_residue;
    localparam DATA_W=16, ACC_W=32, CFG_W=10, NUM_PE=16, SRAM_AW=8, SRAM_DW=32;

    reg clk=0; always #5 clk=~clk;
    reg rst_n=0;
    reg [1:0]                cfg_pe_row=0, cfg_pe_col=0;
    reg [CFG_W-1:0]          cfg_data=0;
    reg                      cfg_load=0, cfg_broadcast=0, clr_acc_all=0, out_en_all=0;
    reg [4*DATA_W-1:0]       ext_in_west=0, ext_in_north=0;
    reg                      per_pe_bypass_en=0;
    reg [NUM_PE*DATA_W-1:0]  per_pe_bypass_flat=0;
    reg [NUM_PE-1:0]         sram_cs_n={NUM_PE{1'b1}}, sram_we_n={NUM_PE{1'b1}};
    reg [NUM_PE*SRAM_AW-1:0] sram_addr=0, sram_raddr2=0;
    reg [NUM_PE-1:0]         sram_wdata_sel=0;
    reg [NUM_PE*SRAM_DW-1:0] sram_wdata_ext=0;
    reg [NUM_PE-1:0]         sel_src_a=0, sel_src_b=0;
    wire [NUM_PE*SRAM_DW-1:0] sram_rdata;
    wire [4*ACC_W-1:0]        ext_out_east;
    wire [NUM_PE*ACC_W-1:0]   all_pe_out;
    wire [NUM_PE-1:0]         mac_err_flat;

    sage16_4x4_mac dut (
        .clk(clk), .rst_n(rst_n),
        .cfg_pe_row(cfg_pe_row), .cfg_pe_col(cfg_pe_col),
        .cfg_data(cfg_data), .cfg_load(cfg_load), .cfg_broadcast(cfg_broadcast),
        .clr_acc_all(clr_acc_all), .out_en_all(out_en_all),
        .ext_in_west(ext_in_west), .ext_in_north(ext_in_north),
        .per_pe_bypass_en(per_pe_bypass_en), .per_pe_bypass_flat(per_pe_bypass_flat),
        .sram_cs_n_flat(sram_cs_n), .sram_we_n_flat(sram_we_n),
        .sram_addr_flat(sram_addr), .sram_wdata_sel(sram_wdata_sel),
        .sram_wdata_ext_flat(sram_wdata_ext),
        .sel_src_a_flat(sel_src_a), .sel_src_b_flat(sel_src_b),
        .sram_raddr2_flat(sram_raddr2),
        // (rail-fault hook removed from the fabric)
        .sram_rdata_flat(sram_rdata), .ext_out_east(ext_out_east), .all_pe_out(all_pe_out),
        .rail_err_w_flat(), .rail_err_n_flat(), .mac_err_flat(mac_err_flat)
    );

    wire [ACC_W-1:0] pe00 = all_pe_out[0 +: ACC_W];

    // sticky mac_err capture for PE0 (driven only from the always block)
    reg cap=0, clr_st=0, st_err=0;
    always @(posedge clk) begin
        if      (clr_st) st_err <= 1'b0;
        else if (cap)    st_err <= st_err | mac_err_flat[0];
    end

    integer pass=0, fail=0;

    task config_macb; begin
        @(negedge clk); cfg_broadcast=1; cfg_data={4'd9,3'd0,3'd0}; clr_acc_all=1;
        @(negedge clk); cfg_broadcast=0; clr_acc_all=0;
    end endtask

    task clr_acc; begin
        @(negedge clk); clr_acc_all=1; @(negedge clk); clr_acc_all=0;
    end endtask

    // one MACB with SRAM[a0] as operand A and north=bval as operand B; same
    // pipeline timing as tb_pe_sram (one accumulate), with a sticky mac_err window.
    task do_sram_mac;
        input [7:0] a0; input [DATA_W-1:0] bval;
        begin
            clr_st=1; cap=0; @(negedge clk); clr_st=0; cap=1;
            sram_raddr2[0 +: SRAM_AW]=a0; @(negedge clk);
            sel_src_a[0]=1; ext_in_north[0 +: DATA_W]=bval; out_en_all=1; @(negedge clk);
            @(negedge clk); @(negedge clk);
            sel_src_a[0]=0; ext_in_north=0; out_en_all=0;
            @(negedge clk); @(negedge clk); @(negedge clk); @(negedge clk);
            cap=0;
        end
    endtask

    // write PE0's accumulator to SRAM[a]; tag is the CARRIED pe_res_q (no mod3 recompute)
    task pe_write; input [7:0] a; begin
        @(negedge clk);
        sram_cs_n[0]=0; sram_we_n[0]=0; sram_addr[0 +: SRAM_AW]=a; sram_wdata_sel[0]=0;
        @(negedge clk);
        sram_cs_n[0]=1; sram_we_n[0]=1; sram_wdata_sel=0;
    end endtask

    initial begin
        rst_n=0; repeat(3) @(negedge clk); rst_n=1; @(negedge clk);
        config_macb;

        $display("=========================================================");
        $display(" SRAM RESIDUE TAG — one-cycle detection of a stored-word fault");
        $display("=========================================================");

        // write 10 -> SRAM[0] of PE0 (external source); tag = mod3(10) = 1
        sram_cs_n[0]=0; sram_we_n[0]=0; sram_addr[0 +: SRAM_AW]=0;
        sram_wdata_sel[0]=1; sram_wdata_ext[0 +: SRAM_DW]=32'd10;
        @(negedge clk); sram_cs_n[0]=1; sram_we_n[0]=1; sram_wdata_sel=0;

        // clean MAC: 10 * 5 = 50, no fault expected
        clr_acc; do_sram_mac(8'd0, 16'd5);
        if (pe00===32'd50 && st_err===1'b0) begin
            $display("  CLEAN : acc=%0d  mac_err=%0b   -> OK (no false alarm)", pe00, st_err);
            pass=pass+1;
        end else begin
            $display("  CLEAN : acc=%0d (exp 50) mac_err=%0b (exp 0) -> FAIL", pe00, st_err);
            fail=fail+1;
        end

        // corrupt the STORED DATA cell, leaving the tag intact (stuck/flipped bit)
        $display("  inject: corrupt SRAM[0] data 10 -> 11 (tag still mod3(10)=1)");
        dut.rg[0].cg[0].u_mem.mem[0] = 32'd11;

        // corrupted MAC: operand is now 11 but the tag says residue-of-10 -> mismatch
        clr_acc; do_sram_mac(8'd0, 16'd5);
        if (st_err===1'b1) begin
            $display("  FAULT : mac_err=%0b   -> OK (stored-word fault caught in-cycle)", st_err);
            pass=pass+1;
        end else begin
            $display("  FAULT : mac_err=%0b (exp 1) -> FAIL (escaped!)", st_err);
            fail=fail+1;
        end

        // ---- Phase 3: PE-accumulator write carries pe_res_q as the tag ----
        $display("  --- PE-write tag carry (no mod3 recompute on the write path) ---");
        pe_write(8'd2);                       // store PE0.out to SRAM[2] with carried tag
        clr_acc; do_sram_mac(8'd2, 16'd1);    // read it back as operand A
        if (st_err===1'b0) begin
            $display("  PE-WRITE clean : mac_err=0 -> OK (carried tag matches stored data)");
            pass=pass+1;
        end else begin
            $display("  PE-WRITE clean : mac_err=%0b -> FAIL (carried tag wrong/misaligned)", st_err);
            fail=fail+1;
        end
        dut.rg[0].cg[0].u_mem.mem[2] = dut.rg[0].cg[0].u_mem.mem[2] + 32'd1;  // +1 flips mod3
        clr_acc; do_sram_mac(8'd2, 16'd1);
        if (st_err===1'b1) begin
            $display("  PE-WRITE fault : mac_err=1 -> OK (carried-tag mismatch caught)");
            pass=pass+1;
        end else begin
            $display("  PE-WRITE fault : mac_err=%0b -> FAIL (escaped)", st_err);
            fail=fail+1;
        end

        $display("---------------------------------------------------------");
        $display(" SUMMARY: pass=%0d fail=%0d / %0d", pass, fail, pass+fail);
        if (fail==0) $display(" RESULT: PASS - SRAM residue tag gives one-cycle stored-word detection.");
        else         $display(" RESULT: FAIL");
        $display("=========================================================");
        #20 $finish;
    end

    initial begin #500000; $display("TIMEOUT"); $finish; end
endmodule

`default_nettype wire
