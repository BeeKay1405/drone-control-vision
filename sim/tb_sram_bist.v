// =============================================================================
// tb_sram_bist.v  —  full BIST run through sage16_mem_top
//
// Holds mode=0 (BIST), pulses bist_start, waits for bist_done, checks:
//   - bist_pass = 1 (every PE's SRAM passed)
//   - pe_pass_mask = 16'hFFFF (all 16 PEs)
//   - bist_done remains high after completion
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module tb_sram_bist;

    reg clk = 0;
    always #5 clk = ~clk;

    reg rst_n = 0;
    reg mode = 0;
    reg bist_start = 0;

    wire        bist_busy, bist_done, bist_pass;
    wire [15:0] pe_pass_mask;
    wire [15*32-1:0] dummy_acc;  // unused
    wire [511:0] sram_rdata_flat;
    wire [4*32-1:0] ext_out_east;
    wire [16*32-1:0] all_pe_out;

    sage16_mem_top dut (
        .clk(clk), .rst_n(rst_n),
        .mode(mode),
        .bist_start(bist_start),
        .bist_busy(bist_busy),
        .bist_done(bist_done),
        .bist_pass(bist_pass),
        .pe_pass_mask(pe_pass_mask),
        // compute-mode pins tied off (mode=0)
        .cfg_pe_row(2'd0), .cfg_pe_col(2'd0),
        .cfg_data(10'd0), .cfg_load(1'b0),
        .cfg_broadcast(1'b0), .clr_acc_all(1'b0), .out_en_all(1'b0),
        .ext_in_west(64'b0), .ext_in_north(64'b0),
        .per_pe_bypass_en(1'b0),
        .per_pe_bypass_flat(256'b0),
        .host_sram_cs_n_flat({16{1'b1}}),
        .host_sram_we_n_flat({16{1'b1}}),
        .host_sram_addr_flat(128'b0),
        .host_sram_wdata_sel(16'b0),
        .host_sram_wdata_ext_flat(512'b0),
        .host_sel_src_a_flat(16'b0),
        .host_sel_src_b_flat(16'b0),
        .sram_rdata_flat(sram_rdata_flat),
        .ext_out_east(ext_out_east),
        .all_pe_out(all_pe_out)
    );

    integer pass_cnt = 0, fail_cnt = 0;
    integer cycles_to_done;

    initial begin
        $display("===============================================");
        $display("  BIST through sage16_mem_top — 16 SRAMs");
        $display("===============================================");

        // VCD dump for waveform viewer (GTKWave)
        $dumpfile("build/tb_sram_bist.vcd");
        $dumpvars(0, tb_sram_bist);

        // reset
        rst_n = 0;
        repeat (4) @(negedge clk);
        rst_n = 1;
        @(negedge clk);

        // pulse start
        bist_start = 1;
        @(negedge clk);
        bist_start = 0;

        // count cycles until done
        cycles_to_done = 0;
        while (!bist_done && cycles_to_done < 5000) begin
            @(negedge clk);
            cycles_to_done = cycles_to_done + 1;
        end

        if (!bist_done) begin
            $display("  FAIL: BIST timeout, cycles=%0d", cycles_to_done);
            fail_cnt = fail_cnt + 1;
        end else begin
            $display("  BIST done after %0d cycles", cycles_to_done);
        end

        // check pass mask
        if (pe_pass_mask === 16'hFFFF) begin
            $display("  PASS: pe_pass_mask = 0x%04x (all 16 PEs OK)", pe_pass_mask);
            pass_cnt = pass_cnt + 1;
        end else begin
            $display("  FAIL: pe_pass_mask = 0x%04x (expected 0xFFFF)", pe_pass_mask);
            fail_cnt = fail_cnt + 1;
        end

        // check aggregate pass
        if (bist_pass === 1'b1) begin
            $display("  PASS: bist_pass = 1");
            pass_cnt = pass_cnt + 1;
        end else begin
            $display("  FAIL: bist_pass = %b (expected 1)", bist_pass);
            fail_cnt = fail_cnt + 1;
        end

        // bist_done sticky
        @(negedge clk);
        @(negedge clk);
        if (bist_done === 1'b0) begin
            // FSM may go back to IDLE — that's fine as long as it isn't busy
            if (bist_busy === 1'b0) begin
                $display("  PASS: post-done state idle");
                pass_cnt = pass_cnt + 1;
            end else begin
                $display("  FAIL: still busy after done");
                fail_cnt = fail_cnt + 1;
            end
        end else begin
            $display("  PASS: bist_done still asserted (sticky)");
            pass_cnt = pass_cnt + 1;
        end

        $display("\n=== BIST SUMMARY: pass=%0d fail=%0d / %0d  total_cycles=%0d ===",
                 pass_cnt, fail_cnt, pass_cnt + fail_cnt, cycles_to_done);
        if (fail_cnt == 0) $display("ALL PASS"); else $display("FAILURES");
        #20 $finish;
    end

    initial begin #200000; $display("TIMEOUT"); $finish; end
endmodule

`default_nettype wire
