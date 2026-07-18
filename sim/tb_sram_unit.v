// =============================================================================
// tb_sram_unit.v  —  unit test for the 256x32 1RW SRAM behavioural model
//
// Checks:
//   T1. Write a value, read it back next cycle.
//   T2. Multiple writes, read back in different order.
//   T3. cs_n=1 holds rdata (no spurious accesses).
//   T4. Sequential walk: write addr=i with data=i*7+1, read all back, verify.
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module tb_sram_unit;

    reg          clk = 0;
    always #5 clk = ~clk;

    reg          cs_n = 1, we_n = 1;
    reg  [7:0]   addr = 0;
    reg  [31:0]  wdata = 0;
    wire [31:0]  rdata;

    wire [31:0] rdata2_unused;
    sram_1rw_256x32 dut (
        .clk(clk), .cs_n(cs_n), .we_n(we_n),
        .addr(addr), .wdata(wdata), .rdata(rdata),
        .raddr2(8'd0), .rdata2(rdata2_unused),   // port B unused in this unit test
        .wtag(2'b0), .rtag(), .rtag2()
    );

    integer pass_cnt = 0, fail_cnt = 0;

    task check;
        input [31:0] got, exp;
        input [63:0] tag;
        begin
            if (got === exp) begin
                $display("  PASS T%0d: rdata=0x%08x", tag, got);
                pass_cnt = pass_cnt + 1;
            end else begin
                $display("  FAIL T%0d: rdata=0x%08x expected=0x%08x", tag, got, exp);
                fail_cnt = fail_cnt + 1;
            end
        end
    endtask

    integer i;
    initial begin
        @(negedge clk);
        // T1: write 0xCAFEBABE to addr 0x10, read back
        $display("\n--- T1: write/read single ---");
        cs_n = 0; we_n = 0; addr = 8'h10; wdata = 32'hCAFEBABE;
        @(negedge clk);
        cs_n = 0; we_n = 1; addr = 8'h10;            // issue read
        @(negedge clk);
        cs_n = 1; we_n = 1;                          // rdata valid this cycle
        #1; check(rdata, 32'hCAFEBABE, 1);

        // T2: write two different addrs, read back out of order
        $display("\n--- T2: multi write/read ---");
        @(negedge clk); cs_n=0; we_n=0; addr=8'h20; wdata=32'h11112222;
        @(negedge clk); cs_n=0; we_n=0; addr=8'h30; wdata=32'h33334444;
        @(negedge clk); cs_n=0; we_n=1; addr=8'h30;   // read 0x30 first
        @(negedge clk); cs_n=0; we_n=1; addr=8'h20;   // read 0x20
        #1; check(rdata, 32'h33334444, 2);
        @(negedge clk); cs_n=1;
        #1; check(rdata, 32'h11112222, 2);

        // T3: deselect — rdata should hold
        $display("\n--- T3: cs_n holds rdata ---");
        @(negedge clk); cs_n=1; addr=8'h00;          // garbage addr
        @(negedge clk);
        #1; check(rdata, 32'h11112222, 3);           // still last value

        // T4: walk the full 256-deep array
        $display("\n--- T4: walk 256 addrs ---");
        for (i = 0; i < 256; i = i + 1) begin
            @(negedge clk);
            cs_n = 0; we_n = 0; addr = i[7:0]; wdata = i * 32'h00000007 + 32'd1;
        end
        // read every address back
        for (i = 0; i < 256; i = i + 1) begin
            @(negedge clk);
            cs_n = 0; we_n = 1; addr = i[7:0];
            if (i > 0) begin
                #1;
                if (rdata !== ((i-1) * 32'h00000007 + 32'd1)) begin
                    $display("  FAIL T4 @ %0d: rdata=0x%08x exp=0x%08x",
                              i-1, rdata, (i-1)*32'h7 + 32'd1);
                    fail_cnt = fail_cnt + 1;
                end else begin
                    pass_cnt = pass_cnt + 1;
                end
            end
        end
        @(negedge clk); cs_n = 1;
        #1;
        if (rdata === (255 * 32'h00000007 + 32'd1)) pass_cnt = pass_cnt + 1;
        else fail_cnt = fail_cnt + 1;
        $display("  T4 walk: %0d entries verified", 256);

        $display("\n=== SRAM UNIT SUMMARY: pass=%0d fail=%0d / %0d ===",
                 pass_cnt, fail_cnt, pass_cnt + fail_cnt);
        if (fail_cnt == 0) $display("ALL PASS"); else $display("FAILURES");
        #10 $finish;
    end

    initial begin #50000; $display("TIMEOUT"); $finish; end
endmodule

`default_nettype wire
