// =============================================================================
// tb_systolic_containment.v  —  the BASELINE half of the containment comparison
//
// Same experiment as tb_fault_containment.v, but on the SYSTOLIC array:
//   1. run a fixed compute, NO fault -> capture all 16 outputs = golden
//   2. for each PE k: re-run with fault_en[k]=1, count how many outputs differ
//
// Expected (systolic): a fault PROPAGATES down its column (partial sums forward
// PE->PE), so a fault near the top corrupts a whole line of outputs.
//   fault at row 0 -> 4 outputs wrong (full column)   ... row 3 -> 1 wrong.
//   max ~ 4 (line error), mean ~ 2.5.
// Contrast with broadcast fabric: every fault = exactly 1 wrong output.
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module tb_systolic_containment;
    localparam N      = 4;
    localparam DATA_W = 16;
    localparam ACC_W  = 32;
    localparam NPE    = N*N;
    localparam [ACC_W-1:0] FAULT_MASK = 32'h0000_0001;

    reg clk = 0; always #5 clk = ~clk;
    reg rst_n, clr, en;
    reg  [NPE*DATA_W-1:0] w_flat;
    reg  [N*DATA_W-1:0]   a_left_flat;
    reg  [NPE-1:0]        fault_en_flat;
    reg  [ACC_W-1:0]      fault_xor;
    wire [NPE*ACC_W-1:0]  c_flat;

    systolic_4x4_mac #(.N(N), .DATA_W(DATA_W), .ACC_W(ACC_W)) dut (
        .clk(clk), .rst_n(rst_n), .clr(clr), .en(en),
        .w_flat(w_flat), .a_left_flat(a_left_flat),
        .fault_en_flat(fault_en_flat), .fault_xor(fault_xor),
        .c_flat(c_flat)
    );

    integer i, k, step;
    reg [ACC_W-1:0] golden [0:NPE-1];
    integer wrong, max_wrong, sum_wrong;

    // run a fixed 8-step compute with constant activations + preloaded weights
    task run_compute;
        begin
            @(negedge clk); clr = 1; en = 0;
            @(negedge clk); clr = 0; en = 1;
            for (step = 0; step < 8; step = step+1) begin
                // constant nonzero activations entering each row from the left
                a_left_flat[0*DATA_W +: DATA_W] = 16'd2;
                a_left_flat[1*DATA_W +: DATA_W] = 16'd3;
                a_left_flat[2*DATA_W +: DATA_W] = 16'd4;
                a_left_flat[3*DATA_W +: DATA_W] = 16'd5;
                @(negedge clk);
            end
            en = 0;
            @(negedge clk);
        end
    endtask

    initial begin
        // weights B[k][j] = small distinct nonzero values
        for (i = 0; i < NPE; i = i+1)
            w_flat[i*DATA_W +: DATA_W] = (i % 5) + 1;
        rst_n = 0; clr = 0; en = 0; fault_en_flat = 0; fault_xor = FAULT_MASK;
        a_left_flat = 0;
        repeat (3) @(negedge clk); rst_n = 1;

        // ---- golden (no fault) ----
        fault_en_flat = 0;
        run_compute;
        for (i = 0; i < NPE; i = i+1) golden[i] = c_flat[i*ACC_W +: ACC_W];

        $display("===========================================================");
        $display(" SYSTOLIC BASELINE — single-PE fault propagation");
        $display(" (partial sums forward PE->PE down each column)");
        $display("===========================================================");

        max_wrong = 0; sum_wrong = 0;
        for (k = 0; k < NPE; k = k+1) begin
            fault_en_flat = 0; fault_en_flat[k] = 1'b1;
            run_compute;
            wrong = 0;
            for (i = 0; i < NPE; i = i+1)
                if (c_flat[i*ACC_W +: ACC_W] !== golden[i]) wrong = wrong + 1;
            $display(" fault PE %2d (row %0d,col %0d) -> %0d output(s) wrong %s",
                     k, k/N, k%N, wrong, (wrong > 1) ? "(LINE/SPREAD)" :
                                          (wrong==1) ? "(1)" : "(masked)");
            if (wrong > max_wrong) max_wrong = wrong;
            sum_wrong = sum_wrong + wrong;
        end
        fault_en_flat = 0;

        $display("-----------------------------------------------------------");
        $display(" SYSTOLIC: max wrong from a single PE fault = %0d", max_wrong);
        $display("           mean wrong over 16 faults        = %0d.%02d",
                 sum_wrong/NPE, (sum_wrong%NPE)*100/NPE);
        $display(" COMPARE — broadcast fabric: max=1, mean=1.00 (contained)");
        if (max_wrong > 1)
            $display(" RESULT: systolic SPREADS faults (line errors) — as expected.");
        else
            $display(" RESULT: unexpected — no spread; check the dataflow model.");
        $display("===========================================================");
        #20 $finish;
    end

    initial begin #200000; $display("TIMEOUT"); $finish; end
endmodule

`default_nettype wire
