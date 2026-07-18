// =============================================================================
// tb_residue_check.v  —  proof that the per-PE residue check catches a fault
//                        AT RUNTIME (the cycle it happens), not at output read.
//
// Simulates a 4-tap MAC accumulation (a*b summed) and at one chosen tap corrupts
// the stored result (flip LSB). The residue checker should flag `err` exactly on
// the corrupted tap, while every clean tap reads err=0.
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module tb_residue_check;
    localparam DATA_W = 16, ACC_W = 32;

    reg  [DATA_W-1:0] a, b;
    reg  [ACC_W-1:0]  c_in, result;
    reg               check_en;
    wire              err;

    pe_residue_checker #(.DATA_W(DATA_W), .ACC_W(ACC_W)) chk (
        .a(a), .b(b), .c_in(c_in), .result(result),
        .check_en(check_en), .err(err)
    );

    // operands for a 4-tap inner product: C = sum_t A[t]*B[t]
    reg [DATA_W-1:0] Av [0:3];
    reg [DATA_W-1:0] Bv [0:3];
    integer t, fault_tap, pass, fail;
    reg [ACC_W-1:0] acc;

    task run_with_fault;
        input integer ft;        // tap at which to corrupt the result (-1 = none)
        integer detected_tap;
        begin
            acc = 0; detected_tap = -1;
            check_en = 1;
            for (t = 0; t < 4; t = t+1) begin
                a = Av[t]; b = Bv[t]; c_in = acc;
                result = a*b + c_in;                 // correct MAC
                if (t == ft) result = result ^ 32'h1; // inject a stored-result fault
                #1;                                  // settle combinational checker
                if (err && detected_tap == -1) detected_tap = t;
                acc = result;                        // accumulate (corrupted if faulted)
                #1;
            end
            // report
            if (ft == -1) begin
                if (detected_tap == -1) begin
                    $display("  clean run            : no error flagged  -> OK");
                    pass = pass + 1;
                end else begin
                    $display("  clean run            : FALSE POSITIVE at tap %0d -> FAIL", detected_tap);
                    fail = fail + 1;
                end
            end else begin
                if (detected_tap == ft) begin
                    $display("  fault at tap %0d       : flagged AT tap %0d (same cycle) -> OK",
                             ft, detected_tap);
                    pass = pass + 1;
                end else begin
                    $display("  fault at tap %0d       : flagged at tap %0d (expected %0d) -> FAIL",
                             ft, detected_tap, ft);
                    fail = fail + 1;
                end
            end
        end
    endtask

    initial begin
        Av[0]=7; Av[1]=3; Av[2]=5; Av[3]=9;
        Bv[0]=2; Bv[1]=4; Bv[2]=6; Bv[3]=8;
        pass = 0; fail = 0;

        $display("===========================================================");
        $display(" RUNTIME PE SELF-CHECK (mod-3 residue) — catch fault in-cycle");
        $display("===========================================================");
        run_with_fault(-1);     // clean
        run_with_fault(0);      // fault at each tap
        run_with_fault(1);
        run_with_fault(2);
        run_with_fault(3);
        $display("-----------------------------------------------------------");
        $display(" SUMMARY: pass=%0d fail=%0d / %0d", pass, fail, pass+fail);
        if (fail==0) $display(" RESULT: PASS — runtime detector flags the fault the cycle it occurs.");
        else         $display(" RESULT: FAIL");
        $display("===========================================================");
        $finish;
    end
endmodule

`default_nettype wire
