// =============================================================================
// tb_residue_coverage.v  —  how good is the runtime residue self-check?
//
// Measures detection coverage of the per-PE check against a permanent-fault
// model that corrupts the stored result by an arbitrary error value e:
//     faulted = correct ^ e   (we sweep e)
//
// Two regimes:
//   (1) ALL single-bit flips (e = 2^p, p=0..31)  -> the dominant defect mode.
//   (2) random multi-bit errors.
//
// Compares mod-3 alone vs mod-3 + mod-7 (two coprime residues). The product
// code mod (3*7=21) escapes only when the error magnitude is a multiple of 21.
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module tb_residue_coverage;
    localparam DATA_W = 16, ACC_W = 32;

    // ---- inline residue helpers (match pe_residue_checker semantics) ----
    function detect3;  // 1 = mod-3 check catches (correct vs faulted) given operands
        input [ACC_W-1:0] correct, faulted;
        detect3 = ((correct % 3) != (faulted % 3));
    endfunction
    function detect7;
        input [ACC_W-1:0] correct, faulted;
        detect7 = ((correct % 7) != (faulted % 7));
    endfunction

    integer p, n, caught3, caught7, caught37, total;
    reg [ACC_W-1:0] correct, e, faulted;
    integer seed;

    initial begin
        seed = 32'hC0FFEE;
        $display("===========================================================");
        $display(" RESIDUE SELF-CHECK COVERAGE (permanent stored-result fault)");
        $display("===========================================================");

        // ---- (1) every single-bit flip on a range of correct values ----
        caught3=0; total=0;
        for (n=0; n<200; n=n+1) begin
            correct = $random(seed);
            for (p=0; p<32; p=p+1) begin
                e = (32'h1 << p);
                faulted = correct ^ e;
                if (detect3(correct, faulted)) caught3 = caught3 + 1;
                total = total + 1;
            end
        end
        $display(" single-bit flips : mod-3 caught %0d / %0d  (%0d%%)",
                 caught3, total, (caught3*100)/total);

        // ---- (2) random multi-bit errors ----
        caught3=0; caught7=0; caught37=0; total=0;
        for (n=0; n<20000; n=n+1) begin
            correct = $random(seed);
            e       = $random(seed);
            if (e == 0) e = 1;              // a fault that changes nothing isn't a fault
            faulted = correct ^ e;
            if (detect3(correct,faulted))                          caught3  = caught3  + 1;
            if (detect7(correct,faulted))                          caught7  = caught7  + 1;
            if (detect3(correct,faulted) || detect7(correct,faulted)) caught37 = caught37 + 1;
            total = total + 1;
        end
        $display(" random multi-bit : mod-3        caught %0d / %0d  (%0d.%0d%%)",
                 caught3, total, (caught3*100)/total, ((caught3*1000)/total)%10);
        $display(" random multi-bit : mod-7        caught %0d / %0d  (%0d.%0d%%)",
                 caught7, total, (caught7*100)/total, ((caught7*1000)/total)%10);
        $display(" random multi-bit : mod-3 + mod-7 caught %0d / %0d  (%0d.%0d%%)",
                 caught37, total, (caught37*100)/total, ((caught37*1000)/total)%10);
        $display("-----------------------------------------------------------");
        $display(" Theory: single-bit flips ALWAYS caught by mod-3 (2^p mod 3 in {1,2}).");
        $display("         dual modulus escapes only if |error| is a multiple of 21.");
        $display("===========================================================");
        $finish;
    end
endmodule

`default_nettype wire
