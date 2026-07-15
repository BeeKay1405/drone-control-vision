// =============================================================================
// mod3_reduce.v — synthesizable mod-3 residue of a W-bit value (no divider)
//
// Identity: 4 == 1 (mod 3), so the sum of a value's 2-bit digits has the same
// residue as the value itself. Reduce by digit-summing, then fold the small
// sum down with the same identity until it fits in 2 bits.
//
//   W=16: 8 digits, sum <= 24  -> 3 folds -> residue
//   W=32: 16 digits, sum <= 48 -> 3 folds -> residue
//
// Pure combinational; a handful of small adders. This is the production form
// of the `% 3` placeholder in pe_residue_checker.v.
// =============================================================================
`default_nettype none

module mod3_reduce #(
    parameter W = 16
)(
    input  wire [W-1:0] x,
    output wire [1:0]   r
);
    // Continuous-assign via a function so it is guaranteed combinational and
    // evaluates at time 0 / on every change of x. (An always @(*) with a
    // variable-indexed for-loop can fail to evaluate in iverilog for a
    // never-toggling constant input, leaving the output at its X init — which
    // showed up only on idle rails. Synthesis infers identical logic.)
    function [1:0] mod3;
        input [W-1:0] xx;
        integer i;
        reg [7:0] s;
        reg [3:0] t;
        begin
            // stage 1: sum all 2-bit digits (each digit weight 4^k == 1 mod 3)
            s = 8'd0;
            for (i = 0; i + 1 < W; i = i + 2)
                s = s + xx[i +: 2];
            if (W % 2) s = s + xx[W-1];              // odd top bit, weight 1 mod 3
            // stage 2..4: fold down with the same identity
            t = s[1:0] + s[3:2] + s[5:4] + s[7:6];   // <= 12
            t = {2'b00, t[1:0]} + {2'b00, t[3:2]};   // <= 6
            t = {2'b00, t[1:0]} + {2'b00, t[3:2]};   // <= 3
            mod3 = (t[1:0] == 2'd3 && t[3:2] == 2'd0) ? 2'd0 : t[1:0];
        end
    endfunction

    assign r = mod3(x);
endmodule

`default_nettype wire
