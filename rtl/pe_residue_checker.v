// =============================================================================
// pe_residue_checker.v  —  per-PE runtime self-check (mod-3 residue code)
//
// Detects an arithmetic error in a PE's MAC THE CYCLE IT HAPPENS — without
// waiting for the output, without a golden reference, without a checksum over
// the array. Each PE checks its OWN result against the residue of its operands.
//
// Invariant (for result = a*b + c_in):
//     result mod 3  ==  ( (a mod 3)*(b mod 3) + (c_in mod 3) ) mod 3
// If a permanent fault corrupts the stored result (e.g. a flipped bit), its
// residue no longer matches the expected residue -> `err` asserts immediately.
//
// Why this composes with our broadcast fabric: a raised flag is instantly
// actionable because the fault is already contained to this PE's one output
// (Pillar 1) and this PE's identity is known for free (output idx = PE idx).
// In a systolic array the flag is less useful — the bad value already streamed
// downstream before the flag mattered.
//
// COST: a few mod-3 reducers + one small multiply/compare per PE. The `% 3`
// below is written for clarity; a production version uses the carry-save
// chunk-sum identity (4 == 1 mod 3 -> sum the 2-bit groups), which is a handful
// of LUTs and no divider.
//
// COVERAGE: mod-3 catches any error whose magnitude is not a multiple of 3
// (incl. all single-bit flips of weight not equal to a multiple of 3). For
// fuller coverage use two coprime moduli (e.g. mod 3 and mod 7).
// =============================================================================
`default_nettype none

module pe_residue_checker #(
    parameter DATA_W = 16,
    parameter ACC_W  = 32
)(
    input  wire [DATA_W-1:0] a,        // multiplier operand A (this cycle)
    input  wire [DATA_W-1:0] b,        // multiplier operand B (this cycle)
    input  wire [ACC_W-1:0]  c_in,     // accumulator-in (in_self)
    input  wire [ACC_W-1:0]  result,   // the PE's stored MAC result to verify
    input  wire              check_en, // 1 = perform the check this cycle
    output wire              err       // 1 = residue mismatch (fault detected)
);
    // mod-3 of each quantity (see header: real impl = chunk-sum, not divider)
    wire [1:0] ra = a      % 3;
    wire [1:0] rb = b      % 3;
    wire [1:0] rc = c_in   % 3;
    wire [1:0] rr = result % 3;

    wire [3:0] prod_res = ra * rb;              // up to 2*2 = 4
    wire [1:0] expected = (prod_res + rc) % 3;

    assign err = check_en & (rr != expected);
endmodule

`default_nettype wire
