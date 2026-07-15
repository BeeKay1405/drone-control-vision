// =============================================================================
// mod7_reduce.v — synthesizable mod-7 residue of a W-bit value (no divider)
//
// Identity: 8 == 1 (mod 7), so 8^k == 1 — the sum of a value's 3-bit digits has
// the same residue mod 7 as the value itself. Sum the 3-bit chunks, then fold
// the small sum down the same way until it fits in 3 bits, and final-correct
// (7 -> 0). Pure combinational; a handful of small adders.
//
// This is the mod-7 sibling of mod3_reduce.v (which uses 2-bit chunks, 4==1
// mod 3). Used as the optional SECOND modulus in the PE residue self-check:
// together mod-3 + mod-7 escape only when the error is a multiple of 21, so
// the in-cycle multi-bit detection rises from ~66% to ~95%.
// =============================================================================
`default_nettype none

module mod7_reduce #(
    parameter W = 32
)(
    input  wire [W-1:0] x,
    output wire [2:0]   r
);
    integer i;
    reg [11:0] s;     // sum of 3-bit chunks: up to 11 chunks * 7 + tail < 4096
    reg [6:0]  t;
    reg [2:0]  rr;

    always @(*) begin
        // stage 1: sum all full 3-bit chunks (each weight 8^k == 1 mod 7)
        s = 12'd0;
        for (i = 0; i + 3 <= W; i = i + 3)
            s = s + x[i +: 3];
        // tail: top (W mod 3) bits sit at a chunk position k -> weight 8^k == 1
        if (W % 3 != 0)
            s = s + (x >> (W - (W % 3)));

        // stage 2..3: fold the small sum back into 3-bit chunks
        t = s[2:0] + s[5:3] + s[8:6] + s[11:9];      // <= 4*7 = 28
        t = {4'b0, t[2:0]} + {4'b0, t[5:3]};          // <= 7 + 3 = 10
        t = {4'b0, t[2:0]} + {4'b0, t[5:3]};          // <= 7 + 1 = 8

        // final correction: t is in 0..8 -> map {7,8} to {0,1}
        rr = (t >= 7'd7) ? (t - 7'd7) : t[2:0];
    end

    assign r = rr;
endmodule

`default_nettype wire
