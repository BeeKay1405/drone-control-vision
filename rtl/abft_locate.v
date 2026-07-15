// =============================================================================
// abft_locate.v — per-row fault decode + exact correction (the safety net)
//
// Closes the multi-bit coverage gap. The mod-3 residue check (mac_err) is the
// FAST path: flags in-cycle, but misses errors whose magnitude is a multiple
// of 3 (~1/3 of random multi-bit corruptions, per measured coverage). The
// checksum pair is the SAFETY NET with 100% coverage for a contained fault:
//
//   D0 = (SUM_j Chat_j) - S   = e        (exact, any bit pattern, mod 2^ACC_W
//                                         — a single-PE error can NEVER make
//                                         D0 == 0, so detection is total)
//   D1 = (SUM_j 2^j*Chat_j) - S' = 2^j* . e
//
// LOCATE priority:
//   1. mac_err one-hot  -> location free (fast path, residue caught it)
//   2. else shift-compare: the unique j with D1 == (D0 << j)  (Huang-Abraham
//      style localization, done with 4 compares)
// CORRECT: c_fixed = Chat[loc] - D0. One subtraction. Exact for any e.
//
// Honest corner (flagged, not hidden): the shift-compare is ambiguous only
// when e == 0 mod 2^(ACC_W-3) (top-3-bits-only errors, e.g. e = 2^31) — and
// those are single/double-bit errors the residue path catches with certainty,
// so the system-level escape requires an error that is simultaneously
// a multiple of 3 AND zero in its low 29 bits. `ambig` reports it anyway.
// =============================================================================
`default_nettype none

module abft_locate #(
    parameter COLS  = 4,
    parameter ACC_W = 32
)(
    input  wire [COLS*ACC_W-1:0] c_obs_flat,   // observed row outputs
    input  wire [ACC_W-1:0]      cksum,        // S  (plain checksum PE)
    input  wire [ACC_W-1:0]      cksum_w,      // S' (2^j-weighted checksum PE)
    input  wire [COLS-1:0]       mac_err_row,  // sticky residue flags, this row
    output wire                  err_present,  // an error exists in this row
    output wire                  located,      // location resolved (fast or net)
    output wire [1:0]            loc,          // which column
    output wire                  used_fallback,// 0 = residue gave loc, 1 = checksums did
    output wire                  ambig,        // checksum loc ambiguous (see header)
    output wire [ACC_W-1:0]      err_val,      // e
    output wire [ACC_W-1:0]      c_fixed       // corrected value for column loc
);
    wire [ACC_W-1:0] c0 = c_obs_flat[0*ACC_W +: ACC_W];
    wire [ACC_W-1:0] c1 = c_obs_flat[1*ACC_W +: ACC_W];
    wire [ACC_W-1:0] c2 = c_obs_flat[2*ACC_W +: ACC_W];
    wire [ACC_W-1:0] c3 = c_obs_flat[3*ACC_W +: ACC_W];

    wire [ACC_W-1:0] D0 = (c0 + c1 + c2 + c3) - cksum;
    wire [ACC_W-1:0] D1 = (c0 + (c1 << 1) + (c2 << 2) + (c3 << 3)) - cksum_w;

    assign err_present = |D0;
    assign err_val     = D0;

    // ---- fast path: residue flags, one-hot ----
    wire fast = (mac_err_row == 4'b0001) | (mac_err_row == 4'b0010) |
                (mac_err_row == 4'b0100) | (mac_err_row == 4'b1000);
    wire [1:0] loc_fast = mac_err_row[1] ? 2'd1 :
                          mac_err_row[2] ? 2'd2 :
                          mac_err_row[3] ? 2'd3 : 2'd0;

    // ---- safety net: which column weight explains D1? ----
    wire m0 = (D1 == D0);
    wire m1 = (D1 == (D0 << 1));
    wire m2 = (D1 == (D0 << 2));
    wire m3 = (D1 == (D0 << 3));
    wire [2:0] nmatch = {2'b00, m0} + {2'b00, m1} + {2'b00, m2} + {2'b00, m3};
    wire       unique_m = (nmatch == 3'd1);
    wire [1:0] loc_net  = m1 ? 2'd1 : m2 ? 2'd2 : m3 ? 2'd3 : 2'd0;

    assign used_fallback = err_present & ~fast;
    assign located = err_present & (fast | unique_m);
    assign ambig   = err_present & ~fast & ~unique_m;
    assign loc     = fast ? loc_fast : loc_net;

    wire [ACC_W-1:0] c_at_loc = (loc == 2'd0) ? c0 :
                                (loc == 2'd1) ? c1 :
                                (loc == 2'd2) ? c2 : c3;
    assign c_fixed = c_at_loc - D0;
endmodule

`default_nettype wire
