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
    output wire [((COLS <= 1) ? 1 : $clog2(COLS))-1:0] loc, // which column
    output wire                  used_fallback,// 0 = residue gave loc, 1 = checksums did
    output wire                  ambig,        // checksum loc ambiguous (see header)
    output wire [ACC_W-1:0]      err_val,      // e
    output wire [ACC_W-1:0]      c_fixed       // corrected value for column loc
);
    localparam LOC_W   = (COLS <= 1) ? 1 : $clog2(COLS);
    localparam COUNT_W = (COLS <= 1) ? 1 : $clog2(COLS + 1);

    integer j;
    reg [ACC_W-1:0] sum_plain, sum_weighted;
    reg [COUNT_W-1:0] fast_count, match_count;
    reg [LOC_W-1:0] loc_fast, loc_net, loc_sel;
    reg [ACC_W-1:0] c_at_loc;
    reg fast, unique_m;

    always @(*) begin
        sum_plain    = {ACC_W{1'b0}};
        sum_weighted = {ACC_W{1'b0}};
        fast_count   = {COUNT_W{1'b0}};
        match_count  = {COUNT_W{1'b0}};
        loc_fast     = {LOC_W{1'b0}};
        loc_net      = {LOC_W{1'b0}};
        c_at_loc     = {ACC_W{1'b0}};

        for (j = 0; j < COLS; j = j + 1) begin
            sum_plain    = sum_plain + c_obs_flat[j*ACC_W +: ACC_W];
            sum_weighted = sum_weighted + (c_obs_flat[j*ACC_W +: ACC_W] << j);

            if (mac_err_row[j]) begin
                fast_count = fast_count + {{(COUNT_W-1){1'b0}}, 1'b1};
                loc_fast   = j[LOC_W-1:0];
            end
        end
    end

    wire [ACC_W-1:0] D0 = sum_plain - cksum;
    wire [ACC_W-1:0] D1 = sum_weighted - cksum_w;

    assign err_present = |D0;
    assign err_val     = D0;

    // ---- fast path: residue flags, one-hot ----
    always @(*) begin
        match_count = {COUNT_W{1'b0}};
        loc_net     = {LOC_W{1'b0}};
        for (j = 0; j < COLS; j = j + 1) begin
            if (D1 == (D0 << j)) begin
                match_count = match_count + {{(COUNT_W-1){1'b0}}, 1'b1};
                loc_net     = j[LOC_W-1:0];
            end
        end
    end

    always @(*) begin
        fast     = (fast_count == {{(COUNT_W-1){1'b0}}, 1'b1});
        unique_m = (match_count == {{(COUNT_W-1){1'b0}}, 1'b1});
        loc_sel  = fast ? loc_fast : loc_net;
        c_at_loc = {ACC_W{1'b0}};
        for (j = 0; j < COLS; j = j + 1)
            if (loc_sel == j[LOC_W-1:0])
                c_at_loc = c_obs_flat[j*ACC_W +: ACC_W];
    end

    assign used_fallback = err_present & ~fast;
    assign located = err_present & (fast | unique_m);
    assign ambig   = err_present & ~fast & ~unique_m;
    assign loc     = loc_sel;
    assign c_fixed = c_at_loc - D0;
endmodule

`default_nettype wire
