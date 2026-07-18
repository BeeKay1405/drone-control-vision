// =============================================================================
// abft_checksum.v — erasure-ABFT checksum PEs: free riders on the broadcast rails
//
// THE IDEA (containment turns errors into erasures):
//   Classical ABFT (Huang-Abraham 1984) needs TWO checksums to locate AND
//   correct one bad value. Our fabric's syndrome (mac_err) already names the
//   faulty PE — location is free. An error with known location is an ERASURE,
//   and t erasures need only t checksums (vs ~2t for unlocated errors).
//
// THE HARDWARE GIFT (broadcast-specific):
//   A checksum PE needs no data routed to it. The row-sum identity:
//       S_i = SUM_j C_ij = SUM_k A_ik * (SUM_j B_kj) = SUM_k A_ik * beta_k
//   beta_k is just the sum of the 4 north rails at tap k — ONE adder, shared
//   by all rows. Each row's checksum PE MACs west_rail[i] * beta_k on the
//   same 4 taps as the real PEs: ZERO extra cycles, ZERO extra memory traffic.
//
// CORRECTION (done by the consumer/sequencer, combinational):
//   1 fault at known column j*:  e = (SUM_j Chat_ij) - S_i ;  C = Chat - e.
//      ONE subtraction. Exact mod 2^ACC_W (wraparound-safe, no caveats).
//   2 faults at known j1<j2: second WEIGHTED checksum S'_i (weights 2^j,
//      beta'_k = SUM_j 2^j * B_kj — a shift-add tree, second rail):
//        D0 = SUM Chat - S  = e1 + e2
//        D1 = SUM 2^j*Chat - S' = 2^j1*e1 + 2^j2*e2
//        e1 = ((2^j2*D0 - D1) >>> j1) * inv(2^(j2-j1)-1)  mod 2^ACC_W
//      inv(1)=1, inv(3)=0xAAAAAAAB, inv(7)=0xB6DB6DB7 (odd -> invertible mod
//      2^32; multiply replaces division — exact, no divider). Exact for
//      |error| < 2^(31-j1); larger errors disambiguated by the mod-3 residue
//      check of the corrected value (<=4 candidates).
//
// Scope: covers PE-internal faults (compute). Rail faults are handled by the
// rail_err syndrome + remap policy, not ABFT — this module taps the rails at
// the same point as the PE array, so its checksums are consistent with what
// the PEs actually received.
//
// Pipeline mirrors pe.v stage-for-stage (operand reg -> product reg -> acc)
// so checksums finish exactly when the PE outputs finish.
// =============================================================================
`default_nettype none

module abft_checksum #(
    parameter ROWS   = 4,
    parameter COLS   = 4,
    parameter DATA_W = 16,
    parameter ACC_W  = 32
)(
    input  wire                       clk, rst_n,
    input  wire                       clr_acc,    // mirror fabric clr_acc_all
    input  wire                       out_en,     // mirror fabric out_en_all
    input  wire [ROWS*DATA_W-1:0]     ext_in_west,
    input  wire [COLS*DATA_W-1:0]     ext_in_north,
    output wire [ROWS*ACC_W-1:0]      cksum_flat,   // S_i  (plain row checksums)
    output wire [ROWS*ACC_W-1:0]      cksum_w_flat  // S'_i (weighted, 2^j)
);
    localparam BETA_W  = DATA_W + ((COLS <= 1) ? 1 : $clog2(COLS));
    localparam BETAW_W = DATA_W + COLS;

    // ---- the checksum rails: one adder (tree) each, shared by all rows ----
    // beta  = SUM_j north_j
    // beta' = SUM_j (north_j << j)
    integer j;
    reg [BETA_W-1:0]  beta;
    reg [BETAW_W-1:0] beta_w;
    reg [BETAW_W-1:0] north_ext;
    always @(*) begin
        beta   = {BETA_W{1'b0}};
        beta_w = {BETAW_W{1'b0}};
        for (j = 0; j < COLS; j = j + 1) begin
            north_ext = {{(BETAW_W-DATA_W){1'b0}}, ext_in_north[j*DATA_W +: DATA_W]};
            beta   = beta   + north_ext[BETA_W-1:0];
            beta_w = beta_w + (north_ext << j);
        end
    end

    // ---- per-row checksum PE: MAC west_i * beta (and * beta'), mod 2^ACC_W ----
    genvar r;
    generate for (r = 0; r < ROWS; r = r+1) begin : crow
        wire [DATA_W-1:0] a_in = ext_in_west[r*DATA_W +: DATA_W];

        // stage 1: operand regs (mirrors pe.v mul_a_q/mul_b_q)
        reg [DATA_W-1:0]  a_q;
        reg [BETA_W-1:0]  b_q;
        reg [BETAW_W-1:0] bw_q;
        always @(posedge clk) begin
            a_q  <= a_in;
            b_q  <= beta;
            bw_q <= beta_w;
        end

        // stage 2: product regs (mirrors mul_prod_q)
        reg [ACC_W-1:0] prod_q, prodw_q;
        always @(posedge clk or negedge rst_n)
            if (!rst_n) begin prod_q <= 0; prodw_q <= 0; end
            else begin
                prod_q  <= a_q * b_q;    // low ACC_W bits (mod 2^32 — exact)
                prodw_q <= a_q * bw_q;
            end

        // stage 3: accumulate (mirrors out)
        reg [ACC_W-1:0] acc, accw;
        always @(posedge clk or negedge rst_n)
            if (!rst_n)       begin acc <= 0; accw <= 0; end
            else if (clr_acc) begin acc <= 0; accw <= 0; end
            else if (out_en)  begin acc <= acc + prod_q; accw <= accw + prodw_q; end

        assign cksum_flat  [r*ACC_W +: ACC_W] = acc;
        assign cksum_w_flat[r*ACC_W +: ACC_W] = accw;
    end endgenerate
endmodule

`default_nettype wire
