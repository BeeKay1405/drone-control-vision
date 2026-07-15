// =============================================================================
// selfval_reduce.v — "one adder does both" : self-validating mean-reduction
//
// THE NOVELTY, IN HARDWARE (checksum-reduction identity):
//   The per-row sum  R[i] = sum_j C[i][j]  is SIMULTANEOUSLY
//     (1) the FUNCTIONAL reduction a DNN layer needs — the per-row mean numerator
//         (LayerNorm-mean over features; the column analog = global-average-pool), and
//     (2) the ACTUAL ABFT checksum that ABFT compares against the input-derived
//         prediction  S_i = sum_k A_ik * beta_k  (produced free on the broadcast
//         rails by abft_checksum).
//   (algebra: sum_j C[i][j] = sum_k A_ik (sum_j B_kj) = S_i)
//
// So this ONE adder tree serves BOTH the reduction AND error detection. And because
// the functional reduction IS the checksum, it is SELF-VALIDATING: a PE fault makes
// R[i] != prediction, so the pooled/normalized output flags the very value it
// corrupted, at zero added checker.
//
// row_reduce_flat is ALWAYS produced (it is the functional output). Only the
// validity comparison is the reliability cost, gated by GEN_CHECK — so GEN_CHECK=0
// gives a plain reduction unit, GEN_CHECK=1 makes that same unit self-checking.
//
// Scope (honest): the identity holds for a reduction taken DIRECTLY on a linear
// (matmul/conv) output; an intervening nonlinearity (ReLU) breaks it. It yields the
// MEAN (sum), not the variance. Cleanest case = global-average-pool / pre-activation
// mean. See SAGE16_architecture.md.
// =============================================================================
`default_nettype none

module selfval_reduce #(
    parameter ROWS      = 4,
    parameter COLS      = 4,
    parameter ACC_W     = 32,
    parameter GEN_CHECK = 1     // 1 = self-validating; 0 = plain reduction unit
)(
    input  wire [ROWS*COLS*ACC_W-1:0] all_pe_out,       // C[i][j]
    input  wire [ROWS*ACC_W-1:0]      pred_flat,         // input-derived ABFT checksum S_i
    output wire [ROWS*ACC_W-1:0]      row_reduce_flat,   // R[i]=sum_jC = mean numerator AND actual checksum
    output wire [ROWS-1:0]            reduce_valid,      // 1 = R[i] matches prediction (self-validated)
    output wire                       any_invalid        // any row's reduction failed self-validation
);
    // ---- the dual-use adder: per-row sum of the PE outputs ----
    // (driven from a module-level always; only READ inside the generate below, to
    //  avoid the iverilog "drive module-scope array from nested generate" pitfall)
    reg [ACC_W-1:0] rsum [0:ROWS-1];
    integer r, c;
    always @(*) begin
        for (r = 0; r < ROWS; r = r+1) begin
            rsum[r] = {ACC_W{1'b0}};
            for (c = 0; c < COLS; c = c+1)
                rsum[r] = rsum[r] + all_pe_out[(r*COLS + c)*ACC_W +: ACC_W];
        end
    end

    genvar gr;
    generate for (gr = 0; gr < ROWS; gr = gr+1) begin : g_out
        assign row_reduce_flat[gr*ACC_W +: ACC_W] = rsum[gr];
        if (GEN_CHECK) begin : g_chk
            // self-validation: the reduction IS the checksum -> compare to prediction
            assign reduce_valid[gr] = (rsum[gr] == pred_flat[gr*ACC_W +: ACC_W]);
        end else begin : g_nochk
            assign reduce_valid[gr] = 1'b1;
        end
    end endgenerate

    assign any_invalid = (GEN_CHECK != 0) ? (reduce_valid != {ROWS{1'b1}}) : 1'b0;
endmodule

`default_nettype wire
