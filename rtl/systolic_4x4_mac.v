// =============================================================================
// systolic_4x4_mac.v  —  4x4 WEIGHT-STATIONARY systolic array (the BASELINE)
//
// The "punching bag" for the containment comparison. This is the conventional
// dataflow used by TPUs et al.:  weights are pre-loaded and held in each PE;
// activations stream in from the LEFT and flow right PE->PE; partial sums flow
// DOWN PE->PE and accumulate. Output C[i][j] emerges at the bottom of column j.
//
// THE KEY DIFFERENCE vs our broadcast fabric:
//   here each PE *forwards* data to its neighbours (act -> east, psum -> south),
//   so a faulty PE corrupts every downstream PE that consumes its forwarded
//   values -> a LINE (column) of wrong outputs.  (Libano et al., IEEE TC 2023:
//   ~70% of single-PE faults become line errors for exactly this reason.)
//
// Same fault-injection hook as pe.v: when fault_en, the PE's MAC result is
// XORed with fault_xor every cycle (permanent-fault model).
//
// Computes C = A * B, 4x4, INT (unsigned here to match the broadcast matmul test):
//   weights  = B  (B[k][j] loaded into PE(k,j))
//   activations stream A row-wise: A[i][k] enters row i from the left, skewed.
//   output   = C[i][j] read from the accumulator of PE(i? ) ...
// To keep the comparison simple + deterministic we use an OUTPUT-at-PE scheme:
//   PE(i,j) accumulates C[i][j] = sum_k A[i][k]*B[k][j], with A streaming east
//   and partial sums streaming south. This is the classic OS-on-systolic mapping
//   where the propagation path (east/south forwarding) is what spreads faults.
//
// NOTE: this baseline is for the fault-PROPAGATION comparison, not for being a
// fast/optimal systolic array. Functional matmul correctness + the propagation
// pattern are what matter.
// =============================================================================
`default_nettype none

module systolic_4x4_mac #(
    parameter N      = 4,
    parameter DATA_W = 16,
    parameter ACC_W  = 32
)(
    input  wire                       clk, rst_n,
    input  wire                       clr,          // clear all accumulators
    input  wire                       en,           // step the array
    // weights B[k][j] preloaded (flat, row-major k*N+j)
    input  wire [N*N*DATA_W-1:0]      w_flat,
    // activation injected at the left of each row i this cycle (A streams in)
    input  wire [N*DATA_W-1:0]        a_left_flat,  // a_left[i]
    // fault injection (per-PE)
    input  wire [N*N-1:0]             fault_en_flat,
    input  wire [ACC_W-1:0]           fault_xor,
    // outputs: every PE accumulator (row-major i*N+j) = C[i][j]
    output wire [N*N*ACC_W-1:0]       c_flat
);
    // per-PE registers
    reg  [DATA_W-1:0] a_reg   [0:N*N-1];   // activation latched in each PE (flows east)
    reg  [ACC_W-1:0]  acc     [0:N*N-1];   // partial-sum accumulator (flows south)

    genvar r, c;
    integer ri, ci;

    // weight access
    function [DATA_W-1:0] wq; input integer kk, jj;
        wq = w_flat[(kk*N+jj)*DATA_W +: DATA_W];
    endfunction

    // activation entering PE(r,c): from the left neighbour, or a_left for col 0
    // (combinational view of the *incoming* activation this cycle)
    generate
    for (r = 0; r < N; r = r+1) begin : sr
        for (c = 0; c < N; c = c+1) begin : sc
            localparam IDX = r*N + c;

            // generate-if (not ternary) so the edge case never elaborates a
            // negative array index — synthesis-clean
            wire [DATA_W-1:0] a_in;
            wire [ACC_W-1:0]  ps_in;
            if (c == 0) begin : g_a_edge
                assign a_in = a_left_flat[r*DATA_W +: DATA_W];
            end else begin : g_a_int
                assign a_in = a_reg[r*N + (c-1)];
            end
            if (r == 0) begin : g_ps_edge
                assign ps_in = {ACC_W{1'b0}};
            end else begin : g_ps_int
                assign ps_in = acc[(r-1)*N + c];
            end

            // MAC: this PE multiplies the activation it currently holds by its
            // weight and adds the partial sum coming from above.
            wire [DATA_W-1:0] wj   = w_flat[(r*N + c)*DATA_W +: DATA_W];
            wire [2*DATA_W-1:0] prod = a_in * wj;
            wire [ACC_W-1:0]  mac  = ps_in + prod[ACC_W-1:0];
            wire [ACC_W-1:0]  mac_fi = fault_en_flat[IDX] ? (mac ^ fault_xor) : mac;

            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    a_reg[IDX] <= 0;
                    acc[IDX]   <= 0;
                end else if (clr) begin
                    a_reg[IDX] <= 0;
                    acc[IDX]   <= 0;
                end else if (en) begin
                    a_reg[IDX] <= a_in;     // forward activation east (next cycle)
                    acc[IDX]   <= mac_fi;   // forward partial sum south (next cycle)
                end
            end

            assign c_flat[IDX*ACC_W +: ACC_W] = acc[IDX];
        end
    end
    endgenerate
endmodule

`default_nettype wire
