// =============================================================================
// syn_systolic_top.v — SYNTHESIS wrapper for the SYSTOLIC baseline
//
// Same harness style as syn_reliability_top (registered inputs, pipelined
// locality-grouped output reduction) so the two synthesize/place/route under
// identical conditions — the harness overhead is the SAME on both sides and
// CANCELS in the broadcast-vs-systolic comparison. Only the dataflow differs.
//
// Note the systolic baseline is PLAIN compute: no per-PE SRAM, no residue
// self-check, no rail protection (a conventional weight-stationary array).
// That makes this a CONSERVATIVE comparison: our broadcast top carries 16
// SRAMs + full reliability that this systolic top does not.
// =============================================================================
`default_nettype none

module syn_systolic_top #(
    parameter N      = 4,
    parameter DATA_W = 16,
    parameter ACC_W  = 32
)(
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 en,
    input  wire [DATA_W-1:0]    din,
    output reg  [ACC_W-1:0]     sig,
    output reg                  flag
);
    localparam NPE = N*N;

    // ---- LFSR + counter ----
    reg [15:0] lfsr;
    reg [7:0]  ctr;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin lfsr <= 16'hBEEF; ctr <= 8'd0; end
        else begin
            lfsr <= {lfsr[14:0], lfsr[15]^lfsr[13]^lfsr[12]^lfsr[10]}
                    ^ (en ? {8'b0, din[7:0]} : 16'b0);
            ctr  <= ctr + 8'd1;
        end
    end

    // ---- registered weights + activations (distinct per element) ----
    reg [NPE*DATA_W-1:0] w_q;
    reg [N*DATA_W-1:0]   a_q;
    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin w_q <= 0; a_q <= 0; end
        else begin
            for (k = 0; k < NPE; k = k+1)
                w_q[k*DATA_W +: DATA_W] <= lfsr ^ (din + (k*16'h0123));
            for (k = 0; k < N; k = k+1)
                a_q[k*DATA_W +: DATA_W] <= lfsr ^ (din + (k*16'h1357) + 16'h2468);
        end
    end

    wire clr = (ctr == 8'h00);
    wire [NPE*ACC_W-1:0] c_flat;

    systolic_4x4_mac #(.N(N), .DATA_W(DATA_W), .ACC_W(ACC_W)) u_sys (
        .clk(clk), .rst_n(rst_n), .clr(clr), .en(en),
        .w_flat(w_q), .a_left_flat(a_q),
        .fault_en_flat({NPE{1'b0}}), .fault_xor({ACC_W{1'b0}}),
        .c_flat(c_flat)
    );

    // ---- pipelined locality-grouped reduction (same shape as broadcast top) -
    reg [ACC_W-1:0] row_sig [0:N-1];
    integer rr;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) for (rr=0;rr<N;rr=rr+1) row_sig[rr] <= 0;
        else for (rr=0;rr<N;rr=rr+1)
            row_sig[rr] <= c_flat[(rr*4+0)*ACC_W +: ACC_W]
                         ^ c_flat[(rr*4+1)*ACC_W +: ACC_W]
                         ^ c_flat[(rr*4+2)*ACC_W +: ACC_W]
                         ^ c_flat[(rr*4+3)*ACC_W +: ACC_W];
    end

    reg [ACC_W-1:0] pr0, pr1;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin pr0<=0; pr1<=0; end
        else begin pr0 <= row_sig[0]^row_sig[1]; pr1 <= row_sig[2]^row_sig[3]; end

    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin sig<=0; flag<=0; end
        else        begin sig <= pr0 ^ pr1; flag <= ^pr0 ^ (^pr1); end
endmodule

`default_nettype wire
