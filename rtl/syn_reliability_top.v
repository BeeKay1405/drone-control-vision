// =============================================================================
// syn_reliability_top.v — SYNTHESIS wrapper for the FULL reliability fabric
//
// Purpose: get REAL post-route area/Fmax/power for the broadcast fabric WITH
// all reliability features, on the actual xc7a35t part. Solves two problems
// that block synthesizing the raw fabric directly:
//   (1) wide flat buses (512-bit all_pe_out, etc.) need >2000 pins -> won't
//       place. Here they are folded into a 32-bit signature + 1-bit flag so
//       the block fits ~50 I/O.
//   (2) if the reliability outputs (mac_err/rail_err) are unconnected the tool
//       PRUNES the residue/rail/checksum/locate logic. Here every reliability
//       output drives the `flag`, so ALL of it is kept and measured.
//
// What is instantiated (everything we want PPA for):
//   - sage16_4x4_mac (PIPELINE=1): 16 PEs, broadcast rails, per-PE residue
//     self-check (mac_err), rail transport checks (rail_err), 16 paired SRAMs
//   - abft_checksum: the 4 free-rider checksum PEs (+ weighted)
//   - abft_locate x4: per-row erasure decode + correct
//
// A small LFSR harness drives the fabric so the 16 PEs synthesize as distinct
// logic (no cross-PE optimization collapsing the array) and the SRAMs stay
// alive (written + read-back folded into the signature). The exact stimulus
// does not change gate count or the compute critical path.
// =============================================================================
`default_nettype none

module syn_reliability_top #(
    parameter DATA_W = 16,
    parameter ACC_W  = 32,
    parameter CFG_W  = 10
)(
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 en,
    input  wire [DATA_W-1:0]    din,
    output reg  [ACC_W-1:0]     sig,     // XOR signature of all outputs + SRAM
    output reg                  flag     // OR of every reliability flag
);
    localparam NUM_PE = 16;
    localparam [3:0] OP_MACB = 4'd9;

    // ---- LFSR + counters (diversify rails, keep SRAM/regs alive) ----
    reg [15:0] lfsr;
    reg [7:0]  actr;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin lfsr <= 16'hACE1; actr <= 8'd0; end
        else begin
            lfsr <= {lfsr[14:0], lfsr[15]^lfsr[13]^lfsr[12]^lfsr[10]}
                    ^ (en ? {8'b0, din[7:0]} : 16'b0);
            actr <= actr + 8'd1;
        end
    end

    // ---- drive the broadcast rails (REGISTERED: each rail its own flop, so
    //      no single net fans out to all 16 PEs — realistic launch points) ----
    reg [4*DATA_W-1:0] west, north;
    integer r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin west <= 0; north <= 0; end
        else for (r = 0; r < 4; r = r+1) begin
            west [r*DATA_W +: DATA_W] <= lfsr ^ (din + (r*16'h1111));
            north[r*DATA_W +: DATA_W] <= lfsr ^ (din + (r*16'h2222) + 16'h0F0F);
        end
    end

    // ---- SRAM control: write the accumulator back + read, to keep 16 BRAMs ----
    wire [NUM_PE-1:0]         cs_n   = {NUM_PE{1'b0}};        // all selected
    wire [NUM_PE-1:0]         we_n   = {NUM_PE{actr[0]}};      // toggle write
    wire [NUM_PE*8-1:0]       addr   = {NUM_PE{actr}};
    wire [NUM_PE*8-1:0]       raddr2 = {NUM_PE{actr ^ 8'h55}};
    wire [NUM_PE-1:0]         wsel   = {NUM_PE{1'b0}};         // wdata = PE acc
    wire                      clr    = (actr == 8'h00);

    // ---- fabric outputs ----
    wire [NUM_PE*ACC_W-1:0]   all_pe_out;
    wire [NUM_PE*32-1:0]      sram_rd;
    wire [NUM_PE-1:0]         mac_err, rerr_w, rerr_n;

    sage16_4x4_mac #(.DATA_W(DATA_W), .ACC_W(ACC_W), .CFG_W(CFG_W), .PIPELINE(1)) u_fab (
        .clk(clk), .rst_n(rst_n), .cfg_pe_row(2'd0), .cfg_pe_col(2'd0),
        .cfg_data({OP_MACB,3'd0,3'd0}), .cfg_load(1'b0), .cfg_broadcast(1'b1),
        .clr_acc_all(clr), .out_en_all(en),
        .ext_in_west(west), .ext_in_north(north),
        .per_pe_bypass_en(1'b0), .per_pe_bypass_flat(256'b0),
        .sram_cs_n_flat(cs_n), .sram_we_n_flat(we_n),
        .sram_addr_flat(addr), .sram_raddr2_flat(raddr2),
        .sram_wdata_sel(wsel), .sram_wdata_ext_flat(512'b0),
        .sel_src_a_flat(16'b0), .sel_src_b_flat(16'b0),
        .sram_rdata_flat(sram_rd), .ext_out_east(), .all_pe_out(all_pe_out),
        .rail_err_w_flat(rerr_w), .rail_err_n_flat(rerr_n), .mac_err_flat(mac_err)
    );

    // NOTE: erasure-ABFT (abft_checksum + abft_locate) is intentionally NOT in
    // this top. abft_locate is a deep COMBINATIONAL chain (sum -> subtract ->
    // compare -> subtract) — a multi-cycle REPAIR path, not a per-cycle path.
    // Wiring it combinationally into the signature created a ~15 ns critical
    // path that hid the fabric's real Fmax. It is measured separately (with its
    // own pipeline). This top measures the FABRIC: 16 PEs + per-PE residue
    // self-check + rail protection + 16 SRAMs.

    // ---- PIPELINED, locality-grouped output reduction --------------------
    // The previous version folded all 44 output words into one register in a
    // single combinational XOR tree (21 levels, chip-wide gather) — that, not
    // the fabric, was the critical path. Here the reduction is a balanced
    // 3-stage pipeline: stage 1 stays WITHIN each row (the 4 PEs of a row are
    // physically adjacent -> short wires), then a 2-level tree combines the 4
    // rows. The harness is therefore never the bottleneck, so the measured
    // Fmax is the fabric's. Extra latency is irrelevant for a synth harness.
    integer rr;

    // stage 1: per-row partial (PEs/SRAMs/checksum of one row are co-located)
    reg [ACC_W-1:0] row_sig [0:3];
    reg [3:0]       row_flg;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (rr=0;rr<4;rr=rr+1) row_sig[rr] <= 0;
            row_flg <= 4'b0;
        end else begin
            for (rr=0;rr<4;rr=rr+1) begin
                row_sig[rr] <= all_pe_out[(rr*4+0)*ACC_W +: ACC_W]
                             ^ all_pe_out[(rr*4+1)*ACC_W +: ACC_W]
                             ^ all_pe_out[(rr*4+2)*ACC_W +: ACC_W]
                             ^ all_pe_out[(rr*4+3)*ACC_W +: ACC_W]
                             ^ sram_rd[(rr*4+0)*32 +: 32] ^ sram_rd[(rr*4+1)*32 +: 32]
                             ^ sram_rd[(rr*4+2)*32 +: 32] ^ sram_rd[(rr*4+3)*32 +: 32];
                row_flg[rr] <= mac_err[rr*4+0] | mac_err[rr*4+1]
                             | mac_err[rr*4+2] | mac_err[rr*4+3]
                             | rerr_w[rr*4+0] | rerr_w[rr*4+1]
                             | rerr_w[rr*4+2] | rerr_w[rr*4+3]
                             | rerr_n[rr*4+0] | rerr_n[rr*4+1]
                             | rerr_n[rr*4+2] | rerr_n[rr*4+3];
            end
        end
    end

    // stage 2: pair the 4 rows
    reg [ACC_W-1:0] pr0, pr1;
    reg             pf0, pf1;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin pr0<=0; pr1<=0; pf0<=0; pf1<=0; end
        else begin
            pr0 <= row_sig[0] ^ row_sig[1];  pr1 <= row_sig[2] ^ row_sig[3];
            pf0 <= row_flg[0] | row_flg[1];  pf1 <= row_flg[2] | row_flg[3];
        end

    // stage 3: final
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin sig <= 0; flag <= 0; end
        else        begin sig <= pr0 ^ pr1; flag <= pf0 | pf1; end
endmodule

`default_nettype wire
