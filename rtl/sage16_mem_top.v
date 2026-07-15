// =============================================================================
// sage16_mem_top.v  —  top-level integration for sage16_mem
//
// Wires the BIST controller to the sage16_4x4_mac fabric.  A 1-bit `mode`
// input selects who drives the per-PE SRAM control buses:
//
//   mode = 1'b0 : BIST mode   — sram_bist_ctrl drives SRAM ports
//   mode = 1'b1 : compute mode — fabric runs kernels via sage16_top
//                                 (SRAM ports tied off; v1 kernels still work)
//
// Tiny register interface for an external host:
//
//   bist_start    (in)  : pulse to launch BIST
//   bist_busy     (out) : 1 while BIST is running
//   bist_done     (out) : 1 when BIST finished (latched until reset)
//   bist_pass     (out) : 1 if every PE passed, else 0
//   pe_pass_mask  (out) : 16-bit per-PE pass/fail (bit i = PE i passed)
//
// For this v2 milestone, compute mode is intentionally minimal:
//   - the 4x4 fabric is exposed with config/operand pins, and any of the
//     existing kernel controllers (matmul_sage16, conv3x3_sage16, quat_sage16,
//     sage16_top) can be wrapped around this top.  The next milestone is
//     wiring a SRAM-aware kernel controller that uses the per-PE memory
//     during compute — for now the top demonstrates the SRAM lifecycle
//     (write → read-back → BIST verifies) end-to-end.
// =============================================================================
`default_nettype none

module sage16_mem_top #(
    parameter ROWS     = 4,
    parameter COLS     = 4,
    parameter DATA_W   = 16,
    parameter ACC_W    = 32,
    parameter CFG_W    = 10,
    parameter PIPELINE = 1,
    parameter SRAM_AW  = 8,
    parameter SRAM_DW  = 32,
    parameter NUM_PE   = ROWS*COLS
)(
    input  wire                          clk, rst_n,

    // ---- mode ----
    input  wire                          mode,        // 0 = BIST, 1 = compute

    // ---- BIST host interface ----
    input  wire                          bist_start,
    output wire                          bist_busy,
    output wire                          bist_done,
    output wire                          bist_pass,
    output wire [NUM_PE-1:0]             pe_pass_mask,

    // ---- compute-mode passthrough (used when mode = 1) ----
    input  wire [$clog2(ROWS)-1:0]       cfg_pe_row,
    input  wire [$clog2(COLS)-1:0]       cfg_pe_col,
    input  wire [CFG_W-1:0]              cfg_data,
    input  wire                          cfg_load,
    input  wire                          cfg_broadcast,
    input  wire                          clr_acc_all,
    input  wire                          out_en_all,
    input  wire [ROWS*DATA_W-1:0]        ext_in_west,
    input  wire [COLS*DATA_W-1:0]        ext_in_north,
    input  wire                          per_pe_bypass_en,
    input  wire [NUM_PE*DATA_W-1:0]      per_pe_bypass_flat,

    // ---- compute-mode SRAM passthrough (only honored when mode = 1) ----
    input  wire [NUM_PE-1:0]             host_sram_cs_n_flat,
    input  wire [NUM_PE-1:0]             host_sram_we_n_flat,
    input  wire [NUM_PE*SRAM_AW-1:0]     host_sram_addr_flat,
    input  wire [NUM_PE-1:0]             host_sram_wdata_sel,
    input  wire [NUM_PE*SRAM_DW-1:0]     host_sram_wdata_ext_flat,
    input  wire [NUM_PE-1:0]             host_sel_src_a_flat,
    input  wire [NUM_PE-1:0]             host_sel_src_b_flat,

    // ---- outputs ----
    output wire [NUM_PE*SRAM_DW-1:0]     sram_rdata_flat,
    output wire [ROWS*ACC_W-1:0]         ext_out_east,
    output wire [NUM_PE*ACC_W-1:0]       all_pe_out
);
    // ---------------- BIST controller ----------------
    wire [NUM_PE-1:0]            bist_cs_n;
    wire [NUM_PE-1:0]            bist_we_n;
    wire [NUM_PE*SRAM_AW-1:0]    bist_addr;
    wire [NUM_PE-1:0]            bist_wdata_sel;
    wire [NUM_PE*SRAM_DW-1:0]    bist_wdata_ext;

    sram_bist_ctrl #(
        .NUM_PE (NUM_PE),
        .SRAM_AW(SRAM_AW),
        .SRAM_DW(SRAM_DW),
        .DEPTH  (1 << SRAM_AW)
    ) u_bist (
        .clk(clk), .rst_n(rst_n),
        .start             (bist_start),
        .busy              (bist_busy),
        .done              (bist_done),
        .pass              (bist_pass),
        .pe_pass_mask      (pe_pass_mask),
        .sram_cs_n_flat    (bist_cs_n),
        .sram_we_n_flat    (bist_we_n),
        .sram_addr_flat    (bist_addr),
        .sram_wdata_sel    (bist_wdata_sel),
        .sram_wdata_ext_flat(bist_wdata_ext),
        .sram_rdata_flat   (sram_rdata_flat)
    );

    // ---------------- SRAM-bus mux: BIST vs host ----------------
    wire [NUM_PE-1:0]            mux_cs_n      = mode ? host_sram_cs_n_flat       : bist_cs_n;
    wire [NUM_PE-1:0]            mux_we_n      = mode ? host_sram_we_n_flat       : bist_we_n;
    wire [NUM_PE*SRAM_AW-1:0]    mux_addr      = mode ? host_sram_addr_flat       : bist_addr;
    wire [NUM_PE-1:0]            mux_wdata_sel = mode ? host_sram_wdata_sel       : bist_wdata_sel;
    wire [NUM_PE*SRAM_DW-1:0]    mux_wdata_ext = mode ? host_sram_wdata_ext_flat  : bist_wdata_ext;
    wire [NUM_PE-1:0]            mux_sel_src_a = mode ? host_sel_src_a_flat       : {NUM_PE{1'b0}};
    wire [NUM_PE-1:0]            mux_sel_src_b = mode ? host_sel_src_b_flat       : {NUM_PE{1'b0}};

    // ---------------- fabric ----------------
    sage16_4x4_mac #(
        .ROWS    (ROWS),
        .COLS    (COLS),
        .DATA_W  (DATA_W),
        .ACC_W   (ACC_W),
        .CFG_W   (CFG_W),
        .PIPELINE(PIPELINE),
        .SRAM_AW (SRAM_AW),
        .SRAM_DW (SRAM_DW)
    ) u_fab (
        .clk(clk), .rst_n(rst_n),
        .cfg_pe_row         (cfg_pe_row),
        .cfg_pe_col         (cfg_pe_col),
        .cfg_data           (cfg_data),
        .cfg_load           (cfg_load),
        .cfg_broadcast      (cfg_broadcast),
        .clr_acc_all        (clr_acc_all),
        .out_en_all         (out_en_all),
        .ext_in_west        (ext_in_west),
        .ext_in_north       (ext_in_north),
        .per_pe_bypass_en   (per_pe_bypass_en),
        .per_pe_bypass_flat (per_pe_bypass_flat),
        .sram_cs_n_flat     (mux_cs_n),
        .sram_we_n_flat     (mux_we_n),
        .sram_addr_flat     (mux_addr),
        .sram_raddr2_flat   (128'b0),
        .sram_wdata_sel     (mux_wdata_sel),
        .sram_wdata_ext_flat(mux_wdata_ext),
        .sel_src_a_flat     (mux_sel_src_a),
        .sel_src_b_flat     (mux_sel_src_b),
        .sram_rdata_flat    (sram_rdata_flat),
        .ext_out_east       (ext_out_east),
        .all_pe_out         (all_pe_out)
    );
endmodule

`default_nettype wire
