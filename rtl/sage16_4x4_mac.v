// =============================================================================
// sage16_4x4_mac.v  — 4x4 fabric with paired per-PE SRAM   (sage16_mem v2)
//
// Each PE (16 total) is paired with one 256x32 1RW SRAM macro.  The fabric
// exposes flat per-PE SRAM control buses driven from outside (BIST or a
// future kernel-with-memory controller).  SRAM write source is the PE's
// own accumulator output `out` (the only thing worth storing).
//
// Flat per-PE SRAM control buses (PE index = r*COLS + c, 0..15):
//   sram_cs_n_flat   [16]                       — chip-select per PE  (active low)
//   sram_we_n_flat   [16]                       — write-enable per PE (active low)
//   sram_addr_flat   [16 * SRAM_AW]             — addr per PE
//   sram_wdata_sel   [16]                       — wdata source per PE:
//                                                   0 = use PE accumulator `out`
//                                                   1 = use sram_wdata_ext_flat
//   sram_wdata_ext_flat [16 * 32]               — external wdata per PE (BIST)
//   sel_src_a_flat   [16]                       — PE op-A source: 0=broadcast, 1=SRAM
//   sel_src_b_flat   [16]                       — PE op-B source: 0=broadcast, 1=SRAM
//
// Backward compatibility:
//   Driving all sram_cs_n_flat = 1 and sel_src_*_flat = 0 puts the fabric
//   into "SRAM disabled" mode, identical to the original sage16_4x4_mac
//   behaviour.  v1 kernel controllers do this and pass the same 1616
//   regression checks.
// =============================================================================
`default_nettype none

module sage16_4x4_mac #(
    parameter ROWS     = 4,
    parameter COLS     = 4,
    parameter DATA_W   = 16,
    parameter ACC_W    = 32,
    parameter CFG_W    = 10,
    parameter PIPELINE = 1,
    parameter GEN_CHECK = 1,           // 1 = reliability in silicon; 0 = plain fabric (PPA baseline)
    parameter RESIDUE_MOD7 = 0,        // threaded to every PE (0 = mod-3 only)
    parameter SRAM_AW  = 8,            // 256 words
    parameter SRAM_DW  = 32
)(
    input  wire                             clk, rst_n,
    input  wire [$clog2(ROWS)-1:0]          cfg_pe_row,
    input  wire [$clog2(COLS)-1:0]          cfg_pe_col,
    input  wire [CFG_W-1:0]                 cfg_data,
    input  wire                             cfg_load,
    input  wire                             cfg_broadcast,
    input  wire                             clr_acc_all,
    input  wire                             out_en_all,
    input  wire [ROWS*DATA_W-1:0]           ext_in_west,
    input  wire [COLS*DATA_W-1:0]           ext_in_north,
    input  wire                             per_pe_bypass_en,
    input  wire [ROWS*COLS*DATA_W-1:0]      per_pe_bypass_flat,
    // --- per-PE SRAM control (flat) ---
    input  wire [ROWS*COLS-1:0]             sram_cs_n_flat,
    input  wire [ROWS*COLS-1:0]             sram_we_n_flat,
    input  wire [ROWS*COLS*SRAM_AW-1:0]     sram_addr_flat,
    input  wire [ROWS*COLS-1:0]             sram_wdata_sel,
    input  wire [ROWS*COLS*SRAM_DW-1:0]     sram_wdata_ext_flat,
    input  wire [ROWS*COLS-1:0]             sel_src_a_flat,
    input  wire [ROWS*COLS-1:0]             sel_src_b_flat,
    // --- SRAM port B: dedicated read addr per PE (for PE operand fetch) ---
    input  wire [ROWS*COLS*SRAM_AW-1:0]     sram_raddr2_flat,
    // --- SRAM read data exposed (for verification / observe) ---
    output wire [ROWS*COLS*SRAM_DW-1:0]     sram_rdata_flat,
    // --- compute fabric outputs ---
    output wire [ROWS*ACC_W-1:0]            ext_out_east,
    output wire [ROWS*COLS*ACC_W-1:0]       all_pe_out,
    // --- runtime fault syndrome (mod-3 end-to-end protection) ---
    // rail_err_*  : transport check at each PE tap — source residue vs the
    //               data that actually arrived. A rail defect fires the flag
    //               at every tap it feeds -> the PATTERN names the rail.
    // mac_err_flat: per-PE compute check (residue-verified MAC) — a PE defect
    //               fires exactly one flag -> the INDEX names the PE.
    output wire [ROWS*COLS-1:0]             rail_err_w_flat,
    output wire [ROWS*COLS-1:0]             rail_err_n_flat,
    output wire [ROWS*COLS-1:0]             mac_err_flat
);
    // ---- rail residue sources ----
    // Residue computed at the DRIVER from the clean input; each tap re-derives
    // the residue from the data that ARRIVED on the rail and compares. A
    // wire-defect is modelled in the TESTBENCH by force/release on west_rail/
    // north_rail (see tb_syndrome) — the silicon rail carries no injection mux.
    wire [1:0]        res_w_src [0:ROWS-1];
    wire [1:0]        res_n_src [0:COLS-1];
    wire [DATA_W-1:0] west_rail [0:ROWS-1];
    wire [DATA_W-1:0] north_rail[0:COLS-1];

    genvar gr, gc;
    generate
        for (gr = 0; gr < ROWS; gr = gr+1) begin : wres
            // residue of the driver value (combinational; pruned by synthesis
            // when GEN_CHECK=0 leaves it unused — kept ungated so the reducer's
            // output net connects cleanly across generate scopes).
            mod3_reduce #(.W(DATA_W)) u_m3
                (.x(ext_in_west[gr*DATA_W +: DATA_W]), .r(res_w_src[gr]));
            assign west_rail[gr] = ext_in_west[gr*DATA_W +: DATA_W];
        end
        for (gc = 0; gc < COLS; gc = gc+1) begin : nres
            mod3_reduce #(.W(DATA_W)) u_m3
                (.x(ext_in_north[gc*DATA_W +: DATA_W]), .r(res_n_src[gc]));
            assign north_rail[gc] = ext_in_north[gc*DATA_W +: DATA_W];
        end
    endgenerate
    wire [ACC_W-1:0]  pe_out_acc  [0:ROWS*COLS-1];
    wire [DATA_W-1:0] pe_out_mesh [0:ROWS*COLS-1];
    wire [SRAM_DW-1:0] sram_rdata_w  [0:ROWS*COLS-1];  // port A read (BIST/verify)
    wire [SRAM_DW-1:0] sram_rdata2_w [0:ROWS*COLS-1];  // port B read (PE operand)
    wire [1:0]         sram_rtag2_w  [0:ROWS*COLS-1];  // port B tag (SRAM operand residue)

    genvar r, c;
    generate for (r = 0; r < ROWS; r = r+1) begin : rg
        for (c = 0; c < COLS; c = c+1) begin : cg
            localparam IDX = r*COLS + c;

            // ---- config-load decode ----
            wire cfl = cfg_broadcast ||
                       (cfg_load && (cfg_pe_row == r) && (cfg_pe_col == c));

            // ---- mesh neighbours ----
            wire [DATA_W-1:0] wn, ws, we, ww;
            if (r == 0)       assign wn = north_rail[c];
            else              assign wn = pe_out_mesh[(r-1)*COLS + c];
            if (r == ROWS-1)  assign ws = {DATA_W{1'b0}};
            else              assign ws = pe_out_mesh[(r+1)*COLS + c];
            if (c == COLS-1)  assign we = {DATA_W{1'b0}};
            else              assign we = pe_out_mesh[r*COLS + (c+1)];
            if (c == 0)       assign ww = west_rail[r];
            else              assign ww = pe_out_mesh[r*COLS + (c-1)];

            // ---- broadcast operand delivery (via protected rails) ----
            wire [DATA_W-1:0] bypass_bcast = west_rail[r];
            wire [DATA_W-1:0] bypass_perpe = per_pe_bypass_flat[IDX*DATA_W +: DATA_W];
            wire [DATA_W-1:0] bypass = per_pe_bypass_en ? bypass_perpe : bypass_bcast;
            wire [DATA_W-1:0] b_col  = north_rail[c];

            // ---- rail transport check at this tap ----
            // Re-derive the residue from the data that ARRIVED and compare to
            // the residue computed at the driver. Always-on: zero data with
            // zero residue is consistent, so idle rails never false-flag.
            // Registered so the mod-3-then-compare is reg-to-reg (off the
            // per-cycle critical path); flags land one cycle later, absorbed by
            // the sticky syndrome — same treatment as the PE residue check.
            // rail-tap residues feed the PE (carried operand residue); ungated so
            // their nets connect cleanly, pruned by synthesis when GEN_CHECK=0.
            wire [1:0] res_w_tap, res_n_tap;
            mod3_reduce #(.W(DATA_W)) u_m3_wtap (.x(west_rail[r]),  .r(res_w_tap));
            mod3_reduce #(.W(DATA_W)) u_m3_ntap (.x(north_rail[c]), .r(res_n_tap));
            // only the syndrome FFs + flag outputs are gated (the reliability cost)
            if (GEN_CHECK) begin : g_railchk
                reg rerr_w_q, rerr_n_q;
                always @(posedge clk or negedge rst_n)
                    if (!rst_n) begin rerr_w_q <= 1'b0; rerr_n_q <= 1'b0; end
                    else begin
                        rerr_w_q <= (res_w_tap != res_w_src[r]);
                        rerr_n_q <= (res_n_tap != res_n_src[c]);
                    end
                assign rail_err_w_flat[IDX] = rerr_w_q;
                assign rail_err_n_flat[IDX] = rerr_n_q;
            end else begin : g_norailchk
                assign rail_err_w_flat[IDX] = 1'b0;
                assign rail_err_n_flat[IDX] = 1'b0;
            end

            // ---- SRAM write-data mux: PE accumulator OR external value ----
            wire [SRAM_DW-1:0] sram_wdata_pe  = pe_out_acc[IDX];
            wire [SRAM_DW-1:0] sram_wdata_ext = sram_wdata_ext_flat[IDX*SRAM_DW +: SRAM_DW];
            wire [SRAM_DW-1:0] sram_wdata     =
                sram_wdata_sel[IDX] ? sram_wdata_ext : sram_wdata_pe;

            // Write tag (mod-3 of the stored word). For PE-accumulator writes we
            // CARRY the PE's already-registered residue (pe_res_q) instead of
            // recomputing mod-3(32b) in the write cycle — that recompute was the
            // Fmax-critical path (out_reg -> 11-LUT mod3 -> tag RAM). Only the
            // external (load/BIST) write recomputes mod-3, and that load path is
            // constrained multicycle (it is not part of the compute loop).
            wire [1:0] pe_res_q;
            wire [1:0] sram_wtag_ext;
            mod3_reduce #(.W(SRAM_DW)) u_m3_wtag (.x(sram_wdata_ext), .r(sram_wtag_ext));
            wire [1:0] sram_wtag = sram_wdata_sel[IDX] ? sram_wtag_ext : pe_res_q;

            // ---- paired SRAM ----
            sram_1rw_256x32 #(.GEN_CHECK(GEN_CHECK)) u_mem (
                .clk   (clk),
                // port A: read/write (BIST, write-back, verify)
                .cs_n  (sram_cs_n_flat[IDX]),
                .we_n  (sram_we_n_flat[IDX]),
                .addr  (sram_addr_flat[IDX*SRAM_AW +: SRAM_AW]),
                .wdata (sram_wdata),
                .rdata (sram_rdata_w[IDX]),
                // port B: dedicated read (PE operand fetch) — concurrent with A
                .raddr2(sram_raddr2_flat[IDX*SRAM_AW +: SRAM_AW]),
                .rdata2(sram_rdata2_w[IDX]),
                // residue tag carried with the word
                .wtag  (sram_wtag),
                .rtag  (),
                .rtag2 (sram_rtag2_w[IDX])
            );

            assign sram_rdata_flat[IDX*SRAM_DW +: SRAM_DW] = sram_rdata_w[IDX];

            // ---- PE ----
            pe #(
                .DATA_W      (DATA_W),
                .ACC_W       (ACC_W),
                .CFG_W       (CFG_W),
                .PIPELINE    (PIPELINE),
                .GEN_CHECK   (GEN_CHECK),
                .RESIDUE_MOD7(RESIDUE_MOD7)
            ) u_pe (
                .clk        (clk),
                .rst_n      (rst_n),
                .cfg_load   (cfl),
                .cfg_in     (cfg_data),
                .out_en     (out_en_all),
                .clr_acc    (clr_acc_all),
                .in_north   (wn),
                .in_south   (ws),
                .in_east    (we),
                .in_west    (ww),
                .in_self    (pe_out_acc[IDX]),
                .in_bypass  (bypass),
                .in_b_col   (b_col),
                .sram_rdata (sram_rdata2_w[IDX]),   // PE reads via port B
                .sel_src_a  (sel_src_a_flat[IDX]),
                .sel_src_b  (sel_src_b_flat[IDX]),
                .res_a_in   (res_w_tap),            // operand-A residue (rail tap, carried)
                .res_b_in   (res_n_tap),            // operand-B residue (rail tap, carried)
                .sram_res   (sram_rtag2_w[IDX]),    // SRAM-word residue (stored tag)
                .out_mesh   (pe_out_mesh[IDX]),
                .out        (pe_out_acc [IDX]),
                .res_q      (pe_res_q),
                .mac_err    (mac_err_flat[IDX])
            );

            if (c == COLS-1)
                assign ext_out_east[r*ACC_W +: ACC_W] = pe_out_acc[IDX];

            assign all_pe_out[IDX*ACC_W +: ACC_W] = pe_out_acc[IDX];
        end
    end endgenerate
endmodule

`default_nettype wire
