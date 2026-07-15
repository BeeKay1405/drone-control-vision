// =============================================================================
// tmr_top.v — Triple Modular Redundancy baseline (reliability yardstick)
//
// Three identical SAGE fabrics (reliability OFF, GEN_CHECK=0) + a bitwise
// majority voter on the compute result. This is the standard fault-tolerance
// reference: it MASKS a fault (the two healthy copies outvote the faulty one)
// but it CANNOT locate the fault, and it triplicates the multipliers (48 DSP).
//
// Synthesize OOC (top = tmr_top) for the area / DSP / Fmax comparison against
// SAGE-full. Expected story: TMR ~3x area + 48 DSP for masking-only, vs SAGE
// at +40% LUT / 16 DSP for detect + LOCATE + repair.
// =============================================================================
`default_nettype none

module tmr_top #(
    parameter ROWS = 4, COLS = 4, DATA_W = 16, ACC_W = 32,
    parameter CFG_W = 10, SRAM_AW = 8, SRAM_DW = 32,
    parameter GEN_CHECK = 0   // unused here; absorbs the synth flow's -generic GEN_CHECK
)(
    input  wire                          clk, rst_n,
    input  wire [1:0]                    cfg_pe_row, cfg_pe_col,
    input  wire [CFG_W-1:0]              cfg_data,
    input  wire                          cfg_load, cfg_broadcast, clr_acc_all, out_en_all,
    input  wire [ROWS*DATA_W-1:0]        ext_in_west,
    input  wire [COLS*DATA_W-1:0]        ext_in_north,
    input  wire                          per_pe_bypass_en,
    input  wire [ROWS*COLS*DATA_W-1:0]   per_pe_bypass_flat,
    input  wire [ROWS*COLS-1:0]          sram_cs_n_flat, sram_we_n_flat,
    input  wire [ROWS*COLS*SRAM_AW-1:0]  sram_addr_flat,
    input  wire [ROWS*COLS-1:0]          sram_wdata_sel,
    input  wire [ROWS*COLS*SRAM_DW-1:0]  sram_wdata_ext_flat,
    input  wire [ROWS*COLS-1:0]          sel_src_a_flat, sel_src_b_flat,
    input  wire [ROWS*COLS*SRAM_AW-1:0]  sram_raddr2_flat,
    output wire [ROWS*COLS*ACC_W-1:0]    all_pe_out_voted,
    output wire                          tmr_mismatch
);
    localparam NW = ROWS*COLS*ACC_W;
    wire [3*NW-1:0] oall;   // packed (slice from generate is iverilog-safe)

    genvar t;
    generate for (t = 0; t < 3; t = t+1) begin : rep
        wire [NW-1:0] o;
        // dont_touch: keep all 3 copies — without it the tool merges the
        // identical replicas (same inputs -> same outputs) down to one.
        (* dont_touch = "true" *)
        sage16_4x4_mac #(.GEN_CHECK(0)) u (
            .clk(clk), .rst_n(rst_n),
            .cfg_pe_row(cfg_pe_row), .cfg_pe_col(cfg_pe_col),
            .cfg_data(cfg_data), .cfg_load(cfg_load), .cfg_broadcast(cfg_broadcast),
            .clr_acc_all(clr_acc_all), .out_en_all(out_en_all),
            .ext_in_west(ext_in_west), .ext_in_north(ext_in_north),
            .per_pe_bypass_en(per_pe_bypass_en), .per_pe_bypass_flat(per_pe_bypass_flat),
            .sram_cs_n_flat(sram_cs_n_flat), .sram_we_n_flat(sram_we_n_flat),
            .sram_addr_flat(sram_addr_flat), .sram_wdata_sel(sram_wdata_sel),
            .sram_wdata_ext_flat(sram_wdata_ext_flat),
            .sel_src_a_flat(sel_src_a_flat), .sel_src_b_flat(sel_src_b_flat),
            .sram_raddr2_flat(sram_raddr2_flat),
            .sram_rdata_flat(), .ext_out_east(), .all_pe_out(o),
            .rail_err_w_flat(), .rail_err_n_flat(), .mac_err_flat()
        );
        assign oall[t*NW +: NW] = o;
    end endgenerate

    wire [NW-1:0] o0 = oall[0*NW +: NW];
    wire [NW-1:0] o1 = oall[1*NW +: NW];
    wire [NW-1:0] o2 = oall[2*NW +: NW];

    assign all_pe_out_voted = (o0 & o1) | (o1 & o2) | (o0 & o2);   // bitwise majority
    assign tmr_mismatch     = |((o0 ^ o1) | (o1 ^ o2));            // any copy disagrees
endmodule

`default_nettype wire
