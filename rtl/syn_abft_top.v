// =============================================================================
// syn_abft_top.v — synthesis wrapper for ABFT checksum + locate hardware
//
// Purpose:
//   Measure the checksum / locate repair-support logic separately from the main
//   SAGE fabric. Inputs are exposed directly so Genus keeps the logic, while a
//   compact signature/flag output prevents pruning of the checksum and locator
//   results.
// =============================================================================
`default_nettype none

module syn_abft_top #(
    parameter ROWS   = 4,
    parameter COLS   = 4,
    parameter DATA_W = 16,
    parameter ACC_W  = 32
)(
    input  wire [ROWS*DATA_W-1:0]      ext_in_west,
    input  wire [COLS*DATA_W-1:0]      ext_in_north,
    input  wire [ROWS*COLS*ACC_W-1:0]  c_obs_flat,
    input  wire [ROWS*COLS-1:0]        mac_err_flat,
    input  wire                        clk,
    input  wire                        rst_n,
    input  wire                        clr_acc,
    input  wire                        out_en,
    output wire [ACC_W-1:0]            sig,
    output wire                        flag
);
    localparam LOC_W = (COLS <= 1) ? 1 : $clog2(COLS);

    wire [ROWS*ACC_W-1:0] cksum_flat;
    wire [ROWS*ACC_W-1:0] cksum_w_flat;
    abft_checksum #(
        .ROWS(ROWS),
        .COLS(COLS),
        .DATA_W(DATA_W),
        .ACC_W(ACC_W)
    ) u_checksum (
        .clk(clk),
        .rst_n(rst_n),
        .clr_acc(clr_acc),
        .out_en(out_en),
        .ext_in_west(ext_in_west),
        .ext_in_north(ext_in_north),
        .cksum_flat(cksum_flat),
        .cksum_w_flat(cksum_w_flat)
    );

    wire [ROWS-1:0]                    err_present_w, located_w, used_fallback_w, ambig_w;
    wire [ROWS*LOC_W-1:0]              loc_flat;
    wire [ROWS*ACC_W-1:0]              err_val_flat, c_fixed_flat;

    genvar r;
    generate
        for (r = 0; r < ROWS; r = r + 1) begin : grow
            abft_locate #(
                .COLS(COLS),
                .ACC_W(ACC_W)
            ) u_locate (
                .c_obs_flat(c_obs_flat[r*COLS*ACC_W +: COLS*ACC_W]),
                .cksum(cksum_flat[r*ACC_W +: ACC_W]),
                .cksum_w(cksum_w_flat[r*ACC_W +: ACC_W]),
                .mac_err_row(mac_err_flat[r*COLS +: COLS]),
                .err_present(err_present_w[r]),
                .located(located_w[r]),
                .loc(loc_flat[r*LOC_W +: LOC_W]),
                .used_fallback(used_fallback_w[r]),
                .ambig(ambig_w[r]),
                .err_val(err_val_flat[r*ACC_W +: ACC_W]),
                .c_fixed(c_fixed_flat[r*ACC_W +: ACC_W])
            );
        end
    endgenerate

    reg [ACC_W-1:0] sig_r;
    integer i;
    always @(*) begin
        sig_r = {ACC_W{1'b0}};
        for (i = 0; i < ROWS; i = i + 1) begin
            sig_r = sig_r
                  ^ cksum_flat[i*ACC_W +: ACC_W]
                  ^ cksum_w_flat[i*ACC_W +: ACC_W]
                  ^ err_val_flat[i*ACC_W +: ACC_W]
                  ^ c_fixed_flat[i*ACC_W +: ACC_W];
        end
    end

    assign sig  = sig_r;
    assign flag = |err_present_w | |located_w | |used_fallback_w | |ambig_w | ^loc_flat;
endmodule

`default_nettype wire
