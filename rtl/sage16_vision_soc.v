// =============================================================================
// sage16_vision_soc.v -- top level that runs the attitude-control loop and the
// host-free CNN vision sequencer on ONE shared SAGE-16 fabric.
//
//   control_top_shared : holds the single scheduler + single fabric. Its
//                        control loop (AC -> MM each tick) is unchanged; its
//                        spare vis_* lane and the fabric's cv_img/cv_k are now
//                        exposed at its boundary.
//   vision_top         : the CNN macro-op sequencer. Drives control_top's
//                        vis_* lane (FC operands) and cv_img/cv_k (conv
//                        operands, Option A direct path). One classify pulse
//                        from the PS runs a full MNIST inference on the PL.
//
// The control loop keeps strict priority in the scheduler, so vision never
// starves it: control can be blocked at most one in-flight vision op
// (~16 cycles, ~160 ns @100 MHz) vs its ~1 ms tick period.
//
// Bus: control_top's existing AXI-lite register file is brought straight out
// (unchanged). Vision's small control surface (classify / image load / status)
// is brought out as ports; in the Vivado BD wire these to a PS AXI-GPIO or a
// second AXI-lite slave. Image pixels are written one at a time via
// img_we/img_addr/img_wdata (676 int16 words, row-major 26x26).
// =============================================================================
`default_nettype none

module sage16_vision_soc #(
    parameter ADDR_W = 10,
    parameter DATA_W = 16,
    parameter ACC_W  = 32
)(
    input  wire                  clk,
    input  wire                  rst_n,

    // ---- control_top AXI-lite register bus (unchanged) ----
    input  wire [ADDR_W-1:0]     addr,
    input  wire [31:0]           wdata,
    output wire [31:0]           rdata,
    input  wire                  we,
    input  wire                  re,
    output wire                  ack,
    output wire                  irq,

    // ---- vision control surface (PS drives via AXI-GPIO / 2nd AXI-lite) ----
    input  wire                  vis_classify,
    output wire                  vis_busy,
    output wire                  vis_result_done,
    output wire [3:0]            vis_digit,
    output wire                  vis_digit_valid,
    input  wire                  vis_img_we,
    input  wire [9:0]            vis_img_addr,
    input  wire signed [DATA_W-1:0] vis_img_wdata
);
    // ---- shared vis lane between vision_top and control_top_shared ----
    wire                  vis_valid, vis_gnt, vis_done;
    wire [1:0]            vis_mode;
    wire [16*DATA_W-1:0]  vis_a_flat, vis_b_flat;
    wire [16*ACC_W-1:0]   vis_result;
    wire [36*DATA_W-1:0]  cv_img;
    wire [ 9*DATA_W-1:0]  cv_k;

    control_top_shared #(.ADDR_W(ADDR_W), .DATA_W(DATA_W), .ACC_W(ACC_W)) u_ctrl (
        .clk(clk), .rst_n(rst_n),
        .addr(addr), .wdata(wdata), .rdata(rdata), .we(we), .re(re), .ack(ack), .irq(irq),
        // shared-fabric vision lane
        .vis_valid(vis_valid), .vis_mode(vis_mode),
        .vis_a_flat(vis_a_flat), .vis_b_flat(vis_b_flat),
        .vis_gnt(vis_gnt), .vis_done(vis_done), .vis_result(vis_result),
        .cv_img(cv_img), .cv_k(cv_k)
    );

    vision_top #(.DATA_W(DATA_W), .ACC_W(ACC_W)) u_vis (
        .clk(clk), .rst_n(rst_n),
        .classify(vis_classify), .busy(vis_busy), .done(vis_result_done),
        .result(vis_digit), .result_valid(vis_digit_valid),
        .img_we(vis_img_we), .img_addr(vis_img_addr), .img_wdata(vis_img_wdata),
        .vis_valid(vis_valid), .vis_mode(vis_mode),
        .vis_a_flat(vis_a_flat), .vis_b_flat(vis_b_flat),
        .vis_gnt(vis_gnt), .vis_done(vis_done), .vis_result(vis_result),
        .cv_img(cv_img), .cv_k(cv_k)
    );
endmodule
`default_nettype wire
