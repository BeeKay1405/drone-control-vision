// =============================================================================
// sage16_shared_axi.v -- AXI4-Lite slave wrapping sage16_vision_soc.
//
// Replaces drone_axi_lite.v. Same register map as drone_axi_lite for
// 0x000..0x2FF (forwarded to control_top_shared), PLUS a vision bank:
//
//   WRITE 0x300  VIS_PIX    bits[25:16]=pixel index 0..675, bits[15:0]=int16 value
//   WRITE 0x304  VIS_CTRL   bit[0]=1 -> start (classify) pulse
//   READ  0x308  VIS_STATUS bit[0]=busy, bit[1]=result_valid, bits[5:2]=digit
//
// Structure mirrors the original drone_axi_lite FSM EXACTLY (one always block,
// waits for ctrl_ack on control transactions). Vision registers complete in a
// single cycle (no ack needed). No signal is driven from more than one block.
// =============================================================================

module sage16_shared_axi #(
    parameter ADDR_W      = 12,   // AXI address width the interconnect drives
    parameter DATA_W      = 32,
    parameter CTRL_ADDR_W = 10    // control_top internal address width
)(
    input               s_axi_aclk,
    input               s_axi_aresetn,

    input  [ADDR_W-1:0] s_axi_awaddr,
    input  [2:0]        s_axi_awprot,
    input               s_axi_awvalid,
    output              s_axi_awready,

    input  [DATA_W-1:0] s_axi_wdata,
    input  [DATA_W/8-1:0] s_axi_wstrb,
    input               s_axi_wvalid,
    output              s_axi_wready,

    output [1:0]        s_axi_bresp,
    output              s_axi_bvalid,
    input               s_axi_bready,

    input  [ADDR_W-1:0] s_axi_araddr,
    input  [2:0]        s_axi_arprot,
    input               s_axi_arvalid,
    output              s_axi_arready,

    output [DATA_W-1:0] s_axi_rdata,
    output [1:0]        s_axi_rresp,
    output              s_axi_rvalid,
    input               s_axi_rready,

    output              irq
);
    // ---- control_top bus ----
    reg  [CTRL_ADDR_W-1:0] ctrl_addr;
    reg  [31:0]            ctrl_wdata;
    wire [31:0]            ctrl_rdata;
    reg                    ctrl_we, ctrl_re;
    wire                   ctrl_ack;

    // ---- vision side ----
    reg                vis_classify;
    wire               vis_busy, vis_result_done, vis_digit_valid;
    wire [3:0]         vis_digit;
    reg                vis_img_we;
    reg  [9:0]         vis_img_addr;
    reg  signed [15:0] vis_img_wdata;

    // status assembled combinationally from live sequencer outputs
    wire [31:0] vis_status = {26'd0, vis_digit, vis_digit_valid, vis_busy};

    // ---- AXI FSM ----
    localparam S_IDLE  = 3'd0,
               S_WBUS  = 3'd1,
               S_BRESP = 3'd2,
               S_RBUS  = 3'd3,
               S_RRESP = 3'd4;

    reg [2:0]        state;
    reg [DATA_W-1:0] rdata_lat;
    reg [ADDR_W-1:0] addr_lat;

    wire aw_w_fire = s_axi_awvalid && s_axi_wvalid && (state == S_IDLE);
    wire ar_fire   = s_axi_arvalid && !aw_w_fire && (state == S_IDLE);

    assign s_axi_awready = aw_w_fire;
    assign s_axi_wready  = aw_w_fire;
    assign s_axi_arready = ar_fire;

    assign s_axi_bvalid  = (state == S_BRESP);
    assign s_axi_bresp   = 2'b00;

    assign s_axi_rvalid  = (state == S_RRESP);
    assign s_axi_rdata   = rdata_lat;
    assign s_axi_rresp   = 2'b00;

    // a write/read targets the vision bank if address[11:8] == 0x3
    wire aw_is_vis = (s_axi_awaddr[11:8] == 4'h3);
    wire ar_is_vis = (s_axi_araddr[11:8] == 4'h3);

    always @(posedge s_axi_aclk or negedge s_axi_aresetn) begin
        if (!s_axi_aresetn) begin
            state         <= S_IDLE;
            ctrl_addr     <= {CTRL_ADDR_W{1'b0}};
            ctrl_wdata    <= 32'd0;
            ctrl_we       <= 1'b0;
            ctrl_re       <= 1'b0;
            rdata_lat     <= 32'd0;
            addr_lat      <= {ADDR_W{1'b0}};
            vis_classify  <= 1'b0;
            vis_img_we    <= 1'b0;
            vis_img_addr  <= 10'd0;
            vis_img_wdata <= 16'd0;
        end else begin
            // one-cycle pulses default low
            ctrl_we      <= 1'b0;
            ctrl_re      <= 1'b0;
            vis_classify <= 1'b0;
            vis_img_we   <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (aw_w_fire) begin
                        addr_lat <= s_axi_awaddr;
                        if (aw_is_vis) begin
                            // ---- vision write: complete immediately ----
                            case (s_axi_awaddr[7:0])
                                8'h00: begin   // VIS_PIX
                                    vis_img_addr  <= s_axi_wdata[25:16];
                                    vis_img_wdata <= s_axi_wdata[15:0];
                                    vis_img_we    <= 1'b1;
                                end
                                8'h04: vis_classify <= s_axi_wdata[0];  // VIS_CTRL
                                default: ;
                            endcase
                            state <= S_BRESP;
                        end else begin
                            // ---- control write: forward to control_top ----
                            ctrl_addr  <= s_axi_awaddr[CTRL_ADDR_W-1:0];
                            ctrl_wdata <= s_axi_wdata;
                            ctrl_we    <= 1'b1;
                            state      <= S_WBUS;
                        end
                    end else if (ar_fire) begin
                        addr_lat <= s_axi_araddr;
                        if (ar_is_vis) begin
                            // ---- vision read: complete immediately ----
                            case (s_axi_araddr[7:0])
                                8'h08:   rdata_lat <= vis_status;
                                default: rdata_lat <= 32'h0;
                            endcase
                            state <= S_RRESP;
                        end else begin
                            ctrl_addr <= s_axi_araddr[CTRL_ADDR_W-1:0];
                            ctrl_re   <= 1'b1;
                            state     <= S_RBUS;
                        end
                    end
                end

                S_WBUS:  if (ctrl_ack) state <= S_BRESP;
                S_BRESP: if (s_axi_bready) state <= S_IDLE;
                S_RBUS:  if (ctrl_ack) begin
                    rdata_lat <= ctrl_rdata;
                    state     <= S_RRESP;
                end
                S_RRESP: if (s_axi_rready) state <= S_IDLE;
                default: state <= S_IDLE;
            endcase
        end
    end

    // ---- SoC: control_top_shared + vision_top on the one shared fabric ----
    sage16_vision_soc #(.ADDR_W(CTRL_ADDR_W)) u_soc (
        .clk   (s_axi_aclk),
        .rst_n (s_axi_aresetn),
        .addr  (ctrl_addr),
        .wdata (ctrl_wdata),
        .rdata (ctrl_rdata),
        .we    (ctrl_we),
        .re    (ctrl_re),
        .ack   (ctrl_ack),
        .irq   (irq),
        .vis_classify   (vis_classify),
        .vis_busy       (vis_busy),
        .vis_result_done(vis_result_done),
        .vis_digit      (vis_digit),
        .vis_digit_valid(vis_digit_valid),
        .vis_img_we     (vis_img_we),
        .vis_img_addr   (vis_img_addr),
        .vis_img_wdata  (vis_img_wdata)
    );
endmodule
