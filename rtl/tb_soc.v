`timescale 1ns/1ps
`default_nettype none
// Drives a full MNIST classification through the SoC, i.e. through the patched
// control_top's internal (shared) scheduler+fabric via the exposed vis lane.
// Control submodules are inert stubs here, so this isolates the vision path
// across the real integration boundary. Checks the digit against the golden.
module tb_soc;
    localparam DATA_W=16, ACC_W=32;
    reg clk=0, rst_n=0; always #5 clk=~clk;

    reg [9:0] addr=0; reg [31:0] wdata=0; wire [31:0] rdata;
    reg we=0, re=0; wire ack, irq;

    reg vis_classify=0; wire vis_busy, vis_result_done, vis_digit_valid;
    wire [3:0] vis_digit;
    reg vis_img_we=0; reg [9:0] vis_img_addr=0; reg signed [15:0] vis_img_wdata=0;

    sage16_vision_soc u_soc (
        .clk(clk), .rst_n(rst_n),
        .addr(addr), .wdata(wdata), .rdata(rdata), .we(we), .re(re), .ack(ack), .irq(irq),
        .vis_classify(vis_classify), .vis_busy(vis_busy),
        .vis_result_done(vis_result_done), .vis_digit(vis_digit),
        .vis_digit_valid(vis_digit_valid),
        .vis_img_we(vis_img_we), .vis_img_addr(vis_img_addr), .vis_img_wdata(vis_img_wdata)
    );

    reg signed [15:0] image [0:675];
    integer p, expected, cyc;
    reg [256*8:1] imgfile;

    initial begin
        if (!$value$plusargs("img=%s", imgfile)) imgfile="img0.mem";
        if (!$value$plusargs("exp=%d", expected)) expected=-1;
        $readmemh(imgfile, image);
        rst_n=0; repeat(6) @(posedge clk); rst_n=1; @(posedge clk);

        for (p=0; p<676; p=p+1) begin
            @(posedge clk); vis_img_we<=1; vis_img_addr<=p[9:0]; vis_img_wdata<=image[p];
        end
        @(posedge clk); vis_img_we<=0;

        @(posedge clk); vis_classify<=1;
        @(posedge clk); vis_classify<=0;

        cyc=0;
        while (!vis_result_done && cyc<2_000_000) begin @(posedge clk); cyc=cyc+1; end
        if (!vis_result_done) begin $display("TIMEOUT"); $finish; end

        $display("SOC img=%0s pred=%0d expected=%0d cycles=%0d  %s",
                 imgfile, vis_digit, expected, cyc,
                 (expected<0)?"(no golden)":((vis_digit==expected)?"PASS":"*** FAIL ***"));
        $finish;
    end
endmodule
`default_nettype wire
