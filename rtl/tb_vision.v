`timescale 1ns/1ps
`default_nettype none
module tb_vision;
    localparam DATA_W=16, ACC_W=32;
    reg clk=0, rst_n=0;
    always #5 clk = ~clk;

    // image load
    reg         img_we=0;
    reg [9:0]   img_addr=0;
    reg signed [15:0] img_wdata=0;

    // vision control
    reg  classify=0;
    wire busy, done, result_valid;
    wire [3:0] result;

    // vision <-> scheduler
    wire        vis_valid, vis_gnt, vis_done;
    wire [1:0]  vis_mode;
    wire [16*DATA_W-1:0] vis_a, vis_b;
    wire [16*ACC_W-1:0]  vis_result;
    // vision -> fabric (Option A direct conv operands)
    wire [36*DATA_W-1:0] cv_img;
    wire [ 9*DATA_W-1:0] cv_k;

    // scheduler -> fabric
    wire        s_start, s_done;
    wire [1:0]  s_mode;
    wire [16*DATA_W-1:0] s_mm_a, s_mm_b;
    wire signed [DATA_W-1:0] s_q1_w,s_q1_x,s_q1_y,s_q1_z;
    wire [16*DATA_W-1:0] s_q2_flat;
    wire [16*ACC_W-1:0]  s_result;

    vision_top u_vis (
        .clk(clk), .rst_n(rst_n),
        .classify(classify), .busy(busy), .done(done),
        .result(result), .result_valid(result_valid),
        .img_we(img_we), .img_addr(img_addr), .img_wdata(img_wdata),
        .vis_valid(vis_valid), .vis_mode(vis_mode),
        .vis_a_flat(vis_a), .vis_b_flat(vis_b),
        .vis_gnt(vis_gnt), .vis_done(vis_done), .vis_result(vis_result),
        .cv_img(cv_img), .cv_k(cv_k)
    );

    sage16_scheduler u_sched (
        .clk(clk), .rst_n(rst_n),
        // control lanes OFF for the pure-vision test
        .ac_valid(1'b0), .ac_mode(2'd0),
        .ac_q1_w(16'd0),.ac_q1_x(16'd0),.ac_q1_y(16'd0),.ac_q1_z(16'd0),
        .ac_q2_flat({16*DATA_W{1'b0}}),
        .ac_gnt(), .ac_done(), .ac_result(),
        .mm_valid(1'b0), .mm_mode(2'd0),
        .mm_a_flat({16*DATA_W{1'b0}}), .mm_b_flat({16*DATA_W{1'b0}}),
        .mm_gnt(), .mm_done(), .mm_result(),
        .vis_valid(vis_valid), .vis_mode(vis_mode),
        .vis_a_flat(vis_a), .vis_b_flat(vis_b),
        .vis_q1_w(16'd0),.vis_q1_x(16'd0),.vis_q1_y(16'd0),.vis_q1_z(16'd0),
        .vis_q2_flat({16*DATA_W{1'b0}}),
        .vis_gnt(vis_gnt), .vis_done(vis_done), .vis_result(vis_result),
        .s_start(s_start), .s_mode(s_mode),
        .s_mm_a(s_mm_a), .s_mm_b(s_mm_b),
        .s_q1_w(s_q1_w), .s_q1_x(s_q1_x), .s_q1_y(s_q1_y), .s_q1_z(s_q1_z),
        .s_q2_flat(s_q2_flat),
        .s_done(s_done), .s_result(s_result)
    );

    sage16_top u_fab (
        .clk(clk), .rst_n(rst_n),
        .start(s_start), .mode(s_mode), .done(s_done), .mode_out(),
        .mm_a(s_mm_a), .mm_b(s_mm_b),
        .cv_img(cv_img), .cv_k(cv_k),
        .qt_q1_w(s_q1_w), .qt_q1_x(s_q1_x), .qt_q1_y(s_q1_y), .qt_q1_z(s_q1_z),
        .qt_q2(s_q2_flat),
        .c_out(s_result)
    );

    // ---- test driver ----
    reg signed [15:0] image [0:675];
    integer p, expected, idx, cyc;
    reg [256*8:1] imgfile;

    initial begin
        if (!$value$plusargs("img=%s", imgfile)) imgfile="img0.mem";
        if (!$value$plusargs("exp=%d", expected)) expected=-1;
        $readmemh(imgfile, image);

        rst_n=0; repeat(4) @(posedge clk);
        rst_n=1; @(posedge clk);

        // load image
        for (p=0; p<676; p=p+1) begin
            @(posedge clk);
            img_we<=1; img_addr<=p[9:0]; img_wdata<=image[p];
        end
        @(posedge clk); img_we<=0;

        // classify
        @(posedge clk); classify<=1;
        @(posedge clk); classify<=0;

        // wait for done (timeout guard)
        cyc=0;
        while (!done && cyc<2_000_000) begin @(posedge clk); cyc=cyc+1; end
        if (!done) begin $display("TIMEOUT after %0d cycles", cyc); $finish; end

        $display("RESULT img=%0s pred=%0d expected=%0d cycles=%0d  %s",
                 imgfile, result, expected, cyc,
                 (expected<0)?"(no golden)":((result==expected)?"PASS":"*** FAIL ***"));
        if ($test$plusargs("dump")) begin : dbg
            integer q; integer sp, sa, sf;
            sp=0; for(q=0;q<288;q=q+1) sp=sp+u_vis.pool_mem[q];
            sa=0; for(q=0;q<256;q=q+1) sa=sa+u_vis.act_mem[q];
            sf=0; for(q=0;q<32;q=q+1)  sf=sf+u_vis.fc1_mem[q];
            $display("  HW sum(pool)=%0d sum(act)=%0d sum(fc1)=%0d", sp, sa, sf);
            $write("  HW fc2=[");
            for(q=0;q<12;q=q+1) $write("%0d ", u_vis.fc2_mem[q]);
            $display("]");
            $write("  HW pool[ch0]=[");
            for(q=0;q<36;q=q+1) $write("%0d ", u_vis.pool_mem[q]);
            $display("]");
            $write("  HW act[0:16]=[");
            for(q=0;q<16;q=q+1) $write("%0d ", u_vis.act_mem[q]);
            $display("]");
        end
        $finish;
    end
endmodule
`default_nettype wire
