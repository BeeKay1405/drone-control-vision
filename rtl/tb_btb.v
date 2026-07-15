`timescale 1ns/1ps
module tb_btb;
  localparam DATA_W=16, ACC_W=32;
  reg clk=0,rst_n=0; always #5 clk=~clk;
  reg vis_valid=0; reg [1:0] vis_mode=2'd3;
  reg [16*DATA_W-1:0] vis_a=0, vis_b=0;
  wire vis_gnt, vis_done; wire [16*ACC_W-1:0] vis_result;
  wire s_start,s_done; wire [1:0] s_mode;
  wire [16*DATA_W-1:0] s_mm_a,s_mm_b; wire signed [15:0] s1w,s1x,s1y,s1z;
  wire [16*DATA_W-1:0] s_q2; wire [16*ACC_W-1:0] s_result;
  sage16_scheduler u_s(.clk(clk),.rst_n(rst_n),
    .ac_valid(1'b0),.ac_mode(2'd0),.ac_q1_w(16'd0),.ac_q1_x(16'd0),.ac_q1_y(16'd0),.ac_q1_z(16'd0),.ac_q2_flat(0),.ac_gnt(),.ac_done(),.ac_result(),
    .mm_valid(1'b0),.mm_mode(2'd0),.mm_a_flat(0),.mm_b_flat(0),.mm_gnt(),.mm_done(),.mm_result(),
    .vis_valid(vis_valid),.vis_mode(vis_mode),.vis_a_flat(vis_a),.vis_b_flat(vis_b),
    .vis_q1_w(16'd0),.vis_q1_x(16'd0),.vis_q1_y(16'd0),.vis_q1_z(16'd0),.vis_q2_flat(0),
    .vis_gnt(vis_gnt),.vis_done(vis_done),.vis_result(vis_result),
    .s_start(s_start),.s_mode(s_mode),.s_mm_a(s_mm_a),.s_mm_b(s_mm_b),
    .s_q1_w(s1w),.s_q1_x(s1x),.s_q1_y(s1y),.s_q1_z(s1z),.s_q2_flat(s_q2),.s_done(s_done),.s_result(s_result));
  sage16_top u_f(.clk(clk),.rst_n(rst_n),.start(s_start),.mode(s_mode),.done(s_done),.mode_out(),
    .mm_a(s_mm_a),.mm_b(s_mm_b),.cv_img(0),.cv_k(0),
    .qt_q1_w(s1w),.qt_q1_x(s1x),.qt_q1_y(s1y),.qt_q1_z(s1z),.qt_q2(s_q2),.c_out(s_result));
  integer i;
  task do_op(input [16*DATA_W-1:0] a, input [16*DATA_W-1:0] b);
    begin
      @(posedge clk); vis_a<=a; vis_b<=b; vis_mode<=2'd3; vis_valid<=1;
      wait(vis_gnt); @(posedge clk); vis_valid<=0;
      wait(vis_done); @(posedge clk);
      $display("  result[0]=%0d", $signed(vis_result[0+:32]));
    end
  endtask
  // A = diag-ish; make two clearly different ops:
  // op1: A=2*I, B=I -> C=2*I -> result[0]=2
  // op2: A=5*I, B=I -> C=5*I -> result[0]=5
  reg [16*DATA_W-1:0] Aa,Bb,Ac;
  initial begin
    // identity B
    Bb=0; Bb[(0*4+0)*16+:16]=16'd1; Bb[(1*4+1)*16+:16]=16'd1; Bb[(2*4+2)*16+:16]=16'd1; Bb[(3*4+3)*16+:16]=16'd1;
    Aa=0; Aa[(0*4+0)*16+:16]=16'd2; Aa[(1*4+1)*16+:16]=16'd2; Aa[(2*4+2)*16+:16]=16'd2; Aa[(3*4+3)*16+:16]=16'd2;
    Ac=0; Ac[(0*4+0)*16+:16]=16'd5; Ac[(1*4+1)*16+:16]=16'd5; Ac[(2*4+2)*16+:16]=16'd5; Ac[(3*4+3)*16+:16]=16'd5;
    rst_n=0; repeat(4)@(posedge clk); rst_n=1;
    $display("op1 expect 2:"); do_op(Aa,Bb);
    $display("op2 expect 5:"); do_op(Ac,Bb);
    $display("op3 expect 2:"); do_op(Aa,Bb);
    $finish;
  end
endmodule
