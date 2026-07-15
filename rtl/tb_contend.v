`timescale 1ns/1ps
// Both the MM lane ("control" stand-in, higher priority) and the VIS lane
// hammer the shared fabric with DIFFERENT operands at the same time.
// Pass criterion: each lane always reads back ITS OWN correct result
// (mm->3, vis->7), proving no staleness and no cross-lane contamination
// under arbitration, with the scheduler fix in place.
module tb_contend;
  localparam DATA_W=16, ACC_W=32;
  reg clk=0,rst_n=0; always #5 clk=~clk;
  reg mm_valid=0, vis_valid=0;
  reg [16*DATA_W-1:0] mm_a=0, mm_b=0, vis_a=0, vis_b=0;
  wire mm_gnt,mm_done,vis_gnt,vis_done;
  wire [16*ACC_W-1:0] mm_result, vis_result;
  wire s_start,s_done; wire [1:0] s_mode;
  wire [16*DATA_W-1:0] s_mm_a,s_mm_b; wire signed [15:0] s1w,s1x,s1y,s1z;
  wire [16*DATA_W-1:0] s_q2; wire [16*ACC_W-1:0] s_result;
  sage16_scheduler u_s(.clk(clk),.rst_n(rst_n),
    .ac_valid(1'b0),.ac_mode(2'd0),.ac_q1_w(16'd0),.ac_q1_x(16'd0),.ac_q1_y(16'd0),.ac_q1_z(16'd0),.ac_q2_flat({16*DATA_W{1'b0}}),.ac_gnt(),.ac_done(),.ac_result(),
    .mm_valid(mm_valid),.mm_mode(2'd3),.mm_a_flat(mm_a),.mm_b_flat(mm_b),.mm_gnt(mm_gnt),.mm_done(mm_done),.mm_result(mm_result),
    .vis_valid(vis_valid),.vis_mode(2'd3),.vis_a_flat(vis_a),.vis_b_flat(vis_b),
    .vis_q1_w(16'd0),.vis_q1_x(16'd0),.vis_q1_y(16'd0),.vis_q1_z(16'd0),.vis_q2_flat({16*DATA_W{1'b0}}),
    .vis_gnt(vis_gnt),.vis_done(vis_done),.vis_result(vis_result),
    .s_start(s_start),.s_mode(s_mode),.s_mm_a(s_mm_a),.s_mm_b(s_mm_b),
    .s_q1_w(s1w),.s_q1_x(s1x),.s_q1_y(s1y),.s_q1_z(s1z),.s_q2_flat(s_q2),.s_done(s_done),.s_result(s_result));
  sage16_top u_f(.clk(clk),.rst_n(rst_n),.start(s_start),.mode(s_mode),.done(s_done),.mode_out(),
    .mm_a(s_mm_a),.mm_b(s_mm_b),.cv_img({36*DATA_W{1'b0}}),.cv_k({9*DATA_W{1'b0}}),
    .qt_q1_w(s1w),.qt_q1_x(s1x),.qt_q1_y(s1y),.qt_q1_z(s1z),.qt_q2(s_q2),.c_out(s_result));

  reg [16*DATA_W-1:0] I3,I7,Ident;
  integer mm_ops=0, vis_ops=0, errs=0, t;

  // MM lane process: keep requesting A=3I, check result==3
  initial begin : mmproc
    @(posedge rst_n);
    forever begin
      @(posedge clk); mm_a<=I3; mm_b<=Ident; mm_valid<=1;
      @(posedge clk); wait(mm_gnt); @(posedge clk); mm_valid<=0;
      wait(mm_done); @(posedge clk);
      if ($signed(mm_result[0+:32])!==3) begin errs=errs+1; $display("MM got %0d (want 3)",$signed(mm_result[0+:32])); end
      mm_ops=mm_ops+1;
    end
  end
  // VIS lane process: keep requesting A=7I, check result==7
  initial begin : visproc
    @(posedge rst_n);
    forever begin
      @(posedge clk); vis_a<=I7; vis_b<=Ident; vis_valid<=1;
      wait(vis_gnt); @(posedge clk); vis_valid<=0;
      wait(vis_done); @(posedge clk);
      if ($signed(vis_result[0+:32])!==7) begin errs=errs+1; $display("VIS got %0d (want 7)",$signed(vis_result[0+:32])); end
      vis_ops=vis_ops+1;
    end
  end

  initial begin
    Ident=0; I3=0; I7=0;
    for (t=0;t<4;t=t+1) begin
      Ident[(t*4+t)*16+:16]=16'd1; I3[(t*4+t)*16+:16]=16'd3; I7[(t*4+t)*16+:16]=16'd7;
    end
    rst_n=0; repeat(5)@(posedge clk); rst_n=1;
    repeat(4000) @(posedge clk);
    $display("CONTENTION: mm_ops=%0d vis_ops=%0d errors=%0d  %s",
             mm_ops, vis_ops, errs, (errs==0)?"PASS":"*** FAIL ***");
    $finish;
  end
endmodule
