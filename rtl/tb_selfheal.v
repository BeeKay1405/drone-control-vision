`timescale 1ns/1ps
// tb_selfheal.v — verifies the IN-FABRIC self-heal path that tb_btb/tb_vision/tb_soc
// leave unexercised: sage16_top's debug fault injection (dbg_fault_*) and the
// erasure-ABFT repair mux (sage_en).
//
// For every PE (16) x fault mode (sa0 / sa1 / bit28-flip):
//   pass A: fault ON, sage_en=0 -> faulted lane DIFFERS from golden, other 15 match
//           (containment through the debug mux)
//   pass B: fault ON, sage_en=1 -> ALL 16 lanes == golden (1-subtraction repair)
// Expected: 48/48 A + 48/48 B.
module tb_selfheal;
  localparam DATA_W=16, ACC_W=32;
  reg clk=0,rst_n=0; always #5 clk=~clk;
  reg vis_valid=0; reg [1:0] vis_mode=2'd0;             // unsigned MM
  reg [16*DATA_W-1:0] vis_a=0, vis_b=0;
  wire vis_gnt, vis_done; wire [16*ACC_W-1:0] vis_result;
  wire s_start,s_done; wire [1:0] s_mode;
  wire [16*DATA_W-1:0] s_mm_a,s_mm_b; wire signed [15:0] s1w,s1x,s1y,s1z;
  wire [16*DATA_W-1:0] s_q2; wire [16*ACC_W-1:0] s_result;

  // debug fault-injection + heal controls (the DUT of this bench)
  reg        dbg_en=0;  reg [3:0] dbg_pe=0;  reg [1:0] dbg_mode=0;  reg sage=0;

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
    .qt_q1_w(s1w),.qt_q1_x(s1x),.qt_q1_y(s1y),.qt_q1_z(s1z),.qt_q2(s_q2),.c_out(s_result),
    .dbg_fault_en(dbg_en),.dbg_fault_pe(dbg_pe),.dbg_fault_mode(dbg_mode),.sage_en(sage));

  reg [16*ACC_W-1:0] golden, got;
  integer i, j, k, m, bad, passA, failA, passB, failB;

  task do_op; begin
      @(posedge clk); vis_valid<=1;
      wait(vis_gnt); @(posedge clk); vis_valid<=0;
      wait(vis_done); @(posedge clk);
      got = vis_result;
  end endtask

  initial begin
    // general matrices: every C entry nonzero (so sa0/sa1 corruption is always visible)
    for (i=0;i<4;i=i+1) for (j=0;j<4;j=j+1) begin
      vis_a[(i*4+j)*16 +: 16] = i+j+1;   // A[i][k]
      vis_b[(i*4+j)*16 +: 16] = i+j+2;   // B[k][j]
    end
    rst_n=0; repeat(4)@(posedge clk); rst_n=1;

    // ---- golden (no fault) ----
    dbg_en=0; sage=0; do_op; golden=got;
    $display("SELF-HEAL BENCH: golden captured (C[0][0]=%0d)", golden[31:0]);

    passA=0; failA=0; passB=0; failB=0;
    for (k=0;k<16;k=k+1) begin
      for (m=0;m<3;m=m+1) begin
        dbg_pe=k[3:0]; dbg_mode=m[1:0];
        // pass A: fault on, heal OFF -> exactly lane k wrong
        dbg_en=1; sage=0; do_op;
        bad=0; for (i=0;i<16;i=i+1)
          if (got[i*ACC_W +: ACC_W] !== golden[i*ACC_W +: ACC_W]) bad=bad+1;
        if (bad==1 && (got[k*ACC_W +: ACC_W] !== golden[k*ACC_W +: ACC_W])) passA=passA+1;
        else begin failA=failA+1;
          $display("  A-FAIL pe=%0d mode=%0d (bad=%0d)", k, m, bad); end
        // pass B: same fault, heal ON -> all 16 match golden
        dbg_en=1; sage=1; do_op;
        bad=0; for (i=0;i<16;i=i+1)
          if (got[i*ACC_W +: ACC_W] !== golden[i*ACC_W +: ACC_W]) bad=bad+1;
        if (bad==0) passB=passB+1;
        else begin failB=failB+1;
          $display("  B-FAIL pe=%0d mode=%0d (bad=%0d, lane=%0d vs %0d)", k, m, bad,
                   got[k*ACC_W +: ACC_W], golden[k*ACC_W +: ACC_W]); end
        dbg_en=0; sage=0;
      end
    end
    $display("-----------------------------------------------");
    $display("  inject visible (sage off): %0d/48", passA);
    $display("  healed exact  (sage on)  : %0d/48", passB);
    if (failA==0 && failB==0) $display("SELF-HEAL RESULT: PASS 96/96");
    else                      $display("SELF-HEAL RESULT: FAIL");
    $finish;
  end
  initial begin #20000000; $display("TIMEOUT"); $finish; end
endmodule
