// =============================================================================
// vision_top.v -- host-free MNIST (MiniCNN-v2) sequencer for the SAGE-16 fabric.
//
// BRAM-FRIENDLY VERSION.
// Large memories (weight ROMs, image, pooled maps, activations) are read ONE
// word per cycle into the operand registers, so Vivado maps them to true block
// RAM (1-2 read ports) instead of collapsing them into LUT mux trees. The
// previous version assembled a 36-word conv patch and a 16-word FC operand
// combinationally in a single cycle, which forced distributed (LUT) memory and
// over-utilised the F7/F8 muxes on the PYNQ-Z1 (xc7z020).
//
// Functionally identical to the verified design: same topology, same integer
// arithmetic, same 12/12 golden results. Only operand-fetch *timing* changed
// (a short per-op serial "fill" before each fabric op).
//
//   26x26 int16 image
//     Conv1 3x3 1->8 (valid) fused with Pool4 4x4 (/16 trunc)  -> 8x6x6
//     Conv2 3x3 8->16 (valid)                                  -> 16x4x4
//     >>3 (arith floor); flatten CHW -> 256
//     FC1 256->32 (signed) -> ReLU ; FC2 32->12 (signed); argmax -> digit
//
//   conv ops -> MODE_CV  (cv_img/cv_k direct to fabric)
//   FC ops   -> MODE_MMS (signed matmul; operands on vis_a/vis_b)
//
// Fill timing model: set *_ra at cycle c -> *_rd valid at cycle c+1.
// A fill loop with counter c issues read c and captures read c-1.
// =============================================================================

module vision_top #(
    parameter DATA_W = 16,
    parameter ACC_W  = 32,
    parameter CONV1_FILE = "conv1_k.mem",
    parameter CONV2_FILE = "conv2_k.mem",
    parameter W1_FILE    = "w1.mem",
    parameter W2_FILE    = "w2.mem"
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    classify,
    output reg                     busy,
    output reg                     done,
    output reg  [3:0]              result,
    output reg                     result_valid,
    input  wire                    img_we,
    input  wire [9:0]              img_addr,
    input  wire signed [DATA_W-1:0] img_wdata,
    output reg                     vis_valid,
    output reg  [1:0]              vis_mode,
    output reg  [16*DATA_W-1:0]    vis_a_flat,
    output reg  [16*DATA_W-1:0]    vis_b_flat,
    input  wire                    vis_gnt,
    input  wire                    vis_done,
    input  wire [16*ACC_W-1:0]     vis_result,
    output reg  [36*DATA_W-1:0]    cv_img,
    output reg  [ 9*DATA_W-1:0]    cv_k
);
    localparam [1:0] MODE_CV = 2'd1, MODE_MMS = 2'd3;

    // ---------------- memories (block-RAM friendly) ----------------
    (* ram_style = "block" *) reg signed [DATA_W-1:0] img_mem  [0:675];
    (* ram_style = "block" *) reg signed [DATA_W-1:0] pool_mem [0:287];
    (* ram_style = "block" *) reg signed [DATA_W-1:0] act_mem  [0:255];
                              reg signed [DATA_W-1:0] fc1_mem  [0:31];
                              reg signed [ACC_W-1:0]  fc2_mem  [0:11];

    (* rom_style = "block" *) reg signed [DATA_W-1:0] conv1_rom [0:71];
    (* rom_style = "block" *) reg signed [DATA_W-1:0] conv2_rom [0:1151];
    (* rom_style = "block" *) reg signed [DATA_W-1:0] w1_rom    [0:8191];
    (* rom_style = "block" *) reg signed [DATA_W-1:0] w2_rom    [0:383];

    initial begin
        $readmemh(CONV1_FILE, conv1_rom);
        $readmemh(CONV2_FILE, conv2_rom);
        $readmemh(W1_FILE,    w1_rom);
        $readmemh(W2_FILE,    w2_rom);
    end

    always @(posedge clk) if (img_we) img_mem[img_addr] <= img_wdata;

    // ---------------- registered-data read ports; COMBINATIONAL addresses ----
    // One register total per memory (the data out) = clean 1-cycle BRAM read:
    // address set in cycle c -> data valid in cycle c+1.
    reg  [9:0]  img_ra;   reg signed [DATA_W-1:0] img_rd;
    reg  [8:0]  pool_ra;  reg signed [DATA_W-1:0] pool_rd;
    reg  [7:0]  act_ra;   reg signed [DATA_W-1:0] act_rd;
    reg  [4:0]  fc1_ra;   reg signed [DATA_W-1:0] fc1_rd;
    reg  [6:0]  c1k_ra;   reg signed [DATA_W-1:0] c1k_rd;
    reg  [10:0] c2k_ra;   reg signed [DATA_W-1:0] c2k_rd;
    reg  [12:0] w1_ra;    reg signed [DATA_W-1:0] w1_rd;
    reg  [8:0]  w2_ra;    reg signed [DATA_W-1:0] w2_rd;

    always @(posedge clk) begin
        img_rd  <= img_mem [img_ra];
        pool_rd <= pool_mem[pool_ra];
        act_rd  <= act_mem [act_ra];
        fc1_rd  <= fc1_mem [fc1_ra];
        c1k_rd  <= conv1_rom[c1k_ra];
        c2k_rd  <= conv2_rom[c2k_ra];
        w1_rd   <= w1_rom  [w1_ra];
        w2_rd   <= w2_rom  [w2_ra];
    end

    // ---------------- FSM ----------------
    localparam [2:0] P_IDLE=3'd0, P_CONV1=3'd1, P_CONV2=3'd2,
                     P_FC1=3'd3, P_FC2=3'd4, P_ARGMAX=3'd5, P_DONE=3'd6;
    localparam [1:0] T_FILL=2'd0, T_REQ=2'd1, T_WAIT=2'd2, T_POST=2'd3;

    reg [2:0] phase;
    reg [1:0] step;

    reg [4:0] co;
    reg [3:0] ci;
    reg [2:0] ty, tx;
    reg [5:0] oc;
    reg [8:0] lb;

    reg [5:0] fc;     // fill counter

    reg signed [ACC_W+7:0] c2_acc [0:15];
    reg signed [ACC_W+7:0] f_acc  [0:3];
    reg signed [ACC_W+7:0] conv1_sum;

    reg signed [ACC_W-1:0] best_val;
    reg [3:0]              best_idx;
    reg [3:0]              am_i;

    integer n;

    function signed [DATA_W-1:0] sat16; input signed [ACC_W+7:0] v;
        begin
            if (v >  32767)      sat16 =  16'sd32767;
            else if (v < -32768) sat16 = -16'sd32768;
            else                 sat16 = v[DATA_W-1:0];
        end
    endfunction

    // next conv-patch address for image (index p=0..35): (4ty+p/6)*26 + (4tx+p%6)
    function [9:0] c1_img_addr; input [5:0] p; input [2:0] ity, itx;
        c1_img_addr = (4*ity + (p/6))*26 + (4*itx + (p%6));
    endfunction

    // -------- combinational read addresses (driven from FSM state + fc) ------
    always @(*) begin
        img_ra  = 10'd0;
        pool_ra = 9'd0;
        act_ra  = 8'd0;
        fc1_ra  = 5'd0;
        c1k_ra  = 7'd0;
        c2k_ra  = 11'd0;
        w1_ra   = 13'd0;
        w2_ra   = 9'd0;
        case (phase)
            P_CONV1: begin
                if (fc < 36) img_ra = c1_img_addr(fc, ty, tx);
                if (fc < 9)  c1k_ra = co*9 + fc;
            end
            P_CONV2: begin
                if (fc < 36) pool_ra = ci*36 + fc;
                if (fc < 9)  c2k_ra  = (co*8 + ci)*9 + fc;
            end
            P_FC1: begin
                if (fc < 4)  act_ra = lb + fc;
                if (fc < 16) w1_ra  = (oc + (fc % 4))*256 + (lb + (fc / 4));
            end
            P_FC2: begin
                if (fc < 4)  fc1_ra = lb + fc;
                if (fc < 16) w2_ra  = (oc + (fc % 4))*32 + (lb + (fc / 4));
            end
            default: ;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase<=P_IDLE; step<=T_FILL;
            busy<=0; done<=0; result<=0; result_valid<=0;
            vis_valid<=0; vis_mode<=MODE_CV;
            co<=0; ci<=0; ty<=0; tx<=0; oc<=0; lb<=0; fc<=0;
            cv_img<=0; cv_k<=0; vis_a_flat<=0; vis_b_flat<=0;
        end else begin
            done <= 0;

            case (phase)
            // =====================================================
            P_IDLE: begin
                busy <= 0;
                if (classify) begin
                    busy<=1; result_valid<=0;
                    co<=0; ci<=0; ty<=0; tx<=0; oc<=0; lb<=0; fc<=0;
                    phase<=P_CONV1; step<=T_FILL;
                end
            end

            // =====================================================
            // CONV1 fused with Pool4.  Fill: read 36 image words + 9 kernel
            // words, one per cycle. counter fc = 0..36.
            //   at fc: issue read fc (if in range); capture read fc-1.
            // =====================================================
            P_CONV1: case (step)
                T_FILL: begin
                    // capture read fc-1
                    if (fc >= 1 && fc <= 36) cv_img[(fc-1)*DATA_W +: DATA_W] <= img_rd;
                    if (fc >= 1 && fc <= 9)  cv_k[(fc-1)*DATA_W +: DATA_W]  <= c1k_rd;
                    if (fc == 36) begin
                        fc <= 0;
                        vis_mode<=MODE_CV; vis_valid<=1'b1; step<=T_REQ;
                    end else fc <= fc + 1;
                end
                T_REQ:  if (vis_gnt) begin vis_valid<=0; step<=T_WAIT; end
                T_WAIT: if (vis_done) begin
                    conv1_sum = 0;
                    for (n=0; n<16; n=n+1)
                        conv1_sum = conv1_sum + $signed(vis_result[n*ACC_W +: ACC_W]);
                    pool_mem[co*36 + ty*6 + tx] <=
                        (conv1_sum >= 0) ? sat16( conv1_sum >>> 4)
                                         : -sat16((-conv1_sum) >>> 4);
                    step <= T_POST;
                end
                T_POST: begin
                    if (tx != 3'd5) tx <= tx + 1;
                    else begin
                        tx <= 0;
                        if (ty != 3'd5) ty <= ty + 1;
                        else begin
                            ty <= 0;
                            if (co != 5'd7) co <= co + 1;
                            else begin co <= 0; ci <= 0; phase <= P_CONV2; end
                        end
                    end
                    fc <= 0; step <= T_FILL;
                end
            endcase

            // =====================================================
            // CONV2: fill 36 pool words + 9 kernel words.
            // =====================================================
            P_CONV2: case (step)
                T_FILL: begin
                    if (fc == 0 && ci == 0)
                        for (n=0; n<16; n=n+1) c2_acc[n] <= 0;
                    if (fc >= 1 && fc <= 36) cv_img[(fc-1)*DATA_W +: DATA_W] <= pool_rd;
                    if (fc >= 1 && fc <= 9)  cv_k[(fc-1)*DATA_W +: DATA_W]  <= c2k_rd;
                    if (fc == 36) begin
                        fc <= 0;
                        vis_mode<=MODE_CV; vis_valid<=1'b1; step<=T_REQ;
                    end else fc <= fc + 1;
                end
                T_REQ:  if (vis_gnt) begin vis_valid<=0; step<=T_WAIT; end
                T_WAIT: if (vis_done) begin
                    for (n=0; n<16; n=n+1)
                        c2_acc[n] <= c2_acc[n] + $signed(vis_result[n*ACC_W +: ACC_W]);
                    step <= T_POST;
                end
                T_POST: begin
                    if (ci != 4'd7) ci <= ci + 1;
                    else begin
                        ci <= 0;
                        for (n=0; n<16; n=n+1)
                            act_mem[co*16 + n] <= sat16(c2_acc[n] >>> 3);
                        if (co != 5'd15) co <= co + 1;
                        else begin co <= 0; oc <= 0; lb <= 0; phase <= P_FC1; end
                    end
                    fc <= 0; step <= T_FILL;
                end
            endcase

            // =====================================================
            // FC1: fill A (4 act words, replicated to 4 rows) + B (16 W1 words).
            // counter fc = 0..16.  A index = fc (0..3); B flat index = fc (0..15).
            //   B[q]=W1[(oc + q%4)*256 + (lb + q/4)]   (transpose from out-major)
            // =====================================================
            P_FC1: case (step)
                T_FILL: begin
                    if (fc == 0 && lb == 0)
                        for (n=0; n<4; n=n+1) f_acc[n] <= 0;
                    if (fc >= 1 && fc <= 4) begin
                        vis_a_flat[(0*4+(fc-1))*DATA_W +: DATA_W] <= act_rd;
                        vis_a_flat[(1*4+(fc-1))*DATA_W +: DATA_W] <= act_rd;
                        vis_a_flat[(2*4+(fc-1))*DATA_W +: DATA_W] <= act_rd;
                        vis_a_flat[(3*4+(fc-1))*DATA_W +: DATA_W] <= act_rd;
                    end
                    if (fc >= 1 && fc <= 16)
                        vis_b_flat[(fc-1)*DATA_W +: DATA_W] <= w1_rd;
                    if (fc == 16) begin
                        fc <= 0;
                        vis_mode<=MODE_MMS; vis_valid<=1'b1; step<=T_REQ;
                    end else fc <= fc + 1;
                end
                T_REQ:  if (vis_gnt) begin vis_valid<=0; step<=T_WAIT; end
                T_WAIT: if (vis_done) begin
                    for (n=0; n<4; n=n+1)
                        f_acc[n] <= f_acc[n] + $signed(vis_result[n*ACC_W +: ACC_W]);
                    step <= T_POST;
                end
                T_POST: begin
                    if (lb != 9'd252) lb <= lb + 4;
                    else begin
                        lb <= 0;
                        for (n=0; n<4; n=n+1)
                            fc1_mem[oc + n] <= (f_acc[n] < 0) ? 16'sd0 : sat16(f_acc[n]);
                        if (oc != 6'd28) oc <= oc + 4;
                        else begin oc <= 0; lb <= 0; phase <= P_FC2; end
                    end
                    fc <= 0; step <= T_FILL;
                end
            endcase

            // =====================================================
            // FC2: same structure, W2 (out-major 12x32) transpose.
            //   B[q]=W2[(oc + q%4)*32 + (lb + q/4)]
            // =====================================================
            P_FC2: case (step)
                T_FILL: begin
                    if (fc == 0 && lb == 0)
                        for (n=0; n<4; n=n+1) f_acc[n] <= 0;
                    if (fc >= 1 && fc <= 4) begin
                        vis_a_flat[(0*4+(fc-1))*DATA_W +: DATA_W] <= fc1_rd;
                        vis_a_flat[(1*4+(fc-1))*DATA_W +: DATA_W] <= fc1_rd;
                        vis_a_flat[(2*4+(fc-1))*DATA_W +: DATA_W] <= fc1_rd;
                        vis_a_flat[(3*4+(fc-1))*DATA_W +: DATA_W] <= fc1_rd;
                    end
                    if (fc >= 1 && fc <= 16)
                        vis_b_flat[(fc-1)*DATA_W +: DATA_W] <= w2_rd;
                    if (fc == 16) begin
                        fc <= 0;
                        vis_mode<=MODE_MMS; vis_valid<=1'b1; step<=T_REQ;
                    end else fc <= fc + 1;
                end
                T_REQ:  if (vis_gnt) begin vis_valid<=0; step<=T_WAIT; end
                T_WAIT: if (vis_done) begin
                    for (n=0; n<4; n=n+1)
                        f_acc[n] <= f_acc[n] + $signed(vis_result[n*ACC_W +: ACC_W]);
                    step <= T_POST;
                end
                T_POST: begin
                    if (lb != 9'd28) lb <= lb + 4;
                    else begin
                        lb <= 0;
                        for (n=0; n<4; n=n+1)
                            fc2_mem[oc + n] <= f_acc[n][ACC_W-1:0];
                        if (oc != 6'd8) oc <= oc + 4;
                        else begin
                            phase<=P_ARGMAX;
                            best_val<=-32'sh7FFFFFFF; best_idx<=0; am_i<=0;
                        end
                    end
                    fc <= 0; step <= T_FILL;
                end
            endcase

            // =====================================================
            P_ARGMAX: begin
                if (fc2_mem[am_i] > best_val) begin
                    best_val <= fc2_mem[am_i];
                    best_idx <= am_i;
                end
                if (am_i == 4'd9) phase <= P_DONE;
                else am_i <= am_i + 1;
            end

            P_DONE: begin
                result<=best_idx; result_valid<=1'b1;
                done<=1'b1; busy<=1'b0;
                phase<=P_IDLE; step<=T_FILL;
            end

            default: phase <= P_IDLE;
            endcase
        end
    end
endmodule
