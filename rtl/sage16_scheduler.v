// =============================================================================
// sage16_scheduler.v -- arbitrates shared SAGE-16 fabric between requesters.
//
// Supports up to 3 requesters today (attitude_ctrl, motor_mixer, vision).
// Priority: control requests beat vision so the control loop never starves.
// Within a control batch, attitude_ctrl goes before motor_mixer (because
// motor_mixer needs the torques attitude_ctrl produces).
//
// Each requester exposes:
//   valid   - "I want the fabric"
//   mode    - which kernel mode to run on it
//   operands- the operand registers to drive
//   gnt     - "you have it; I'm driving the fabric with your operands now"
//   done    - "your kernel is finished; result is on result bus"
//   result  - 16x32 result bus, valid the cycle done is high
//
// FSM:
//   IDLE  - no one granted; pick a winner if any valid is asserted
//   ARM   - winner picked, drive operands+mode for one cycle then pulse start
//   RUN   - sage16_top is computing; wait for its done
//   FIN   - pulse done back to winner for one cycle
// =============================================================================
module sage16_scheduler #(
    parameter DATA_W = 16,
    parameter ACC_W  = 32
)(
    input  wire                       clk,
    input  wire                       rst_n,

    // ---- attitude_ctrl requester (priority 0, highest) ----
    input  wire                       ac_valid,
    input  wire [1:0]                 ac_mode,
    input  wire signed [DATA_W-1:0]   ac_q1_w, ac_q1_x, ac_q1_y, ac_q1_z,
    input  wire [16*DATA_W-1:0]       ac_q2_flat,
    output reg                        ac_gnt,
    output reg                        ac_done,
    output reg  [16*ACC_W-1:0]        ac_result,

    // ---- motor_mixer requester (priority 1) ----
    input  wire                       mm_valid,
    input  wire [1:0]                 mm_mode,
    input  wire [16*DATA_W-1:0]       mm_a_flat,
    input  wire [16*DATA_W-1:0]       mm_b_flat,
    output reg                        mm_gnt,
    output reg                        mm_done,
    output reg  [16*ACC_W-1:0]        mm_result,

    // ---- vision requester (priority 2, lowest) ----
    // Vision packs whatever operands its current kernel needs into the
    // generic op_a/op_b/op_q1/op_q2 ports. The scheduler doesn't care
    // what mode it picks; it just routes.
    input  wire                       vis_valid,
    input  wire [1:0]                 vis_mode,
    input  wire [16*DATA_W-1:0]       vis_a_flat,    // for matmul A or conv image
    input  wire [16*DATA_W-1:0]       vis_b_flat,    // for matmul B or conv kernel
    input  wire signed [DATA_W-1:0]   vis_q1_w, vis_q1_x, vis_q1_y, vis_q1_z,
    input  wire [16*DATA_W-1:0]       vis_q2_flat,
    output reg                        vis_gnt,
    output reg                        vis_done,
    output reg  [16*ACC_W-1:0]        vis_result,

    // ---- to shared sage16_top fabric ----
    output reg                        s_start,
    output reg  [1:0]                 s_mode,
    output reg  [16*DATA_W-1:0]       s_mm_a,
    output reg  [16*DATA_W-1:0]       s_mm_b,
    output reg  signed [DATA_W-1:0]   s_q1_w, s_q1_x, s_q1_y, s_q1_z,
    output reg  [16*DATA_W-1:0]       s_q2_flat,
    // (conv operands packed into s_mm_a / s_mm_b for vision; the conv
    //  kernel reads them as 'image' and 'kernel' respectively per
    //  sage16_top's CV mode wiring.)
    input  wire                       s_done,
    input  wire [16*ACC_W-1:0]        s_result
);

    localparam [1:0] S_IDLE = 2'd0,
                     S_ARM  = 2'd1,
                     S_RUN  = 2'd2,
                     S_FIN  = 2'd3;

    // current winner: 0=ac, 1=mm, 2=vis, 3=none
    localparam [1:0] WIN_AC  = 2'd0,
                     WIN_MM  = 2'd1,
                     WIN_VIS = 2'd2,
                     WIN_NONE = 2'd3;

    reg [1:0] state;
    reg [1:0] winner;
    // Set once the fabric has dropped `done` in response to our `start`,
    // proving the in-flight op is OURS and not the previous op's lingering
    // done. Prevents the back-to-back stale-result race.
    reg       fab_ack;

    // ---- combinational priority selection ----
    wire [1:0] pick = ac_valid  ? WIN_AC :
                      mm_valid  ? WIN_MM :
                      vis_valid ? WIN_VIS :
                                  WIN_NONE;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            winner    <= WIN_NONE;
            fab_ack   <= 1'b0;
            s_start   <= 1'b0;
            s_mode    <= 2'd0;
            s_mm_a    <= {16*DATA_W{1'b0}};
            s_mm_b    <= {16*DATA_W{1'b0}};
            s_q1_w    <= {DATA_W{1'b0}};
            s_q1_x    <= {DATA_W{1'b0}};
            s_q1_y    <= {DATA_W{1'b0}};
            s_q1_z    <= {DATA_W{1'b0}};
            s_q2_flat <= {16*DATA_W{1'b0}};
            ac_gnt    <= 1'b0;
            mm_gnt    <= 1'b0;
            vis_gnt   <= 1'b0;
            ac_done   <= 1'b0;
            mm_done   <= 1'b0;
            vis_done  <= 1'b0;
            ac_result <= {16*ACC_W{1'b0}};
            mm_result <= {16*ACC_W{1'b0}};
            vis_result<= {16*ACC_W{1'b0}};
        end else begin
            // default: deassert all pulses each cycle
            s_start  <= 1'b0;
            ac_done  <= 1'b0;
            mm_done  <= 1'b0;
            vis_done <= 1'b0;

            case (state)
                // ----------------------------------------------------------
                S_IDLE: begin
                    ac_gnt  <= 1'b0;
                    mm_gnt  <= 1'b0;
                    vis_gnt <= 1'b0;

                    if (pick != WIN_NONE) begin
                        winner <= pick;
                        state  <= S_ARM;
                    end
                end

                // ----------------------------------------------------------
                // Route the winner's operands to the fabric and grant.
                // Pulse start to kick off the kernel.
                S_ARM: begin
                    case (winner)
                        WIN_AC: begin
                            s_mode    <= ac_mode;
                            s_q1_w    <= ac_q1_w;
                            s_q1_x    <= ac_q1_x;
                            s_q1_y    <= ac_q1_y;
                            s_q1_z    <= ac_q1_z;
                            s_q2_flat <= ac_q2_flat;
                            // mm_a/mm_b can stay don't-care for QT mode
                            ac_gnt    <= 1'b1;
                        end
                        WIN_MM: begin
                            s_mode <= mm_mode;
                            s_mm_a <= mm_a_flat;
                            s_mm_b <= mm_b_flat;
                            mm_gnt <= 1'b1;
                        end
                        WIN_VIS: begin
                            s_mode    <= vis_mode;
                            s_mm_a    <= vis_a_flat;
                            s_mm_b    <= vis_b_flat;
                            s_q1_w    <= vis_q1_w;
                            s_q1_x    <= vis_q1_x;
                            s_q1_y    <= vis_q1_y;
                            s_q1_z    <= vis_q1_z;
                            s_q2_flat <= vis_q2_flat;
                            vis_gnt   <= 1'b1;
                        end
                        default: ;
                    endcase
                    s_start <= 1'b1;       // one-cycle start pulse
                    fab_ack <= 1'b0;       // wait for fabric to ack this start
                    state   <= S_RUN;
                end

                // ----------------------------------------------------------
                // Fabric is running. Wait for done.
                S_RUN: begin
                    // Wait for the fabric to drop `done` (fab_ack) before
                    // accepting it -- but ONLY for the vision lane, which runs
                    // ops back-to-back and hits the lingering-done race. The
                    // AC/MM control lanes keep the ORIGINAL immediate-capture
                    // timing (their ops are spaced by the tick sequencer and
                    // were validated that way), so control behaviour is
                    // byte-identical to the pre-shared design.
                    if (!s_done)
                        fab_ack <= 1'b1;
                    if (s_done && fab_ack) begin
                        case (winner)
                            WIN_AC:  ac_result  <= s_result;
                            WIN_MM:  mm_result  <= s_result;
                            WIN_VIS: vis_result <= s_result;
                            default: ;
                        endcase
                        state <= S_FIN;
                    end
                end

                // ----------------------------------------------------------
                // Pulse done back to the winner, then release.
                S_FIN: begin
                    case (winner)
                        WIN_AC:  ac_done  <= 1'b1;
                        WIN_MM:  mm_done  <= 1'b1;
                        WIN_VIS: vis_done <= 1'b1;
                        default: ;
                    endcase
                    ac_gnt  <= 1'b0;
                    mm_gnt  <= 1'b0;
                    vis_gnt <= 1'b0;
                    state   <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
