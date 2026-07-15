// =============================================================================
// control_top.v  (v4 -- adds on-chip self-tick so the control loop runs
//                 host-free; v3 operand-capture debug retained)
//
// New vs v3:
//   * On-chip tick generator (clock divider). When enabled, the fabric
//     paces its own control loop -- the host no longer clocks each tick.
//         0x208[0]  AUTO_EN   : 1 = self-tick on
//         0x20C     TICK_DIV  : period in clk cycles; rate = f_clk/(TICK_DIV+1)
//     Both default OFF, so the manual 0x200 pulse path is unchanged.
//   * Input snapshot: q/omega/thrust are frozen at the instant each tick
//     launches, so the PS can refresh them asynchronously without tearing
//     the sample mid-loop.
// =============================================================================
`default_nettype none

module control_top_shared #(
    parameter ADDR_W = 10,
    parameter DATA_W = 16,
    parameter ACC_W  = 32
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire [ADDR_W-1:0]     addr,
    input  wire [31:0]           wdata,
    output reg  [31:0]           rdata,
    input  wire                  we,
    input  wire                  re,
    output reg                   ack,
    output wire                  irq,

    // ---- shared-fabric VISION lane (driven by vision_top at SoC level) ----
    // Brought out from the internal scheduler's vis_* lane and the fabric's
    // conv operand ports. Control-loop logic below is UNCHANGED; these were
    // previously tied to 0. The single scheduler + single fabric still live
    // in here -- vision just plugs into the spare lane.
    input  wire                  vis_valid,
    input  wire [1:0]            vis_mode,
    input  wire [16*DATA_W-1:0]  vis_a_flat,
    input  wire [16*DATA_W-1:0]  vis_b_flat,
    output wire                  vis_gnt,
    output wire                  vis_done,
    output wire [16*ACC_W-1:0]   vis_result,
    input  wire [36*DATA_W-1:0]  cv_img,   // conv 6x6 patch  (Option A direct)
    input  wire [ 9*DATA_W-1:0]  cv_k      // conv 3x3 kernel
);

    wire signed [31:0] dbg_errxr;
    assign irq = 1'b0;

    // =========================================================
    // Input registers (written by the host over the bus)
    // =========================================================
    reg signed [DATA_W-1:0] q_w_in, q_x_in, q_y_in, q_z_in;
    reg signed [DATA_W-1:0] omega_x_in, omega_y_in, omega_z_in;
    reg signed [DATA_W-1:0] thrust_in;

    // Self-tick control registers
    reg        auto_en;     // 0x208[0]: 1 = fabric generates its own ticks
    reg [31:0] tick_div;    // 0x20C   : tick period in clk cycles

    // DEBUG fault-injection registers (debug bitstream only)
    reg        dbg_fault_en;    // 0x210[0]
    reg [3:0]  dbg_fault_pe;    // 0x214[3:0]
    reg [1:0]  dbg_fault_mode;  // 0x218[1:0]
    reg        sage_en;         // 0x21C[0]: 1 = SAGE self-heal repair ON

    // =========================================================
    // On-chip tick generator (programmable clock divider)
    //   Free-running counter; emits a 1-cycle auto_tick every
    //   (tick_div + 1) clocks while auto_en is set. This is the
    //   control loop's heartbeat -- nothing external paces it.
    //   tick rate = f_clk / (tick_div + 1)
    //   NOTE: keep tick_div well above one loop's worst-case length
    //   (AC+MM is a few hundred cycles) so a pulse always lands in IDLE.
    // =========================================================
    reg [31:0] tick_cnt;
    reg        auto_tick;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tick_cnt  <= 32'd0;
            auto_tick <= 1'b0;
        end else begin
            auto_tick <= 1'b0;                 // default: no pulse this cycle
            if (auto_en) begin
                if (tick_cnt >= tick_div) begin
                    tick_cnt  <= 32'd0;
                    auto_tick <= 1'b1;         // one-cycle heartbeat
                end else begin
                    tick_cnt  <= tick_cnt + 32'd1;
                end
            end else begin
                tick_cnt <= 32'd0;             // held in reset while disabled
            end
        end
    end

    // =========================================================
    // Tick-request latch (manual AXI pulse OR on-chip auto_tick)
    // =========================================================
    reg tick_req;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            tick_req <= 1'b0;
        else if (tick_req)
            tick_req <= 1'b0;
        else if ((we && addr == 10'h200 && wdata[0]) || auto_tick)
            tick_req <= 1'b1;
    end

    // =========================================================
    // Sequencer
    // =========================================================
    localparam [2:0] SEQ_IDLE = 3'd0,
                     SEQ_AC   = 3'd1,
                     SEQ_WAIT = 3'd2,
                     SEQ_MM   = 3'd3,
                     SEQ_DONE = 3'd4;

    reg [2:0] seq_state;
    reg       ac_start, mm_start;
    reg       tick_busy;
    reg [7:0] ac_calls, mm_calls;

    wire ac_done_w, mm_done_w;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            seq_state <= SEQ_IDLE;
            ac_start  <= 1'b0;
            mm_start  <= 1'b0;
            tick_busy <= 1'b0;
            ac_calls  <= 8'd0;
            mm_calls  <= 8'd0;
        end else begin
            ac_start <= 1'b0;
            mm_start <= 1'b0;

            case (seq_state)
                SEQ_IDLE: begin
                    tick_busy <= 1'b0;
                    if (tick_req) begin
                        ac_start  <= 1'b1;
                        ac_calls  <= ac_calls + 8'd1;
                        tick_busy <= 1'b1;
                        seq_state <= SEQ_AC;
                    end
                end
                SEQ_AC:   if (ac_done_w) seq_state <= SEQ_WAIT;
                SEQ_WAIT: begin
                    mm_start  <= 1'b1;
                    mm_calls  <= mm_calls + 8'd1;
                    seq_state <= SEQ_MM;
                end
                SEQ_MM:   if (mm_done_w) seq_state <= SEQ_DONE;
                SEQ_DONE: begin tick_busy <= 1'b0; seq_state <= SEQ_IDLE; end
                default:  seq_state <= SEQ_IDLE;
            endcase
        end
    end

    // =========================================================
    // Input snapshot
    //   Freeze a coherent copy of all inputs at the instant the loop
    //   launches (IDLE + tick_req). The compute modules read the
    //   snapshot, so the host can keep writing q/omega/thrust between
    //   ticks without corrupting a sample mid-loop.
    // =========================================================
    reg signed [DATA_W-1:0] q_w_a, q_x_a, q_y_a, q_z_a;
    reg signed [DATA_W-1:0] omega_x_a, omega_y_a, omega_z_a;
    reg signed [DATA_W-1:0] thrust_a;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q_w_a <= 16'sd100; q_x_a <= 0; q_y_a <= 0; q_z_a <= 0;
            omega_x_a <= 0; omega_y_a <= 0; omega_z_a <= 0;
            thrust_a <= 16'sd1000;
        end else if (seq_state == SEQ_IDLE && tick_req) begin
            q_w_a <= q_w_in; q_x_a <= q_x_in; q_y_a <= q_y_in; q_z_a <= q_z_in;
            omega_x_a <= omega_x_in; omega_y_a <= omega_y_in; omega_z_a <= omega_z_in;
            thrust_a  <= thrust_in;
        end
    end

    // =========================================================
    // attitude_ctrl
    // =========================================================
    wire signed [DATA_W-1:0] ac_torque_x, ac_torque_y, ac_torque_z;
    wire signed [ACC_W-1:0]  ac_torque_full_x, ac_torque_full_y, ac_torque_full_z;
    wire                     ac_kreq_valid, ac_kreq_gnt, ac_kreq_done;
    wire [1:0]               ac_kreq_mode;
    wire [16*DATA_W-1:0]     ac_kreq_q2_flat;
    wire signed [DATA_W-1:0] ac_kreq_q1_w, ac_kreq_q1_x, ac_kreq_q1_y, ac_kreq_q1_z;
    wire [16*ACC_W-1:0]      ac_kreq_result;

    reg signed [ACC_W-1:0] err_x_lat, err_y_lat, err_z_lat;
    reg signed [31:0] torque_x_lat;
    reg signed [31:0] torque_y_lat;
    reg signed [31:0] torque_z_lat;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            torque_x_lat <= 0;
            torque_y_lat <= 0;
            torque_z_lat <= 0;
        end
        else if (ac_done_w) begin
            torque_x_lat <= ac_torque_full_x;
            torque_y_lat <= ac_torque_full_y;
            torque_z_lat <= ac_torque_full_z;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin err_x_lat<=0; err_y_lat<=0; err_z_lat<=0; end
        else if (ac_kreq_done) begin
            err_x_lat <= $signed(ac_kreq_result[1*ACC_W +: ACC_W]);
            err_y_lat <= $signed(ac_kreq_result[2*ACC_W +: ACC_W]);
            err_z_lat <= $signed(ac_kreq_result[3*ACC_W +: ACC_W]);
        end
    end

    attitude_ctrl u_ac (
        .clk(clk), .rst_n(rst_n), .start(ac_start),
        .q_w(q_w_a), .q_x(q_x_a), .q_y(q_y_a), .q_z(q_z_a),
        .omega_x(omega_x_a), .omega_y(omega_y_a), .omega_z(omega_z_a),
        .torque_x(ac_torque_x), .torque_y(ac_torque_y), .torque_z(ac_torque_z),
        .torque_full_x(ac_torque_full_x), .torque_full_y(ac_torque_full_y),
        .torque_full_z(ac_torque_full_z), .dbg_errxr(dbg_errxr),
        .done(ac_done_w), 
        .kreq_valid(ac_kreq_valid), .kreq_gnt(ac_kreq_gnt),
        .kreq_done(ac_kreq_done),  .kreq_mode(ac_kreq_mode),
        .kreq_q2_flat(ac_kreq_q2_flat),
        .kreq_q1_w(ac_kreq_q1_w), .kreq_q1_x(ac_kreq_q1_x),
        .kreq_q1_y(ac_kreq_q1_y), .kreq_q1_z(ac_kreq_q1_z),
        .kreq_result(ac_kreq_result)
    );

    // =========================================================
    // motor_mixer
    // =========================================================
    wire signed [ACC_W-1:0] mm_motor0, mm_motor1, mm_motor2, mm_motor3;
    wire                     mm_kreq_valid, mm_kreq_gnt, mm_kreq_done;
    wire [1:0]               mm_kreq_mode;
    wire [16*DATA_W-1:0]     mm_kreq_a, mm_kreq_b;
    wire [16*ACC_W-1:0]      mm_kreq_result;

    motor_mixer_sage16 u_mm (
        .clk(clk), .rst_n(rst_n), .start(mm_start),
        .thrust(thrust_a),
        .torque_x(ac_torque_x), .torque_y(ac_torque_y), .torque_z(ac_torque_z),
        .motor0(mm_motor0), .motor1(mm_motor1),
        .motor2(mm_motor2), .motor3(mm_motor3),
        .done(mm_done_w),
        .kreq_valid(mm_kreq_valid), .kreq_gnt(mm_kreq_gnt),
        .kreq_done(mm_kreq_done),  .kreq_mode(mm_kreq_mode),
        .kreq_mm_a(mm_kreq_a),     .kreq_mm_b(mm_kreq_b),
        .kreq_result(mm_kreq_result)
    );

    reg signed [ACC_W-1:0] motor0_r, motor1_r, motor2_r, motor3_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin motor0_r<=0; motor1_r<=0; motor2_r<=0; motor3_r<=0; end
        else if (mm_done_w) begin
            motor0_r <= mm_motor0; motor1_r <= mm_motor1;
            motor2_r <= mm_motor2; motor3_r <= mm_motor3;
        end
    end

    // =========================================================
    // Scheduler
    // =========================================================
    wire                  s_start_w;
    wire [1:0]            s_mode_w;
    wire [16*DATA_W-1:0]  s_mm_a_w, s_mm_b_w;
    wire signed [DATA_W-1:0] s_q1_w_w, s_q1_x_w, s_q1_y_w, s_q1_z_w;
    wire [16*DATA_W-1:0]  s_q2_flat_w;
    wire                  s_done_w;
    wire [16*ACC_W-1:0]   s_result_w;

    sage16_scheduler u_sched (
        .clk(clk), .rst_n(rst_n),
        .ac_valid(ac_kreq_valid), .ac_mode(ac_kreq_mode),
        .ac_q1_w(ac_kreq_q1_w),  .ac_q1_x(ac_kreq_q1_x),
        .ac_q1_y(ac_kreq_q1_y),  .ac_q1_z(ac_kreq_q1_z),
        .ac_q2_flat(ac_kreq_q2_flat),
        .ac_gnt(ac_kreq_gnt),    .ac_done(ac_kreq_done),
        .ac_result(ac_kreq_result),
        .mm_valid(mm_kreq_valid), .mm_mode(mm_kreq_mode),
        .mm_a_flat(mm_kreq_a),    .mm_b_flat(mm_kreq_b),
        .mm_gnt(mm_kreq_gnt),    .mm_done(mm_kreq_done),
        .mm_result(mm_kreq_result),
        .vis_valid(vis_valid), .vis_mode(vis_mode),
        .vis_a_flat(vis_a_flat), .vis_b_flat(vis_b_flat),
        .vis_q1_w({DATA_W{1'b0}}),  .vis_q1_x({DATA_W{1'b0}}),
        .vis_q1_y({DATA_W{1'b0}}),  .vis_q1_z({DATA_W{1'b0}}),
        .vis_q2_flat({16*DATA_W{1'b0}}),
        .vis_gnt(vis_gnt), .vis_done(vis_done), .vis_result(vis_result),
        .s_start(s_start_w), .s_mode(s_mode_w),
        .s_mm_a(s_mm_a_w),   .s_mm_b(s_mm_b_w),
        .s_q1_w(s_q1_w_w),   .s_q1_x(s_q1_x_w),
        .s_q1_y(s_q1_y_w),   .s_q1_z(s_q1_z_w),
        .s_q2_flat(s_q2_flat_w),
        .s_done(s_done_w), .s_result(s_result_w)
    );

    // =========================================================
    // sage16_top
    // =========================================================
    sage16_top u_fab (
        .clk(clk), .rst_n(rst_n),
        .start(s_start_w), .mode(s_mode_w), .done(s_done_w),
        .mm_a(s_mm_a_w), .mm_b(s_mm_b_w),
        .cv_img(cv_img), .cv_k(cv_k),
        .qt_q1_w(s_q1_w_w), .qt_q1_x(s_q1_x_w),
        .qt_q1_y(s_q1_y_w), .qt_q1_z(s_q1_z_w),
        .qt_q2(s_q2_flat_w),
        .c_out(s_result_w), .mode_out(),
        .dbg_fault_en(dbg_fault_en),
        .dbg_fault_pe(dbg_fault_pe),
        .dbg_fault_mode(dbg_fault_mode),
        .sage_en(sage_en)
    );

    // =========================================================
    // DEBUG: capture operands at every s_start pulse
    // These let us see exactly what the fabric was given each call.
    // 0x140 = B[0][0] (should be thrust for matmul call, 0 for quat)
    // 0x144 = A[0][0] (should be 1 for mixer row0, Q matrix entry for quat)
    // 0x148 = mode that fired
    // 0x14C = s_result[0] captured at s_done (what fabric returned)
    // =========================================================
    reg [31:0] dbg_b00, dbg_a00, dbg_mode, dbg_result0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dbg_b00 <= 32'hBBBBBBBB;
            dbg_a00 <= 32'hAAAAAAAA;
            dbg_mode<= 32'hCCCCCCCC;
            dbg_result0 <= 32'hDDDDDDDD;
        end else begin
            if (s_start_w) begin
                // sign-extend 16-bit words to 32
                dbg_b00  <= {{16{s_mm_b_w[DATA_W-1]}}, s_mm_b_w[DATA_W-1:0]};
                dbg_a00  <= {{16{s_mm_a_w[DATA_W-1]}}, s_mm_a_w[DATA_W-1:0]};
                dbg_mode <= {30'd0, s_mode_w};
            end
            if (s_done_w)
                dbg_result0 <= s_result_w[ACC_W-1:0];  // word 0 of result
        end
    end

    // =========================================================
    // Register bus
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q_w_in    <= 16'sd100;
            q_x_in    <= 0; q_y_in <= 0; q_z_in <= 0;
            omega_x_in<= 0; omega_y_in <= 0; omega_z_in <= 0;
            thrust_in <= 16'sd1000;
            auto_en   <= 1'b0;            // manual mode at reset (back-compatible)
            tick_div  <= 32'd100_000;     // ~1 kHz at a 100 MHz PL clock
            dbg_fault_en   <= 1'b0;       // injection OFF at reset (clean behaviour)
            dbg_fault_pe   <= 4'd0;
            dbg_fault_mode <= 2'd0;
            sage_en        <= 1'b0;       // repair OFF at reset (unprotected)
            ack       <= 1'b0;
            rdata     <= 32'd0;
        end else begin
            ack <= 1'b0;
            if (we) begin
                case (addr)
                    10'h000: q_w_in     <= wdata[DATA_W-1:0];
                    10'h004: q_x_in     <= wdata[DATA_W-1:0];
                    10'h008: q_y_in     <= wdata[DATA_W-1:0];
                    10'h00C: q_z_in     <= wdata[DATA_W-1:0];
                    10'h010: omega_x_in <= wdata[DATA_W-1:0];
                    10'h014: omega_y_in <= wdata[DATA_W-1:0];
                    10'h018: omega_z_in <= wdata[DATA_W-1:0];
                    10'h01C: thrust_in  <= wdata[DATA_W-1:0];
                    10'h208: auto_en    <= wdata[0];
                    10'h20C: tick_div   <= wdata;
                    10'h210: dbg_fault_en   <= wdata[0];
                    10'h214: dbg_fault_pe   <= wdata[3:0];
                    10'h218: dbg_fault_mode <= wdata[1:0];
                    10'h21C: sage_en        <= wdata[0];
                    default: ;
                endcase
                ack <= 1'b1;
            end
            if (re) begin
                case (addr)
                    10'h000: rdata <= {{16{q_w_in[DATA_W-1]}},     q_w_in};
                    10'h004: rdata <= {{16{q_x_in[DATA_W-1]}},     q_x_in};
                    10'h008: rdata <= {{16{q_y_in[DATA_W-1]}},     q_y_in};
                    10'h00C: rdata <= {{16{q_z_in[DATA_W-1]}},     q_z_in};
                    10'h010: rdata <= {{16{omega_x_in[DATA_W-1]}}, omega_x_in};
                    10'h014: rdata <= {{16{omega_y_in[DATA_W-1]}}, omega_y_in};
                    10'h018: rdata <= {{16{omega_z_in[DATA_W-1]}}, omega_z_in};
                    10'h01C: rdata <= {{16{thrust_in[DATA_W-1]}},  thrust_in};
                    10'h100: rdata <= motor0_r;
                    10'h104: rdata <= motor1_r;
                    10'h108: rdata <= motor2_r;
                    10'h10C: rdata <= motor3_r;
                    10'h110: rdata <= torque_x_lat;
                    10'h114: rdata <= torque_y_lat;
                    10'h118: rdata <= torque_z_lat;
                    10'h11C: rdata <= err_x_lat;
                    10'h120: rdata <= err_y_lat;
                    10'h124: rdata <= err_z_lat;
                    10'h130: rdata <= {{16{ac_torque_x[15]}}, ac_torque_x};
                    10'h134: rdata <= dbg_errxr;
                    10'h140: rdata <= dbg_b00;
                    10'h144: rdata <= dbg_a00;
                    10'h148: rdata <= dbg_mode;
                    10'h14C: rdata <= dbg_result0;
                    10'h204: rdata <= {8'd0, mm_calls, ac_calls,
                                       5'd0, tick_busy, mm_done_w, ac_done_w};
                    10'h208: rdata <= {31'd0, auto_en};
                    10'h20C: rdata <= tick_div;
                    10'h210: rdata <= {31'd0, dbg_fault_en};
                    10'h214: rdata <= {28'd0, dbg_fault_pe};
                    10'h218: rdata <= {30'd0, dbg_fault_mode};
                    10'h21C: rdata <= {31'd0, sage_en};
                    default: rdata <= 32'hDEADBEEF;
                endcase
                ack <= 1'b1;
            end
        end
    end

endmodule
`default_nettype wire
