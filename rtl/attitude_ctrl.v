// =============================================================================
// attitude_ctrl.v  -- attitude PD controller, shared-fabric version
//
// CHANGES vs v1:
//   - Does NOT instantiate its own quat_sage16. Instead, exposes a kernel
//     request interface (kreq_*) that a parent scheduler arbitrates onto
//     the one shared sage16_top fabric. This is the architectural fix
//     required for the paper's "shared 16-DSP fabric" claim.
//   - q_target is now a parameter (still defaults to identity = upright).
//   - Torque outputs saturate to int16 before exiting, so the downstream
//     motor_mixer's int16 inputs don't see overflow wraparound on large
//     tilts. The internal int32 values are preserved on torque_full_*
//     for the dashboard / debugging.
//   - DONE_ST holds done for exactly one cycle and is robust to start
//     re-pulses (start arriving during DONE_ST queues the next iteration
//     via the IDLE->IDLE path; no edge is lost).
//
// Kernel request protocol (towards scheduler / shared fabric):
//   1. Assert kreq_valid with kreq_mode=2 (QT) and operands packed in
//      the kreq_q1_*/kreq_q2_flat ports.
//   2. Scheduler eventually asserts kreq_gnt (grant). It can be the same
//      cycle as kreq_valid if the fabric is idle.
//   3. Scheduler holds the fabric driving our operands until it pulses
//      kreq_done with kreq_result holding the 16x32 result.
//   4. We deassert kreq_valid in response and continue with the PD math.
//
// The q2 buffer layout for QT mode (matches sage16_top.v MODE_QT):
//   slot j (j = 0..3) holds quaternion j as 4 components (w, x, y, z)
//   row-major. For this controller we use slot 0 = q_current* and zero
//   out slots 1..3.
// =============================================================================
module attitude_ctrl #(
    parameter DATA_W = 16,
    parameter ACC_W  = 32,

    parameter signed [DATA_W-1:0] KP = 6,
    parameter signed [DATA_W-1:0] KD = 2,

    // Target quaternion -- defaults to identity (level, no yaw).
    // Scale convention: identity = (100, 0, 0, 0) as elsewhere in this
    // codebase. Override at instantiation for off-level setpoints.
    parameter signed [DATA_W-1:0] Q_TARGET_W = 16'sd100,
    parameter signed [DATA_W-1:0] Q_TARGET_X = 16'sd0,
    parameter signed [DATA_W-1:0] Q_TARGET_Y = 16'sd0,
    parameter signed [DATA_W-1:0] Q_TARGET_Z = 16'sd0
)(
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       start,

    // current state from IMU / estimator
    input  wire signed [DATA_W-1:0]   q_w, q_x, q_y, q_z,
    input  wire signed [DATA_W-1:0]   omega_x, omega_y, omega_z,

    // saturated int16 torques (safe to wire directly into motor_mixer)
    output reg  signed [DATA_W-1:0]   torque_x,
    output reg  signed [DATA_W-1:0]   torque_y,
    output reg  signed [DATA_W-1:0]   torque_z,

    // full-precision int32 torques (for debug / dashboard)
    output reg  signed [ACC_W-1:0]    torque_full_x,
    output reg  signed [ACC_W-1:0]    torque_full_y,
    output reg  signed [ACC_W-1:0]    torque_full_z,
    output reg signed [ACC_W-1:0] dbg_errxr,

    output reg                        done,

    // -------- shared-fabric kernel request interface --------
    output reg                        kreq_valid,
    input  wire                       kreq_gnt,
    input  wire                       kreq_done,
    output reg  [1:0]                 kreq_mode,        // 2 = QT
    output wire [16*DATA_W-1:0]       kreq_q2_flat,
    output wire signed [DATA_W-1:0]   kreq_q1_w,
    output wire signed [DATA_W-1:0]   kreq_q1_x,
    output wire signed [DATA_W-1:0]   kreq_q1_y,
    output wire signed [DATA_W-1:0]   kreq_q1_z,
    input  wire [16*ACC_W-1:0]        kreq_result
);

    localparam [2:0] S_IDLE  = 3'd0,
                     S_KREQ  = 3'd1,   // assert request, wait for grant
                     S_KWAIT = 3'd2,   // grant received, wait for done
                     S_PD    = 3'd3,   // compute PD law
                     S_DONE  = 3'd4;

    localparam [1:0] MODE_QT = 2'd2;

    reg [2:0] state;

    // captured error components (registered on kreq_done)
    reg signed [ACC_W-1:0] err_x_r, err_y_r, err_z_r;

    // saturation helper: int32 -> int16
    function signed [DATA_W-1:0] sat16;
        input signed [ACC_W-1:0] v;
        begin
            if      (v >  $signed({{(ACC_W-DATA_W+1){1'b0}}, {(DATA_W-1){1'b1}}}))
                sat16 = {1'b0, {(DATA_W-1){1'b1}}};      // +32767
            else if (v <  $signed({{(ACC_W-DATA_W){1'b1}}, 1'b1, {(DATA_W-1){1'b0}}}))
                sat16 = {1'b1, {(DATA_W-1){1'b0}}};      // -32768
            else
                sat16 = v[DATA_W-1:0];
        end
    endfunction

    // q2[0] = q_current* = (q_w, -q_x, -q_y, -q_z); slots 1..3 zero.
    // q1 = target quaternion (parameter, identity = upright).
    // NOTE: these MUST be continuous assigns, not always@(*). The q1 block
    // reads only parameters, so an always@(*) has an EMPTY sensitivity list
    // and is non-portable: simulators may never evaluate it and some synth
    // flows infer a latch / tie it to 0 instead of the parameter value,
    // which silently zeroes the attitude error. Continuous assigns are
    // unambiguous in both simulation and synthesis.
    assign kreq_q2_flat = { {12*DATA_W{1'b0}}, -q_z, -q_y, -q_x, q_w };
    assign kreq_q1_w = Q_TARGET_W;
    assign kreq_q1_x = Q_TARGET_X;
    assign kreq_q1_y = Q_TARGET_Y;
    assign kreq_q1_z = Q_TARGET_Z;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_IDLE;
            kreq_valid    <= 1'b0;
            kreq_mode     <= 2'd0;
            err_x_r       <= 0;
            err_y_r       <= 0;
            err_z_r       <= 0;
            torque_x      <= 0;
            torque_y      <= 0;
            torque_z      <= 0;
            torque_full_x <= 0;
            torque_full_y <= 0;
            torque_full_z <= 0;
            dbg_errxr <= 0;
            done          <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)
                S_IDLE: begin
                    kreq_valid <= 1'b0;
                    if (start) begin
                        kreq_valid <= 1'b1;
                        kreq_mode  <= MODE_QT;
                        state      <= S_KREQ;
                    end
                end

                // Hold the request until the scheduler grants the fabric.
                S_KREQ: begin
                    if (kreq_gnt) begin
                        kreq_valid <= 1'b0;
                        state      <= S_KWAIT;
                    end
                end

                // Fabric is running our kernel; wait for completion.
                S_KWAIT: begin
                    if (kreq_done) begin
                        // capture err_x, err_y, err_z (err_w not used)
                        err_x_r    <= $signed(kreq_result[1*ACC_W +: ACC_W]);
                        dbg_errxr <= $signed(kreq_result[1*ACC_W +: ACC_W]);
                        err_y_r    <= $signed(kreq_result[2*ACC_W +: ACC_W]);
                        err_z_r    <= $signed(kreq_result[3*ACC_W +: ACC_W]);
                        kreq_valid <= 1'b0;
                        state      <= S_PD;
                    end
                end

                // One cycle to compute the PD law and saturate.
                S_PD: begin : pd_blk
                    reg signed [ACC_W-1:0] tx_full, ty_full, tz_full;
                    reg signed [ACC_W-1:0] omega_x_ext, omega_y_ext, omega_z_ext;

                    // Sign-extend omega from 16 -> 32 bits
                    omega_x_ext = {{(ACC_W-DATA_W){omega_x[DATA_W-1]}}, omega_x};
                    omega_y_ext = {{(ACC_W-DATA_W){omega_y[DATA_W-1]}}, omega_y};
                    omega_z_ext = {{(ACC_W-DATA_W){omega_z[DATA_W-1]}}, omega_z};

                    tx_full = $signed(KP) * err_x_r - $signed(KD) * omega_x_ext;
                    ty_full = $signed(KP) * err_y_r - $signed(KD) * omega_y_ext;
                    tz_full = $signed(KP) * err_z_r - $signed(KD) * omega_z_ext;

                    torque_full_x <= tx_full;
                    torque_full_y <= ty_full;
                    torque_full_z <= tz_full;
                    torque_x      <= sat16(tx_full);
                    torque_y      <= sat16(ty_full);
                    torque_z      <= sat16(tz_full);

                    state <= S_DONE;
                end

                S_DONE: begin
                    done  <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
