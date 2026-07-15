// =============================================================================
// motor_mixer_sage16.v  -- quad-X motor mixer, shared-fabric version
//
// CHANGES vs v1:
//   - Does NOT instantiate its own matmul_sage16. Drives the shared
//     sage16_top via the kernel-request interface, identical in shape
//     to attitude_ctrl's kreq_* interface.
//   - Uses MODE_MMS (signed matmul, mode=3) because the mixer matrix
//     has -1 entries. Mode 0 (unsigned matmul) would mis-interpret
//     those as 0xFFFF = 65535 and produce garbage.
//   - The fixed mixer matrix is built combinationally into mm_a_flat
//     and exposed via the kreq_mm_a port. The B operand is
//     [thrust; tau_x; tau_y; tau_z] in column 0 (rest zero).
//   - One-pulse done; safe under start re-pulse.
//
// Mixer matrix (standard quad-X, unchanged from v1):
//
//     [ 1  1  1 -1 ]
//     [ 1 -1  1  1 ]
//     [ 1 -1 -1 -1 ]
//     [ 1  1 -1  1 ]
//
// motor_i = C[i][0] = row_i . [thrust, tau_x, tau_y, tau_z]
// =============================================================================
module motor_mixer_sage16 #(
    parameter DATA_W = 16,
    parameter ACC_W  = 32
)(
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       start,

    input  wire signed [DATA_W-1:0]   thrust,
    input  wire signed [DATA_W-1:0]   torque_x,
    input  wire signed [DATA_W-1:0]   torque_y,
    input  wire signed [DATA_W-1:0]   torque_z,

    output reg  signed [ACC_W-1:0]    motor0,
    output reg  signed [ACC_W-1:0]    motor1,
    output reg  signed [ACC_W-1:0]    motor2,
    output reg  signed [ACC_W-1:0]    motor3,

    output reg                        done,

    // -------- shared-fabric kernel request interface --------
    output reg                        kreq_valid,
    input  wire                       kreq_gnt,
    input  wire                       kreq_done,
    output reg  [1:0]                 kreq_mode,             // 3 = MMS (signed matmul)
    output reg  [16*DATA_W-1:0]       kreq_mm_a,
    output reg  [16*DATA_W-1:0]       kreq_mm_b,
    input  wire [16*ACC_W-1:0]        kreq_result
);

    localparam [1:0] S_IDLE  = 2'd0,
                     S_KREQ  = 2'd1,
                     S_KWAIT = 2'd2,
                     S_DONE  = 2'd3;

    localparam [1:0] MODE_MMS = 2'd3;

    reg [1:0] state;

    // -------- build the A matrix (mixer) combinationally --------
    // Row-major: A[i*4+j] = entry at row i, col j.
    // sage16_top MODE_MM/MMS reads A as row-broadcast (west), so this
    // packing matches matmul_sage16.v's a_in convention.
    always @(*) begin
        kreq_mm_a = {16*DATA_W{1'b0}};
        // Row 0:  [ 1,  1,  1, -1]
        kreq_mm_a[ 0*DATA_W +: DATA_W] = 16'sd1;
        kreq_mm_a[ 1*DATA_W +: DATA_W] = 16'sd1;
        kreq_mm_a[ 2*DATA_W +: DATA_W] = 16'sd1;
        kreq_mm_a[ 3*DATA_W +: DATA_W] = -16'sd1;
        // Row 1:  [ 1, -1,  1,  1]
        kreq_mm_a[ 4*DATA_W +: DATA_W] = 16'sd1;
        kreq_mm_a[ 5*DATA_W +: DATA_W] = -16'sd1;
        kreq_mm_a[ 6*DATA_W +: DATA_W] = 16'sd1;
        kreq_mm_a[ 7*DATA_W +: DATA_W] = 16'sd1;
        // Row 2:  [ 1, -1, -1, -1]
        kreq_mm_a[ 8*DATA_W +: DATA_W] = 16'sd1;
        kreq_mm_a[ 9*DATA_W +: DATA_W] = -16'sd1;
        kreq_mm_a[10*DATA_W +: DATA_W] = -16'sd1;
        kreq_mm_a[11*DATA_W +: DATA_W] = -16'sd1;
        // Row 3:  [ 1,  1, -1,  1]
        kreq_mm_a[12*DATA_W +: DATA_W] = 16'sd1;
        kreq_mm_a[13*DATA_W +: DATA_W] = 16'sd1;
        kreq_mm_a[14*DATA_W +: DATA_W] = -16'sd1;
        kreq_mm_a[15*DATA_W +: DATA_W] = 16'sd1;
    end

    // -------- build the B matrix --------
    // sage16_top reads B as col-broadcast (north). To get C[:,0]
    // we put [thrust; tau_x; tau_y; tau_z] in column 0 of B and
    // zeros elsewhere. Row-major packing: B[k*4+j] = B[k][j].
    always @(*) begin
        kreq_mm_b = {16*DATA_W{1'b0}};
        kreq_mm_b[ 0*DATA_W +: DATA_W] = thrust;     // B[0][0]
        kreq_mm_b[ 4*DATA_W +: DATA_W] = torque_x;   // B[1][0]
        kreq_mm_b[ 8*DATA_W +: DATA_W] = torque_y;   // B[2][0]
        kreq_mm_b[12*DATA_W +: DATA_W] = torque_z;   // B[3][0]
        // all other entries stay zero
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            kreq_valid <= 1'b0;
            kreq_mode  <= 2'd0;
            motor0     <= 0;
            motor1     <= 0;
            motor2     <= 0;
            motor3     <= 0;
            done       <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)
                S_IDLE: begin
                    kreq_valid <= 1'b0;
                    if (start) begin
                        kreq_valid <= 1'b1;
                        kreq_mode  <= MODE_MMS;
                        state      <= S_KREQ;
                    end
                end

                S_KREQ: begin
                    if (kreq_gnt) begin
                        kreq_valid <= 1'b0;
                        state      <= S_KWAIT;
                    end
                end

                S_KWAIT: begin
                    if (kreq_done) begin
                        // motor_i = C[i][0] = row-major index i*4+0
                        motor0     <= $signed(kreq_result[ 0*ACC_W +: ACC_W]);
                        motor1     <= $signed(kreq_result[ 4*ACC_W +: ACC_W]);
                        motor2     <= $signed(kreq_result[ 8*ACC_W +: ACC_W]);
                        motor3     <= $signed(kreq_result[12*ACC_W +: ACC_W]);
                        kreq_valid <= 1'b0;
                        state      <= S_DONE;
                    end
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
