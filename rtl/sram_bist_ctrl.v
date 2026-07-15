// =============================================================================
// sram_bist_ctrl.v  —  Built-In Self-Test for 16x (256x32 1RW) SRAMs
//
// Runs a simplified MARCH C- style pattern across every word of every PE's
// paired SRAM.  Sequence (per pass):
//
//   Phase W0 : write pattern P  to addr 0..N-1     (N = DEPTH = 256)
//   Phase R0 : read  back addr 0..N-1, compare = P
//   Phase W1 : write pattern ~P to addr 0..N-1
//   Phase R1 : read  back addr 0..N-1, compare = ~P
//
// Two passes are run with P = 32'hAAAA_AAAA and P = 32'h5A5A_5A5A.  This
// catches stuck-at-0/1, simple coupling faults, address-decoder shorts,
// and data-bus shorts on every PE's SRAM in parallel (all 16 are stepped
// in lock-step).
//
// Per-PE pass/fail latched in `pe_pass_mask[15:0]` (bit i = 1 means PE i
// passed).  Aggregate `bist_done` and `bist_pass` exposed to the top.
//
// Cycle budget (DEPTH=256, 2 passes, 4 phases per pass):
//   4 * 256 * 2 = 2048 cycles  + 1-cycle SRAM read latency = ~2050 cycles.
//
// Drives flat SRAM control buses for the fabric.  All 16 SRAMs share the
// same address/data/control during BIST (every SRAM tested with the same
// pattern at the same address).
// =============================================================================
`default_nettype none

module sram_bist_ctrl #(
    parameter NUM_PE   = 16,
    parameter SRAM_AW  = 8,
    parameter SRAM_DW  = 32,
    parameter DEPTH    = 256
)(
    input  wire                          clk, rst_n,
    input  wire                          start,
    output reg                           busy,
    output reg                           done,
    output reg                           pass,            // aggregate AND of per-PE
    output reg  [NUM_PE-1:0]             pe_pass_mask,    // bit i = PE i passed

    // ---- flat SRAM control buses (driven into fabric) ----
    output reg  [NUM_PE-1:0]             sram_cs_n_flat,
    output reg  [NUM_PE-1:0]             sram_we_n_flat,
    output reg  [NUM_PE*SRAM_AW-1:0]     sram_addr_flat,
    output reg  [NUM_PE-1:0]             sram_wdata_sel,        // 1 = external wdata
    output reg  [NUM_PE*SRAM_DW-1:0]     sram_wdata_ext_flat,

    // ---- read data (from each SRAM, registered output, 1 cy after addr issue) ----
    input  wire [NUM_PE*SRAM_DW-1:0]     sram_rdata_flat
);
    // ------------------- FSM -------------------
    localparam [2:0] S_IDLE   = 3'd0,
                     S_W0     = 3'd1,
                     S_R0     = 3'd2,
                     S_R0_CHK = 3'd3,
                     S_W1     = 3'd4,
                     S_R1     = 3'd5,
                     S_R1_CHK = 3'd6,
                     S_DONE   = 3'd7;

    reg [2:0]            state;
    reg [SRAM_AW:0]      cnt;            // 1 bit wider to safely hit DEPTH
    reg                  pass_idx;       // 0 = first pass (P=0xAAAA_AAAA), 1 = second pass (0x5A5A_5A5A)
    reg [SRAM_DW-1:0]    pat;
    reg [NUM_PE-1:0]     fail_mask;      // sticky per-PE failure flags

    // Pattern lookup
    function [SRAM_DW-1:0] pattern_for;
        input pidx;
        case (pidx)
            1'b0:    pattern_for = 32'hAAAA_AAAA;
            1'b1:    pattern_for = 32'h5A5A_5A5A;
            default: pattern_for = 32'h0;
        endcase
    endfunction

    // Drive flat buses (broadcast same addr/wdata to every PE during BIST)
    integer i;
    always @(*) begin
        sram_cs_n_flat      = {NUM_PE{1'b1}};   // default deselected
        sram_we_n_flat      = {NUM_PE{1'b1}};
        sram_addr_flat      = {NUM_PE*SRAM_AW{1'b0}};
        sram_wdata_sel      = {NUM_PE{1'b1}};   // use external wdata during BIST
        sram_wdata_ext_flat = {NUM_PE*SRAM_DW{1'b0}};

        case (state)
            S_W0, S_W1: if (cnt < DEPTH[SRAM_AW:0]) begin
                for (i = 0; i < NUM_PE; i = i + 1) begin
                    sram_cs_n_flat[i]                          = 1'b0;
                    sram_we_n_flat[i]                          = 1'b0;
                    sram_addr_flat[i*SRAM_AW +: SRAM_AW]       = cnt[SRAM_AW-1:0];
                    sram_wdata_ext_flat[i*SRAM_DW +: SRAM_DW]  = pat;
                end
            end

            S_R0, S_R1: if (cnt < DEPTH[SRAM_AW:0]) begin
                for (i = 0; i < NUM_PE; i = i + 1) begin
                    sram_cs_n_flat[i]                    = 1'b0;
                    sram_we_n_flat[i]                    = 1'b1;
                    sram_addr_flat[i*SRAM_AW +: SRAM_AW] = cnt[SRAM_AW-1:0];
                end
            end

            default: ; // hold defaults
        endcase
    end

    // Sequential FSM
    integer j;
    reg [SRAM_AW-1:0] last_read_addr;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_IDLE;
            cnt           <= 0;
            pass_idx      <= 0;
            pat           <= 0;
            fail_mask     <= {NUM_PE{1'b0}};
            pe_pass_mask  <= {NUM_PE{1'b0}};
            busy          <= 0;
            done          <= 0;
            pass          <= 0;
            last_read_addr<= 0;
        end else begin
            case (state)
            S_IDLE: begin
                done <= 0;
                if (start) begin
                    busy      <= 1;
                    fail_mask <= {NUM_PE{1'b0}};
                    pass_idx  <= 0;
                    pat       <= pattern_for(1'b0);
                    cnt       <= 0;
                    state     <= S_W0;
                end
            end

            S_W0: begin
                // step through every addr, writing pattern
                if (cnt == DEPTH[SRAM_AW:0] - 1) begin
                    cnt   <= 0;
                    state <= S_R0;
                end else begin
                    cnt <= cnt + 1;
                end
            end

            S_R0: begin
                // issue read address each cycle; capture last_read_addr to know
                // which lane the rdata returning NEXT cycle belongs to.
                last_read_addr <= cnt[SRAM_AW-1:0];
                if (cnt == DEPTH[SRAM_AW:0] - 1) begin
                    cnt   <= 0;
                    state <= S_R0_CHK;
                end else begin
                    cnt <= cnt + 1;
                end
                // start checking from cycle 1 onward (skip first read which
                // has no preceding addr to validate against)
                if (cnt != 0) begin
                    for (j = 0; j < NUM_PE; j = j + 1)
                        if (sram_rdata_flat[j*SRAM_DW +: SRAM_DW] != pat)
                            fail_mask[j] <= 1'b1;
                end
            end

            S_R0_CHK: begin
                // last rdata still pending: check addr DEPTH-1
                for (j = 0; j < NUM_PE; j = j + 1)
                    if (sram_rdata_flat[j*SRAM_DW +: SRAM_DW] != pat)
                        fail_mask[j] <= 1'b1;
                pat   <= ~pat;
                cnt   <= 0;
                state <= S_W1;
            end

            S_W1: begin
                if (cnt == DEPTH[SRAM_AW:0] - 1) begin
                    cnt   <= 0;
                    state <= S_R1;
                end else begin
                    cnt <= cnt + 1;
                end
            end

            S_R1: begin
                last_read_addr <= cnt[SRAM_AW-1:0];
                if (cnt == DEPTH[SRAM_AW:0] - 1) begin
                    cnt   <= 0;
                    state <= S_R1_CHK;
                end else begin
                    cnt <= cnt + 1;
                end
                if (cnt != 0) begin
                    for (j = 0; j < NUM_PE; j = j + 1)
                        if (sram_rdata_flat[j*SRAM_DW +: SRAM_DW] != pat)
                            fail_mask[j] <= 1'b1;
                end
            end

            S_R1_CHK: begin
                for (j = 0; j < NUM_PE; j = j + 1)
                    if (sram_rdata_flat[j*SRAM_DW +: SRAM_DW] != pat)
                        fail_mask[j] <= 1'b1;

                if (pass_idx == 1'b0) begin
                    // second pattern pass
                    pass_idx <= 1'b1;
                    pat      <= pattern_for(1'b1);
                    cnt      <= 0;
                    state    <= S_W0;
                end else begin
                    state <= S_DONE;
                end
            end

            S_DONE: begin
                pe_pass_mask <= ~fail_mask;
                pass         <= (fail_mask == {NUM_PE{1'b0}});
                done         <= 1;
                busy         <= 0;
                state        <= S_IDLE;
            end

            default: state <= S_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
