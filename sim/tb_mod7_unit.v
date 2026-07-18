// tb_mod7_unit.v — verify mod7_reduce against the reference (% 7) before wiring
// it into the PE. Checks W=32 and W=16 over edge cases + 25k random vectors.
`timescale 1ns/1ps
`default_nettype none
module tb_mod7_unit;
    reg  [31:0] x32; wire [2:0] r32;
    reg  [15:0] x16; wire [2:0] r16;
    mod7_reduce #(.W(32)) u32 (.x(x32), .r(r32));
    mod7_reduce #(.W(16)) u16 (.x(x16), .r(r16));
    integer i, fail, seed;
    initial begin
        seed = 32'h0D7_1234; fail = 0;
        // edge cases
        x32=0;          #1; if (r32 !== 0) begin fail=fail+1; $display("edge 0"); end
        x32=7;          #1; if (r32 !== 0) begin fail=fail+1; $display("edge 7"); end
        x32=100;        #1; if (r32 !== 2) begin fail=fail+1; $display("edge 100 got %0d", r32); end
        x32=32'hFFFFFFFF;#1;if (r32 !== (32'hFFFFFFFF % 7)) begin fail=fail+1;
                            $display("edge FFFFFFFF got %0d exp %0d", r32, 32'hFFFFFFFF%7); end
        // random 32-bit
        for (i=0;i<25000;i=i+1) begin
            x32 = $random(seed); #1;
            if (r32 !== (x32 % 7)) begin
                fail=fail+1; if (fail<8) $display("W32 x=%h got %0d exp %0d", x32, r32, x32%7);
            end
        end
        // random 16-bit
        for (i=0;i<8000;i=i+1) begin
            x16 = $random(seed); #1;
            if (r16 !== (x16 % 7)) begin
                fail=fail+1; if (fail<8) $display("W16 x=%h got %0d exp %0d", x16, r16, x16%7);
            end
        end
        $display("=== mod7_reduce unit: %s (%0d fails) ===", (fail==0)?"PASS":"FAIL", fail);
        $finish;
    end
endmodule
`default_nettype wire
