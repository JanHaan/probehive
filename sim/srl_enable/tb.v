`timescale 1ns/1ps
// Shifts 1..40 in with every third clock stalled, then prints both ends.
module tb;
    reg clk = 0, en = 0;
    reg [7:0] d = 0;
    wire [7:0] head, tail;
    integer i;
    shreg dut(.clk(clk), .en(en), .d(d), .head(head), .tail(tail));
    always #5 clk = ~clk;
    initial begin
        for (i = 1; i <= 40; i = i + 1) begin
            @(negedge clk); en = (i % 3 != 0); d = i;
        end
        @(negedge clk); en = 0;
        repeat (5) @(negedge clk);
        $display("head %0d tail %0d", head, tail);
        $finish;
    end
endmodule
