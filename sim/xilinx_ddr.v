// Behavioural models of the 7-series DDR I/O registers, as gateware/dap.py uses them, for the
// simulations: ODDR and IDDR in SAME_EDGE mode.

module ODDR #(parameter DDR_CLK_EDGE = "SAME_EDGE", parameter INIT = 0, parameter SRTYPE = "SYNC")
             (output reg Q = 0, input C, input CE, input D1, input D2, input R, input S);
    reg second = 0;
    always @(posedge C) begin Q <= D1; second <= D2; end      // D1 for the first half, D2 for
    always @(negedge C) Q <= second;                           // the second
endmodule

module IDDR #(parameter DDR_CLK_EDGE = "SAME_EDGE", parameter INIT_Q1 = 0,
              parameter INIT_Q2 = 0, parameter SRTYPE = "SYNC")
             (output reg Q1 = 0, output reg Q2 = 0, input C, input CE, input D, input R, input S);
    // SAME_EDGE: Q1 is D at this rising edge, Q2 D at the falling edge before it;
    // SAME_EDGE_PIPELINED delays Q1 a cycle, so that both are the same cycle's
    reg rising = 0, falling = 0;
    always @(posedge C) rising <= D;
    always @(negedge C) falling <= D;
    always @(posedge C) begin
        Q1 <= (DDR_CLK_EDGE == "SAME_EDGE_PIPELINED") ? rising : D;
        Q2 <= falling;
    end
endmodule
