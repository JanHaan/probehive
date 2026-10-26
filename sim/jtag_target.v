// A JTAG target for sim/jtag_sim.py: one TAP with a 5-bit instruction register (as the ESP32's),
// IDCODE, BYPASS and a 32-bit scratch register that keeps what is shifted into it.  The TAP takes
// TMS and TDI on the rising edge of TCK and moves TDO on the falling edge; TDO is high outside
// the shift states.  sim/jtag_sim.py's Tap is the same machine.

module jtag_target #(parameter [31:0] ID = 32'h120034E5)
                    (input tck, input tms, input tdi, output reg tdo = 1);
    localparam TLR = 4'd0, RTI = 4'd1, SEL_DR = 4'd2, CAP_DR = 4'd3, SHIFT_DR = 4'd4, EXIT1_DR = 4'd5,
               PAUSE_DR = 4'd6, EXIT2_DR = 4'd7, UPD_DR = 4'd8, SEL_IR = 4'd9, CAP_IR = 4'd10,
               SHIFT_IR = 4'd11, EXIT1_IR = 4'd12, PAUSE_IR = 4'd13, EXIT2_IR = 4'd14, UPD_IR = 4'd15;
    localparam [4:0] I_IDCODE = 5'h01, I_SCRATCH = 5'h02;

    reg [3:0]  state = TLR;
    reg [4:0]  ir = I_IDCODE, ir_sr = 0;
    reg [31:0] dr = 0, scratch = 0;
    wire       bypass = ir != I_IDCODE && ir != I_SCRATCH;

    function [3:0] next(input [3:0] s, input m);
        case (s)
            TLR:      next = m ? TLR : RTI;
            RTI:      next = m ? SEL_DR : RTI;
            SEL_DR:   next = m ? SEL_IR : CAP_DR;
            CAP_DR:   next = m ? EXIT1_DR : SHIFT_DR;
            SHIFT_DR: next = m ? EXIT1_DR : SHIFT_DR;
            EXIT1_DR: next = m ? UPD_DR : PAUSE_DR;
            PAUSE_DR: next = m ? EXIT2_DR : PAUSE_DR;
            EXIT2_DR: next = m ? UPD_DR : SHIFT_DR;
            UPD_DR:   next = m ? SEL_DR : RTI;
            SEL_IR:   next = m ? TLR : CAP_IR;
            CAP_IR:   next = m ? EXIT1_IR : SHIFT_IR;
            SHIFT_IR: next = m ? EXIT1_IR : SHIFT_IR;
            EXIT1_IR: next = m ? UPD_IR : PAUSE_IR;
            PAUSE_IR: next = m ? EXIT2_IR : PAUSE_IR;
            EXIT2_IR: next = m ? UPD_IR : SHIFT_IR;
            UPD_IR:   next = m ? SEL_DR : RTI;
            default:  next = m ? TLR : RTI;               // From power-up, as five TMS highs would
        endcase
    endfunction

    always @(posedge tck) begin
        case (state)
            TLR:      ir <= I_IDCODE;
            CAP_DR:   dr <= ir == I_IDCODE ? ID : ir == I_SCRATCH ? scratch : 32'd0;
            SHIFT_DR: dr <= bypass ? {31'd0, tdi} : {tdi, dr[31:1]};
            UPD_DR:   if (ir == I_SCRATCH) scratch <= dr;
            CAP_IR:   ir_sr <= 5'b00001;
            SHIFT_IR: ir_sr <= {tdi, ir_sr[4:1]};
            UPD_IR:   ir <= ir_sr;
            default: ;
        endcase
        state <= next(state, tms);
    end
    always @(negedge tck)
        tdo <= state == SHIFT_DR ? dr[0] : state == SHIFT_IR ? ir_sr[0] : 1'b1;
endmodule
