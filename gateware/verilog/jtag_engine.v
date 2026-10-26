`default_nettype none
//
// jtag_engine
// ===========
//
// Runs one CMSIS-DAP command of a JTAG slot, read from the slot's request buffer, and writes its
// response into the response buffer: DAP_JTAG_Sequence, and DAP_SWJ_Sequence with its bits on
// TMS.  The CPU parses everything else, and hands these over with where the command starts and
// where its response goes; it gets back how many bytes each took (as swd_exec does).
//
// The wire follows ARM's CMSIS-DAP reference (DAP.c, JTAG_Sequence and SWJ_Sequence): for each bit
// TMS and TDI are set and TCK goes low, then TCK goes high, where the target takes TMS and TDI; TDO
// is captured while TCK is high, before the falling edge where the target moves it on.  TCK rests
// high.  TCK, TMS and TDI keep their last levels between commands, and the CPU sets them only
// while the engine is idle (DAP_SWJ_Pins).  TDI starts low: on the ESP32 it is MTDI (GPIO12), which
// the chip reads at reset for its flash supply, and high there means 1.8 V.
//
// TCK is lo + hi half cycles of `clk`, low first: the engine works two half cycles a cycle and
// hands its pins out as pairs, [0] for the first half (as swd_phy does, through ODDRs in
// SAME_EDGE mode).  TDO is taken `cfg_sample` half cycles after the rising edge (0: half way
// through the high phase), from an IDDR in SAME_EDGE mode.
//
// Bits run back to back while the request keeps up; when it does not, TCK stays high until the
// next bit is there.

module jtag_engine #(parameter AW = 12) (
                input wire             clk,
                input wire             rst,

        // The command: fields stable from before `cmd_req` toggles until `cmd_done` does ==============
                input wire             cmd_req,
                input wire [AW-1:0]    cmd_rq_base,                        // Its first byte, the command
                input wire [AW-1:0]    cmd_rs_base,                         // Where its response goes
                output reg        cmd_done,
                output reg [AW-1:0] cmd_rq_len,                         // Bytes of request it took
                output reg [AW-1:0] cmd_rs_len,                         // Bytes of response written

        // Configuration, read when a command starts =======================================================
                input wire [9:0]       clk_hi,                        // TCK high and low, in half cycles
                input wire [9:0]       clk_lo,                                         // each at least 2
                input wire [6:0]       cfg_sample,         // TDO taken this far into the high phase; 0: half way

        // The pins' levels from the CPU: taken when `pins_set` toggles and the engine is idle ============
                input wire             pins_set,
                input wire             pins_tck,
                input wire             pins_tms,
                input wire             pins_tdi,

        // Buffers ========================================================================================
                output reg [AW-1:0] rq_addr,          // Request byte read: rq_data two cycles after
                input wire [7:0]       rq_data,
                output reg        rs_we,                                // Response byte written
                output reg [AW-1:0] rs_addr,
                output reg [7:0]  rs_data,

        // Pins ===========================================================================================
                output reg [1:0]  tck,
                output reg [1:0]  tms,
                output reg [1:0]  tdi,
                input wire [1:0]       tdo,           // From an IDDR in SAME_EDGE mode: [1] the first half's
                output wire            busy
             );

   localparam [7:0] JTAG_SEQUENCE = 8'h14, SWJ_SEQUENCE = 8'h12;
   localparam [7:0] DAP_OK = 8'h00, DAP_ERROR = 8'hFF;

   // Synchronisers ======================================================================================
   reg [2:0]  req_s, pins_s;
   always @(posedge clk) begin
      req_s  <= {req_s[1:0], cmd_req};
      pins_s <= {pins_s[1:0], pins_set};
   end
   wire       req_new  = req_s[2] != req_s[1];
   wire       pins_new = pins_s[2] != pins_s[1];

   // The request: commands parsed into byte operations ================================================
   //
   // An operation is up to eight bits of one sequence: their data, LSB first, on TDI (with TMS
   // fixed) or on TMS (with TDI as it is), and whether their TDO is kept.
   localparam P_IDLE = 4'd0, P_READ = 4'd1, P_CMD = 4'd2, P_COUNT = 4'd3, P_INFO = 4'd4,
              P_DATA = 4'd5, P_PUSH = 4'd6, P_NEXT = 4'd7, P_DRAIN = 4'd8, P_HDR0 = 4'd9,
              P_HDR1 = 4'd10, P_DONE = 4'd11, P_SWJ = 4'd12;
   reg [3:0]  pst, pret;
   reg [AW-1:0] rq_base, rs_base, rp, wp;
   reg [1:0]  fwait;
   reg [7:0]  cmd, status;
   reg [7:0]  seqs;                                                 // Sequences left, after this one
   reg [8:0]  bits;                                              // Bits of this sequence left
   reg        seq_tms, seq_cap, swj;
   reg [7:0]  data;
   reg        parsing;
   reg        running;
   reg        tdo_byte_valid;
   reg  [7:0] tdo_byte;
   wire       wire_idle;

   // The queue of operations, two deep
   reg [15:0] q_data;                                                   // Two of each, by q_wr
   reg [7:0]  q_n;                                                                // Bits, 1..8
   reg [3:0]  q_flags;                                                   // {on TMS, TDO kept}
   reg [1:0]  q_tms;
   reg        q_wr, q_rd;
   reg [1:0]  q_count;
   wire       q_full = q_count == 2;
   wire       q_pop;

   wire [8:0] take = bits > 9'd8 ? 9'd8 : bits;

   always @(posedge clk) begin
      if (rst) begin
         pst <= P_IDLE; parsing <= 0; cmd_done <= 0; rs_we <= 0;
      end else begin
         rs_we <= 0;
         case (pst)
         P_IDLE:
            if (req_new) begin
               rq_base <= cmd_rq_base; rs_base <= cmd_rs_base;
               rp <= cmd_rq_base; wp <= cmd_rs_base + 2'd2;
               status <= DAP_OK; parsing <= 1;
               rq_addr <= cmd_rq_base; fwait <= 2; pret <= P_CMD; pst <= P_READ;
            end
         P_READ:                                     // rq_data is the byte at rq_addr two cycles on
            if (fwait != 0) fwait <= fwait - 1'b1;
            else begin
               data <= rq_data; rp <= rp + 1'b1; pst <= pret;
            end
         P_CMD: begin
            cmd <= data;
            if (data == JTAG_SEQUENCE || data == SWJ_SEQUENCE) begin
               swj <= data == SWJ_SEQUENCE;
               rq_addr <= rp; fwait <= 2; pret <= P_COUNT; pst <= P_READ;
            end else begin
               status <= DAP_ERROR; pst <= P_DRAIN;
            end
         end
         P_COUNT:
            if (swj) begin                                       // SWJ_Sequence: bits, 0 for 256
               bits <= data == 0 ? 9'd256 : {1'b0, data};
               seq_tms <= 0; seq_cap <= 0; seqs <= 0;
               pst <= P_NEXT;
            end else if (data == 0) pst <= P_DRAIN;
            else begin
               seqs <= data - 1'b1;
               rq_addr <= rp; fwait <= 2; pret <= P_INFO; pst <= P_READ;
            end
         P_INFO: begin                                   // TCK cycles in [5:0] (0 for 64), TMS, TDO
            bits <= data[5:0] == 0 ? 9'd64 : {3'b0, data[5:0]};
            seq_tms <= data[6]; seq_cap <= data[7];
            pst <= P_NEXT;
         end
         P_NEXT:                                                   // The next byte of this sequence
            if (bits != 0) begin
               rq_addr <= rp; fwait <= 2; pret <= P_PUSH; pst <= P_READ;
            end else if (seqs != 0) begin
               seqs <= seqs - 1'b1;
               rq_addr <= rp; fwait <= 2; pret <= P_INFO; pst <= P_READ;
            end else pst <= P_DRAIN;
         P_PUSH:
            if (!q_full) begin
               bits <= bits - take;
               pst <= P_NEXT;
            end
         P_DRAIN: begin                                      // Every bit out and its TDO written
            parsing <= 0;
            if (!parsing && wire_idle) pst <= P_HDR0;
         end
         P_HDR0: begin
            rs_we <= 1; rs_addr <= rs_base; rs_data <= cmd; pst <= P_HDR1;
         end
         P_HDR1: begin
            rs_we <= 1; rs_addr <= rs_base + 1'b1; rs_data <= status;
            cmd_rq_len <= rp - rq_base; cmd_rs_len <= status == DAP_OK ? wp - rs_base : 2'd2;
            pst <= P_DONE;
         end
         P_DONE: begin
            cmd_done <= ~cmd_done; pst <= P_IDLE;
         end
         default: pst <= P_IDLE;
         endcase
         // TDO bytes, as they come
         if (tdo_byte_valid) begin
            rs_we <= 1; rs_addr <= wp; rs_data <= tdo_byte; wp <= wp + 1'b1;
         end
      end
   end

   // The queue
   wire q_push = pst == P_PUSH && !q_full;
   always @(posedge clk) begin
      if (rst) begin
         q_wr <= 0; q_rd <= 0; q_count <= 0;
      end else begin
         if (q_push) begin
            q_data[q_wr * 8 +: 8] <= data; q_n[q_wr * 4 +: 4] <= take[3:0];
            q_flags[q_wr * 2 +: 2] <= {swj, seq_cap}; q_tms[q_wr] <= seq_tms;
            q_wr <= ~q_wr;
         end
         if (q_pop) q_rd <= ~q_rd;
         q_count <= q_count + q_push - q_pop;
      end
   end

   // The wire ============================================================================================
   //
   // A bit takes lo + hi half cycles: low from its start, high from `lo`, its TDO taken at `samp`,
   // over after `per`.  The engine works two half cycles a cycle and decides both from registers:
   // `c` is the bit's count at the cycle's first half, and f0 and f1 say for that half and the next
   // whether TCK is still low, TDO is taken and the bit ends there, worked out a cycle ahead.  With
   // lo and hi at least two, a bit's first two half cycles are low, take nothing and end nothing,
   // and at most one bit starts in a cycle.
   //
   // `nxt` holds the operation that comes next, `cur` the one whose bits are going out: sr its data
   // still to go, LSB first.
   reg [9:0]  hi, lo, per, samp;
   reg [2:0]  f2, f3;                            // {low, sample, end} for counts 2 and 3: fixed
   reg        inbit;
   reg [9:0]  c;
   reg [2:0]  f0, f1;                                                    // {low, sample, end}
   reg        nxt_v, cur_v;
   reg [7:0]  nxt_data, sr;
   reg [3:0]  nxt_n, cur_left;
   reg [1:0]  nxt_flags, cur_flags;                                        // {on TMS, TDO kept}
   reg        nxt_tms, cur_tms;
   reg [2:0]  cur_i;
   reg        lvl_tck, lvl_tms, lvl_tdi;
   reg        b_cap, b_last;                                   // The bit in the wire
   reg [2:0]  b_i;

   function [2:0] flags(input [9:0] k);
      flags = {k < lo, k == samp, k == per - 1'b1};
   endfunction

   // This cycle
   wire       have    = cur_v | nxt_v;
   wire       end0    = inbit & f0[0];
   wire       start0  = running & !inbit & have;
   wire       start1  = running & end0 & have;
   wire       start   = start0 | start1;
   wire       from_nxt = start & !cur_v;
   wire       bitval  = cur_v ? sr[0] : nxt_data[0];
   wire [1:0] bflags  = cur_v ? cur_flags : nxt_flags;
   wire       btms    = cur_v ? cur_tms : nxt_tms;
   wire       new_tms = bflags[1] ? bitval : btms;
   wire       new_tdi = bflags[1] ? lvl_tdi : bitval;
   wire       go1     = inbit & !end0;                                 // the bit goes on into half 1
   wire       end1    = go1 & f1[0];

   wire       tck0 = inbit ? !f0[2] : start0 ? 1'b0 : lvl_tck;
   wire       tck1 = go1 ? !f1[2] : (start0 | start1) ? 1'b0 : end0 ? 1'b1 : lvl_tck;
   wire       tms0 = start0 ? new_tms : lvl_tms, tms1 = start ? new_tms : lvl_tms;
   wire       tdi0 = start0 ? new_tdi : lvl_tdi, tdi1 = start ? new_tdi : lvl_tdi;
   wire [1:0] ev = {go1 & f1[1] & b_cap, inbit & f0[1] & b_cap};
   wire [5:0] ev_i = {b_i, b_i};
   wire [1:0] ev_last = {b_last, b_last};
   wire       c_go = go1 & !end1;                                       // still in the bit next cycle
   wire [9:0] c2 = c + 2'd2;

   assign q_pop = (!nxt_v || from_nxt) && q_count != 0;

   always @(posedge clk) begin
      if (rst) begin
         running <= 0; inbit <= 0; cur_v <= 0; nxt_v <= 0;
         lvl_tck <= 1; lvl_tms <= 1; lvl_tdi <= 0;
         tck <= 2'b11; tms <= 2'b11; tdi <= 2'b00;
      end else begin
         if (pst == P_IDLE && req_new) begin
            running <= 1;
            hi <= clk_hi; lo <= clk_lo; per <= clk_hi + clk_lo;
            samp <= clk_lo - 1'b1 + (cfg_sample == 0 ? (clk_hi + 1'b1) >> 1 :
                                     cfg_sample > clk_hi ? clk_hi : cfg_sample);
         end else if (pst == P_DONE) running <= 0;
         // Counts 2 and 3 against the clock just taken, a cycle before any bit can need them
         f2 <= {10'd2 < lo, 10'd2 == samp, 10'd2 == per - 1'b1};
         f3 <= {10'd3 < lo, 10'd3 == samp, 10'd3 == per - 1'b1};

         if (!running && pins_new) begin
            lvl_tck <= pins_tck; lvl_tms <= pins_tms; lvl_tdi <= pins_tdi;
            tck <= {2{pins_tck}}; tms <= {2{pins_tms}}; tdi <= {2{pins_tdi}};
         end else if (running) begin
            tck <= {tck1, tck0}; tms <= {tms1, tms0}; tdi <= {tdi1, tdi0};
            lvl_tck <= tck1; lvl_tms <= tms1; lvl_tdi <= tdi1;

            // The bit's count and flags for the next cycle
            if (start0) begin
               inbit <= 1; c <= 10'd2; f0 <= f2; f1 <= f3;
            end else if (start1) begin
               inbit <= 1; c <= 10'd1; f0 <= 3'b100; f1 <= f2;
            end else if (c_go) begin
               c <= c2; f0 <= flags(c2); f1 <= flags(c2 + 1'b1);
            end else inbit <= 0;

            if (start) begin
               b_cap <= bflags[0];
               b_i <= cur_v ? cur_i : 3'd0;
               b_last <= cur_v ? cur_left == 1 : nxt_n == 1;
               if (cur_v) begin
                  sr <= sr >> 1; cur_i <= cur_i + 1'b1; cur_left <= cur_left - 1'b1;
                  if (cur_left == 1) cur_v <= 0;
               end else begin
                  sr <= nxt_data >> 1; cur_i <= 3'd1; cur_left <= nxt_n - 1'b1;
                  cur_flags <= nxt_flags; cur_tms <= nxt_tms;
                  cur_v <= nxt_n != 1;
               end
            end
         end

         // The next operation, from the queue
         if (q_pop) begin
            nxt_v <= 1;
            nxt_data <= q_data[q_rd * 8 +: 8]; nxt_n <= q_n[q_rd * 4 +: 4];
            nxt_flags <= q_flags[q_rd * 2 +: 2]; nxt_tms <= q_tms[q_rd];
         end else if (from_nxt) nxt_v <= 0;
      end
   end

   // TDO ==================================================================================================
   //
   // A pair goes out a cycle after it is set and the IDDR gives back that cycle's pin a cycle after
   // that: two cycles on, tdo[1] is the line at the end of the pair's first half, tdo[0] at the end
   // of its second.  The samples follow the pins through the same two stages.
   reg [1:0]  ev0, ev1, ev2;
   reg [5:0]  ev0_i, ev1_i, ev2_i;
   reg [1:0]  ev0_last, ev1_last, ev2_last;
   always @(posedge clk) begin
      if (rst) begin
         ev0 <= 0; ev1 <= 0; ev2 <= 0;
      end else begin
         ev0 <= running ? ev : 2'b00; ev0_i <= ev_i; ev0_last <= ev_last;
         ev1 <= ev0; ev1_i <= ev0_i; ev1_last <= ev0_last;
         ev2 <= ev1; ev2_i <= ev1_i; ev2_last <= ev1_last;
      end
   end
   // At most one sample a cycle: a bit is at least four half cycles
   wire       s_v    = ev2[0] | ev2[1];
   wire       s_bit  = ev2[0] ? tdo[1] : tdo[0];
   wire [2:0] s_i    = ev2[0] ? ev2_i[2:0] : ev2_i[5:3];
   wire       s_last = ev2[0] ? ev2_last[0] : ev2_last[1];
   reg  [7:0] acc;
   always @(posedge clk) begin
      tdo_byte_valid <= 0;
      if (s_v) begin
         if (s_last) begin
            tdo_byte <= (acc & ~(8'd1 << s_i)) | ({7'd0, s_bit} << s_i);
            tdo_byte_valid <= 1; acc <= 0;
         end else acc <= (acc & ~(8'd1 << s_i)) | ({7'd0, s_bit} << s_i);
      end
      if (rst) acc <= 0;
   end

   assign wire_idle = q_count == 0 && !nxt_v && !cur_v && !inbit && ev0 == 0 && ev1 == 0 && ev2 == 0 &&
                    !tdo_byte_valid;
   assign busy = running;
endmodule
