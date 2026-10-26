`default_nettype none
//
// SPDX-License-Identifier: MIT OR Apache-2.0
//
// swd_phy
// =======
//
// SWD on the wire, one operation after another: a transfer, a run of bits driven or read (the
// sequences), or a calibration.  SWCLK and the point at which SWDIO is read are set in ticks of
// half a clock cycle: the engine works two ticks a cycle and hands its pins out as pairs, the value
// for each half of the cycle, for DDR registers at the pins (ODDR for SWCLK, SWDIO and its
// direction, IDDR for SWDIO in).
//
// The executor offers the next operation while one runs; the engine takes it in the idle slot after
// the running transfer, once that transfer's result is in, so transfers follow each other with one
// idle slot and no stop of the clock.  Runs of bits, driven or read, follow each other with none,
// as DAP.c clocks a DAP_SWD_Sequence: a line reset of 50 ones may span operations, and a TARGETSEL
// of bits driven, read and driven again (as OpenOCD sends it) lands on the cycle.  A run's bits
// read come in a round trip after its slots, so they are taken by a capture of their own and given
// as its result while the runs after it go on; the engine does not park before they are in.  With nothing offered the engine parks: SWCLK high, SWDIO
// driven low.  A transfer that fails (WAIT after its retries, FAULT, no or a late ACK, a parity
// error) halts the engine: it takes nothing more until `resume`.
//
// A bit the target puts out on a rising edge reaches the probe a round trip later, which at tens
// of MHz is more than half a period.  The engine therefore reads each bit `sample` ticks after
// the rising edge that sent it, whatever the period, and decides on the ACK as soon as its last
// bit is in: after a WAIT or FAULT on a read the line must be driven low before the target looks
// for the next start bit.  If the ACK is not in by then, the transfer is reported late (ACK 7),
// never guessed.
//
// Calibration reads DPIDR four times at every sampling delay, with SWDIO released throughout
// whatever the ACK seems to be, so a wrong delay cannot make both sides drive the line.  A delay
// is good when every read had ACK OK in time, good parity and the same word: a delay that reads
// the bits shifted cannot see ACK OK, which is 1 0 0 between a released line and the data.  The
// engine then samples in the middle of the longest run of good delays.
//
// Slots of a transfer, T turnaround cycles and I idle cycles; a slot's bit is put out by the
// target on the rising edge before it, and read by the target on the rising edge that ends it.
//
//     1-8 header | T trn | 3 ack | read OK:  32 data, parity, T trn | I idle
//                                | write OK: T trn, 32 data, parity | I idle
//                                | other:    T trn, then low (2, or 33 with the data phase)
//
// The engine decides what it drives in slot 12+2T, the first that differs, once the ACK is in.

module swd_phy #(parameter DEFAULT_SAMPLE = 5) (
                input wire             clk,
                input wire             rst,

        // Pins, as [first half, second half] of the cycle after this one =================================
                output reg        own,                     // The engine drives the pins, not the slot
                output wire [1:0]      swclk,
                output wire [1:0]      swdo,
                output wire [1:0]      swwr,                                        // 1: SWDIO driven
                input wire  [1:0]      swdi,          // From an IDDR in SAME_EDGE mode: [0] the first
                                                 // half of the cycle the outputs set in the cycle
                                                 // before were on the pins, [1] the second half
                                                 // of the one before that

        // Configuration, unchanged while the engine runs =================================================
                input wire [9:0]       cfg_hi,                           // SWCLK high, in ticks (2 or more)
                input wire [9:0]       cfg_lo,                            // SWCLK low, in ticks (2 or more)
                input wire [1:0]       cfg_turn,                                         // Turnaround less 1
                input wire [7:0]       cfg_idle,                         // Idle cycles after each transfer
                input wire             cfg_dataphase,                    // Data phase after WAIT and FAULT
                input wire [6:0]       cfg_sample,                // [6]: sample [5:0] ticks, not calibrated

        // Operations: fields stable while op_valid, taken when op_ready pulses ===========================
                input wire             op_valid,
                output reg        op_ready,
                input wire [1:0]       op_kind,          // 0 transfer, 1 bits driven, 2 bits read, 3 calibrate
                input wire             op_rnw,
                input wire             op_apndp,
                input wire [1:0]       op_addr,                                              // Address [3:2]
                input wire [31:0]      op_data,                         // Word written, or bits, LSB first
                input wire [4:0]       op_bits,                                    // Bits less 1 (1 to 32)
                input wire [15:0]      op_retries,                                      // Retries after WAIT
                input wire [3:0]       op_tag,                          // Returned with the result as it is
                input wire             resume,                                     // Take operations again
                input wire             res_hold,          // Bits read wait: the executor stores a result

        // Results, one cycle each: every operation taken gives one ========================================
                output reg        res_valid,
                output reg [2:0]  res_ack,             // ACK, 7: none or late; bits and calibration: 1
                output reg        res_perr,                                            // Parity error
                output reg [31:0] res_data,                       // Word read, or bits read, LSB first
                output reg [3:0]  res_tag,
                output reg        halted,                // A transfer failed: nothing more till resume
                output wire            busy,                       // Running, or an operation not yet taken

        // Calibration ====================================================================================
                output reg [5:0]  cal_sample,                 // Sampling delay in use (low six bits)
                output reg [5:0]  cal_first,                                // Good delays found: first
                output reg [6:0]  cal_count,                                //   and how many
                output reg        cal_ok,                               // The last calibration found some
                output reg        cal_seen                       // It saw an ACK OK, timely or not
             );

   localparam K_XFER = 2'd0, K_OUT = 2'd1, K_IN = 2'd2, K_CAL = 2'd3;
   localparam CAL_READS = 4;
   localparam CAL_FIRST = 1, CAL_LAST = 63;                      // Delays calibration tries
   localparam CAL_TICKS = 16;                     // A calibration slower than this runs at this
   localparam LINGER = 4;                        // Idle slots waited for an operation before parking

   reg        rst_q = 1;
   always @(posedge clk) rst_q <= rst;

   // The operation offered, registered a cycle after it appears, two after one was taken ===============
   reg        nx_have;
   reg [1:0]  nx_kind;
   reg        nx_rnw;
   reg [7:0]  nx_header;
   reg [32:0] nx_wdata;
   reg [4:0]  nx_bits;
   reg [15:0] nx_retries;
   reg [3:0]  nx_tag;
   reg [1:0]  take_hold;
   always @(posedge clk) begin
      nx_have    <= op_valid && !op_ready && (take_hold == 0) && !halted;
      take_hold  <= op_ready ? 2'd2 : (take_hold != 0) ? take_hold - 1 : 2'd0;
      nx_kind    <= op_kind;
      nx_rnw     <= (op_kind == K_CAL) ? 1'b1 : op_rnw;
      nx_header  <= (op_kind == K_CAL) ?
                    {1'b1, 1'b0, 1'b1, 2'b00, 1'b1, 1'b0, 1'b1} :                  // DP read of DPIDR
                    {1'b1, 1'b0, op_apndp ^ op_rnw ^ op_addr[1] ^ op_addr[0],
                     op_addr[1], op_addr[0], op_rnw, op_apndp, 1'b1};
      nx_wdata   <= (op_kind == K_XFER) ? {^op_data, op_data} : {1'b0, op_data};
      nx_bits    <= op_bits;
      nx_retries <= op_retries;
      nx_tag     <= op_tag;
   end

   // The operation running ==============================================================================
   reg [1:0]  kind;
   reg        rnw;
   reg [7:0]  header;
   reg [32:0] wdata;
   reg [4:0]  bits;
   reg [15:0] retries;
   reg [3:0]  tag;
   reg [2:0]  turn;                                                             // Turnaround cycles
   reg [7:0]  idle;
   reg        dataphase;
   reg        is_cal, is_xfer, is_in, is_out;                     // kind, decoded when it was taken
   reg        cal_run;                                         // The engine runs for a calibration
   wire       fixed = is_cal;                                // Every transfer clocked as a read OK

   // SWCLK ==============================================================================================
   // A down counter of ticks, two a cycle: cnt0 is the first half's, cnt1 the second's.  SWCLK is
   // high while the count is `lo` or more; the slot's value goes out after the tick where it is
   // `lo - lo/2`, in the middle of the low half.  A calibration asked for at a clock slower than
   // CAL_TICKS runs at CAL_TICKS: it measures the round trip, which does not depend on the clock.
   reg [10:0] period_m1, ev_cnt, lo, cfg_period_m1, ev_cnt_m1;
   wire       cal_clock = cal_run && (cfg_hi + cfg_lo > CAL_TICKS);
   always @(posedge clk) begin
      cfg_period_m1 <= cfg_hi + cfg_lo - 1;
      period_m1 <= cal_clock ? CAL_TICKS - 1 : cfg_hi + cfg_lo - 1;
      lo        <= cal_clock ? CAL_TICKS / 2 : cfg_lo;
      ev_cnt    <= cal_clock ? CAL_TICKS / 4 : cfg_lo - (cfg_lo >> 1);
      ev_cnt_m1 <= (cal_clock ? CAL_TICKS / 4 : cfg_lo - (cfg_lo >> 1)) - 1;
   end

   reg  [10:0] cnt0;
   wire [10:0] cnt1      = (cnt0 == 0) ? period_m1 : cnt0 - 1;
   wire [10:0] cnt0_next = (cnt1 == 0) ? period_m1 : cnt1 - 1;
   reg         running, go, arm, stopping;
   // The compares on the count are made a cycle early and held: they start every path that decides
   // what the pins and the input side do.  The count steps down by two a cycle around the period,
   // so it becomes X next cycle when it is X + 2 (around the period) now: a compare of two
   // registers.  The comparands are held a cycle or two behind what they follow, which changes
   // only between runs, or in calibration, which then idles for it.
   reg         rise0_q, rise1_q, evA_q, evB_q, stb0_q, stb1_q;
   wire        rise0 = running && rise0_q;                       // The first high tick
   wire        rise1 = running && rise1_q;                       // cnt1 is period_m1 then
   // The slot changes after the tick with count ev_cnt.  The engine commits in the cycle whose
   // pins carry the new value first: if that is the second half, evA; the first, evB.  A cycle
   // later would be too late for the ODDR, a cycle earlier needlessly early for the ACK.
   wire        evA   = running && evA_q;
   wire        evB   = running && evB_q;
   wire        ev    = evA || evB;

   // Output side ========================================================================================
   localparam SQ_PRE = 3'd0, SQ_HDR = 3'd1, SQ_REL = 3'd2, SQ_DECIDE = 3'd3, SQ_DATA = 3'd4,
              SQ_LOW = 3'd5, SQ_NEXT = 3'd6, SQ_PARK = 3'd7;
   reg [2:0]  sq;
   reg [5:0]  sq_left;                                         // Slots left in this state, less 1
   reg [7:0]  hdr_sr;
   reg [32:0] wdata_sr;
   reg        writing;                                // SQ_DATA drives wdata_sr rather than releases
   reg        after_op;                                  // SQ_NEXT follows an operation, not the start
   reg        first;                                       // The next rising edge starts slot 1
   reg        line_o, line_oe;                               // What SWDIO carries since the event
   reg [2:0]  linger;                                     // Idle slots left before parking

   // Runs of bits read.  A run's first rising edge is marked in rhist, as mhist marks an
   // operation's; its length and tag wait in a queue until that edge's strobe, and a capture of
   // its own takes the bits, so the run may chain into the next while they are on the way.  Its
   // result is queued, and given in a cycle no other result is, when the executor can take it.
   reg        rawf;                                     // The next rising edge is a run's first
   reg [15:0] rhist;
   reg [8:0]  rq_mem [0:3];                                             // {bits less 1, tag}
   reg [2:0]  rq_w, rq_r;
   reg        rcap;                                                   // Capturing a run's bits
   reg [4:0]  rleft, ridx;
   reg [31:0] racc;
   reg [3:0]  rtag;
   reg        raw_done;
   reg [31:0] raw_data;
   reg [3:0]  raw_tag;
   reg [35:0] rres_mem [0:3];                                         // {tag, bits read}
   reg [2:0]  rres_w, rres_r;
   wire       raw_pending = (rq_w != rq_r) || rcap;          // Bits still to come in: keep clocking
   reg        gave;                                     // A result was given in this cycle

   // Results from the input side, for the operation running
   reg        ack_in, data_in;                                     // ACK in, word or bits read in
   reg        bad_ack, bad_data;          // Some of it was read while the probe drove the line itself
   reg [2:0]  ack_val;
   reg        par_ok;
   reg [31:0] rdata;
   reg        clr;                                               // An operation starts: forget them
   reg        late;                                              // This transfer's ACK came too late

   // Calibration bookkeeping.  The first good delay is the round trip, which does not change with
   // the clock: at the clock calibrated the engine samples in the middle of the good delays, at
   // another half a period after the round trip.
   reg [10:0] cal_period_m1;
   reg [5:0]  first_good;                                             // cal_first, kept here
   reg [10:0] auto_sample;
   reg        same_clock;
   always @(posedge clk) begin
     same_clock  <= cal_ok && (cfg_period_m1 == cal_period_m1);
     auto_sample <= same_clock ? first_good + ((cal_count - 1) >> 1) : first_good + (cfg_period_m1 >> 1);
   end
   reg [10:0] sample;                                                 // Sampling delay, in ticks
   reg [10:0] smod;                                             // sample, less whole periods,
   reg [4:0]  sper;                                             //   and how many periods those are
   reg [10:0] scnt;                                         // The count of the ticks it reads at
   reg [5:0]  try;                                                           // Delay being tried
   reg [2:0]  try_reads;
   reg [31:0] try_ref;                                // What the first read at this delay gave
   reg        try_good;
   reg [5:0]  run_first, best_first;
   reg [6:0]  run_len, best_len;
   reg  [1:0] resample;            // Idle slots after the delay changed: its compares follow it

   // Results, registered once: a slot is at least two cycles, so they are in time for the event
   // after the one that changed what they depend on
   reg        ack_in_r, data_in_r, ack_ok_r, data_ok_r, retry_r, bad_ack_r, cal_good_r, run_better;
   always @(posedge clk) begin
      ack_in_r    <= ack_in;
      data_in_r   <= data_in;
      ack_ok_r    <= ack_in && (ack_val == 3'b001) && !bad_ack;
      data_ok_r   <= data_in && par_ok && !bad_data;
      retry_r     <= ack_in && (ack_val == 3'b010) && !bad_ack && (retries != 0);
      bad_ack_r   <= bad_ack;
      cal_good_r  <= ack_in && (ack_val == 3'b001) && data_in && par_ok && !bad_ack && !bad_data &&
                     (try_reads == 0 || rdata == try_ref);
      run_better  <= (run_len + 1 > best_len);
   end

   // Controller =========================================================================================
   localparam C_IDLE = 3'd0, C_PREP = 3'd1, C_RUN = 3'd2, C_DONE = 3'd3;
   reg [2:0]  ctl;
   reg        accounted;                          // The last operation's result has been given
   reg        prep_wait;                            // C_PREP: the clock settings follow `kind` first
   assign busy = (ctl != C_IDLE) || nx_have || (op_valid && !halted);

   // The operation offered can follow the running one in this run of the clock: not a calibration,
   // whose clock may differ, and none after one
   wire       nx_ok = nx_have && (nx_kind != K_CAL) && !is_cal;

   localparam A_WAIT = 4'd0, A_RESAMPLE = 4'd1, A_START = 4'd2, A_TAKE_START = 4'd3,
              A_RETRY = 4'd4, A_FAIL = 4'd5, A_PERR = 4'd6, A_DONE_TAKE_START = 4'd7,
              A_DONE = 4'd8, A_PARK = 4'd9, A_CAL_READ = 4'd10, A_CAL_DELAY_DONE = 4'd11,
              A_CAL_NEXT = 4'd12, A_CAL_FINISH = 4'd13, A_LINGER = 4'd14;
   // What SQ_NEXT does: the last operation's result, then what next.  Decided a cycle before it is
   // used: an event is two cycles after the one before at least, and what the decision reads has
   // settled a cycle after that one.
   // Its inputs as the input side has them, not registered again: the decision's own register is
   // their second, as the registered results its actions use are
   wire       ack_ok_c  = ack_in && (ack_val == 3'b001) && !bad_ack;
   wire       retry_c   = ack_in && (ack_val == 3'b010) && !bad_ack && (retries != 0);
   wire       data_ok_c = data_in && par_ok && !bad_data;
   reg [3:0]  act_c, act;
   always @(posedge clk) act <= act_c;
   always @(*) begin
      if (resample != 0)
        act_c = A_RESAMPLE;
      else if (after_op && !accounted) begin
         if (is_cal)
           act_c = !(data_in || late) ? A_WAIT :
                 (try_reads != CAL_READS - 1) ? A_CAL_READ : A_CAL_DELAY_DONE;
         else if (is_out)
           act_c = nx_ok ? A_DONE_TAKE_START : A_DONE;
         else if (is_in)
           act_c = nx_ok ? A_DONE_TAKE_START : A_DONE;              // Its result: from the capture
         else if (late || bad_ack || (ack_in && !ack_ok_c && !retry_c))
           act_c = A_FAIL;
         else if (retry_c)
           act_c = A_RETRY;
         else if (ack_ok_c && (!rnw || data_in))
           act_c = (rnw && !data_ok_c) ? A_PERR : nx_ok ? A_DONE_TAKE_START : A_DONE;
         else
           act_c = A_WAIT;                                         // Not in yet: an idle slot
      end else if (is_cal && after_op)
        act_c = (try == CAL_LAST) ? A_CAL_FINISH : A_CAL_NEXT;
      else if (!after_op)
        act_c = A_START;                                // The operation taken when the engine started
      else if (nx_ok)
        act_c = A_TAKE_START;
      else if (raw_pending)
        act_c = A_WAIT;
      else if (linger != 0)
        act_c = A_LINGER;
      else
        act_c = A_PARK;
   end
   wire       taking   = (sq == SQ_NEXT) && (act == A_TAKE_START || act == A_DONE_TAKE_START);
   wire       starting = (sq == SQ_NEXT) &&
                         (act == A_START || taking || act == A_RETRY || act == A_CAL_READ);
   // Runs of bits follow each other with no idle slot: the next is taken in the last slot's event
   wire       chain    = (sq == SQ_DATA) && (sq_left == 0) && (is_out || is_in) &&
                         nx_ok && (nx_kind == K_OUT || nx_kind == K_IN);
   // What starts: the operation taken now, or the one running
   wire [1:0] s_kind   = taking ? nx_kind : kind;
   wire       s_xfer   = (s_kind == K_XFER) || (s_kind == K_CAL);
   // A transfer starting at an evB commit, with the rising edge in the same cycle's second half:
   // that edge is the one before its slot 1
   wire       mark_now = (ctl == C_RUN) && starting && s_xfer && evB && rise1;
   wire       mark_in  = first || mark_now;
   // A run read starts in this event: its first bit is the one the target puts out on the next
   // rising edge, which ends the slot this event puts on the line (the last driven, or released)
   wire       raw_start = (ctl == C_RUN) && ev &&
                          ((starting && s_kind == K_IN) || (chain && nx_kind == K_IN));
   wire       rmark_now = raw_start && evB && rise1;
   wire       rmark_in  = rawf || rmark_now;
   wire [8:0] raw_desc  = (chain || taking) ? {nx_bits, nx_tag} : {bits, tag};

   // What the event puts on the line: released for the turnaround, the ACK and a read's data; the
   // header, a write's data, or low otherwise.  The ACK decision reads the ACK as it comes in.
   wire       ack_last;                                  // The input side takes the last ACK bit now
   wire [2:0] ack_val_now;
   wire       ack_have   = ack_in || ack_last;
   wire       drv_in;                     // The bit being read was taken while the probe drove
   wire       ack_ok_now = ack_in ? (ack_val == 3'b001 && !bad_ack) :
                           (ack_last && ack_val_now == 3'b001 && !bad_ack && !drv_in);
   wire       read_on = fixed || (rnw && ack_ok_now);
   // Bits read keep the line released until the last is in: with a round trip that is after the
   // run's last slot, as it is for a read's parity
   // A run of bits read is released from the slot before its first, as a host clocking it releases
   // the line and then reads in each low half
   wire       drive = !((sq == SQ_REL) || (sq == SQ_DATA && !writing) || (sq == SQ_DECIDE && read_on) ||
                        (sq == SQ_NEXT && is_in && after_op && !accounted) ||
                        (sq == SQ_NEXT && starting && s_kind == K_IN));
   wire       value = (sq == SQ_HDR) ? hdr_sr[0] :
                      (sq == SQ_DATA) ? wdata_sr[0] :
                      (sq == SQ_DECIDE && !rnw && ack_ok_now) ? wdata[0] : 1'b0;

   // The pins.  Parked, SWCLK is high and SWDIO driven low.
   wire       new_o  = (ctl == C_RUN && ev) ? value : line_o;
   wire       new_oe = (ctl == C_RUN && ev) ? drive : line_oe;
   assign swclk = running ? {cnt1 >= lo, cnt0 >= lo} : 2'b11;
   assign swdo  = (ctl == C_RUN && evA) ? {new_o, line_o} : {new_o, new_o};
   assign swwr  = (ctl == C_RUN && evA) ? {new_oe, line_oe} : {new_oe, new_oe};

   // Taking the operation offered: its fields become the running one's
   task take;
      begin
         op_ready <= 1;
         kind     <= nx_kind;
         is_cal   <= (nx_kind == K_CAL);
         is_xfer  <= (nx_kind == K_XFER);
         is_in    <= (nx_kind == K_IN);
         is_out   <= (nx_kind == K_OUT);
         rnw      <= nx_rnw;
         header   <= nx_header;
         wdata    <= nx_wdata;
         bits     <= nx_bits;
         retries  <= nx_retries;
         tag      <= nx_tag;
      end
   endtask

   // The result of the running operation
   task result(input [2:0] ack, input perr);
      begin
         gave       = 1;
         res_valid <= 1;
         res_ack   <= ack;
         res_perr  <= perr;
         res_data  <= rdata;
         res_tag   <= tag;
      end
   endtask

   always @(posedge clk) begin
      gave       = 0;
      op_ready  <= 0;
      res_valid <= 0;
      clr       <= 0;
      go        <= arm;
      arm       <= 0;
      if (resume) halted <= 0;

      // SWCLK
      if (go) begin
         running <= 1; cnt0 <= lo;                          // first the falling edge, mid-cycle
      end else if (running) begin
         cnt0 <= cnt0_next;
         if (rise0 || rise1) begin first <= 0; rawf <= 0; end
         if ((rise0 || rise1) && stopping) begin
            running <= 0;                                     // SWCLK stays high from here
            ctl     <= C_DONE;
         end
      end

      case (ctl)
        C_IDLE:
          if (nx_have) begin
             take;
             cal_run    <= (nx_kind == K_CAL);
             turn       <= {1'b0, cfg_turn} + 3'd1;
             idle       <= cfg_idle;
             dataphase  <= cfg_dataphase;
             after_op   <= 0; accounted <= 0; late <= 0; first <= 0; resample <= 0;
             writing    <= 0; stopping <= 0; linger <= LINGER;
             if (nx_kind == K_CAL) begin
                try <= CAL_FIRST; try_reads <= 0; try_good <= 1; sample <= CAL_FIRST;
                smod <= CAL_FIRST;
                cal_seen <= 0;
                run_len <= 0; best_len <= 0; best_first <= 0; run_first <= 0;
             end else begin
                sample <= cfg_sample[6] ? {5'd0, cfg_sample[5:0]} : auto_sample;
                smod   <= cfg_sample[6] ? {5'd0, cfg_sample[5:0]} : auto_sample;
             end
             sper <= 0; prep_wait <= 1;
             // Take the pins, parked as the slot leaves them
             own <= 1; line_o <= 0; line_oe <= 1;
             sq  <= SQ_PRE; sq_left <= 1;
             ctl <= C_PREP;
          end

        C_PREP:
          // The sampling delay in whole periods and the rest, once the clock settings follow `kind`
          if (prep_wait)
             prep_wait <= 0;
          else if (smod > period_m1) begin
             smod <= smod - period_m1 - 1;
             sper <= sper + 1;
          end else begin
             scnt       <= period_m1 - smod;
             cal_sample <= sample[5:0];
             ctl        <= C_RUN;
             arm        <= 1;
          end

        C_RUN:
          if (ev) begin
             // This event puts the next slot on the line ...
             line_o  <= value;
             line_oe <= drive;
             if (raw_start) begin
                rawf <= !rmark_now;
                rq_mem[rq_w[1:0]] <= raw_desc;
                rq_w <= rq_w + 1'b1;
             end

             // ... and moves on
             if (starting) begin
                after_op <= 1; accounted <= 0; late <= 0; clr <= 1; first <= s_xfer && !mark_now;
                linger <= LINGER;
                if (s_xfer) begin
                   // The next event drives the start bit, the first of SQ_HDR
                   sq <= SQ_HDR; sq_left <= 7; hdr_sr <= taking ? nx_header : header;
                end else begin
                   // A run of bits: the next event drives or releases the first
                   sq <= SQ_DATA; sq_left <= {1'b0, taking ? nx_bits : bits};
                   writing  <= (s_kind == K_OUT);
                   wdata_sr <= taking ? nx_wdata : wdata;
                end
             end
             case (sq)
               SQ_PRE:                                        // Two idle cycles before the first
                 if (sq_left == 0) sq <= SQ_NEXT; else sq_left <= sq_left - 1;

               SQ_HDR: begin
                  hdr_sr <= hdr_sr >> 1;
                  if (sq_left == 0) begin
                     sq <= SQ_REL; sq_left <= {turn, 1'b0} + 6'd2;         // 2T+3 slots, less 1
                  end else sq_left <= sq_left - 1;
               end

               SQ_REL:
                 if (sq_left == 0) sq <= SQ_DECIDE; else sq_left <= sq_left - 1;

               SQ_DECIDE: begin                                  // Slot 12+2T: what the ACK says
                  late <= !ack_have;
                  if (read_on) begin
                     sq <= SQ_DATA; sq_left <= 31; writing <= 0;             // The rest of a read
                  end else if (!rnw && ack_ok_now) begin
                     sq <= SQ_DATA; sq_left <= 31; writing <= 1; wdata_sr <= wdata >> 1;
                  end else begin
                     sq <= SQ_LOW; sq_left <= dataphase ? 6'd32 : 6'd1;
                  end
               end

               SQ_DATA:
                 if (sq_left == 0) begin
                    if (chain) begin
                       // The next run of bits follows in the next slot; a run read gives its result
                       // from the capture
                       if (!is_in) result(3'b001, 1'b0);
                       take;
                       sq_left  <= {1'b0, nx_bits};
                       wdata_sr <= nx_wdata;
                       writing  <= (nx_kind == K_OUT);
                    end else if (is_xfer || is_cal) begin
                       if (idle != 0) begin sq <= SQ_LOW; sq_left <= idle - 1; end
                       else sq <= SQ_NEXT;
                    end else
                       sq <= SQ_NEXT;
                 end else begin
                    sq_left  <= sq_left - 1;
                    wdata_sr <= wdata_sr >> 1;
                 end

               SQ_LOW:
                 if (sq_left == 0) sq <= SQ_NEXT; else sq_left <= sq_left - 1;

               SQ_NEXT:
                 // An idle slot (low).  First the last operation's result is given, once it is in:
                 // with a long round trip that is some slots later.  Then the next operation starts,
                 // the engine waits a few slots for one, or it parks.
                 case (act)
                   A_RESAMPLE: resample <= resample - 1;
                   A_RETRY: retries <= retries - 1;                   // WAIT: the same again
                   A_FAIL: begin
                      result((late || bad_ack_r) ? 3'b111 : ack_val, 1'b0); // Failed: stop here
                      halted <= 1;
                      sq     <= SQ_PARK;
                   end
                   A_PERR: begin
                      result(3'b001, 1'b1);                           // Parity error
                      halted <= 1;
                      sq     <= SQ_PARK;
                   end
                   A_DONE: begin
                      if (!is_in) result(3'b001, 1'b0);
                      accounted <= 1;
                   end
                   A_LINGER: linger <= linger - 1;
                   A_PARK: sq <= SQ_PARK;
                   A_CAL_READ: begin
                      if (!(cal_good_r && !late)) try_good <= 0;
                      if (ack_ok_r) cal_seen <= 1;
                      if (try_reads == 0) try_ref <= rdata;
                      try_reads <= try_reads + 1;
                   end
                   A_CAL_DELAY_DONE: begin
                      // This delay is done: extend or end the run of good ones
                      if (ack_ok_r) cal_seen <= 1;
                      if (try_good && cal_good_r && !late) begin
                         if (run_len == 0) run_first <= try;
                         run_len <= run_len + 1;
                         if (run_better) begin
                            best_len   <= run_len + 1;
                            best_first <= (run_len == 0) ? try : run_first;
                         end
                      end else
                        run_len <= 0;
                      accounted <= 1;
                   end
                   A_CAL_NEXT: begin
                      // One tick later; the period is CAL_TICKS or less, so this is all it takes
                      try <= try + 1; try_reads <= 0; try_good <= 1;
                      resample <= 2; accounted <= 0; after_op <= 0;
                      if (smod == period_m1) begin
                         smod <= 0; sper <= sper + 1; scnt <= period_m1;
                      end else begin
                         smod <= smod + 1; scnt <= scnt - 1;
                      end
                   end
                   A_CAL_FINISH: begin
                      cal_ok <= (best_len != 0);
                      if (best_len != 0) begin
                         cal_first     <= best_first;
                         first_good    <= best_first;
                         cal_count     <= best_len;
                         cal_period_m1 <= period_m1;        // CAL_TICKS-1 when it ran at that
                      end
                      result((best_len != 0) ? 3'b001 : 3'b111, 1'b0);
                      sq <= SQ_PARK;
                   end
                   default: ;                                     // A_WAIT, A_START: as above
                 endcase

               SQ_PARK: stopping <= 1;                       // Low, then stop with SWCLK high

               default: sq <= SQ_PARK;
             endcase

             // Taken in SQ_NEXT, with the result of the one before
             if (taking) begin
                if (act == A_DONE_TAKE_START && !is_in) result(3'b001, 1'b0);
                take;
                cal_run <= 1'b0;
             end
          end

        C_DONE: begin
           own <= 0;
           ctl <= C_IDLE;
        end

        default: ctl <= C_IDLE;
      endcase

      // Runs read: their results, queued as they come in, out in cycles with no other
      if (raw_done) begin
         rres_mem[rres_w[1:0]] <= {raw_tag, raw_data};
         rres_w <= rres_w + 1'b1;
      end
      if (rres_w != rres_r && !gave && !res_hold) begin
         res_valid <= 1;
         res_ack   <= 3'b001;
         res_perr  <= 1'b0;
         res_data  <= rres_mem[rres_r[1:0]][31:0];
         res_tag   <= rres_mem[rres_r[1:0]][35:32];
         rres_r    <= rres_r + 1'b1;
      end

      // A reset stops the engine and parks the pins; everything else starts from its initial
      // value, and a reset reaching every register would cost the whole cycle in routing
      if (rst_q) begin
         ctl <= C_IDLE; own <= 0; running <= 0; arm <= 0; go <= 0; halted <= 0;
         line_o <= 0; line_oe <= 1; rq_w <= 0; rres_w <= 0; rres_r <= 0; rawf <= 0;
      end
   end

   // Input side =========================================================================================
   // The tick `sample` ticks after a rising edge is the one with count scnt, sper whole periods
   // later.  `first` marks the edge before an operation's slot 1: the marks of the last rising edges
   // are kept, and a strobe takes the one from sper periods back.  Strobes and marks then wait for
   // the IDDR's samples of the same ticks.
   reg [15:0] mhist;                                       // first, at each rising edge, latest in [0]
   wire       stb0 = running && stb0_q;
   // cnt1 == scnt, compared on cnt0: against scnt + 1, or 0 when scnt is the top count
   reg  [10:0] stb1_cnt;
   always @(posedge clk) stb1_cnt <= (scnt == period_m1) ? 11'd0 : scnt + 11'd1;
   wire       stb1 = running && stb1_q;
   function [10:0] plus2(input [10:0] x, input [10:0] top);       // x + 2 around 0..top
      plus2 = (x + 11'd2 > top) ? x + 11'd2 - top - 11'd1 : x + 11'd2;
   endfunction
   reg  [10:0] p2_rise0, p2_rise1, p2_evA, p2_evB, p2_stb0, p2_stb1;
   always @(posedge clk) begin
      p2_rise0 <= plus2(period_m1, period_m1);
      p2_rise1 <= plus2(11'd0, period_m1);
      p2_evA   <= plus2(ev_cnt, period_m1);
      p2_evB   <= plus2(ev_cnt_m1, period_m1);
      p2_stb0  <= plus2(scnt, period_m1);
      p2_stb1  <= plus2(stb1_cnt, period_m1);
      // At the start the count is set to lo
      rise0_q <= go ? (lo == period_m1) : (cnt0 == p2_rise0);
      rise1_q <= go ? (lo == 0)         : (cnt0 == p2_rise1);
      evA_q   <= go ? (lo == ev_cnt)    : (cnt0 == p2_evA);
      evB_q   <= go ? (lo == ev_cnt_m1) : (cnt0 == p2_evB);
      stb0_q  <= go ? (lo == scnt)      : (cnt0 == p2_stb0);
      stb1_q  <= go ? (lo == stb1_cnt)  : (cnt0 == p2_stb1);
   end
   // An edge in this cycle at or before the strobe has not reached mhist yet
   wire       edge_before = (rise0 && (stb0 || stb1)) || (rise1 && stb1);
   wire [16:0] hist_now = {mhist, mark_in};
   wire       mark = edge_before ? hist_now[sper] : mhist[sper];
   reg        s0_d, m0_d;                                     // A first half's: a cycle later
   reg [1:0]  s1_d, m1_d;                                     // A second half's: two cycles later
   wire [16:0] rhist_now = {rhist, rmark_in};
   wire       rmark = edge_before ? rhist_now[sper] : rhist[sper];
   reg        r0_d;
   reg [1:0]  r1_d;
   always @(posedge clk) begin
      if (go || !running) begin mhist <= 0; rhist <= 0; end
      else if (rise0 || rise1) begin
         mhist <= {mhist[14:0], mark_in};
         rhist <= {rhist[14:0], rmark_in};
      end
      s0_d <= stb0;
      m0_d <= stb0 && mark;
      r0_d <= stb0 && rmark;
      s1_d <= {s1_d[0], stb1};
      m1_d <= {m1_d[0], stb1 && mark};
      r1_d <= {r1_d[0], stb1 && rmark};
   end
   // Strobes are two cycles apart at least, so the two never meet
   wire       stb    = s0_d || s1_d[1];
   wire       mstb   = s0_d ? m0_d : m1_d[1];
   wire       rmstb  = s0_d ? r0_d : r1_d[1];
   wire       bit_in = s0_d ? swdi[0] : swdi[1];
   // Whether the probe drove the line when the IDDR took it.  Output pairs reach the pins a cycle
   // after they are set, and the IDDR's samples come a cycle after that: swdi[0] is the line at the
   // rising edge, where the last pair's second half gives way to the next pair's first; swdi[1] at
   // the falling edge between a pair's halves.  An edge where either drove counts as driven.
   reg [1:0]  wr_p1, wr_p2;                              // swwr one and two cycles ago
   always @(posedge clk) begin wr_p1 <= swwr; wr_p2 <= wr_p1; end
   assign     drv_in = s0_d ? (wr_p2[1] | wr_p1[0]) : (wr_p2[0] | wr_p2[1]);

   // Counted from slot 1: a transfer waits for the ACK, takes three ACK bits, then on a read OK the
   // data and parity; a run of bits read takes them from slot 1.  Bits land at their index.
   localparam IN_OFF = 3'd0, IN_PRE = 3'd1, IN_ACK = 3'd2, IN_DATA = 3'd3, IN_PAR = 3'd4;
   reg [2:0]  in_st;
   reg [5:0]  in_left;
   reg [4:0]  in_idx;
   reg        in_par;
   assign ack_last    = stb && !mstb && (in_st == IN_ACK) && (in_left == 0);
   assign ack_val_now = {bit_in, ack_val[2:1]};
   always @(posedge clk) begin
      if (stb) begin
         if (mstb) begin
            in_st <= IN_PRE; in_left <= {3'b0, turn} + 6'd6;       // Slots 2 to 8+T, less 1
         end else case (in_st)
           IN_PRE:
             if (in_left == 0) begin in_st <= IN_ACK; in_left <= 2; end
             else in_left <= in_left - 1;
           IN_ACK: begin
              ack_val <= {bit_in, ack_val[2:1]};
              if (drv_in) bad_ack <= 1;
              if (in_left == 0) begin
                 ack_in <= 1;
                 if (fixed || (rnw && {bit_in, ack_val[2:1]} == 3'b001)) begin
                    in_st <= IN_DATA; in_left <= 31; in_idx <= 0; in_par <= 0;
                 end else
                    in_st <= IN_OFF;
              end else in_left <= in_left - 1;
           end
           IN_DATA: begin
              rdata[in_idx] <= bit_in;
              if (drv_in) bad_data <= 1;
              in_idx <= in_idx + 1;
              in_par <= in_par ^ bit_in;
              if (in_left == 0) in_st <= IN_PAR;
              else in_left <= in_left - 1;
           end
           IN_PAR: begin
              par_ok  <= (in_par == bit_in);
              if (drv_in) bad_data <= 1;
              data_in <= 1;
              in_st   <= IN_OFF;
           end
           default: ;
         endcase
      end
      // An operation starts: what is still coming in of the one before (a calibration's read at a
      // late delay, whose result was given as late) is not its
      if (clr) begin
         ack_in <= 0; data_in <= 0; bad_ack <= 0; bad_data <= 0; in_st <= IN_OFF;
      end
      if (rst_q) in_st <= IN_OFF;
   end

   // A run's bits: from the strobe of its first edge, one a strobe.  Bits read while the probe
   // still drove the line (the first after bits driven, at a short delay) are the probe's own, as
   // in DAP.c's turnaround; they are given as read.
   wire [31:0] racc_now = racc | ({31'd0, bit_in} << ridx);
   always @(posedge clk) begin
      raw_done <= 0;
      if (stb) begin
         if (rmstb) begin
            racc  <= {31'd0, bit_in};
            rtag  <= rq_mem[rq_r[1:0]][3:0];
            rq_r  <= rq_r + 1'b1;
            if (rq_mem[rq_r[1:0]][8:4] == 0) begin
               raw_done <= 1; raw_data <= {31'd0, bit_in}; raw_tag <= rq_mem[rq_r[1:0]][3:0];
            end else begin
               rcap <= 1; ridx <= 1; rleft <= rq_mem[rq_r[1:0]][8:4];
            end
         end else if (rcap) begin
            racc <= racc_now;
            ridx <= ridx + 1'b1;
            if (rleft == 1) begin
               rcap <= 0; raw_done <= 1; raw_data <= racc_now; raw_tag <= rtag;
            end else rleft <= rleft - 1'b1;
         end
      end
      if (rst_q) begin rq_r <= 0; rcap <= 0; end
   end

   initial begin
      ctl = C_IDLE; own = 0; running = 0; arm = 0; go = 0; line_o = 0; line_oe = 1;
      op_ready = 0; res_valid = 0; halted = 0; nx_have = 0; take_hold = 0;
      sample = DEFAULT_SAMPLE; cal_sample = DEFAULT_SAMPLE; cal_run = 0;
      cal_first = DEFAULT_SAMPLE; first_good = DEFAULT_SAMPLE; cal_count = 0; cal_ok = 0;
      cal_period_m1 = 0; cal_seen = 0; ack_in = 0; data_in = 0; in_st = 0; mhist = 0;
      s0_d = 0; s1_d = 0; m0_d = 0; m1_d = 0; r0_d = 0; r1_d = 0; rhist = 0; rawf = 0;
      rq_w = 0; rq_r = 0; rcap = 0; raw_done = 0; rres_w = 0; rres_r = 0;
   end

endmodule
