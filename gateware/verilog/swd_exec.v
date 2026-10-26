`default_nettype none
//
// swd_exec
// ========
//
// Runs one CMSIS-DAP command of an SWD slot, read from the slot's request buffer, and writes its
// response into the response buffer: DAP_Transfer, DAP_TransferBlock, DAP_WriteABORT,
// DAP_SWJ_Sequence and DAP_SWD_Sequence, the commands that put bits on the wire.  The CPU parses
// everything else, and hands these over with where the command starts and where its response
// goes; it gets back how many bytes each took.
//
// The order of transfers follows ARM's CMSIS-DAP reference (DAP.c, DAP_SWD_Transfer and
// DAP_SWD_TransferBlock): an AP read is posted and its word comes with the next AP read or a read
// of RDBUFF; a read with value match repeats until the masked word matches or the match retries
// run out; a write is checked with a read of RDBUFF at the end; the first transfer that fails ends
// the command, and the rest of its requests are skipped.  swd_phy retries a transfer after WAIT.
//
// The executor offers the next transfer while one runs, so they follow each other on the wire.
// Results come back in order, each tagged with whether it counts as a request done and how many of
// its bytes are stored in the response.
//
// Before a transfer, after a clock change, the engine is calibrated; if calibration finds the
// target answering too late for the clock, the clock is lengthened until it does or reaches 16
// ticks a cycle.  With no target answering, calibration waits for the next read of DPIDR.

module swd_exec #(parameter AW = 12) (
                input wire             clk,
                input wire             rst,

        // The command: fields stable from before `cmd_req` toggles until `cmd_done` does ==============
                input wire             cmd_req,
                input wire [AW-1:0]    cmd_rq_base,                        // Its first byte, the command
                input wire [AW-1:0]    cmd_rs_base,                         // Where its response goes
                output reg        cmd_done,
                output reg [AW-1:0] cmd_rq_len,                         // Bytes of request it took
                output reg [AW-1:0] cmd_rs_len,                         // Bytes of response written
                input wire             abort,                             // DAP_TransferAbort, any clock

        // Configuration ==================================================================================
                input wire [15:0]      cfg_retry,                                      // Retries after WAIT
                input wire [15:0]      cfg_match_retry,
                input wire [6:0]       cfg_sample,            // [6] set: a fixed sampling delay, no calibration
                input wire             clk_set,           // Toggles with a new clock: calibrate before the next
                input wire [9:0]       clk_hi,                          // The clock asked for, in ticks
                input wire [9:0]       clk_lo,
                output reg [9:0]  phy_hi,                     // The clock in use, after any fallback
                output reg [9:0]  phy_lo,

        // Buffers ========================================================================================
                output reg [AW-1:0] rq_addr,          // Request byte read: rq_data two cycles after
                input wire [7:0]       rq_data,
                output reg        rs_we,                                // Response byte written
                output reg [AW-1:0] rs_addr,
                output reg [7:0]  rs_data,

        // swd_phy ========================================================================================
                output reg        op_valid,
                input wire             op_ready,
                output reg [1:0]  op_kind,
                output reg        op_rnw,
                output reg        op_apndp,
                output reg [1:0]  op_addr,
                output reg [31:0] op_data,
                output reg [4:0]  op_bits,
                output reg [15:0] op_retries,
                output reg [3:0]  op_tag,
                output reg        resume,
                output wire            res_hold,      // A result's bytes still being stored: bits read wait
                input wire             res_valid,
                input wire [2:0]       res_ack,
                input wire             res_perr,
                input wire [31:0]      res_data,
                input wire [3:0]       res_tag,
                input wire             cal_ok,
                input wire             cal_seen
             );

   localparam K_XFER = 2'd0, K_OUT = 2'd1, K_IN = 2'd2, K_CAL = 2'd3;

   // Request bits of DAP_Transfer
   localparam R_APNDP = 0, R_RNW = 1, R_MATCH_VALUE = 4, R_MATCH_MASK = 5;
   // DP RDBUFF read, as a request byte: RnW, A[3:2] = 3
   localparam [7:0] RDBUFF = 8'b0000_1110;


   // Synchronisers ======================================================================================
   reg [2:0]  req_s, clk_s;
   reg [1:0]  abort_s;
   reg        rst_q = 1;
   always @(posedge clk) begin
      req_s   <= {req_s[1:0], cmd_req};
      clk_s   <= {clk_s[1:0], clk_set};
      abort_s <= {abort_s[0], abort};
      rst_q   <= rst;
   end
   wire       req_new = req_s[2] != req_s[1];
   wire       clk_new = clk_s[2] != clk_s[1];
   wire       aborted = abort_s[1];

   // State ==============================================================================================
   localparam E_IDLE = 6'd0, E_FETCH = 6'd1, E_CMD = 6'd2, E_ISSUE = 6'd3, E_DRAIN = 6'd4,
              E_CAL = 6'd5, E_CAL_WAIT = 6'd6, E_HDR = 6'd7, E_DONE = 6'd8,
              T_COUNT = 6'd10, T_ITEM = 6'd11, T_RV = 6'd12, T_WRITE = 6'd13, T_WRITE_DO = 6'd14,
              T_READ = 6'd15, T_MATCH = 6'd16, T_MATCH_POST = 6'd17, T_MATCH_READ = 6'd18,
              T_MATCH_CHECK = 6'd19, T_END = 6'd20, T_SKIP = 6'd21, T_MASK = 6'd22,
              B_COUNT = 6'd24, B_RV = 6'd25, B_READ = 6'd26, B_WRITE = 6'd27, B_WRITE_DO = 6'd28,
              A_DATA = 6'd32, A_DO = 6'd33,
              J_COUNT = 6'd36, J_CHUNK = 6'd37, J_ISSUE = 6'd38,
              S_COUNT = 6'd40, S_INFO = 6'd41, S_GOT_INFO = 6'd42, S_ISSUE = 6'd43, S_OUT = 6'd44;
   localparam [3:0] TAG_CAL = 4'b0111;                      // A calibration's result: not a transfer's
   reg [5:0]  st, ret, iret;
   reg [AW-1:0] rq_base, rs_base;
   reg [AW-1:0] rp;                                             // Next request byte to read
   reg [AW-1:0] wp;                                              // Next response byte for data
   reg [1:0]  fwait;
   reg [2:0]  fleft;                                          // Bytes still to fetch into fword
   reg [31:0] fword;                    // Bytes fetched, shifted in at the top: LSB first when four
   reg [7:0]  cmd;
   reg [7:0]  rv;                                                   // The request byte in hand
   reg [15:0] n;                                                     // Requests or bits left
   reg [15:0] k;
   reg        post, check_write;
   reg [31:0] mvalue, mask;
   reg [15:0] mretry;
   reg [6:0]  bits_left;                                     // DAP_SWD_Sequence: of this sequence
   reg        din;
   reg [7:0]  seqs;
   reg [1:0]  cal_needed;                     // 1: before any transfer, 2: before a read of DPIDR
   reg        cal_tried;                         // E_ISSUE calibrated for the transfer it offers
   reg [2:0]  hdr_i;

   // Results: each counts a request done, stores its bytes, or ends the command.  A transfer whose
   // failure is of no interest (DAP_WriteABORT's) neither counts nor fails.
   reg [15:0] count;                                                        // Requests done
   reg [7:0]  value;                                                  // The response's status
   reg        failed;
   reg        ignore_fail;
   reg [31:0] last_data;
   reg        matched;                          // last_data, masked, is the value a match wants
   reg [7:0]  outst;                                      // Operations taken, results not yet in
   reg [31:0] wbuf;
   reg [2:0]  wleft;                                              // Bytes still to write of wbuf
   // A result given now is taken the cycle after next, when at most the last byte is left
   assign res_hold = wleft > 1;
   wire       res_xfer = res_valid && (res_tag != TAG_CAL);
   wire       res_good = (res_ack == 3'b001) && !res_perr;
   wire       fail_now = res_xfer && !res_good && !ignore_fail;

   // The operation E_ISSUE offers
   reg [1:0]  p_kind;
   reg [7:0]  p_rv;
   reg [31:0] p_data;
   reg [4:0]  p_bits;
   reg [3:0]  p_tag;

   wire       drained    = (outst == 0) && !op_valid && (wleft == 0) && !res_valid;
   wire       p_dpidr    = (p_kind == K_XFER) && p_rv[R_RNW] && !p_rv[R_APNDP] && (p_rv[3:2] == 2'd0);
   wire       cal_wanted = !cfg_sample[6] && (p_kind == K_XFER) && !cal_tried &&
                           ((cal_needed == 2'd1) || (cal_needed == 2'd2 && p_dpidr));

   // Runs of bits: a sequence's next run is 32 bits or what is left, in this many bytes
   wire [5:0] j_run   = (n >= 32) ? 6'd32 : {1'b0, n[4:0]};
   wire [2:0] j_bytes = (j_run + 6'd7) >> 3;
   wire [5:0] s_run   = (bits_left >= 32) ? 6'd32 : bits_left[5:0];
   wire [2:0] s_bytes = (s_run + 6'd7) >> 3;
   // A run of bits driven, as its bytes are fetched: its length, held, and its bytes, fetched into
   // fword's top and moved down to bit 0
   reg  [5:0] run;
   reg  [2:0] run_bytes;
   wire [31:0] run_word = fword >> {3'd4 - run_bytes, 3'b000};

   // Fetch `bytes` request bytes into fword, then go to `to`
   task fetch(input [5:0] to, input [2:0] bytes);
      begin
         rq_addr <= rp;
         rp      <= rp + 1;
         fwait   <= 2;
         fleft   <= bytes;
         ret     <= to;
         st      <= E_FETCH;
      end
   endtask

   // Offer an operation, then go to `to`; nothing is offered once the command failed
   task issue(input [1:0] kind, input [7:0] r, input [31:0] d, input [4:0] b, input [3:0] t,
              input [5:0] to);
      begin
         p_kind <= kind; p_rv <= r; p_data <= d; p_bits <= b; p_tag <= t;
         iret   <= to;
         st     <= E_ISSUE;
      end
   endtask

   // Drain, then go to `to`
   task drain(input [5:0] to);
      begin
         ret <= to;
         st  <= E_DRAIN;
      end
   endtask

   // Write a header byte of the response
   task put(input [AW-1:0] at, input [7:0] b);
      begin
         rs_we <= 1; rs_addr <= at; rs_data <= b;
      end
   endtask

   always @(posedge clk) begin
      resume <= 0;
      rs_we  <= 0;

      // Results ----------------------------------------------------------------------------------------
      if (op_ready && !res_valid) outst <= outst + 1;
      else if (res_valid && !op_ready) outst <= outst - 1;
      if (op_ready) op_valid <= 0;

      if (wleft != 0) begin
         rs_we   <= 1;
         rs_addr <= wp;
         rs_data <= wbuf[7:0];
         wbuf    <= wbuf >> 8;
         wp      <= wp + 1;
         wleft   <= wleft - 1;
      end

      if (res_xfer) begin
         last_data <= res_data;
         matched   <= ((res_data & mask) == mvalue);
         if (res_good || ignore_fail) begin
            if (res_good && !ignore_fail) value <= 8'h01;
            if (res_tag[3]) count <= count + 1;
            if (res_tag[2:0] != 0) begin
               wbuf  <= res_data;
               wleft <= res_tag[2:0];
            end
         end else begin
            failed   <= 1;
            value    <= res_perr ? 8'h08 : {5'd0, res_ack};
            op_valid <= 0;                       // The operation offered and not taken is dropped
         end
      end

      if (clk_new) begin
         phy_hi <= clk_hi; phy_lo <= clk_lo; cal_needed <= 2'd1;
      end

      // The command ------------------------------------------------------------------------------------
      case (st)
        E_IDLE:
          if (req_new) begin
             rq_base <= cmd_rq_base; rs_base <= cmd_rs_base;
             count   <= 0; value <= 0; failed <= 0; ignore_fail <= 0;
             post    <= 0; check_write <= 0;
             resume  <= 1;
             // The command byte: fetch() would read rp before it is set
             rq_addr <= cmd_rq_base; rp <= cmd_rq_base + 1;
             fwait <= 2; fleft <= 1; ret <= E_CMD; st <= E_FETCH;
          end

        E_FETCH:
          if (fwait != 0)
            fwait <= fwait - 1;
          else begin
             fword <= {rq_data, fword[31:8]};
             if (fleft > 1) begin
                rq_addr <= rp; rp <= rp + 1; fwait <= 2; fleft <= fleft - 1;
             end else
                st <= ret;
          end

        E_CMD: begin
           cmd <= fword[31:24];
           case (fword[31:24])
             8'h05: begin wp <= rs_base + 3; fetch(T_COUNT, 2); end    // index, count
             8'h06: begin wp <= rs_base + 4; fetch(B_COUNT, 3); end    // index, count
             8'h08: begin wp <= rs_base + 2; fetch(A_DATA, 5); end     // index, data
             8'h12: begin wp <= rs_base + 2; fetch(J_COUNT, 1); end
             8'h1D: begin wp <= rs_base + 2; fetch(S_COUNT, 1); end
             default: begin wp <= rs_base + 2; value <= 8'hFF; st <= E_HDR; end
           endcase
        end

        // Offer p_*, calibrating first when that is due ================================================
        E_ISSUE:
          if (failed || fail_now)
            st <= iret;
          else if (cal_wanted) begin
             if (drained) begin
                op_kind <= K_CAL; op_tag <= TAG_CAL; op_valid <= 1;
                st <= E_CAL;
             end
          end else if (!op_valid || op_ready) begin
             cal_tried <= 0;
             op_valid <= 1;
             op_kind  <= p_kind;
             op_rnw   <= p_rv[R_RNW];
             op_apndp <= p_rv[R_APNDP];
             op_addr  <= p_rv[3:2];
             op_data  <= p_data;
             op_bits  <= p_bits;
             op_tag   <= p_tag;
             op_retries <= (cmd == 8'h08) ? 16'd0 : cfg_retry;          // DAP_WriteABORT: one try
             st       <= iret;
          end

        E_CAL:                                       // Wait until the calibration is taken and in
          if (!op_valid && outst == 0 && !res_valid) st <= E_CAL_WAIT;

        E_CAL_WAIT: begin
           if (!cal_ok && cal_seen && (phy_hi + phy_lo < 16)) begin
              // The target answers, too late for this clock: a slower one, and again
              if (phy_lo <= phy_hi) phy_lo <= phy_lo + 1; else phy_hi <= phy_hi + 1;
           end else begin
              // Found, or no target answering yet: then before the next read of DPIDR, and this
              // transfer goes out as it is
              cal_needed <= cal_ok ? 2'd0 : 2'd2;
              cal_tried  <= 1;
           end
           st <= E_ISSUE;
        end

        E_DRAIN:
          if (drained) st <= ret;

        // DAP_Transfer ===================================================================================
        T_COUNT: begin n <= {8'd0, fword[31:24]}; st <= T_ITEM; end

        T_ITEM:
          if (n == 0) drain(T_END);
          else fetch(T_RV, 1);

        T_RV: begin
           rv <= fword[31:24];
           n  <= n - 1;
           if (failed || fail_now || aborted) begin
              // Skipped: only its data, if any, is read past
              if (!fword[24 + R_RNW] || fword[24 + R_MATCH_VALUE]) fetch(T_SKIP, 4);
              else st <= T_ITEM;
           end else if (fword[24 + R_RNW]) begin
              check_write <= 0;
              if (post && fword[24 + R_APNDP] && !fword[24 + R_MATCH_VALUE])
                // The previous AP read's word, and the next posted: one AP read
                issue(K_XFER, fword[31:24], 0, 0, 4'b1100, T_ITEM);
              else if (post) begin
                 // The previous AP read's word first, from RDBUFF
                 post <= 0;
                 issue(K_XFER, RDBUFF, 0, 0, 4'b0100, T_READ);
              end else
                 st <= T_READ;
           end else begin
              if (post) begin
                 post <= 0;
                 issue(K_XFER, RDBUFF, 0, 0, 4'b0100, T_WRITE);
              end else
                 st <= T_WRITE;
           end
        end

        T_SKIP: st <= T_ITEM;

        T_WRITE: fetch(T_WRITE_DO, 4);

        T_WRITE_DO:
          if (rv[R_MATCH_MASK])
            drain(T_MASK);
          else begin
             check_write <= 1;
             issue(K_XFER, rv, fword, 0, 4'b1000, T_ITEM);
          end

        T_MASK: begin
           if (!failed) begin mask <= fword; count <= count + 1; value <= 8'h01; end
           st <= T_ITEM;
        end

        T_READ:
          if (rv[R_MATCH_VALUE])
            fetch(T_MATCH, 4);
          else if (rv[R_APNDP]) begin
             post <= 1;                                  // Posted: its word comes with the next
             issue(K_XFER, rv, 0, 0, 4'b1000, T_ITEM);
          end else
             issue(K_XFER, rv, 0, 0, 4'b1100, T_ITEM);

        T_MATCH: begin
           mvalue <= fword;
           mretry <= cfg_match_retry;
           if (rv[R_APNDP]) issue(K_XFER, rv, 0, 0, 4'b0000, T_MATCH_POST);
           else st <= T_MATCH_POST;
        end

        T_MATCH_POST: issue(K_XFER, rv, 0, 0, 4'b0000, T_MATCH_READ);

        T_MATCH_READ: drain(T_MATCH_CHECK);

        T_MATCH_CHECK:
          if (failed)
            st <= T_ITEM;
          else if (matched) begin
             count <= count + 1;
             st    <= T_ITEM;
          end else if (mretry != 0 && !aborted) begin
             mretry <= mretry - 1;
             st     <= T_MATCH_POST;
          end else begin
             value  <= value | 8'h10;                   // Mismatch: the command ends here
             failed <= 1;
             st     <= T_ITEM;
          end

        T_END:
          // Everything is in: the last AP read's word, or a check of the last write
          if (!failed && value == 8'h01 && post) begin
             post <= 0;
             issue(K_XFER, RDBUFF, 0, 0, 4'b0100, T_END);
          end else if (!failed && value == 8'h01 && check_write) begin
             check_write <= 0;
             issue(K_XFER, RDBUFF, 0, 0, 4'b0000, T_END);
          end else
             drain(E_HDR);

        // DAP_TransferBlock ==============================================================================
        B_COUNT: begin
           n <= fword[31:16];
           if (fword[31:16] == 0) begin
              rp <= rp + 1;                                       // The request byte, unread
              drain(E_HDR);
           end else
              fetch(B_RV, 1);
        end

        B_RV: begin
           rv <= fword[31:24];
           k  <= 0;
           if (fword[24 + R_RNW]) begin
              if (fword[24 + R_APNDP]) issue(K_XFER, fword[31:24], 0, 0, 4'b0000, B_READ);
              else st <= B_READ;
           end else
              st <= B_WRITE;
        end

        B_READ:
          if (k == n || failed || fail_now)
            drain(E_HDR);
          else begin
             k <= k + 1;
             issue(K_XFER, (k == n - 1 && rv[R_APNDP]) ? RDBUFF : rv, 0, 0, 4'b1100, B_READ);
          end

        B_WRITE:
          if (k == n)
            issue(K_XFER, RDBUFF, 0, 0, 4'b0000, A_DO);                 // Check the last write
          else begin
             k <= k + 1;
             fetch(B_WRITE_DO, 4);
          end

        B_WRITE_DO: issue(K_XFER, rv, fword, 0, 4'b1000, B_WRITE);

        // DAP_WriteABORT =================================================================================
        A_DATA: begin
           ignore_fail <= 1;
           issue(K_XFER, 8'b0000_0000, fword, 0, 4'b0000, A_DO);   // DP write of ABORT
        end

        A_DO: drain(E_HDR);

        // DAP_SWJ_Sequence: bits driven, in runs of 32 ===================================================
        J_COUNT: begin
           n  <= (fword[31:24] == 0) ? 16'd256 : {8'd0, fword[31:24]};
           st <= J_CHUNK;
        end

        J_CHUNK:
          if (n == 0) drain(E_HDR);
          else begin
             run <= j_run; run_bytes <= j_bytes;
             fetch(J_ISSUE, j_bytes);
          end

        J_ISSUE: begin
           n <= n - run;
           issue(K_OUT, 0, run_word, run - 1, 4'b0000, J_CHUNK);
        end

        // DAP_SWD_Sequence: runs of bits driven or read, up to 64 a sequence =============================
        S_COUNT: begin seqs <= fword[31:24]; st <= S_INFO; end

        S_INFO:
          if (seqs == 0) drain(E_HDR);
          else begin
             seqs <= seqs - 1;
             fetch(S_GOT_INFO, 1);
          end

        S_GOT_INFO: begin
           din       <= fword[31];
           bits_left <= (fword[29:24] == 0) ? 7'd64 : {1'b0, fword[29:24]};
           st        <= S_ISSUE;
        end

        S_ISSUE:
          if (bits_left == 0)
            st <= S_INFO;
          else if (din) begin
             bits_left <= bits_left - s_run;
             issue(K_IN, 0, 0, s_run - 1, {1'b0, s_bytes}, S_ISSUE);
          end else
          begin
             run <= s_run; run_bytes <= s_bytes;
             fetch(S_OUT, s_bytes);
          end

        S_OUT: begin
           bits_left <= bits_left - run;
           issue(K_OUT, 0, run_word, run - 1, 4'b0000, S_ISSUE);
        end

        // The response's header ==========================================================================
        E_HDR: begin
           // The command, then its count and status
           case (hdr_i)
             0: put(rs_base, cmd);
             1: put(rs_base + 1, (cmd == 8'h05 || cmd == 8'h06) ? count[7:0] :
                                 (value == 8'hFF) ? 8'hFF : 8'h00);
             2: if (cmd == 8'h05) put(rs_base + 2, value);
                else if (cmd == 8'h06) put(rs_base + 2, count[15:8]);
             3: if (cmd == 8'h06) put(rs_base + 3, value);
             default: ;
           endcase
           if (hdr_i == 3) begin
              hdr_i <= 0;
              st    <= E_DONE;
           end else
              hdr_i <= hdr_i + 1;
        end

        E_DONE: begin
           cmd_rq_len <= rp - rq_base;
           cmd_rs_len <= wp - rs_base;
           cmd_done   <= !cmd_done;
           resume     <= 1;
           st         <= E_IDLE;
        end

        default: st <= E_IDLE;
      endcase

      if (rst_q) begin
         st <= E_IDLE; op_valid <= 0; hdr_i <= 0; outst <= 0; wleft <= 0;
      end
   end

   initial begin
      st = E_IDLE; op_valid = 0; cmd_done = 0; hdr_i = 0; outst = 0; wleft = 0; rs_we = 0;
      resume = 0; cal_needed = 2'd1; mask = 32'hFFFFFFFF; phy_hi = 2; phy_lo = 2;
      din = 0; failed = 0; ignore_fail = 0; cal_tried = 0;
   end

endmodule
