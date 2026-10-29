// A behavioural SWD target with a DP and a MEM-AP, for sim/slot_sim.py.
//
// On the wire it reads the line, the host's drive or the
// pull-up; a 1 in idle is a start bit; a malformed request puts it in lockout, which only a line
// reset (50 or more 1s, then a 0) ends; the next start bit may follow a transfer at once.  It
// changes what it puts out after the falling edge, the later of the two edges a target may use.
//
// Behind the wire, the registers slot_sim.py's reference model has:
//   DP  read:  0 DPIDR, 4 CTRL/STAT (STICKYERR in bit 5), 8 RESEND (the last AP word), C RDBUFF
//   DP  write: 0 ABORT (clears STICKYERR), 4 CTRL/STAT, 8 SELECT
//   AP  0 CSW, 4 TAR, 8 a plain register, C DRW: memory at TAR, TAR then advancing by 4
// An AP read answers the word of the AP read before it, and RDBUFF that word again.
// Every request but a read of DPIDR counts; the WAIT_EVERY-th of them waits, the FAULT_AT-th
// faults, and so does the WRITE_FAULT_AT-th write.  After a FAULT the AP faults (STICKYERR) until
// a write of ABORT.
//
// With MULTIDROP it is an SWD-DP on a multi-drop bus, as the RP2040's: after a line reset it
// answers nothing until a write of TARGETSEL, which it does not acknowledge, carries TARGETID;
// any other request, or another ID, leaves it deselected until the next line reset.  TARGETSEL is
// taken bit by bit as it comes: request, five cycles (turnaround, ACK, turnaround), 32 bits and
// parity, so a host that clocks one cycle more or less selects nothing.
//
// With DORMANT it starts in the dormant state, as an RP2040 does, and answers nothing until the
// ADIv5.2 selection alert, then exactly four cycles low and the SWD activation code; it then needs
// a line reset.  After a line reset, the SWD-to-dormant sequence (0xE3BC) puts it back.  A cycle
// more or less between the alert and the code wakes nothing.
module swd_target #(parameter WAIT_EVERY = 0, parameter FAULT_AT = 0, parameter WRITE_FAULT_AT = 0,
                   parameter [31:0] DPIDR = 32'h0BC12477, parameter MEM_WORDS = 1024,
                   parameter MULTIDROP = 0, parameter [31:0] TARGETID = 32'h01002927,
                   parameter DORMANT = 0)
                 (input swclk, input line,
                  output reg drive = 0, output reg out_bit = 1);
    integer requests = 0;
    integer writes = 0;
    integer lockouts = 0;
    reg waiting = 0;
    reg faulting = 0;
    reg [7:0] header = 0;
    integer collected = 0;            // header bits collected, 0 when idle
    integer phase = -1;               // bit of the transfer after the header
    integer ones = 0;                 // consecutive 1s, for the line reset
    integer ones_before = 0;
    reg locked = 0;
    reg rnw = 0, apndp = 0;
    reg [1:0] a = 0;
    reg dpidr = 0;                    // a DP read of DPIDR: never WAIT or FAULT, not counted
    reg [31:0] value = 0;             // what a read answers
    reg [31:0] word = 0;              // what a write carries
    reg selected = !MULTIDROP;        // multi-drop: selected by TARGETSEL
    reg awaiting = 0;                 // multi-drop: a line reset was the last thing
    reg ts = 0;                       // the transfer is a TARGETSEL
    // Dormant: the last 128 bits, the first sent in [0]; the bits after the alert; the bits after
    // a line reset
    localparam [127:0] ALERT = 128'h19BC0EA2_E3DDAFE9_86852D95_6209F392;
    reg dormant = DORMANT;
    reg [127:0] hist = 0;
    integer act_n = -1;
    reg [11:0] act_bits = 0;
    integer pr_n = 0;
    reg [15:0] pr_bits = 0;

    // Registers
    reg sticky = 0;
    reg [31:0] ctrlstat = 0, select = 0, csw = 32'h23000052, tar = 0, ap8 = 0, ap_buf = 0;
    reg [31:0] mem [0:MEM_WORDS-1];
    integer i;
    initial for (i = 0; i < MEM_WORDS; i = i + 1) mem[i] = 32'hA5A50000 ^ (i * 32'h01010101);

    function [31:0] ap_read(input [1:0] r);
        begin
            case (r)
                0: ap_read = csw;
                1: ap_read = tar;
                2: ap_read = ap8;
                3: begin ap_read = mem[(tar >> 2) % MEM_WORDS]; tar = tar + 4; end
            endcase
        end
    endfunction

    always @(posedge swclk) begin
        ones_before = ones;
        ones = line ? ones + 1 : 0;
        hist = {line, hist[127:1]};
        if (dormant) begin
            if (act_n >= 0) begin
                act_bits[act_n] = line;
                act_n = act_n + 1;
                if (act_n == 12) begin
                    act_n = -1;
                    if (act_bits == 12'h1A0) begin
                        dormant = 0; locked = 1;          // awake, waiting for a line reset
                        $display("WOKEN %0t", $time);
                    end else
                        $display("NOT-WOKEN %0t %h", $time, act_bits);
                end
            end else if (hist == ALERT)
                act_n = 0;
        end else begin
            // SWD-to-dormant: 16 bits from the 0 that ends a line reset
            if (!line && ones_before >= 50) begin
                pr_n = 1; pr_bits = 0;
            end else if (pr_n > 0) begin
                pr_bits[pr_n] = line;
                pr_n = pr_n + 1;
                if (pr_n == 16) begin
                    pr_n = 0;
                    if (pr_bits == 16'hE3BC) begin
                        dormant = 1; locked = 0; phase = -1; collected = 0; ts = 0; awaiting = 0;
                        selected = !MULTIDROP;
                        $display("DORMANT %0t", $time);
                    end
                end
            end
            if (dormant) ;                                   // just sent there
            else if (locked) begin
                if (!line && ones_before >= 50) begin
                    locked = 0;
                    if (MULTIDROP) begin selected = 0; awaiting = 1; end
                    $display("LINE-RESET %0t", $time);
                end
            end else if (phase >= 0) begin
                if (!rnw && phase >= 5 && phase <= 36)
                    word[phase - 5] = line;
                if (ts && phase == 37) begin
                    selected = (word == TARGETID) && (line == ^word);
                    $display("TARGETSEL %0t %h %s", $time, word, selected ? "selected" : "not selected");
                end else if (!rnw && phase == 37 && !waiting && !faulting) begin
                    $display("WD %0t %s%0d %h", $time, apndp ? "AP" : "DP", a * 4, word);
                    if (!apndp) case (a)
                        0: if (word != 0) sticky = 0;
                        1: ctrlstat = word;
                        2: select = word;
                        default: ;
                    endcase else case (a)
                        0: csw = word;
                        1: tar = word;
                        2: ap8 = word;
                        3: begin mem[(tar >> 2) % MEM_WORDS] = word; tar = tar + 4; end
                    endcase
                end
                phase = phase + 1;
                if ((waiting || faulting) && phase == 5) begin
                    phase = -1;              // no data phase after WAIT or FAULT
                end else if (phase == 38) begin
                    // A read's turnaround or a write's parity bit was the last; ADIv5 lets the
                    // next start bit follow at once.
                    phase = -1;
                    ts = 0;
                end
            end else if (collected == 0) begin
                if (line) begin
                    header = 8'b1;
                    collected = 1;
                end
            end else begin
                header[collected] = line;
                collected = collected + 1;
                if (collected == 8) begin
                    collected = 0;
                    if (!selected) begin
                        // Multi-drop and not selected: a TARGETSEL right after a line reset is taken,
                        // anything else is not answered, and nothing more is until a line reset
                        if (awaiting && header == 8'h99) begin
                            ts = 1; phase = 0; rnw = 0; apndp = 0; a = 3; waiting = 0; faulting = 0;
                        end else begin
                            locked = 1;
                            $display("DESELECTED %0t %h", $time, header);
                        end
                        awaiting = 0;
                    end else if (!header[6] && header[7] && (header[5] == ^header[4:1])) begin
                        apndp = header[1];
                        rnw = header[2];
                        a = header[4:3];
                        dpidr = rnw && !apndp && a == 0;
                        phase = 0;
                        faulting = 0;
                        waiting = 0;
                        if (!dpidr) begin
                            requests = requests + 1;
                            if (!rnw) writes = writes + 1;
                            faulting = ((FAULT_AT != 0) && (requests == FAULT_AT)) ||
                                       ((WRITE_FAULT_AT != 0) && !rnw && (writes == WRITE_FAULT_AT)) ||
                                       (sticky && apndp);
                            waiting = !faulting && (WAIT_EVERY != 0) && (requests % WAIT_EVERY == 0);
                            if (faulting) sticky = 1;
                        end
                        if (rnw && !waiting && !faulting) begin
                            if (dpidr) value = DPIDR;
                            else if (!apndp) case (a)
                                1: value = ctrlstat | (sticky ? 32'h20 : 0);
                                2: value = ap_buf;
                                3: value = ap_buf;
                                default: value = 0;
                            endcase else begin
                                value = ap_buf;
                                ap_buf = ap_read(a);
                            end
                        end
                        $display("T %0t %s %s%0d %s", $time, rnw ? "R" : "W", apndp ? "AP" : "DP", a * 4,
                                 faulting ? "FAULT" : waiting ? "WAIT" : "OK");
                    end else begin
                        locked = 1;
                        lockouts = lockouts + 1;
                        $display("LOCKOUT %0t %h", $time, header);
                    end
                end
            end
        end
    end
    // The target drives after the falling edge, the host samples on the rising one.
    always @(negedge swclk) begin
        drive = 0;
        if (!locked && phase >= 0 && !ts) begin
            if (phase + 1 >= 2 && phase + 1 <= 4) begin
                drive = 1;
                // ACK OK is 1 0 0, WAIT 0 1 0 and FAULT 0 0 1, LSB first
                out_bit = faulting ? (phase + 1 == 4) : waiting ? (phase + 1 == 3) : (phase + 1 == 2);
            end else if (rnw && !waiting && !faulting && phase + 1 >= 5 && phase + 1 <= 36) begin
                drive = 1;
                out_bit = value[phase + 1 - 5];
            end else if (rnw && !waiting && !faulting && phase + 1 == 37) begin
                drive = 1;
                out_bit = ^value;
            end
        end
    end
endmodule
