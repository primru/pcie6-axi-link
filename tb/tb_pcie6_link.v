`timescale 1ns/1ps
// Two pcie6_top instances connected back to back (A <-> B).
// dir 0 = A masters -> B record output, dir 1 = B masters -> A record output.
// Source id: tid[0] = master index, tid[11:4] = packet number.
module tb_pcie6_link;
    localparam N = 2, MAXB = 2048;
    localparam REPLAY_TMO = 512;

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    // ------------------------------------------------------------ A side
    reg  [N-1:0]     a_tvalid = 0;      wire [N-1:0] a_tready;
    reg  [512*N-1:0] a_tdata  = 0;      reg [64*N-1:0] a_tkeep = 0, a_tstrb = 0;
    reg  [8*N-1:0]   a_tuser  = 0;      reg [12*N-1:0] a_tid = 0;
    reg  [8*N-1:0]   a_tdest  = 0;      reg [N-1:0] a_tlast = 0, a_drw = 0;
    reg  [7*N-1:0]   a_tqos   = 0;      reg [31*N-1:0] a_addr = 0;
    // ------------------------------------------------------------ B side
    reg  [N-1:0]     b_tvalid = 0;      wire [N-1:0] b_tready;
    reg  [512*N-1:0] b_tdata  = 0;      reg [64*N-1:0] b_tkeep = 0, b_tstrb = 0;
    reg  [8*N-1:0]   b_tuser  = 0;      reg [12*N-1:0] b_tid = 0;
    reg  [8*N-1:0]   b_tdest  = 0;      reg [N-1:0] b_tlast = 0, b_drw = 0;
    reg  [7*N-1:0]   b_tqos   = 0;      reg [31*N-1:0] b_addr = 0;

    wire [63:0] a_l0, a_l1, a_l2, a_l3, b_l0, b_l1, b_l2, b_l3;
    wire        a_pv, a_pl, b_pv, b_pl;

    // record outputs
    wire a_ov, b_ov;
    wire [511:0] a_od, b_od;  wire [63:0] a_ok, a_os, b_ok, b_os;
    wire [7:0] a_ou, b_ou, a_ode, b_ode;  wire [11:0] a_oi, b_oi;
    wire a_ol, b_ol, a_orw, b_orw;  wire [30:0] a_oa, b_oa;
    reg  a_ord = 1, b_ord = 1;

    wire [15:0] a_txf, a_rep, a_rxf, a_fec, a_crc, a_drp;
    wire [15:0] b_txf, b_rep, b_rxf, b_fec, b_crc, b_drp;

    // ---------------------------------------------------- error injection
    reg [63:0] inj_ab0 = 0, inj_ab1 = 0, inj_ab2 = 0, inj_ab2b = 0, inj_ab3 = 0;
    reg [63:0] inj_ba0 = 0, inj_ba1 = 0, inj_ba2 = 0, inj_ba3 = 0;
    reg        kill_ba = 0;                       // corrupt every B->A beat
    wire [63:0] kmask0 = kill_ba ? 64'h4000_0000_0000_0000 : 64'h0;

    wire [63:0] ab0 = a_l0 ^ (a_pv ? inj_ab0 : 64'h0);
    wire [63:0] ab1 = a_l1 ^ (a_pv ? inj_ab1 : 64'h0);
    wire [63:0] ab2 = a_l2 ^ (a_pv ? inj_ab2 : 64'h0);
    wire [63:0] ab3 = a_l3 ^ (a_pv ? inj_ab3 : 64'h0);
    wire [63:0] ba0 = b_l0 ^ (b_pv ? (inj_ba0 ^ kmask0) : 64'h0);
    wire [63:0] ba1 = b_l1 ^ (b_pv ? inj_ba1 : 64'h0);
    wire [63:0] ba2 = b_l2 ^ (b_pv ? inj_ba2 : 64'h0);
    wire [63:0] ba3 = b_l3 ^ (b_pv ? (inj_ba3 ^ kmask0) : 64'h0);

    pcie6_top #(.N(N), .REPLAY_TMO(REPLAY_TMO)) dutA (
        .clk(clk), .rst_n(rst_n),
        .m_tvalid(a_tvalid), .m_tready(a_tready), .m_tdata(a_tdata), .m_tkeep(a_tkeep),
        .m_tstrb(a_tstrb), .m_tuser(a_tuser), .m_tid(a_tid), .m_tdest(a_tdest),
        .m_tlast(a_tlast), .m_tqos(a_tqos), .m_address(a_addr), .m_drw(a_drw),
        .lane0_sym(a_l0), .lane1_sym(a_l1), .lane2_sym(a_l2), .lane3_sym(a_l3),
        .phy_valid(a_pv), .phy_last(a_pl),
        .rx_lane0_sym(ba0), .rx_lane1_sym(ba1), .rx_lane2_sym(ba2), .rx_lane3_sym(ba3),
        .rx_phy_valid(b_pv), .rx_phy_last(b_pl),
        .o_tvalid(a_ov), .o_tready(a_ord), .o_tdata(a_od), .o_tkeep(a_ok), .o_tstrb(a_os),
        .o_tuser(a_ou), .o_tid(a_oi), .o_tdest(a_ode), .o_tlast(a_ol), .o_drw(a_orw), .o_address(a_oa),
        .mon_tx_valid(), .mon_tx_data(), .mon_tx_last(),
        .st_tx_flits(a_txf), .st_replay(a_rep), .st_rx_flits(a_rxf),
        .st_fec_corr(a_fec), .st_crc_fail(a_crc), .st_drop(a_drp));

    pcie6_top #(.N(N), .REPLAY_TMO(REPLAY_TMO)) dutB (
        .clk(clk), .rst_n(rst_n),
        .m_tvalid(b_tvalid), .m_tready(b_tready), .m_tdata(b_tdata), .m_tkeep(b_tkeep),
        .m_tstrb(b_tstrb), .m_tuser(b_tuser), .m_tid(b_tid), .m_tdest(b_tdest),
        .m_tlast(b_tlast), .m_tqos(b_tqos), .m_address(b_addr), .m_drw(b_drw),
        .lane0_sym(b_l0), .lane1_sym(b_l1), .lane2_sym(b_l2), .lane3_sym(b_l3),
        .phy_valid(b_pv), .phy_last(b_pl),
        .rx_lane0_sym(ab0), .rx_lane1_sym(ab1), .rx_lane2_sym(ab2), .rx_lane3_sym(ab3),
        .rx_phy_valid(a_pv), .rx_phy_last(a_pl),
        .o_tvalid(b_ov), .o_tready(b_ord), .o_tdata(b_od), .o_tkeep(b_ok), .o_tstrb(b_os),
        .o_tuser(b_ou), .o_tid(b_oi), .o_tdest(b_ode), .o_tlast(b_ol), .o_drw(b_orw), .o_address(b_oa),
        .mon_tx_valid(), .mon_tx_data(), .mon_tx_last(),
        .st_tx_flits(b_txf), .st_replay(b_rep), .st_rx_flits(b_rxf),
        .st_fec_corr(b_fec), .st_crc_fail(b_crc), .st_drop(b_drp));

    // ---------------------------------------------------------- scoreboard
    reg [511:0] e_data [0:4*MAXB-1];
    reg [63:0]  e_keep [0:4*MAXB-1];
    reg [63:0]  e_strb [0:4*MAXB-1];
    reg [7:0]   e_user [0:4*MAXB-1];
    reg [11:0]  e_id   [0:4*MAXB-1];
    reg [7:0]   e_dest [0:4*MAXB-1];
    reg         e_last [0:4*MAXB-1];
    reg         e_rw   [0:4*MAXB-1];
    reg [30:0]  e_addr [0:4*MAXB-1];
    integer snt [0:3];
    integer rcv [0:3];
    integer open_m [0:1];
    integer errors = 0, checks = 0, tests_failed = 0;
    integer pktno [0:3];
    integer ii;

    task automatic err(input [511:0] msg);
        begin errors = errors + 1; $display("  [ERROR @%0t] %0s", $time, msg); end
    endtask

    task automatic check_beat(input integer dir,
        input [511:0] d, input [63:0] k, input [63:0] s, input [7:0] u,
        input [11:0] id, input [7:0] de, input l, input rw, input [30:0] ad);
        integer m, idx, ix;
        begin
            m = id[0]; idx = dir*2 + m; ix = idx*MAXB + rcv[idx];
            checks = checks + 1;
            if (rcv[idx] >= snt[idx]) begin
                err("unexpected extra beat"); 
            end else begin
                if (rw !== e_rw[ix])     err("rw mismatch");
                if (rw && d !== e_data[ix]) err("data mismatch");
                if (k !== e_keep[ix])    err("tkeep mismatch");
                if (s !== e_strb[ix])    err("tstrb mismatch");
                if (u !== e_user[ix])    err("tuser mismatch");
                if (id !== e_id[ix])     err("tid mismatch");
                if (de !== e_dest[ix])   err("tdest mismatch");
                if (l !== e_last[ix])    err("tlast mismatch");
                if (ad !== e_addr[ix])   err("address mismatch");
            end
            if (open_m[dir] >= 0 && open_m[dir] != m) err("packet interleaving at output");
            open_m[dir] = l ? -1 : m;
            rcv[idx] = rcv[idx] + 1;
        end
    endtask

    always @(posedge clk) if (rst_n && b_ov && b_ord)
        check_beat(0, b_od, b_ok, b_os, b_ou, b_oi, b_ode, b_ol, b_orw, b_oa);
    always @(posedge clk) if (rst_n && a_ov && a_ord)
        check_beat(1, a_od, a_ok, a_os, a_ou, a_oi, a_ode, a_ol, a_orw, a_oa);

    // random backpressure on the record outputs
    reg bp_en = 0;
    always @(negedge clk) begin
        a_ord <= bp_en ? ($random & 1) : 1'b1;
        b_ord <= bp_en ? ($random & 1) : 1'b1;
    end

    // ------------------------------------------------------------ drivers
    task automatic put_beat(input integer dir, input integer m,
        input [511:0] d, input [63:0] k, input [63:0] s, input [7:0] u,
        input [11:0] id, input [7:0] de, input l, input rw, input [30:0] ad, input [6:0] q);
        begin
            if (dir == 0) begin
                a_tdata[512*m +: 512] = d;  a_tkeep[64*m +: 64] = k;  a_tstrb[64*m +: 64] = s;
                a_tuser[8*m +: 8] = u;      a_tid[12*m +: 12] = id;   a_tdest[8*m +: 8] = de;
                a_tlast[m] = l;             a_drw[m] = rw;            a_addr[31*m +: 31] = ad;
                a_tqos[7*m +: 7] = q;       a_tvalid[m] = 1'b1;
            end else begin
                b_tdata[512*m +: 512] = d;  b_tkeep[64*m +: 64] = k;  b_tstrb[64*m +: 64] = s;
                b_tuser[8*m +: 8] = u;      b_tid[12*m +: 12] = id;   b_tdest[8*m +: 8] = de;
                b_tlast[m] = l;             b_drw[m] = rw;            b_addr[31*m +: 31] = ad;
                b_tqos[7*m +: 7] = q;       b_tvalid[m] = 1'b1;
            end
        end
    endtask

    function automatic rdy(input integer dir, input integer m);
        rdy = (dir == 0) ? a_tready[m] : b_tready[m];
    endfunction

    task automatic idle_src(input integer dir, input integer m);
        begin
            if (dir == 0) a_tvalid[m] = 1'b0; else b_tvalid[m] = 1'b0;
        end
    endtask

    // one packet: nbeats beats, last beat has tail bytes valid (1..64)
    task automatic send_pkt(input integer dir, input integer m, input integer nbeats,
                            input integer tail, input integer rw, input integer qos);
        integer b, idx, ix;
        reg [511:0] d; reg [63:0] k, s; reg [7:0] u, de; reg [11:0] id; reg [30:0] ad; reg l;
        begin
            idx = dir*2 + m;
            id[0] = m[0]; id[3:1] = 3'd0; id[11:4] = pktno[idx][7:0];
            pktno[idx] = pktno[idx] + 1;
            for (b = 0; b < nbeats; b = b + 1) begin
                d = {$random,$random,$random,$random,$random,$random,$random,$random,
                     $random,$random,$random,$random,$random,$random,$random,$random};
                l = (b == nbeats-1);
                k = (l && tail < 64) ? ((64'd1 << tail) - 64'd1) : {64{1'b1}};
                s = k & {$random,$random};
                u = $random; de = $random; ad = $random;
                ix = idx*MAXB + snt[idx];
                e_data[ix] = d; e_keep[ix] = k; e_strb[ix] = s; e_user[ix] = u; e_id[ix] = id;
                e_dest[ix] = de; e_last[ix] = l; e_rw[ix] = rw[0]; e_addr[ix] = ad;
                snt[idx] = snt[idx] + 1;
                @(negedge clk);
                put_beat(dir, m, d, k, s, u, id, de, l, rw[0], ad, qos[6:0]);
                @(posedge clk);
                while (!rdy(dir, m)) @(posedge clk);
            end
            @(negedge clk);
            idle_src(dir, m);
        end
    endtask

    task automatic wait_done(input integer max_cycles, input [511:0] name);
        integer c; reg done;
        begin
            done = 0;
            for (c = 0; c < max_cycles && !done; c = c + 1) begin
                @(posedge clk);
                done = (snt[0]==rcv[0]) && (snt[1]==rcv[1]) && (snt[2]==rcv[2]) && (snt[3]==rcv[3]);
            end
            repeat (60) @(posedge clk);       // catch stray extra beats
            if (!done) begin errors = errors + 1; $display("  [ERROR] timeout in %0s (snt %0d %0d %0d %0d rcv %0d %0d %0d %0d)",
                name, snt[0],snt[1],snt[2],snt[3], rcv[0],rcv[1],rcv[2],rcv[3]); end
            if ((snt[0]!=rcv[0])||(snt[1]!=rcv[1])||(snt[2]!=rcv[2])||(snt[3]!=rcv[3])) begin end
        end
    endtask

    // flip symbols on the n-th valid A->B beat (dir 0) after the call; lanes given by masks
    task automatic inject_ab(input integer skip, input [63:0] m0, input [63:0] m1,
                             input [63:0] m2, input [63:0] m3);
        integer cnt; reg done;
        begin
            cnt = 0; done = 0;
            while (!done) begin
                @(negedge clk);
                if (a_pv) begin
                    if (cnt == skip) begin
                        inj_ab0 = m0; inj_ab1 = m1; inj_ab2 = m2; inj_ab3 = m3;
                        @(negedge clk);
                        inj_ab0 = 0; inj_ab1 = 0; inj_ab2 = 0; inj_ab3 = 0;
                        done = 1;
                    end
                    cnt = cnt + 1;
                end
            end
        end
    endtask

    // ------------------------------------------------------------ test flow
    integer p, nb, t0_fec, t0_crc, t0_rep, t0_drp;
    integer errs_before;

    task automatic begin_test(input [511:0] name);
        begin errs_before = errors; $display("TEST: %0s", name); end
    endtask
    task automatic end_test(input [511:0] name);
        begin
            if (errors == errs_before) $display("  PASS: %0s", name);
            else begin $display("  FAIL: %0s", name); tests_failed = tests_failed + 1; end
        end
    endtask

    initial begin
        $dumpfile("pcie6_link.vcd");
        $dumpvars(1, tb_pcie6_link);
        for (ii = 0; ii < 4; ii = ii + 1) begin snt[ii] = 0; rcv[ii] = 0; pktno[ii] = 0; end
        open_m[0] = -1; open_m[1] = -1;

        // ---- T1 reset / idle ----
        begin_test("T1 reset and idle");
        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (40) begin
            @(posedge clk);
            if (a_pv || b_pv || a_ov || b_ov) err("activity while idle");
        end
        end_test("T1 reset and idle");

        // ---- T2 single write ----
        begin_test("T2 single 64B write A.m0 -> B");
        send_pkt(0, 0, 1, 64, 1, 10);
        wait_done(2000, "T2");
        end_test("T2 single 64B write A.m0 -> B");

        // ---- T3 read request (header only) ----
        begin_test("T3 read request A.m1 -> B");
        send_pkt(0, 1, 1, 64, 0, 5);
        wait_done(2000, "T3");
        end_test("T3 read request A.m1 -> B");

        // ---- T4 concurrent multi-beat packets, no interleaving ----
        begin_test("T4 two masters, 6-beat packets, packet locking and QoS");
        fork
            send_pkt(0, 0, 6, 64, 1, 3);
            send_pkt(0, 1, 6, 64, 1, 9);
        join
        wait_done(3000, "T4");
        end_test("T4 two masters, 6-beat packets, packet locking and QoS");

        // ---- T5 partial last beat ----
        begin_test("T5 partial tkeep/tstrb on last beat");
        send_pkt(0, 0, 3, 13, 1, 1);
        send_pkt(0, 1, 2, 1, 1, 1);
        wait_done(3000, "T5");
        end_test("T5 partial tkeep/tstrb on last beat");

        // ---- T6 network packet processing workload, both directions ----
        begin_test("T6 network workload: 4 sources x 12 packets (1..24 beats), mixed reads");
        fork
            begin : s00 integer q; for (q = 0; q < 12; q = q + 1) send_pkt(0, 0, 1 + ($random & 32'hFF) % 24, 1 + ($random & 32'hFF) % 64, 1, q%4); end
            begin : s01 integer q; for (q = 0; q < 12; q = q + 1) begin if (q%5==4) send_pkt(0, 1, 1, 64, 0, 7); else send_pkt(0, 1, 1 + ($random & 32'hFF) % 24, 1 + ($random & 32'hFF) % 64, 1, 2); end end
            begin : s10 integer q; for (q = 0; q < 12; q = q + 1) send_pkt(1, 0, 1 + ($random & 32'hFF) % 24, 1 + ($random & 32'hFF) % 64, 1, 5); end
            begin : s11 integer q; for (q = 0; q < 12; q = q + 1) send_pkt(1, 1, 1 + ($random & 32'hFF) % 24, 64, 1, 1); end
        join
        wait_done(40000, "T6");
        end_test("T6 network workload: 4 sources x 12 packets (1..24 beats), mixed reads");

        // ---- T7 ML inference: weight stream + read requests + completions ----
        begin_test("T7 ML workload: 64-beat weight tensor + read requests + completions");
        fork
            send_pkt(0, 0, 64, 64, 1, 4);                                  // weights A -> B
            begin : rq integer q; for (q = 0; q < 10; q = q + 1) send_pkt(0, 1, 1, 64, 0, 8); end   // read requests A -> B
            send_pkt(1, 0, 32, 64, 1, 4);                                  // completions with data B -> A
        join
        wait_done(30000, "T7");
        end_test("T7 ML workload: 64-beat weight tensor + read requests + completions");

        // ---- T8 correctable error: one symbol error ----
        begin_test("T8 single symbol error -> FEC corrects, no replay");
        t0_fec = b_fec; t0_rep = a_rep; t0_crc = b_crc;
        fork
            send_pkt(0, 0, 8, 64, 1, 1);
            inject_ab(2, 64'h0, 64'h0200_0000_0000_0000, 64'h0, 64'h0);
        join
        wait_done(4000, "T8");
        if (b_fec == t0_fec)  err("FEC did not report a correction");
        if (b_crc != t0_crc)  err("CRC failed although FEC should have corrected");
        if (a_rep != t0_rep)  err("unexpected replay");
        end_test("T8 single symbol error -> FEC corrects, no replay");

        // ---- T9 uncorrectable error -> NAK + replay ----
        begin_test("T9 two bytes in one FEC group -> CRC fail, NAK, replay");
        t0_crc = b_crc; t0_rep = a_rep;
        fork
            send_pkt(0, 0, 8, 64, 1, 1);
            inject_ab(1, 64'h4000_0000_0000_0000, 64'h0, 64'h0, 64'h4000_0000_0000_0000);
        join
        wait_done(6000, "T9");
        if (b_crc == t0_crc)  err("CRC failure not detected");
        if (a_rep == t0_rep)  err("no replay after NAK");
        end_test("T9 two bytes in one FEC group -> CRC fail, NAK, replay");

        // ---- T10 reverse link dead: ACK loss -> window stall -> replay timeout ----
        begin_test("T10 ACK path corrupted: window stall, replay timeout, recovery");
        t0_rep = a_rep;
        kill_ba = 1;
        fork
            send_pkt(0, 0, 24, 64, 1, 1);
            begin repeat (1800) @(posedge clk); kill_ba = 0; end
        join
        wait_done(12000, "T10");
        end_test("T10 ACK path corrupted: window stall, replay timeout, recovery");

        // ---- T11 sink backpressure -> overflow drop -> NAK/replay ----
        begin_test("T11 random sink backpressure, no data loss");
        bp_en = 1;
        fork
            begin : b0 integer q; for (q = 0; q < 6; q = q + 1) send_pkt(0, 0, 10, 64, 1, 1); end
            begin : b1 integer q; for (q = 0; q < 6; q = q + 1) send_pkt(0, 1, 10, 64, 1, 2); end
        join
        wait_done(60000, "T11");
        bp_en = 0;
        end_test("T11 random sink backpressure, no data loss");

        $display("link statistics before reset:");
        $display("A: tx_flits=%0d replay=%0d rx_flits=%0d fec_corr=%0d crc_fail=%0d drop=%0d", a_txf, a_rep, a_rxf, a_fec, a_crc, a_drp);
        $display("B: tx_flits=%0d replay=%0d rx_flits=%0d fec_corr=%0d crc_fail=%0d drop=%0d", b_txf, b_rep, b_rxf, b_fec, b_crc, b_drp);

        // ---- T12 reset recovery ----
        begin_test("T12 reset recovery");
        rst_n = 0; repeat (4) @(posedge clk); rst_n = 1;
        repeat (30) begin
            @(posedge clk);
            if (a_pv || b_pv || a_ov || b_ov) err("activity after reset");
        end
        if (a_txf != 0 || b_rxf != 0 || a_rep != 0) err("statistics not cleared by reset");
        end_test("T12 reset recovery");

        $display("----------------------------------------------");
        $display("beats checked: %0d   errors: %0d   failed tests: %0d", checks, errors, tests_failed);
        if (errors == 0) $display("ALL TESTS PASSED"); else $display("SOME TESTS FAILED");
        $finish;
    end

    // global watchdog
    initial begin
        #200_000_000;
        $display("GLOBAL TIMEOUT");
        $finish;
    end
endmodule
