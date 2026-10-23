//##############################################################################
// flit_receiver : assembles eight 256-bit beats (MSB first) into one FLIT.
// Framing check: rx_last must coincide with the 8th beat, otherwise the
// partial FLIT is discarded (f_err pulse).
//##############################################################################
module flit_receiver (
    input  wire          aclk,
    input  wire          aresetn,
    input  wire          rx_valid,
    input  wire [255:0]  rx_data,
    input  wire          rx_last,
    output reg           f_valid,
    output wire [2047:0] f_flit,
    output reg           f_err
);
    reg [2047:0] sh;
    reg [2:0]    cnt;
    assign f_flit = sh;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            sh <= 2048'b0; cnt <= 3'd0; f_valid <= 1'b0; f_err <= 1'b0;
        end else begin
            f_valid <= 1'b0;
            f_err   <= 1'b0;
            if (rx_valid) begin
                if (rx_last != (cnt == 3'd7)) begin
                    cnt <= 3'd0; f_err <= 1'b1;
                end else begin
                    sh  <= {sh[1791:0], rx_data};
                    cnt <= cnt + 1'b1;
                    if (cnt == 3'd7) f_valid <= 1'b1;
                end
            end
        end
    end
endmodule


//##############################################################################
// flit_check : receive-side FEC correction + CRC check, pipelined, in order.
//   slot RAM -> FEC syndrome (reuses pcie6_fec_flit) -> single-symbol
//   correction per 3-way interleave group -> CRC over corrected bytes 0..241
//   -> o_ok = CRC match and no uncorrectable group.
// A FLIT arriving while all slots are busy is dropped (drop pulse); the
// sequence-number logic recovers it with NAK/replay.
//##############################################################################
module flit_check #(
    parameter integer DEPTH_LOG2 = 2
)(
    input  wire          clk,
    input  wire          rst_n,
    input  wire          in_valid,
    input  wire [2047:0] in_flit,
    output reg           drop,
    output wire          o_valid,
    input  wire          o_ready,
    output wire [2047:0] o_flit,
    output wire          o_ok,
    output reg           corr_pulse,
    output reg           crcfail_pulse
);
    localparam integer DEPTH = 1 << DEPTH_LOG2;

    function [7:0] gf_mul;
        input [7:0] a, b;
        integer i;
        reg [14:0] p;
        begin
            p = 15'd0;
            for (i = 0; i < 8; i = i + 1)
                if (b[i]) p = p ^ ({7'b0, a} << i);
            for (i = 14; i >= 8; i = i - 1)
                if (p[i]) p = p ^ (15'h11D << (i - 8));
            gf_mul = p[7:0];
        end
    endfunction

    function [7:0] apow;
        input integer n;
        integer i;
        reg [7:0] v;
        begin
            v = 8'h01;
            for (i = 0; i < n; i = i + 1) v = gf_mul(v, 8'h02);
            apow = v;
        end
    endfunction

    reg [2047:0]       mem [0:DEPTH-1];
    reg [DEPTH-1:0]    unc, okf;
    reg [DEPTH_LOG2:0] wr, fi, fd, ci, cd, rd;

    wire [DEPTH_LOG2-1:0] wr_s = wr[DEPTH_LOG2-1:0];
    wire [DEPTH_LOG2-1:0] fi_s = fi[DEPTH_LOG2-1:0];
    wire [DEPTH_LOG2-1:0] ci_s = ci[DEPTH_LOG2-1:0];
    wire [DEPTH_LOG2-1:0] cd_s = cd[DEPTH_LOG2-1:0];
    wire [DEPTH_LOG2-1:0] rd_s = rd[DEPTH_LOG2-1:0];

    wire [DEPTH_LOG2:0] occ   = wr - rd;
    wire                space = ~occ[DEPTH_LOG2];

    // ---------------- FEC syndrome ----------------
    wire                  fec_in_v = (fi != wr);
    wire [47:0]           fec_val;
    wire                  fec_ov;
    wire [DEPTH_LOG2-1:0] fec_tag;
    wire [2047:0]         fi_word = mem[fi_s];

    pcie6_fec_flit #(.TAGW(DEPTH_LOG2)) u_fec (
        .clk(clk), .rst_n(rst_n),
        .in_valid(fec_in_v), .in_tag(fi_s), .in_data(fi_word[2047:48]),
        .fec_out(fec_val), .out_valid(fec_ov), .out_tag(fec_tag)
    );

    wire [2047:0] cw  = mem[fec_tag];
    wire [47:0]   syn = fec_val ^ cw[47:0];       // {P2,C2,P1,C1,P0,C0} syndromes

    wire [1999:0] corr_body;
    wire [249:0]  match_v;
    genvar gi;
    generate
        for (gi = 0; gi < 250; gi = gi + 1) begin : g_corr
            localparam integer G  = gi % 3;
            localparam integer K  = gi / 3;
            localparam [7:0]   AK = apow(K);
            wire [7:0] s0 = syn[8 + 16*G +: 8];
            wire [7:0] s1 = syn[16*G +: 8];
            wire       m  = (s0 != 8'h00) && (s1 == gf_mul(s0, AK));
            assign match_v[gi] = m;
            assign corr_body[1999 - 8*gi -: 8] = cw[2047 - 8*gi -: 8] ^ (m ? s0 : 8'h00);
        end
    endgenerate

    reg [2:0] gm, gs0, gs1;
    integer q;
    always @* begin
        gm = 3'b000; gs0 = 3'b000; gs1 = 3'b000;
        for (q = 0; q < 250; q = q + 1)
            if (match_v[q]) gm[q % 3] = 1'b1;
        for (q = 0; q < 3; q = q + 1) begin
            gs0[q] = (syn[8 + 16*q +: 8] != 8'h00);
            gs1[q] = (syn[16*q +: 8]     != 8'h00);
        end
    end
    wire unc_now = |(gs0 & gs1 & ~gm);   // both syndromes set but no valid position
    wire any_err = |syn;

    // ---------------- CRC over corrected bytes 0..241 ----------------
    reg        busy;
    reg [2:0]  cidx;
    wire [2047:0] ci_word = mem[ci_s];
    wire [2047:0] padded  = {112'b0, ci_word[2047:112]};
    wire [255:0]  chunk   = padded[2047 - 256*cidx -: 256];
    wire          more    = ((ci + 1'b1) != fd);
    wire [63:0]   crc_val;
    wire          crc_vld;

    pcie6_crc_pipeline #(.FCR(0), .CHUNK_BYTES(32)) u_crc (
        .clk(clk), .rst_n(rst_n),
        .in_valid(busy), .in_first(cidx == 3'd0), .in_last(cidx == 3'd7),
        .message_chunk(chunk),
        .crc_out(crc_val), .crc_valid_out(crc_vld)
    );

    wire [2047:0] cd_word = mem[cd_s];
    wire          crc_good = (crc_val == cd_word[111:48]) && !unc[cd_s];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr <= 0; fi <= 0; fd <= 0; ci <= 0; cd <= 0; rd <= 0;
            busy <= 1'b0; cidx <= 3'd0; unc <= 0; okf <= 0;
            drop <= 1'b0; corr_pulse <= 1'b0; crcfail_pulse <= 1'b0;
        end else begin
            drop <= 1'b0; corr_pulse <= 1'b0; crcfail_pulse <= 1'b0;

            if (in_valid) begin
                if (space) begin mem[wr_s] <= in_flit; wr <= wr + 1'b1; end
                else       drop <= 1'b1;
            end

            if (fec_in_v) fi <= fi + 1'b1;

            if (fec_ov) begin
                mem[fec_tag] <= {corr_body, cw[47:0]};
                unc[fec_tag] <= unc_now;
                fd <= fd + 1'b1;
                if (any_err & ~unc_now) corr_pulse <= 1'b1;
            end

            if (!busy) begin
                if (ci != fd) begin busy <= 1'b1; cidx <= 3'd0; end
            end else begin
                cidx <= cidx + 1'b1;
                if (cidx == 3'd7) begin ci <= ci + 1'b1; busy <= more; end
            end

            if (crc_vld) begin
                okf[cd_s] <= crc_good;
                if (!crc_good) crcfail_pulse <= 1'b1;
                cd <= cd + 1'b1;
            end

            if (o_valid & o_ready) rd <= rd + 1'b1;
        end
    end

    assign o_valid = (rd != cd);
    assign o_flit  = mem[rd_s];
    assign o_ok    = okf[rd_s];
endmodule


//##############################################################################
// rx_link_ctl : sequence tracking, ACK/NAK generation, ACK/NAK forwarding
//   data flit, seq == expected : accept (if depacketizer free), ACK coalesced
//   data flit, seq ahead       : drop, NAK(expected) (re-armed after NAK_TMO)
//   data flit, seq behind      : drop, immediate ACK (duplicate after replay)
//   bad CRC                    : drop, NAK(expected)
//   control flit               : forwarded to replay_tx
//##############################################################################
module rx_link_ctl #(
    parameter integer ACK_EVERY = 2,
    parameter integer ACK_TMO   = 16,
    parameter integer NAK_TMO   = 256
)(
    input  wire          clk,
    input  wire          rst_n,

    input  wire          c_valid,
    output wire          c_ready,
    input  wire [2047:0] c_flit,
    input  wire          c_ok,

    output wire          dp_load,
    output wire [1887:0] dp_payload,
    output wire [3:0]    dp_nrec,
    input  wire          dp_busy,

    output reg           ack_v,
    output reg  [9:0]    ack_seq,
    output reg           nak_v,
    output reg  [9:0]    nak_seq,

    output reg           ctl_v,
    output reg           ctl_nak,
    output reg  [9:0]    ctl_seq
);
    wire [47:0] dlp  = c_flit[159:112];
    wire [1:0]  ty   = dlp[47:46];
    wire [9:0]  seq  = dlp[45:36];
    wire [3:0]  nrec = dlp[35:32];

    reg [9:0] expected;
    reg       nak_sent;
    reg [8:0] nak_timer;
    reg [7:0] ack_cnt, ack_timer;
    reg       ack_pending;

    wire [9:0] d       = seq - expected;
    wire is_data = c_ok & (ty == 2'b00);
    wire accept  = is_data & (d == 10'd0);
    wire ahead   = is_data & (d != 10'd0) & (d < 10'd512);
    wire dup     = is_data & ~accept & ~ahead;

    assign c_ready    = c_valid & ~(accept & dp_busy);
    assign dp_load    = c_valid & accept & ~dp_busy;
    assign dp_payload = c_flit[2047:160];
    assign dp_nrec    = nrec;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            expected <= 10'd0; nak_sent <= 1'b0; nak_timer <= 9'd0;
            ack_cnt <= 8'd0; ack_timer <= 8'd0; ack_pending <= 1'b0;
            ack_v <= 1'b0; ack_seq <= 10'd0; nak_v <= 1'b0; nak_seq <= 10'd0;
            ctl_v <= 1'b0; ctl_nak <= 1'b0; ctl_seq <= 10'd0;
        end else begin
            ack_v <= 1'b0; nak_v <= 1'b0; ctl_v <= 1'b0;

            if (ack_pending && ack_timer != 8'hFF) ack_timer <= ack_timer + 1'b1;
            if (nak_sent) begin
                nak_timer <= nak_timer + 1'b1;
                if (nak_timer >= NAK_TMO) begin nak_sent <= 1'b0; nak_timer <= 9'd0; end
            end

            if (c_valid & c_ready) begin
                if (!c_ok) begin
                    if (!nak_sent) begin ctl_v <= 1'b1; ctl_nak <= 1'b1; ctl_seq <= expected; nak_sent <= 1'b1; nak_timer <= 9'd0; end
                end else if (ty == 2'b01) begin
                    ack_v <= 1'b1; ack_seq <= seq;
                end else if (ty == 2'b10) begin
                    nak_v <= 1'b1; nak_seq <= seq;
                end else if (accept) begin
                    expected    <= expected + 1'b1;
                    nak_sent    <= 1'b0;
                    ack_cnt     <= ack_cnt + 1'b1;
                    ack_pending <= 1'b1;
                    ack_timer   <= 8'd0;
                end else if (ahead) begin
                    if (!nak_sent) begin ctl_v <= 1'b1; ctl_nak <= 1'b1; ctl_seq <= expected; nak_sent <= 1'b1; nak_timer <= 9'd0; end
                end else if (dup) begin
                    ctl_v <= 1'b1; ctl_nak <= 1'b0; ctl_seq <= expected;
                end
            end else if (ack_pending && (ack_cnt >= ACK_EVERY || ack_timer >= ACK_TMO)) begin
                ctl_v <= 1'b1; ctl_nak <= 1'b0; ctl_seq <= expected;
                ack_pending <= 1'b0; ack_cnt <= 8'd0; ack_timer <= 8'd0;
            end
        end
    end
endmodule


//##############################################################################
// rec_depack : unpacks the records of one accepted payload, one per clock
//##############################################################################
module rec_depack (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          load,
    input  wire [1887:0] payload,
    input  wire [3:0]    nrec,
    output wire          busy,
    output wire          o_tvalid,
    input  wire          o_tready,
    output wire [511:0]  o_tdata,
    output wire [63:0]   o_tkeep,
    output wire [63:0]   o_tstrb,
    output wire [7:0]    o_tuser,
    output wire [11:0]   o_tid,
    output wire [7:0]    o_tdest,
    output wire          o_tlast,
    output wire          o_drw,
    output wire [30:0]   o_address
);
    reg [1887:0] pay;
    reg [3:0]    cnt;
    wire [191:0] h = pay[1887:1696];

    assign busy      = (cnt != 4'd0);
    assign o_tvalid  = busy;
    assign o_drw     = h[191];
    assign o_tlast   = h[190];
    assign o_address = h[189:159];
    assign o_tid     = h[158:147];
    assign o_tdest   = h[146:139];
    assign o_tuser   = h[138:131];
    assign o_tkeep   = h[130:67];
    assign o_tstrb   = h[66:3];
    assign o_tdata   = o_drw ? pay[1695:1184] : 512'b0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pay <= 1888'b0; cnt <= 4'd0;
        end else if (load) begin
            pay <= payload; cnt <= nrec;
        end else if (busy & o_tready) begin
            pay <= o_drw ? (pay << 704) : (pay << 192);
            cnt <= cnt - 1'b1;
        end
    end
endmodule
