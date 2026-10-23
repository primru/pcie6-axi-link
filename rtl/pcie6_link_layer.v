module pcie_link_layer #(
    parameter integer REPLAY_LOG2 = 3,     
    parameter integer FLUSH_TMO   = 16,    
    parameter integer REPLAY_TMO  = 2048, 
    parameter integer NAK_TMO     = 256,
    parameter integer ACK_EVERY   = 2,
    parameter integer ACK_TMO     = 16
)(
    input  wire          aclk,
    input  wire          aresetn,

    input  wire          s_valid,
    output wire          s_ready,
    input  wire [30:0]   s_address,
    input  wire          s_drw,
    input  wire [511:0]  s_data,
    input  wire [63:0]   s_keep,
    input  wire [63:0]   s_strb,
    input  wire [11:0]   s_id,
    input  wire [7:0]    s_dest,
    input  wire [7:0]    s_user,
    input  wire          s_last,

    output wire          tx_valid,
    input  wire          tx_ready,
    output wire [255:0]  tx_data,
    output wire          tx_last,

    input  wire          rx_valid,
    input  wire [255:0]  rx_data,
    input  wire          rx_last,

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
    output wire [30:0]   o_address,

    output reg  [15:0]   st_tx_flits,
    output reg  [15:0]   st_replay,
    output reg  [15:0]   st_rx_flits,
    output reg  [15:0]   st_fec_corr,
    output reg  [15:0]   st_crc_fail,
    output reg  [15:0]   st_drop
);
    wire          rec_valid, rec_ready;
    wire [703:0]  rec;
    transaction_builder u_builder (
        .aclk(aclk), .aresetn(aresetn),
        .s_valid(s_valid), .s_ready(s_ready),
        .s_address(s_address), .s_drw(s_drw), .s_data(s_data),
        .s_keep(s_keep), .s_strb(s_strb), .s_id(s_id),
        .s_dest(s_dest), .s_user(s_user), .s_last(s_last),
        .m_valid(rec_valid), .m_ready(rec_ready), .m_record(rec)
    );

    wire          pk_valid, pk_ready;
    wire [1887:0] pk_payload;
    wire [47:0]   pk_dlp;
    wire [9:0]    ack_next;
    flit_packer #(.WIN(1 << REPLAY_LOG2), .FLUSH_TMO(FLUSH_TMO)) u_packer (
        .aclk(aclk), .aresetn(aresetn),
        .s_valid(rec_valid), .s_ready(rec_ready), .s_record(rec),
        .ack_next(ack_next),
        .m_valid(pk_valid), .m_ready(pk_ready),
        .m_payload(pk_payload), .m_dlp(pk_dlp)
    );

    wire       ctl_v, ctl_nak;
    wire [9:0] ctl_seq;
    reg        cp_pend, cp_nak;
    reg [9:0]  cp_seq;
    wire       prot_s_ready;
    wire       prot_s_valid = cp_pend | pk_valid;
    wire [1887:0] prot_payload = cp_pend ? 1888'b0 : pk_payload;
    wire [47:0]   prot_dlp     = cp_pend ? {(cp_nak ? 2'b10 : 2'b01), cp_seq, 36'b0} : pk_dlp;
    assign pk_ready = prot_s_ready & ~cp_pend;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            cp_pend <= 1'b0; cp_nak <= 1'b0; cp_seq <= 10'd0;
        end else begin
            if (cp_pend & prot_s_ready) cp_pend <= 1'b0;
            if (ctl_v && !(cp_pend && cp_nak && !ctl_nak)) begin   
                cp_pend <= 1'b1; cp_nak <= ctl_nak; cp_seq <= ctl_seq;
            end
        end
    end

    wire          pr_valid, pr_ready;
    wire [2047:0] pr_flit;
    flit_protect u_protect (
        .aclk(aclk), .aresetn(aresetn),
        .s_valid(prot_s_valid), .s_ready(prot_s_ready),
        .s_payload(prot_payload), .s_dlp(prot_dlp),
        .m_valid(pr_valid), .m_ready(pr_ready), .m_flit(pr_flit)
    );

    wire          t_valid, t_ready;
    wire [2047:0] t_flit;
    wire          ack_v, nak_v, replay_pulse;
    wire [9:0]    ack_seq, nak_seq;
    replay_tx #(.DEPTH_LOG2(REPLAY_LOG2), .REPLAY_TMO(REPLAY_TMO)) u_replay (
        .clk(aclk), .rst_n(aresetn),
        .p_valid(pr_valid), .p_ready(pr_ready), .p_flit(pr_flit),
        .t_valid(t_valid), .t_ready(t_ready), .t_flit(t_flit),
        .ack_v(ack_v), .ack_seq(ack_seq), .nak_v(nak_v), .nak_seq(nak_seq),
        .ack_next(ack_next), .replay_pulse(replay_pulse)
    );

    flit_transmitter u_flit_tx (
        .aclk(aclk), .aresetn(aresetn),
        .s_valid(t_valid), .s_ready(t_ready), .s_flit(t_flit),
        .tx_valid(tx_valid), .tx_ready(tx_ready),
        .tx_data(tx_data), .tx_last(tx_last)
    );

    wire          f_valid, f_err;
    wire [2047:0] f_flit;
    flit_receiver u_flit_rx (
        .aclk(aclk), .aresetn(aresetn),
        .rx_valid(rx_valid), .rx_data(rx_data), .rx_last(rx_last),
        .f_valid(f_valid), .f_flit(f_flit), .f_err(f_err)
    );

    wire          c_valid, c_ready, c_ok;
    wire [2047:0] c_flit;
    wire          chk_drop, chk_corr, chk_crcfail;
    flit_check #(.DEPTH_LOG2(2)) u_check (
        .clk(aclk), .rst_n(aresetn),
        .in_valid(f_valid), .in_flit(f_flit),
        .drop(chk_drop),
        .o_valid(c_valid), .o_ready(c_ready), .o_flit(c_flit), .o_ok(c_ok),
        .corr_pulse(chk_corr), .crcfail_pulse(chk_crcfail)
    );

    wire          dp_load, dp_busy;
    wire [1887:0] dp_payload;
    wire [3:0]    dp_nrec;
    rx_link_ctl #(.ACK_EVERY(ACK_EVERY), .ACK_TMO(ACK_TMO), .NAK_TMO(NAK_TMO)) u_ctl (
        .clk(aclk), .rst_n(aresetn),
        .c_valid(c_valid), .c_ready(c_ready), .c_flit(c_flit), .c_ok(c_ok),
        .dp_load(dp_load), .dp_payload(dp_payload), .dp_nrec(dp_nrec), .dp_busy(dp_busy),
        .ack_v(ack_v), .ack_seq(ack_seq), .nak_v(nak_v), .nak_seq(nak_seq),
        .ctl_v(ctl_v), .ctl_nak(ctl_nak), .ctl_seq(ctl_seq)
    );

    rec_depack u_depack (
        .clk(aclk), .rst_n(aresetn),
        .load(dp_load), .payload(dp_payload), .nrec(dp_nrec), .busy(dp_busy),
        .o_tvalid(o_tvalid), .o_tready(o_tready), .o_tdata(o_tdata),
        .o_tkeep(o_tkeep), .o_tstrb(o_tstrb), .o_tuser(o_tuser), .o_tid(o_tid),
        .o_tdest(o_tdest), .o_tlast(o_tlast), .o_drw(o_drw), .o_address(o_address)
    );

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            st_tx_flits <= 16'd0; st_replay <= 16'd0; st_rx_flits <= 16'd0;
            st_fec_corr <= 16'd0; st_crc_fail <= 16'd0; st_drop <= 16'd0;
        end else begin
            if (t_valid & t_ready)  st_tx_flits <= st_tx_flits + 1'b1;
            if (replay_pulse)       st_replay   <= st_replay + 1'b1;
            if (dp_load)            st_rx_flits <= st_rx_flits + 1'b1;
            if (chk_corr)           st_fec_corr <= st_fec_corr + 1'b1;
            if (chk_crcfail)        st_crc_fail <= st_crc_fail + 1'b1;
            if (chk_drop | f_err)   st_drop     <= st_drop + 1'b1;
        end
    end
endmodule

module transaction_builder (
    input  wire          aclk,
    input  wire          aresetn,
    input  wire          s_valid,
    output wire          s_ready,
    input  wire [30:0]   s_address,
    input  wire          s_drw,
    input  wire [511:0]  s_data,
    input  wire [63:0]   s_keep,
    input  wire [63:0]   s_strb,
    input  wire [11:0]   s_id,
    input  wire [7:0]    s_dest,
    input  wire [7:0]    s_user,
    input  wire          s_last,
    output wire          m_valid,
    input  wire          m_ready,
    output wire [703:0]  m_record
);
    reg          valid_reg;
    reg [703:0]  rec_reg;

    assign s_ready  = ~valid_reg | m_ready;
    assign m_valid  = valid_reg;
    assign m_record = rec_reg;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            valid_reg <= 1'b0;
            rec_reg   <= 704'b0;
        end else if (s_ready) begin
            valid_reg <= s_valid;
            if (s_valid)
                rec_reg <= {s_drw, s_last, s_address, s_id, s_dest, s_user,
                            s_keep, s_strb, 3'b000,
                            (s_drw ? s_data : 512'b0)};
        end
    end
endmodule

module flit_packer #(
    parameter integer WIN       = 8,
    parameter integer FLUSH_TMO = 16
)(
    input  wire          aclk,
    input  wire          aresetn,
    input  wire          s_valid,
    output wire          s_ready,
    input  wire [703:0]  s_record,
    input  wire [9:0]    ack_next,
    output reg           m_valid,
    input  wire          m_ready,
    output reg  [1887:0] m_payload,
    output reg  [47:0]   m_dlp
);
    reg [1887:0] acc;
    reg [7:0]    fill;
    reg [3:0]    nrec;
    reg [9:0]    seq;
    reg [7:0]    idle;

    wire        rw     = s_record[703];
    wire [7:0]  rb     = rw ? 8'd88 : 8'd24;
    wire [8:0]  fsum   = {1'b0, fill} + {1'b0, rb};
    wire        fits   = (fsum <= 9'd236);
    wire        can_emit = ~m_valid | m_ready;
    wire [9:0]  outst  = seq - ack_next;
    wire        win_ok = (outst < WIN);
    wire        has    = (nrec != 4'd0);
    wire        full   = has & (fill > 8'd212);            
    wire        tmo    = has & (idle >= FLUSH_TMO);
    wire        want_emit = has & ((s_valid & ~fits) | full | tmo);
    wire        emit   = want_emit & can_emit & win_ok;

    assign s_ready = fits & ~full & ~tmo;
    wire take = s_valid & s_ready;

    wire [1887:0] placed = acc | ({s_record, 1184'b0} >> {fill, 3'b000});

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            acc <= 1888'b0; fill <= 8'd0; nrec <= 4'd0; seq <= 10'd0; idle <= 8'd0;
            m_valid <= 1'b0; m_payload <= 1888'b0; m_dlp <= 48'b0;
        end else begin
            if (m_valid & m_ready) m_valid <= 1'b0;

            if (emit) begin
                m_payload <= acc;
                m_dlp     <= {2'b00, seq, nrec, fill, 24'b0};
                m_valid   <= 1'b1;
                seq       <= seq + 1'b1;
                acc <= 1888'b0; fill <= 8'd0; nrec <= 4'd0; idle <= 8'd0;
            end else if (take) begin
                acc  <= placed;
                fill <= fsum[7:0];
                nrec <= nrec + 1'b1;
                idle <= 8'd0;
            end else if (has && idle != 8'hFF) begin
                idle <= idle + 1'b1;
            end
        end
    end
endmodule


module replay_tx #(
    parameter integer DEPTH_LOG2  = 3,
    parameter integer REPLAY_TMO  = 2048
)(
    input  wire          clk,
    input  wire          rst_n,
    input  wire          p_valid,
    output wire          p_ready,
    input  wire [2047:0] p_flit,
    output wire          t_valid,
    input  wire          t_ready,
    output wire [2047:0] t_flit,
    input  wire          ack_v,
    input  wire [9:0]    ack_seq,
    input  wire          nak_v,
    input  wire [9:0]    nak_seq,
    output reg  [9:0]    ack_next,
    output reg           replay_pulse
);
    localparam integer DEPTH = 1 << DEPTH_LOG2;

    reg [2047:0] rbuf [0:DEPTH-1];
    reg [9:0]    sent_next;
    reg [9:0]    rptr;
    reg          replay;
    reg [15:0]   tmo;

    wire [9:0] outst = sent_next - ack_next;
    wire       pdata = (p_flit[159:158] == 2'b00);

    wire       any_v   = ack_v | nak_v;
    wire [9:0] new_ack = nak_v ? nak_seq : ack_seq;
    wire [9:0] a_d     = new_ack - ack_next;
    wire       a_ok    = any_v & (a_d <= outst);
    wire [9:0] r_d     = new_ack - rptr;
    wire       r_ahead = (r_d != 10'd0) & (r_d < 10'd512);

    wire [DEPTH_LOG2-1:0] rslot = rptr[DEPTH_LOG2-1:0];
    wire [DEPTH_LOG2-1:0] wslot = sent_next[DEPTH_LOG2-1:0];

    assign t_valid = replay ? (rptr != sent_next) : p_valid;
    assign t_flit  = replay ? rbuf[rslot] : p_flit;
    assign p_ready = ~replay & t_ready;
    wire send_new  = ~replay & p_valid & t_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sent_next <= 10'd0; ack_next <= 10'd0; rptr <= 10'd0;
            replay <= 1'b0; tmo <= 16'd0; replay_pulse <= 1'b0;
        end else begin
            replay_pulse <= 1'b0;

            if (send_new && pdata) begin
                rbuf[wslot] <= p_flit;
                sent_next   <= sent_next + 1'b1;
            end

            if (replay && t_ready && rptr != sent_next) rptr <= rptr + 1'b1;
            if (replay && rptr == sent_next)            replay <= 1'b0;

            if (outst != 10'd0 && !replay) begin
                tmo <= tmo + 1'b1;
                if (tmo >= REPLAY_TMO) begin
                    replay <= 1'b1; rptr <= ack_next; tmo <= 16'd0; replay_pulse <= 1'b1;
                end
            end else begin
                tmo <= 16'd0;
            end

            if (a_ok) begin
                ack_next <= new_ack;
                tmo      <= 16'd0;
                if (replay && r_ahead) rptr <= new_ack;
                if (nak_v && (nak_seq != sent_next)) begin
                    replay <= 1'b1; rptr <= nak_seq; replay_pulse <= 1'b1;
                end
            end
        end
    end
endmodule
