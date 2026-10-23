
module axi_interconnect #(
    parameter N       = 2,
    parameter DATA_W  = 512,
    parameter KEEP_W  = 64,
    parameter USER_W  = 8,
    parameter ID_W    = 12,
    parameter DEST_W  = 8,
    parameter QW      = 7
)(
    input  wire                      aclk,
    input  wire                      aresetn,

    input  wire [N-1:0]              valid_in,
    input  wire [DATA_W*N-1:0]       data_in,
    input  wire [KEEP_W*N-1:0]       keep_in,
    input  wire [KEEP_W*N-1:0]       strb_in,
    input  wire [USER_W*N-1:0]       user_in,
    input  wire [ID_W*N-1:0]         id_in,
    input  wire [DEST_W*N-1:0]       dest_in,
    input  wire [N-1:0]              last_in,
    input  wire [N-1:0]              drw_in,
    input  wire [QW*N-1:0]           tqos_in,

    output wire [N-1:0]              grant,
    output wire                      tready,
    output wire [$clog2(N>1?N:2)-1:0] sel,

    output wire                      dvalid,
    output wire [DATA_W-1:0]         ddata,
    output wire [KEEP_W-1:0]         dkeep,
    output wire [KEEP_W-1:0]         dstrb,
    output wire [USER_W-1:0]         duser,
    output wire [ID_W-1:0]           did,
    output wire [DEST_W-1:0]         ddest,
    output wire                      dlast,
    output wire                      ddrw,
    output wire                      dwrite,
    output wire                      dread,

    input  wire                      dready
);
    localparam SELW = (N > 1) ? $clog2(N) : 1;

    reg              locked;
    reg [SELW-1:0]   lock_sel;
    reg [SELW-1:0]   rr;

    function [SELW-1:0] pick;
        input [N-1:0]    v;
        input [QW*N-1:0] q;
        input [SELW-1:0] r;
        integer k, idx;
        reg found;
        reg [SELW-1:0] best;
        reg [QW-1:0]   bq;
        begin
            found = 1'b0; best = {SELW{1'b0}}; bq = {QW{1'b0}};
            for (k = 0; k < N; k = k + 1) begin
                idx = (r + k) % N;
                if (v[idx] && (!found || q[QW*idx +: QW] > bq)) begin
                    found = 1'b1;
                    best  = idx;
                    bq    = q[QW*idx +: QW];
                end
            end
            pick = best;
        end
    endfunction

    wire [SELW-1:0] arb_sel = pick(valid_in, tqos_in, rr);
    wire [SELW-1:0] cur     = locked ? lock_sel : arb_sel;

    assign sel    = cur;
    assign dvalid = valid_in[cur];
    assign ddata  = dvalid ? data_in[DATA_W*cur +: DATA_W] : {DATA_W{1'b0}};
    assign dkeep  = dvalid ? keep_in[KEEP_W*cur +: KEEP_W] : {KEEP_W{1'b0}};
    assign dstrb  = dvalid ? strb_in[KEEP_W*cur +: KEEP_W] : {KEEP_W{1'b0}};
    assign duser  = dvalid ? user_in[USER_W*cur +: USER_W] : {USER_W{1'b0}};
    assign did    = dvalid ? id_in[ID_W*cur +: ID_W]       : {ID_W{1'b0}};
    assign ddest  = dvalid ? dest_in[DEST_W*cur +: DEST_W] : {DEST_W{1'b0}};
    assign dlast  = dvalid & last_in[cur];
    assign ddrw   = dvalid & drw_in[cur];
    assign dwrite = dvalid &  ddrw;
    assign dread  = dvalid & ~ddrw;
    assign tready = dready;

    wire fire = dvalid & dready;
    assign grant = fire ? ({{(N-1){1'b0}}, 1'b1} << cur) : {N{1'b0}};

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            locked   <= 1'b0;
            lock_sel <= {SELW{1'b0}};
            rr       <= {SELW{1'b0}};
        end else if (fire) begin
            if (dlast) begin
                locked <= 1'b0;
                rr     <= (cur + 1) % N;
            end else begin
                locked   <= 1'b1;
                lock_sel <= cur;
            end
        end
    end
endmodule
