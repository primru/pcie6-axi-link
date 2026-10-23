module flit_protect #(
    parameter integer DEPTH_LOG2 = 2
)(
    input  wire          aclk,
    input  wire          aresetn,
    input  wire          s_valid,
    output wire          s_ready,
    input  wire [1887:0] s_payload,
    input  wire [47:0]   s_dlp,
    output wire          m_valid,
    input  wire          m_ready,
    output wire [2047:0] m_flit
);
    localparam integer DEPTH = 1 << DEPTH_LOG2;

    reg [1935:0] body_mem [0:DEPTH-1];
    reg [63:0]   crc_mem  [0:DEPTH-1];
    reg [47:0]   fec_mem  [0:DEPTH-1];
    reg [DEPTH-1:0] done;

    reg [DEPTH_LOG2:0] wr_ptr, iss_ptr, crc_ptr, rd_ptr;

    wire [DEPTH_LOG2-1:0] wr_slot  = wr_ptr [DEPTH_LOG2-1:0];
    wire [DEPTH_LOG2-1:0] iss_slot = iss_ptr[DEPTH_LOG2-1:0];
    wire [DEPTH_LOG2-1:0] crc_slot = crc_ptr[DEPTH_LOG2-1:0];
    wire [DEPTH_LOG2-1:0] rd_slot  = rd_ptr [DEPTH_LOG2-1:0];

    wire [DEPTH_LOG2:0] occ = wr_ptr - rd_ptr;   
    assign s_ready = ~occ[DEPTH_LOG2];
    wire accept = s_valid & s_ready;

    reg        busy;
    reg [2:0]  cidx;
    wire [2047:0] padded = {112'b0, body_mem[iss_slot]};
    wire [255:0]  chunk  = padded[2047 - 256*cidx -: 256];
    wire          more   = ((iss_ptr + 1'b1) != wr_ptr);

    wire [63:0] crc_val;
    wire        crc_vld;

    pcie6_crc_pipeline #(.FCR(0), .CHUNK_BYTES(32)) u_crc (
        .clk(aclk), .rst_n(aresetn),
        .in_valid(busy), .in_first(cidx == 3'd0), .in_last(cidx == 3'd7),
        .message_chunk(chunk),
        .crc_out(crc_val), .crc_valid_out(crc_vld)
    );

    wire [47:0]            fec_val;
    wire                   fec_vld;
    wire [DEPTH_LOG2-1:0]  fec_tag;

    pcie6_fec_flit #(.TAGW(DEPTH_LOG2)) u_fec (
        .clk(aclk), .rst_n(aresetn),
        .in_valid(crc_vld), .in_tag(crc_slot),
        .in_data({body_mem[crc_slot], crc_val}),
        .fec_out(fec_val), .out_valid(fec_vld), .out_tag(fec_tag)
    );
    
    reg [DEPTH-1:0] done_n;
    wire pop = m_valid & m_ready;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            wr_ptr <= 0; iss_ptr <= 0; crc_ptr <= 0; rd_ptr <= 0;
            busy <= 1'b0; cidx <= 3'd0; done <= {DEPTH{1'b0}};
        end else begin
            if (accept) begin
                body_mem[wr_slot] <= {s_payload, s_dlp};
                wr_ptr <= wr_ptr + 1'b1;
            end

            if (!busy) begin
                if (iss_ptr != wr_ptr) begin busy <= 1'b1; cidx <= 3'd0; end
            end else begin
                cidx <= cidx + 1'b1;
                if (cidx == 3'd7) begin
                    iss_ptr <= iss_ptr + 1'b1;
                    busy    <= more;       
                end
            end

            if (crc_vld) begin
                crc_mem[crc_slot] <= crc_val;
                crc_ptr <= crc_ptr + 1'b1;
            end

            done_n = done;
            if (fec_vld) begin
                fec_mem[fec_tag] <= fec_val;
                done_n[fec_tag]   = 1'b1;
            end
            if (pop) begin
                done_n[rd_slot] = 1'b0;
                rd_ptr <= rd_ptr + 1'b1;
            end
            done <= done_n;
        end
    end

    assign m_valid = done[rd_slot];
    assign m_flit  = {body_mem[rd_slot], crc_mem[rd_slot], fec_mem[rd_slot]};
endmodule

module pcie6_crc_pipeline #(
    parameter integer FCR         = 0,
    parameter integer CHUNK_BYTES = 32
)(
    input  wire                     clk,
    input  wire                     rst_n,
    input  wire                     in_valid,
    input  wire                     in_first,
    input  wire                     in_last,
    input  wire [8*CHUNK_BYTES-1:0] message_chunk,
    output reg  [63:0]              crc_out,
    output reg                      crc_valid_out
);
    localparam integer W    = 8*CHUNK_BYTES;
    localparam integer NSEG = W/64;

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

    function [7:0] gf_alpha_pow;
        input integer n;
        integer i;
        reg [7:0] v;
        begin
            v = 8'h01;
            for (i = 0; i < n; i = i + 1) v = gf_mul(v, 8'h02);
            gf_alpha_pow = v;
        end
    endfunction

    function [71:0] gen_poly;
        input integer fcr;
        integer k, i;
        reg [71:0] g, n;
        reg [7:0]  a;
        begin
            g = 72'd1;
            for (k = 0; k < 8; k = k + 1) begin
                a = gf_alpha_pow(fcr + k);
                for (i = 0; i <= 8; i = i + 1)
                    n[8*i +: 8] = ((i > 0) ? g[8*(i-1) +: 8] : 8'h00)
                                  ^ gf_mul(a, g[8*i +: 8]);
                g = n;
            end
            gen_poly = g;
        end
    endfunction

    function [63:0] lfsr_step;
        input [63:0] r;
        input [7:0]  b;
        input [71:0] g;
        reg [7:0]  fb;
        reg [63:0] n;
        integer i;
        begin
            fb = r[63:56] ^ b;
            n[7:0] = gf_mul(fb, g[7:0]);
            for (i = 1; i < 8; i = i + 1)
                n[8*i +: 8] = r[8*(i-1) +: 8] ^ gf_mul(fb, g[8*i +: 8]);
            lfsr_step = n;
        end
    endfunction

    function [63:0] lfsr_chunk;
        input [63:0]  r;
        input [W-1:0] c;
        input [71:0]  g;
        integer k;
        reg [63:0] t;
        begin
            t = r;
            for (k = 0; k < CHUNK_BYTES; k = k + 1)
                t = lfsr_step(t, c[W-1 - 8*k -: 8], g);
            lfsr_chunk = t;
        end
    endfunction

    function [64*64-1:0] build_A;
        input [71:0] g;
        integer i, j;
        reg [63:0] col;
        begin
            for (j = 0; j < 64; j = j + 1) begin
                col = lfsr_chunk(64'd1 << j, {W{1'b0}}, g);
                for (i = 0; i < 64; i = i + 1) build_A[64*i + j] = col[i];
            end
        end
    endfunction

    function [64*W-1:0] build_B;
        input [71:0] g;
        integer i, j;
        reg [63:0] col;
        reg [W-1:0] e;
        begin
            for (j = 0; j < W; j = j + 1) begin
                e = {W{1'b0}};
                e[j] = 1'b1;
                col = lfsr_chunk(64'd0, e, g);
                for (i = 0; i < 64; i = i + 1) build_B[W*i + j] = col[i];
            end
        end
    endfunction

    localparam [71:0]       GEN = gen_poly(FCR);
    localparam [64*64-1:0]  MA  = build_A(GEN);
    localparam [64*W-1:0]   MB  = build_B(GEN);

    reg [W-1:0]       c1;
    reg               v1, f1, l1;
    reg [64*NSEG-1:0] dpart2;
    reg               v2, f2, l2;
    reg [63:0]        R;

    wire [64*NSEG-1:0] dpart;
    wire [63:0]        ar;
    reg  [63:0]        dsum;
    wire [63:0]        rn;

    genvar gi, gs;
    generate
        for (gs = 0; gs < NSEG; gs = gs + 1) begin : g_seg
            for (gi = 0; gi < 64; gi = gi + 1) begin : g_bit
                assign dpart[64*gs + gi] = ^(c1[64*gs +: 64] & MB[W*gi + 64*gs +: 64]);
            end
        end
        for (gi = 0; gi < 64; gi = gi + 1) begin : g_ar
            assign ar[gi] = ^(R & MA[64*gi +: 64]);
        end
    endgenerate

    integer s;
    always @* begin
        dsum = 64'd0;
        for (s = 0; s < NSEG; s = s + 1) dsum = dsum ^ dpart2[64*s +: 64];
    end

    assign rn = (f2 ? 64'd0 : ar) ^ dsum;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            v1 <= 1'b0; f1 <= 1'b0; l1 <= 1'b0;
            v2 <= 1'b0; f2 <= 1'b0; l2 <= 1'b0;
            c1 <= {W{1'b0}}; dpart2 <= {(64*NSEG){1'b0}};
            R <= 64'd0; crc_out <= 64'd0; crc_valid_out <= 1'b0;
        end else begin
            v1 <= in_valid; f1 <= in_first; l1 <= in_valid & in_last;
            if (in_valid) c1 <= message_chunk;
            v2 <= v1; f2 <= f1; l2 <= l1;
            dpart2 <= dpart;
            crc_valid_out <= 1'b0;
            if (v2) begin
                R <= rn;
                if (l2) begin
                    crc_out       <= rn;
                    crc_valid_out <= 1'b1;
                end
            end
        end
    end
endmodule

module pcie6_fec_flit #(
    parameter integer TAGW = 2
)(
    input  wire            clk,
    input  wire            rst_n,
    input  wire            in_valid,
    input  wire [TAGW-1:0] in_tag,
    input  wire [1999:0]   in_data,
    output reg  [47:0]     fec_out,
    output reg             out_valid,
    output reg  [TAGW-1:0] out_tag
);
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

    function [48*128-1:0] build_lane;
        input integer l;
        integer j, b, o, idx, g, k, p;
        reg [47:0] vec;
        reg [7:0]  v;
        begin
            build_lane = {(48*128){1'b0}};
            for (j = 0; j < 16; j = j + 1) begin
                idx = 16*l + j;
                if (idx < 250) begin
                    g = idx % 3;
                    k = idx / 3;
                    for (b = 0; b < 8; b = b + 1) begin
                        v = 8'h01 << b;
                        vec = 48'd0;
                        vec[8 + 16*g +: 8] = v;
                        vec[16*g +: 8]     = gf_mul(v, apow(k));
                        p = 120 - 8*j + b;
                        for (o = 0; o < 48; o = o + 1)
                            build_lane[128*o + p] = vec[o];
                    end
                end
            end
        end
    endfunction

    wire [2047:0] dpad = {in_data, 48'd0};

    reg [47:0] lp [0:15];
    reg [47:0] s2 [0:3];
    reg        v1, v2;
    reg [TAGW-1:0] t1, t2;

    genvar l, o;
    generate
        for (l = 0; l < 16; l = l + 1) begin : g_lane
            localparam [48*128-1:0] MF = build_lane(l);
            wire [127:0] lane = dpad[2047 - 128*l -: 128];
            wire [47:0]  part;
            for (o = 0; o < 48; o = o + 1) begin : g_o
                assign part[o] = ^(lane & MF[128*o +: 128]);
            end
            always @(posedge clk) lp[l] <= part;
        end
    endgenerate

    integer q;
    always @(posedge clk) begin
        for (q = 0; q < 4; q = q + 1)
            s2[q] <= lp[4*q] ^ lp[4*q+1] ^ lp[4*q+2] ^ lp[4*q+3];
        fec_out <= s2[0] ^ s2[1] ^ s2[2] ^ s2[3];
        t1 <= in_tag; t2 <= t1; out_tag <= t2;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin v1 <= 1'b0; v2 <= 1'b0; out_valid <= 1'b0; end
        else begin v1 <= in_valid; v2 <= v1; out_valid <= v2; end
    end
endmodule

module flit_transmitter (
    input  wire          aclk,
    input  wire          aresetn,
    input  wire          s_valid,
    output wire          s_ready,
    input  wire [2047:0] s_flit,
    output wire          tx_valid,
    input  wire          tx_ready,
    output wire [255:0]  tx_data,
    output wire          tx_last
);
    reg [2047:0] shift_reg;
    reg [2:0]    beat_count;
    reg          busy;

    wire last_beat_done = busy & tx_ready & (beat_count == 3'd7);

    assign s_ready  = ~busy | last_beat_done;
    assign tx_valid = busy;
    assign tx_data  = shift_reg[2047:1792];
    assign tx_last  = busy & (beat_count == 3'd7);

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            shift_reg <= 2048'b0; beat_count <= 3'd0; busy <= 1'b0;
        end else begin
            if (s_valid & s_ready) begin
                shift_reg  <= s_flit;
                beat_count <= 3'd0;
                busy       <= 1'b1;
            end else if (busy & tx_ready) begin
                if (beat_count == 3'd7) begin
                    busy <= 1'b0; beat_count <= 3'd0;
                end else begin
                    shift_reg  <= {shift_reg[1791:0], 256'b0};
                    beat_count <= beat_count + 1'b1;
                end
            end
        end
    end
endmodule
