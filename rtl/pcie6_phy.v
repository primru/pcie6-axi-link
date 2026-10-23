module pcie6_proto_to_phy #(
    parameter integer PRECODE  = 1,
    parameter integer SCRAMBLE = 1
)(
    input  wire         clk,
    input  wire         rst_n,

    input  wire [255:0] flow_data,
    input  wire         flow_valid,
    input  wire         flow_last,

    output reg  [63:0]  lane0_sym,
    output reg  [63:0]  lane1_sym,
    output reg  [63:0]  lane2_sym,
    output reg  [63:0]  lane3_sym,
    output reg          phy_valid,
    output reg          phy_last
);
    function [86:0] scr_adv;      
        input [22:0] s;
        integer i;
        reg [22:0] st;
        reg [63:0] ks;
        reg fb;
        begin
            st = s;
            for (i = 0; i < 64; i = i + 1) begin
                ks[63-i] = st[22];
                fb = st[22] ^ st[20] ^ st[15] ^ st[7] ^ st[4] ^ st[1];
                st = {st[21:0], fb};
            end
            scr_adv = {st, ks};
        end
    endfunction

    function [65:0] enc_lane;     
        input [63:0] d;
        input [1:0]  pin;
        integer t;
        reg [1:0] g, p;
        reg [63:0] o;
        begin
            p = pin;
            for (t = 0; t < 32; t = t + 1) begin
                g = {d[63-2*t], d[63-2*t] ^ d[62-2*t]};
                if (PRECODE != 0) begin
                    p = g - p;
                    o[63-2*t -: 2] = p;
                end else begin
                    o[63-2*t -: 2] = g;
                end
            end
            enc_lane = {p, o};
        end
    endfunction

    function [63:0] stripe;       
        input [255:0] f;
        input integer l;
        integer m;
        reg [63:0] r;
        begin
            for (m = 0; m < 8; m = m + 1)
                r[63-8*m -: 8] = f[255 - 8*(4*m + l) -: 8];
            stripe = r;
        end
    endfunction

    localparam [22:0] SEED0 = 23'h1DBFBC, SEED1 = 23'h0607BB,
                      SEED2 = 23'h1EC760, SEED3 = 23'h18C0DB;

    reg        first;
    reg [22:0] s0, s1, s2, s3;
    reg [1:0]  pp0, pp1, pp2, pp3;

    wire [86:0] a0 = scr_adv(first ? SEED0 : s0);
    wire [86:0] a1 = scr_adv(first ? SEED1 : s1);
    wire [86:0] a2 = scr_adv(first ? SEED2 : s2);
    wire [86:0] a3 = scr_adv(first ? SEED3 : s3);

    wire [63:0] k0 = (SCRAMBLE != 0) ? a0[63:0] : 64'b0;
    wire [63:0] k1 = (SCRAMBLE != 0) ? a1[63:0] : 64'b0;
    wire [63:0] k2 = (SCRAMBLE != 0) ? a2[63:0] : 64'b0;
    wire [63:0] k3 = (SCRAMBLE != 0) ? a3[63:0] : 64'b0;

    wire [65:0] e0 = enc_lane(stripe(flow_data, 0) ^ k0, first ? 2'b00 : pp0);
    wire [65:0] e1 = enc_lane(stripe(flow_data, 1) ^ k1, first ? 2'b00 : pp1);
    wire [65:0] e2 = enc_lane(stripe(flow_data, 2) ^ k2, first ? 2'b00 : pp2);
    wire [65:0] e3 = enc_lane(stripe(flow_data, 3) ^ k3, first ? 2'b00 : pp3);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lane0_sym <= 64'b0; lane1_sym <= 64'b0; lane2_sym <= 64'b0; lane3_sym <= 64'b0;
            phy_valid <= 1'b0; phy_last <= 1'b0;
            first <= 1'b1;
            s0 <= SEED0; s1 <= SEED1; s2 <= SEED2; s3 <= SEED3;
            pp0 <= 2'b00; pp1 <= 2'b00; pp2 <= 2'b00; pp3 <= 2'b00;
        end else begin
            if (flow_valid) begin
                lane0_sym <= e0[63:0]; lane1_sym <= e1[63:0];
                lane2_sym <= e2[63:0]; lane3_sym <= e3[63:0];
                phy_valid <= 1'b1;
                phy_last  <= flow_last;
                s0 <= a0[86:64]; s1 <= a1[86:64]; s2 <= a2[86:64]; s3 <= a3[86:64];
                pp0 <= e0[65:64]; pp1 <= e1[65:64]; pp2 <= e2[65:64]; pp3 <= e3[65:64];
                first <= flow_last;
            end else begin
                phy_valid <= 1'b0;
                phy_last  <= 1'b0;
            end
        end
    end
endmodule


module pcie6_phy_to_proto #(
    parameter integer PRECODE  = 1,
    parameter integer SCRAMBLE = 1
)(
    input  wire         clk,
    input  wire         rst_n,

    input  wire [63:0]  lane0_sym,
    input  wire [63:0]  lane1_sym,
    input  wire [63:0]  lane2_sym,
    input  wire [63:0]  lane3_sym,
    input  wire         phy_valid,
    input  wire         phy_last,

    output reg  [255:0] flow_data,
    output reg          flow_valid,
    output reg          flow_last
);
    function [86:0] scr_adv;
        input [22:0] s;
        integer i;
        reg [22:0] st;
        reg [63:0] ks;
        reg fb;
        begin
            st = s;
            for (i = 0; i < 64; i = i + 1) begin
                ks[63-i] = st[22];
                fb = st[22] ^ st[20] ^ st[15] ^ st[7] ^ st[4] ^ st[1];
                st = {st[21:0], fb};
            end
            scr_adv = {st, ks};
        end
    endfunction

    function [63:0] dec_lane;     
        input [63:0] sy;
        input [1:0]  pin;
        integer t;
        reg [1:0] g, p, pprev;
        reg [63:0] o;
        begin
            pprev = pin;
            for (t = 0; t < 32; t = t + 1) begin
                p = sy[63-2*t -: 2];
                if (PRECODE != 0) g = p + pprev;
                else              g = p;
                pprev = p;
                o[63-2*t]  = g[1];
                o[62-2*t]  = g[1] ^ g[0];
            end
            dec_lane = o;
        end
    endfunction

    localparam [22:0] SEED0 = 23'h1DBFBC, SEED1 = 23'h0607BB,
                      SEED2 = 23'h1EC760, SEED3 = 23'h18C0DB;

    reg        first;
    reg [22:0] s0, s1, s2, s3;
    reg [1:0]  pp0, pp1, pp2, pp3;

    wire [86:0] a0 = scr_adv(first ? SEED0 : s0);
    wire [86:0] a1 = scr_adv(first ? SEED1 : s1);
    wire [86:0] a2 = scr_adv(first ? SEED2 : s2);
    wire [86:0] a3 = scr_adv(first ? SEED3 : s3);

    wire [63:0] k0 = (SCRAMBLE != 0) ? a0[63:0] : 64'b0;
    wire [63:0] k1 = (SCRAMBLE != 0) ? a1[63:0] : 64'b0;
    wire [63:0] k2 = (SCRAMBLE != 0) ? a2[63:0] : 64'b0;
    wire [63:0] k3 = (SCRAMBLE != 0) ? a3[63:0] : 64'b0;

    wire [63:0] d0 = dec_lane(lane0_sym, first ? 2'b00 : pp0) ^ k0;
    wire [63:0] d1 = dec_lane(lane1_sym, first ? 2'b00 : pp1) ^ k1;
    wire [63:0] d2 = dec_lane(lane2_sym, first ? 2'b00 : pp2) ^ k2;
    wire [63:0] d3 = dec_lane(lane3_sym, first ? 2'b00 : pp3) ^ k3;

    reg [255:0] beat;
    integer m;
    always @* begin
        for (m = 0; m < 8; m = m + 1) begin
            beat[255 - 8*(4*m + 0) -: 8] = d0[63-8*m -: 8];
            beat[255 - 8*(4*m + 1) -: 8] = d1[63-8*m -: 8];
            beat[255 - 8*(4*m + 2) -: 8] = d2[63-8*m -: 8];
            beat[255 - 8*(4*m + 3) -: 8] = d3[63-8*m -: 8];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            flow_data <= 256'b0; flow_valid <= 1'b0; flow_last <= 1'b0;
            first <= 1'b1;
            s0 <= SEED0; s1 <= SEED1; s2 <= SEED2; s3 <= SEED3;
            pp0 <= 2'b00; pp1 <= 2'b00; pp2 <= 2'b00; pp3 <= 2'b00;
        end else begin
            flow_valid <= phy_valid;
            flow_last  <= phy_valid & phy_last;
            if (phy_valid) begin
                flow_data <= beat;
                s0 <= a0[86:64]; s1 <= a1[86:64]; s2 <= a2[86:64]; s3 <= a3[86:64];
                pp0 <= lane0_sym[1:0]; pp1 <= lane1_sym[1:0];
                pp2 <= lane2_sym[1:0]; pp3 <= lane3_sym[1:0];
                first <= phy_last;
            end
        end
    end
endmodule
