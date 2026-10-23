`timescale 1ns/1ps

module pcie6_top #(
    parameter integer N            = 2,
    parameter integer DATA_W       = 512,
    parameter integer KEEP_W       = 64,
    parameter integer USER_W       = 8,
    parameter integer ID_W         = 12,
    parameter integer DEST_W       = 8,
    parameter integer QW           = 7,
    parameter integer REPLAY_LOG2  = 3,
    parameter integer REPLAY_TMO   = 2048,
    parameter integer FLUSH_TMO    = 16,
    parameter integer PRECODE      = 1,
    parameter integer SCRAMBLE     = 1
)(
    input  wire clk,
    input  wire rst_n,

    input  wire [N-1:0]        m_tvalid,
    output wire [N-1:0]        m_tready,
    input  wire [DATA_W*N-1:0] m_tdata,
    input  wire [KEEP_W*N-1:0] m_tkeep,
    input  wire [KEEP_W*N-1:0] m_tstrb,
    input  wire [USER_W*N-1:0] m_tuser,
    input  wire [ID_W*N-1:0]   m_tid,
    input  wire [DEST_W*N-1:0] m_tdest,
    input  wire [N-1:0]        m_tlast,
    input  wire [QW*N-1:0]     m_tqos,
    input  wire [31*N-1:0]     m_address,
    input  wire [N-1:0]        m_drw,         
    
    output wire [63:0]  lane0_sym,
    output wire [63:0]  lane1_sym,
    output wire [63:0]  lane2_sym,
    output wire [63:0]  lane3_sym,
    output wire         phy_valid,
    output wire         phy_last,

   
    input  wire [63:0]  rx_lane0_sym,
    input  wire [63:0]  rx_lane1_sym,
    input  wire [63:0]  rx_lane2_sym,
    input  wire [63:0]  rx_lane3_sym,
    input  wire         rx_phy_valid,
    input  wire         rx_phy_last,

    
    output wire                o_tvalid,
    input  wire                o_tready,
    output wire [DATA_W-1:0]   o_tdata,
    output wire [KEEP_W-1:0]   o_tkeep,
    output wire [KEEP_W-1:0]   o_tstrb,
    output wire [USER_W-1:0]   o_tuser,
    output wire [ID_W-1:0]     o_tid,
    output wire [DEST_W-1:0]   o_tdest,
    output wire                o_tlast,
    output wire                o_drw,
    output wire [30:0]         o_address,

   
    output wire         mon_tx_valid,
    output wire [255:0] mon_tx_data,
    output wire         mon_tx_last,
    output wire [15:0]  st_tx_flits,
    output wire [15:0]  st_replay,
    output wire [15:0]  st_rx_flits,
    output wire [15:0]  st_fec_corr,
    output wire [15:0]  st_crc_fail,
    output wire [15:0]  st_drop
);
    wire [N-1:0]        master_valid, grant;
    wire [DATA_W*N-1:0] master_data;
    wire [KEEP_W*N-1:0] master_keep, master_strb;
    localparam integer UW = USER_W + 31;        
    wire [UW*N-1:0]     master_user;
    wire [ID_W*N-1:0]   master_id;
    wire [DEST_W*N-1:0] master_dest;
    wire [N-1:0]        master_last, master_drw;

    wire                dvalid, dready, dlast, ddrw, dwrite, dread, xbar_tready;
    wire [DATA_W-1:0]   ddata;
    wire [KEEP_W-1:0]   dkeep, dstrb;
    wire [UW-1:0]       duser_x;
    wire [USER_W-1:0]   duser = duser_x[USER_W-1:0];
    wire [ID_W-1:0]     did;
    wire [DEST_W-1:0]   ddest;
    wire [$clog2(N>1?N:2)-1:0] sel;
    wire [30:0]         daddress = duser_x[UW-1:USER_W];

    genvar g;
    generate for (g = 0; g < N; g = g + 1) begin : gen_slave_interfaces
        axi4s_slave_if #(.DATA_W(DATA_W), .KEEP_W(KEEP_W), .USER_W(UW), .ID_W(ID_W), .DEST_W(DEST_W)) u_slave_if (
            .aclk(clk), .aresetn(rst_n),
            .tvalid(m_tvalid[g]), .tready(m_tready[g]),
            .tdata(m_tdata[DATA_W*g +: DATA_W]),
            .tkeep(m_tkeep[KEEP_W*g +: KEEP_W]),
            .tstrb(m_tstrb[KEEP_W*g +: KEEP_W]),
            .tuser({m_address[31*g +: 31], m_tuser[USER_W*g +: USER_W]}),
            .tid(m_tid[ID_W*g +: ID_W]), .tdest(m_tdest[DEST_W*g +: DEST_W]),
            .tlast(m_tlast[g]), .tdrw(m_drw[g]), .grant_in(grant[g]),
            .valid(master_valid[g]), .data1(master_data[DATA_W*g +: DATA_W]),
            .data2(), .keepreg(master_keep[KEEP_W*g +: KEEP_W]),
            .strbreg(master_strb[KEEP_W*g +: KEEP_W]),
            .userreg(master_user[UW*g +: UW]),
            .idreg(master_id[ID_W*g +: ID_W]), .destreg(master_dest[DEST_W*g +: DEST_W]),
            .lastreg(master_last[g]), .drwreg(master_drw[g])
        );
    end endgenerate

    axi_interconnect #(.N(N), .DATA_W(DATA_W), .KEEP_W(KEEP_W), .USER_W(UW), .ID_W(ID_W), .DEST_W(DEST_W), .QW(QW)) u_interconnect (
        .aclk(clk), .aresetn(rst_n), .valid_in(master_valid), .data_in(master_data),
        .keep_in(master_keep), .strb_in(master_strb), .user_in(master_user),
        .id_in(master_id), .dest_in(master_dest), .last_in(master_last),
        .drw_in(master_drw), .tqos_in(m_tqos), .grant(grant), .tready(xbar_tready),
        .sel(sel), .dvalid(dvalid), .ddata(ddata), .dkeep(dkeep), .dstrb(dstrb),
        .duser(duser_x), .did(did), .ddest(ddest), .dlast(dlast), .ddrw(ddrw),
        .dwrite(dwrite), .dread(dread), .dready(dready)
    );

    wire         tx_valid_wire, tx_last_wire;
    wire [255:0] tx_data_wire;
    wire         rx_valid_wire, rx_last_wire;
    wire [255:0] rx_data_wire;

    pcie_link_layer #(.REPLAY_LOG2(REPLAY_LOG2), .REPLAY_TMO(REPLAY_TMO), .FLUSH_TMO(FLUSH_TMO)) u_link_layer (
        .aclk(clk), .aresetn(rst_n),
        .s_valid(dvalid), .s_ready(dready),
        .s_address(daddress), .s_drw(ddrw), .s_data(ddata), .s_keep(dkeep),
        .s_strb(dstrb), .s_id(did), .s_dest(ddest), .s_user(duser), .s_last(dlast),
        .tx_valid(tx_valid_wire), .tx_ready(1'b1), .tx_data(tx_data_wire), .tx_last(tx_last_wire),
        .rx_valid(rx_valid_wire), .rx_data(rx_data_wire), .rx_last(rx_last_wire),
        .o_tvalid(o_tvalid), .o_tready(o_tready), .o_tdata(o_tdata), .o_tkeep(o_tkeep),
        .o_tstrb(o_tstrb), .o_tuser(o_tuser), .o_tid(o_tid), .o_tdest(o_tdest),
        .o_tlast(o_tlast), .o_drw(o_drw), .o_address(o_address),
        .st_tx_flits(st_tx_flits), .st_replay(st_replay), .st_rx_flits(st_rx_flits),
        .st_fec_corr(st_fec_corr), .st_crc_fail(st_crc_fail), .st_drop(st_drop)
    );

    pcie6_proto_to_phy #(.PRECODE(PRECODE), .SCRAMBLE(SCRAMBLE)) u_proto_to_phy (
        .clk(clk), .rst_n(rst_n), .flow_data(tx_data_wire), .flow_valid(tx_valid_wire),
        .flow_last(tx_last_wire), .lane0_sym(lane0_sym), .lane1_sym(lane1_sym),
        .lane2_sym(lane2_sym), .lane3_sym(lane3_sym), .phy_valid(phy_valid), .phy_last(phy_last)
    );

    pcie6_phy_to_proto #(.PRECODE(PRECODE), .SCRAMBLE(SCRAMBLE)) u_phy_to_proto (
        .clk(clk), .rst_n(rst_n),
        .lane0_sym(rx_lane0_sym), .lane1_sym(rx_lane1_sym),
        .lane2_sym(rx_lane2_sym), .lane3_sym(rx_lane3_sym),
        .phy_valid(rx_phy_valid), .phy_last(rx_phy_last),
        .flow_data(rx_data_wire), .flow_valid(rx_valid_wire), .flow_last(rx_last_wire)
    );

    assign mon_tx_valid = tx_valid_wire;
    assign mon_tx_data  = tx_data_wire;
    assign mon_tx_last  = tx_last_wire;
endmodule
