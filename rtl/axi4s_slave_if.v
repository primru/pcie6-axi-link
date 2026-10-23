module axi4s_slave_if #(
    parameter DATA_W = 512,
    parameter KEEP_W = 64,
    parameter USER_W = 8,
    parameter ID_W   = 12,
    parameter DEST_W = 8
)(
    input  wire                  aclk,
    input  wire                  aresetn,

    input  wire                  tvalid,
    output wire                  tready,
    input  wire [DATA_W-1:0]     tdata,
    input  wire [KEEP_W-1:0]     tkeep,
    input  wire [KEEP_W-1:0]     tstrb,
    input  wire [USER_W-1:0]     tuser,
    input  wire [ID_W-1:0]       tid,
    input  wire [DEST_W-1:0]     tdest,
    input  wire                  tlast,
    input  wire                  tdrw,      // 1 = write beat, 0 = read beat

    input  wire                  grant_in,

    output wire                  valid,
    output reg  [DATA_W-1:0]     data1,
    output reg  [DATA_W-1:0]     data2,
    output reg  [KEEP_W-1:0]     keepreg,
    output reg  [KEEP_W-1:0]     strbreg,
    output reg  [USER_W-1:0]     userreg,
    output reg  [ID_W-1:0]       idreg,
    output reg  [DEST_W-1:0]     destreg,
    output reg                   lastreg,
    output reg                   drwreg     // registered r/w bit of the held beat
);

    reg                   data1_valid;
    reg                   skid_drw;

    reg                   skid_valid;
    reg  [DATA_W-1:0]     skid_data;
    reg  [KEEP_W-1:0]     skid_keep;
    reg  [KEEP_W-1:0]     skid_strb;
    reg  [USER_W-1:0]     skid_user;
    reg  [ID_W-1:0]       skid_id;
    reg  [DEST_W-1:0]     skid_dest;
    reg                   skid_last;

    assign valid  = data1_valid;
    assign tready = ~skid_valid;

    wire fire_in  = tvalid & tready;
    wire fire_out = data1_valid & grant_in;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            data1_valid  <= 1'b0;
            skid_valid   <= 1'b0;
            data1        <= {DATA_W{1'b0}};
            data2        <= {DATA_W{1'b0}};
            keepreg      <= {KEEP_W{1'b0}};
            strbreg      <= {KEEP_W{1'b0}};
            userreg      <= {USER_W{1'b0}};
            idreg        <= {ID_W{1'b0}};
            destreg      <= {DEST_W{1'b0}};
            lastreg      <= 1'b0;
            drwreg       <= 1'b0;
        end else begin

            if (!data1_valid || fire_out) begin
                if (skid_valid) begin
                    data1       <= skid_data;
                    keepreg     <= skid_keep;
                    strbreg     <= skid_strb;
                    userreg     <= skid_user;
                    idreg       <= skid_id;
                    destreg     <= skid_dest;
                    lastreg     <= skid_last;
                    drwreg      <= skid_drw;
                    data1_valid <= 1'b1;
                    skid_valid  <= 1'b0;
                end else if (fire_in) begin
                    data1       <= tdata;
                    keepreg     <= tkeep;
                    strbreg     <= tstrb;
                    userreg     <= tuser;
                    idreg       <= tid;
                    destreg     <= tdest;
                    lastreg     <= tlast;
                    drwreg      <= tdrw;
                    data1_valid <= 1'b1;
                end else begin
                    data1_valid <= 1'b0;
                end
            end
            else if (fire_in) begin
                data2        <= tdata;
                skid_data    <= tdata;
                skid_keep    <= tkeep;
                skid_strb    <= tstrb;
                skid_user    <= tuser;
                skid_id      <= tid;
                skid_dest    <= tdest;
                skid_last    <= tlast;
                skid_drw     <= tdrw;
                skid_valid   <= 1'b1;
            end
        end
    end

endmodule