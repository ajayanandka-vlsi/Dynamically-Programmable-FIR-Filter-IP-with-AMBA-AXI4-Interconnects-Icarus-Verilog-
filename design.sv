`timescale 1ns / 1ps
// =============================================================================
// Dynamically programmable 4-tap FIR filter IP
//   * AXI4-Lite slave  : coefficient registers (double-buffered) + commit
//   * AXI4-Stream      : 16-bit signed samples in, 34-bit signed results out
//
// Register map (byte addresses, 32-bit registers)
//   0x00  C0  [15:0] R/W  shadow coefficient 0 (reads return the shadow value)
//   0x04  C1  [15:0] R/W  shadow coefficient 1
//   0x08  C2  [15:0] R/W  shadow coefficient 2
//   0x0C  C3  [15:0] R/W  shadow coefficient 3
//   0x10  CTRL  [0]   W   write 1 -> copy all four shadows into the active
//                         coefficients in ONE clock edge (reads return 0)
//   any other address  -> SLVERR (2'b10), no register changes
//
// Changes compared with v1
//   1. WSTRB honoured per byte (v1 checked only wstrb[0] but wrote 2 bytes)
//   2. Unmapped addresses answer SLVERR instead of a silent OKAY
//   3. Coefficient reads return the shadow (what the CPU last wrote)
//   4. A new read address is accepted only when the previous read data has
//      been taken (v1 could accept a 2nd read while RVALID was still high)
//   5. FIR stalls only when its OUTPUT holds valid data the sink is not taking
//      (v1 froze the whole pipeline whenever m_axis_tready was low, even if
//      the pipeline was empty)
//   6. One reset style everywhere: synchronous, active-low (AXI convention)
// =============================================================================

module axi4_lite_slave #(
    parameter ADDR_WIDTH = 5,
    parameter DATA_WIDTH = 32
)(
    input  wire                      aclk,
    input  wire                      aresetn,
    input  wire [ADDR_WIDTH-1:0]     awaddr,
    input  wire                      awvalid,
    output reg                       awready,
    input  wire [DATA_WIDTH-1:0]     wdata,
    input  wire [(DATA_WIDTH/8)-1:0] wstrb,
    input  wire                      wvalid,
    output reg                       wready,
    output reg  [1:0]                bresp,
    output reg                       bvalid,
    input  wire                      bready,
    input  wire [ADDR_WIDTH-1:0]     araddr,
    input  wire                      arvalid,
    output reg                       arready,
    output reg  [DATA_WIDTH-1:0]     rdata,
    output reg  [1:0]                rresp,
    output reg                       rvalid,
    input  wire                      rready,
    output reg  [15:0]               c0,
    output reg  [15:0]               c1,
    output reg  [15:0]               c2,
    output reg  [15:0]               c3
);

    localparam RESP_OKAY   = 2'b00;
    localparam RESP_SLVERR = 2'b10;

    reg        aw_en;
    reg [15:0] shadow_c0, shadow_c1, shadow_c2, shadow_c3;

    // A write happens on the edge where both the AW and W handshakes complete
    wire wr_fire = awready && awvalid && wready && wvalid;
    // A read address is accepted on the edge where AR handshakes
    wire rd_fire = arready && arvalid;

    // Address decode: only 0x00, 0x04, 0x08, 0x0C, 0x10 exist
    wire wr_addr_ok = (awaddr == 5'h00) || (awaddr == 5'h04) || (awaddr == 5'h08) ||
                      (awaddr == 5'h0C) || (awaddr == 5'h10);
    wire rd_addr_ok = (araddr == 5'h00) || (araddr == 5'h04) || (araddr == 5'h08) ||
                      (araddr == 5'h0C) || (araddr == 5'h10);

    // ---------------- write address ready ----------------
    // aw_en blocks a new write until the previous write response is accepted
    always @(posedge aclk) begin
        if (!aresetn) begin
            awready <= 1'b0;
            aw_en   <= 1'b1;
        end else begin
            if (!awready && awvalid && wvalid && aw_en) begin
                awready <= 1'b1;
                aw_en   <= 1'b0;
            end else if (bready && bvalid) begin
                aw_en   <= 1'b1;
                awready <= 1'b0;
            end else begin
                awready <= 1'b0;
            end
        end
    end

    // ---------------- write data ready ----------------
    always @(posedge aclk) begin
        if (!aresetn) begin
            wready <= 1'b0;
        end else begin
            if (!wready && wvalid && awvalid && aw_en)
                wready <= 1'b1;
            else
                wready <= 1'b0;
        end
    end

    // ---------------- register writes (byte strobes honoured) ----------------
    always @(posedge aclk) begin
        if (!aresetn) begin
            c0 <= 16'h0000; c1 <= 16'h0000; c2 <= 16'h0000; c3 <= 16'h0000;
            shadow_c0 <= 16'h0000; shadow_c1 <= 16'h0000;
            shadow_c2 <= 16'h0000; shadow_c3 <= 16'h0000;
        end else if (wr_fire) begin
            case (awaddr)
                5'h00: begin
                    if (wstrb[0]) shadow_c0[7:0]  <= wdata[7:0];
                    if (wstrb[1]) shadow_c0[15:8] <= wdata[15:8];
                end
                5'h04: begin
                    if (wstrb[0]) shadow_c1[7:0]  <= wdata[7:0];
                    if (wstrb[1]) shadow_c1[15:8] <= wdata[15:8];
                end
                5'h08: begin
                    if (wstrb[0]) shadow_c2[7:0]  <= wdata[7:0];
                    if (wstrb[1]) shadow_c2[15:8] <= wdata[15:8];
                end
                5'h0C: begin
                    if (wstrb[0]) shadow_c3[7:0]  <= wdata[7:0];
                    if (wstrb[1]) shadow_c3[15:8] <= wdata[15:8];
                end
                5'h10: begin
                    // commit: all four active coefficients change on the same edge
                    if (wstrb[0] && wdata[0]) begin
                        c0 <= shadow_c0;
                        c1 <= shadow_c1;
                        c2 <= shadow_c2;
                        c3 <= shadow_c3;
                    end
                end
                default: ; // unmapped: no change (answered with SLVERR below)
            endcase
        end
    end

    // ---------------- write response ----------------
    always @(posedge aclk) begin
        if (!aresetn) begin
            bvalid <= 1'b0;
            bresp  <= RESP_OKAY;
        end else begin
            if (wr_fire && !bvalid) begin
                bvalid <= 1'b1;
                bresp  <= wr_addr_ok ? RESP_OKAY : RESP_SLVERR;
            end else if (bready && bvalid) begin
                bvalid <= 1'b0;
            end
        end
    end

    // ---------------- read address ready ----------------
    // Accept a new read only when no read data is waiting to be taken
    always @(posedge aclk) begin
        if (!aresetn) begin
            arready <= 1'b0;
        end else begin
            if (!arready && arvalid && !rvalid)
                arready <= 1'b1;
            else
                arready <= 1'b0;
        end
    end

    // ---------------- read data / response ----------------
    always @(posedge aclk) begin
        if (!aresetn) begin
            rvalid <= 1'b0;
            rresp  <= RESP_OKAY;
            rdata  <= {DATA_WIDTH{1'b0}};
        end else begin
            if (rd_fire && !rvalid) begin
                rvalid <= 1'b1;
                rresp  <= rd_addr_ok ? RESP_OKAY : RESP_SLVERR;
                case (araddr)
                    5'h00:   rdata <= {16'd0, shadow_c0};
                    5'h04:   rdata <= {16'd0, shadow_c1};
                    5'h08:   rdata <= {16'd0, shadow_c2};
                    5'h0C:   rdata <= {16'd0, shadow_c3};
                    default: rdata <= {DATA_WIDTH{1'b0}};   // 0x10 and unmapped read 0
                endcase
            end else if (rvalid && rready) begin
                rvalid <= 1'b0;
            end
        end
    end
endmodule


module axi_stream_fir #(
    parameter D_WIDTH = 16,
    parameter C_WIDTH = 16
)(
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire signed [C_WIDTH-1:0]    c0,
    input  wire signed [C_WIDTH-1:0]    c1,
    input  wire signed [C_WIDTH-1:0]    c2,
    input  wire signed [C_WIDTH-1:0]    c3,
    input  wire [D_WIDTH-1:0]           s_axis_tdata,
    input  wire                         s_axis_tvalid,
    output wire                         s_axis_tready,
    output wire [D_WIDTH+C_WIDTH+1:0]   m_axis_tdata,
    output wire                         m_axis_tvalid,
    input  wire                         m_axis_tready
);

    wire signed [C_WIDTH-1:0] coeff [0:3];
    assign coeff[0] = c0;
    assign coeff[1] = c1;
    assign coeff[2] = c2;
    assign coeff[3] = c3;

    reg signed [D_WIDTH-1:0]         shift_reg [0:3];   // x[n], x[n-1], x[n-2], x[n-3]
    reg signed [D_WIDTH+C_WIDTH-1:0] mult_reg  [0:3];   // 16x16 -> 32 bits
    reg signed [D_WIDTH+C_WIDTH:0]   add_stg1_0;        // 33 bits
    reg signed [D_WIDTH+C_WIDTH:0]   add_stg1_1;
    reg signed [D_WIDTH+C_WIDTH+1:0] add_stg2;          // 34 bits: no overflow possible

    reg [3:0] valid_pipe;   // valid bit travelling with each pipeline stage

    // Freeze the pipeline only when the output holds a VALID result that the
    // sink is not accepting. Empty stages (bubbles) keep moving.
    wire stall    = m_axis_tvalid && !m_axis_tready;
    wire in_fire  = s_axis_tvalid && !stall;       // input handshake

    assign s_axis_tready = !stall;

    integer i;

    always @(posedge clk) begin
        if (!rst_n) begin
            for (i = 0; i < 4; i = i + 1) begin
                shift_reg[i] <= 0;
                mult_reg[i]  <= 0;
            end
            add_stg1_0 <= 0;
            add_stg1_1 <= 0;
            add_stg2   <= 0;
            valid_pipe <= 4'b0000;
        end else if (!stall) begin
            // Stage 1: shift in a new sample only when one is transferred
            if (in_fire) begin
                shift_reg[0] <= $signed(s_axis_tdata);
                for (i = 1; i < 4; i = i + 1)
                    shift_reg[i] <= shift_reg[i-1];
            end
            // Stage 2: four multiplies in parallel
            for (i = 0; i < 4; i = i + 1)
                mult_reg[i] <= shift_reg[i] * coeff[i];
            // Stages 3 and 4: adder tree
            add_stg1_0 <= mult_reg[0] + mult_reg[1];
            add_stg1_1 <= mult_reg[2] + mult_reg[3];
            add_stg2   <= add_stg1_0 + add_stg1_1;

            valid_pipe <= {valid_pipe[2:0], in_fire};
        end
    end

    assign m_axis_tdata  = add_stg2;
    assign m_axis_tvalid = valid_pipe[3];
endmodule


module top_axi_fir_ip (
    input  wire        aclk,
    input  wire        aresetn,

    input  wire [4:0]  s_axi_awaddr,
    input  wire        s_axi_awvalid,
    output wire        s_axi_awready,
    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    input  wire        s_axi_wvalid,
    output wire        s_axi_wready,
    output wire [1:0]  s_axi_bresp,
    output wire        s_axi_bvalid,
    input  wire        s_axi_bready,
    input  wire [4:0]  s_axi_araddr,
    input  wire        s_axi_arvalid,
    output wire        s_axi_arready,
    output wire [31:0] s_axi_rdata,
    output wire [1:0]  s_axi_rresp,
    output wire        s_axi_rvalid,
    input  wire        s_axi_rready,

    input  wire [15:0] s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,

    output wire [33:0] m_axis_tdata,
    output wire        m_axis_tvalid,
    input  wire        m_axis_tready
);

    wire [15:0] c0_int, c1_int, c2_int, c3_int;

    axi4_lite_slave #(
        .ADDR_WIDTH(5),
        .DATA_WIDTH(32)
    ) axi_ctrl (
        .aclk(aclk),        .aresetn(aresetn),
        .awaddr(s_axi_awaddr), .awvalid(s_axi_awvalid), .awready(s_axi_awready),
        .wdata(s_axi_wdata),   .wstrb(s_axi_wstrb),     .wvalid(s_axi_wvalid), .wready(s_axi_wready),
        .bresp(s_axi_bresp),   .bvalid(s_axi_bvalid),   .bready(s_axi_bready),
        .araddr(s_axi_araddr), .arvalid(s_axi_arvalid), .arready(s_axi_arready),
        .rdata(s_axi_rdata),   .rresp(s_axi_rresp),     .rvalid(s_axi_rvalid), .rready(s_axi_rready),
        .c0(c0_int), .c1(c1_int), .c2(c2_int), .c3(c3_int)
    );

    axi_stream_fir #(
        .D_WIDTH(16),
        .C_WIDTH(16)
    ) datapath (
        .clk(aclk),          .rst_n(aresetn),
        .c0(c0_int), .c1(c1_int), .c2(c2_int), .c3(c3_int),
        .s_axis_tdata(s_axis_tdata),   .s_axis_tvalid(s_axis_tvalid), .s_axis_tready(s_axis_tready),
        .m_axis_tdata(m_axis_tdata),   .m_axis_tvalid(m_axis_tvalid), .m_axis_tready(m_axis_tready)
    );

endmodule
