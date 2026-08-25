`timescale 1ns / 1ps

module axi4_lite_slave #(
    parameter ADDR_WIDTH = 5,
    parameter DATA_WIDTH = 32
)(
    input wire aclk,
    input wire aresetn,
    input wire [ADDR_WIDTH-1:0] awaddr,
    input wire awvalid,
    output reg awready,
    input wire [DATA_WIDTH-1:0] wdata,
    input wire [(DATA_WIDTH/8)-1:0] wstrb,
    input wire wvalid,
    output reg wready,
    output reg [1:0] bresp,
    output reg bvalid,
    input wire bready,
    input wire [ADDR_WIDTH-1:0] araddr,
    input wire arvalid,
    output reg arready,
    output reg [DATA_WIDTH-1:0] rdata,
    output reg [1:0] rresp,
    output reg rvalid,
    input wire rready,
    output reg [15:0] c0,
    output reg [15:0] c1,
    output reg [15:0] c2,
    output reg [15:0] c3
);

    reg aw_en;
    reg [15:0] shadow_c0, shadow_c1, shadow_c2, shadow_c3;

    always @(posedge aclk) begin
        if (~aresetn) begin
            awready <= 1'b0;
            aw_en <= 1'b1;
        end else begin
            if (~awready && awvalid && wvalid && aw_en) begin
                awready <= 1'b1;
                aw_en <= 1'b0;
            end else if (bready && bvalid) begin
                aw_en <= 1'b1;
                awready <= 1'b0;
            end else begin
                awready <= 1'b0;
            end
        end
    end

    always @(posedge aclk) begin
        if (~aresetn) begin
            wready <= 1'b0;
        end else begin
            if (~wready && wvalid && awvalid && aw_en) begin
                wready <= 1'b1;
            end else begin
                wready <= 1'b0;
            end
        end
    end

    always @(posedge aclk) begin
        if (~aresetn) begin
            c0 <= 16'h0000;
            c1 <= 16'h0000;
            c2 <= 16'h0000;
            c3 <= 16'h0000;
            shadow_c0 <= 16'h0000;
            shadow_c1 <= 16'h0000;
            shadow_c2 <= 16'h0000;
            shadow_c3 <= 16'h0000;
        end else begin
            if (wready && wvalid && awready && awvalid) begin
                case (awaddr)
                    5'h00: if (wstrb[0]) shadow_c0 <= wdata[15:0];
                    5'h04: if (wstrb[0]) shadow_c1 <= wdata[15:0];
                    5'h08: if (wstrb[0]) shadow_c2 <= wdata[15:0];
                    5'h0C: if (wstrb[0]) shadow_c3 <= wdata[15:0];
                    5'h10: begin
                        if (wdata[0] == 1'b1) begin
                            c0 <= shadow_c0;
                            c1 <= shadow_c1;
                            c2 <= shadow_c2;
                            c3 <= shadow_c3;
                        end
                    end
                endcase
            end
        end
    end

    always @(posedge aclk) begin
        if (~aresetn) begin
            bvalid <= 1'b0;
            bresp <= 2'b00;
        end else begin
            if (awready && awvalid && ~bvalid && wready && wvalid) begin
                bvalid <= 1'b1;
                bresp <= 2'b00;
            end else if (bready && bvalid) begin
                bvalid <= 1'b0;
            end
        end
    end

    always @(posedge aclk) begin
        if (~aresetn) begin
            arready <= 1'b0;
        end else begin
            if (~arready && arvalid) begin
                arready <= 1'b1;
            end else begin
                arready <= 1'b0;
            end
        end
    end

    always @(posedge aclk) begin
        if (~aresetn) begin
            rvalid <= 1'b0;
            rresp <= 2'b00;
        end else begin
            if (arready && arvalid && ~rvalid) begin
                rvalid <= 1'b1;
                rresp <= 2'b00;
            end else if (rvalid && rready) begin
                rvalid <= 1'b0;
            end
        end
    end

    always @(posedge aclk) begin
        if (~aresetn) begin
            rdata <= 0;
        end else begin
            if (arready && arvalid && ~rvalid) begin
                case (araddr)
                    5'h00: rdata <= {16'd0, c0};
                    5'h04: rdata <= {16'd0, c1};
                    5'h08: rdata <= {16'd0, c2};
                    5'h0C: rdata <= {16'd0, c3};
                    default: rdata <= 32'd0;
                endcase
            end
        end
    end
endmodule

module axi_stream_fir #(
    parameter D_WIDTH = 16,
    parameter C_WIDTH = 16
)(
    input  wire clk,
    input  wire rst_n,
    input  wire signed [C_WIDTH-1:0] c0,
    input  wire signed [C_WIDTH-1:0] c1,
    input  wire signed [C_WIDTH-1:0] c2,
    input  wire signed [C_WIDTH-1:0] c3,
    input  wire [D_WIDTH-1:0] s_axis_tdata,
    input  wire s_axis_tvalid,
    output wire s_axis_tready,
    output wire [D_WIDTH+C_WIDTH+1:0] m_axis_tdata,
    output wire m_axis_tvalid,
    input  wire m_axis_tready
);

    wire signed [C_WIDTH-1:0] coeff [0:3];
    assign coeff[0] = c0;
    assign coeff[1] = c1;
    assign coeff[2] = c2;
    assign coeff[3] = c3;

    reg signed [D_WIDTH-1:0] shift_reg [0:3];
    reg signed [D_WIDTH+C_WIDTH-1:0] mult_reg [0:3];
    reg signed [D_WIDTH+C_WIDTH:0] add_stg1_0;
    reg signed [D_WIDTH+C_WIDTH:0] add_stg1_1;
    reg signed [D_WIDTH+C_WIDTH+1:0] add_stg2;

    reg [3:0] valid_pipe;
    wire stall;
    
    assign stall = !m_axis_tready;
    assign s_axis_tready = m_axis_tready;

    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < 4; i = i + 1) begin
                shift_reg[i] <= 0;
                mult_reg[i] <= 0;
            end
            add_stg1_0 <= 0;
            add_stg1_1 <= 0;
            add_stg2 <= 0;
            valid_pipe <= 0;
        end else if (!stall) begin
            if (s_axis_tvalid) begin
                shift_reg[0] <= $signed(s_axis_tdata);
                for (i = 1; i < 4; i = i + 1) begin
                    shift_reg[i] <= shift_reg[i-1];
                end
            end

            for (i = 0; i < 4; i = i + 1) begin
                mult_reg[i] <= shift_reg[i] * coeff[i];
            end

            add_stg1_0 <= mult_reg[0] + mult_reg[1];
            add_stg1_1 <= mult_reg[2] + mult_reg[3];
            add_stg2 <= add_stg1_0 + add_stg1_1;
            
            valid_pipe <= {valid_pipe[2:0], s_axis_tvalid};
        end
    end

    assign m_axis_tdata = add_stg2;
    assign m_axis_tvalid = valid_pipe[3];
endmodule

module top_axi_fir_ip (
    input wire aclk,
    input wire aresetn,
    
    input wire [4:0] s_axi_awaddr,
    input wire s_axi_awvalid,
    output wire s_axi_awready,
    input wire [31:0] s_axi_wdata,
    input wire [3:0] s_axi_wstrb,
    input wire s_axi_wvalid,
    output wire s_axi_wready,
    output wire [1:0] s_axi_bresp,
    output wire s_axi_bvalid,
    input wire s_axi_bready,
    input wire [4:0] s_axi_araddr,
    input wire s_axi_arvalid,
    output wire s_axi_arready,
    output wire [31:0] s_axi_rdata,
    output wire [1:0] s_axi_rresp,
    output wire s_axi_rvalid,
    input wire s_axi_rready,

    input wire [15:0] s_axis_tdata,
    input wire s_axis_tvalid,
    output wire s_axis_tready,
    
    output wire [33:0] m_axis_tdata,
    output wire m_axis_tvalid,
    input wire m_axis_tready
);

    wire [15:0] c0_int;
    wire [15:0] c1_int;
    wire [15:0] c2_int;
    wire [15:0] c3_int;

    axi4_lite_slave #(
        .ADDR_WIDTH(5),
        .DATA_WIDTH(32)
    ) axi_ctrl (
        .aclk(aclk),
        .aresetn(aresetn),
        .awaddr(s_axi_awaddr),
        .awvalid(s_axi_awvalid),
        .awready(s_axi_awready),
        .wdata(s_axi_wdata),
        .wstrb(s_axi_wstrb),
        .wvalid(s_axi_wvalid),
        .wready(s_axi_wready),
        .bresp(s_axi_bresp),
        .bvalid(s_axi_bvalid),
        .bready(s_axi_bready),
        .araddr(s_axi_araddr),
        .arvalid(s_axi_arvalid),
        .arready(s_axi_arready),
        .rdata(s_axi_rdata),
        .rresp(s_axi_rresp),
        .rvalid(s_axi_rvalid),
        .rready(s_axi_rready),
        .c0(c0_int),
        .c1(c1_int),
        .c2(c2_int),
        .c3(c3_int)
    );

    axi_stream_fir #(
        .D_WIDTH(16),
        .C_WIDTH(16)
    ) datapath (
        .clk(aclk),
        .rst_n(aresetn),
        .c0(c0_int),
        .c1(c1_int),
        .c2(c2_int),
        .c3(c3_int),
        .s_axis_tdata(s_axis_tdata),
        .s_axis_tvalid(s_axis_tvalid),
        .s_axis_tready(s_axis_tready),
        .m_axis_tdata(m_axis_tdata),
        .m_axis_tvalid(m_axis_tvalid),
        .m_axis_tready(m_axis_tready)
    );

endmodule