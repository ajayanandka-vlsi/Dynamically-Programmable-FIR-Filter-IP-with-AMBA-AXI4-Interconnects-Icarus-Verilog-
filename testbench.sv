`timescale 1ns / 1ps

module tb_top;

    reg aclk;
    reg aresetn;

    reg [4:0] awaddr;
    reg awvalid;
    wire awready;
    reg [31:0] wdata;
    reg [3:0] wstrb;
    reg wvalid;
    wire wready;
    wire [1:0] bresp;
    wire bvalid;
    reg bready;
    reg [4:0] araddr;
    reg arvalid;
    wire arready;
    wire [31:0] rdata;
    wire [1:0] rresp;
    wire rvalid;
    reg rready;

    reg [15:0] s_axis_tdata;
    reg s_axis_tvalid;
    wire s_axis_tready;
    
    wire [33:0] m_axis_tdata;
    wire m_axis_tvalid;
    reg m_axis_tready;

    top_axi_fir_ip dut (
        .aclk(aclk),
        .aresetn(aresetn),
        .s_axi_awaddr(awaddr),
        .s_axi_awvalid(awvalid),
        .s_axi_awready(awready),
        .s_axi_wdata(wdata),
        .s_axi_wstrb(wstrb),
        .s_axi_wvalid(wvalid),
        .s_axi_wready(wready),
        .s_axi_bresp(bresp),
        .s_axi_bvalid(bvalid),
        .s_axi_bready(bready),
        .s_axi_araddr(araddr),
        .s_axi_arvalid(arvalid),
        .s_axi_arready(arready),
        .s_axi_rdata(rdata),
        .s_axi_rresp(rresp),
        .s_axi_rvalid(rvalid),
        .s_axi_rready(rready),
        .s_axis_tdata(s_axis_tdata),
        .s_axis_tvalid(s_axis_tvalid),
        .s_axis_tready(s_axis_tready),
        .m_axis_tdata(m_axis_tdata),
        .m_axis_tvalid(m_axis_tvalid),
        .m_axis_tready(m_axis_tready)
    );

    always #5 aclk = ~aclk;

    task axi_write;
        input [4:0] wr_addr;
        input [31:0] wr_data;
        begin
            @(posedge aclk);
            awaddr <= wr_addr;
            awvalid <= 1;
            wdata <= wr_data;
            wstrb <= 4'hF;
            wvalid <= 1;
            bready <= 1;
            
            while (!(awready && wready)) @(posedge aclk);
            
            awvalid <= 0;
            wvalid <= 0;
            
            while (!bvalid) @(posedge aclk);
            
            bready <= 0;
        end
    endtask

    initial begin
        $dumpfile("top_axi_fir.vcd");
        $dumpvars(0, tb_top);
        
        aclk = 0;
        aresetn = 0;
        awaddr = 0; awvalid = 0; wdata = 0; wstrb = 0; wvalid = 0; bready = 0;
        araddr = 0; arvalid = 0; rready = 0;
        s_axis_tdata = 0; s_axis_tvalid = 0; m_axis_tready = 1;

        #20;
        aresetn = 1;
        #20;

        axi_write(5'h00, 32'h00001000);
        axi_write(5'h04, 32'h00002000);
        axi_write(5'h08, 32'h00002000);
        axi_write(5'h0C, 32'h00001000);
        axi_write(5'h10, 32'h00000001);
        
        #50;
        
        @(posedge aclk);
        s_axis_tvalid <= 1;
        s_axis_tdata <= 16'd100;
        
        @(posedge aclk);
        s_axis_tdata <= 16'd0;
        
        @(posedge aclk);
        s_axis_tvalid <= 0;
        
        #100;
        
        @(posedge aclk);
        s_axis_tvalid <= 1;
        s_axis_tdata <= 16'd50;
        
        @(posedge aclk);
        s_axis_tdata <= -16'd25;
        
        @(posedge aclk);
        s_axis_tvalid <= 0;

        #150;
        $finish;
    end
endmodule