`timescale 1ns / 1ps
// =============================================================================
// Self-checking testbench for top_axi_fir_ip
//   1. Register tests : write/read-back, byte strobes, SLVERR on bad addresses
//   2. Stream test A  : 300 samples, random gaps on the input, random
//                       back-pressure on the output, CPU writing NEW shadow
//                       coefficients at the same time (must NOT affect output)
//   3. Commit         : once the pipeline is drained, commit coefficient set B
//   4. Stream test B  : 300 more samples with set B (incl. negative coeffs)
//   A reference model computes every expected output; a checker also verifies
//   that m_axis_tdata/tvalid stay stable while the sink holds tready low.
// =============================================================================
module tb_top;

    // ------------------------------------------------------------ DUT signals
    reg         aclk;
    reg         aresetn;

    reg  [4:0]  awaddr;
    reg         awvalid;
    wire        awready;
    reg  [31:0] wdata;
    reg  [3:0]  wstrb;
    reg         wvalid;
    wire        wready;
    wire [1:0]  bresp;
    wire        bvalid;
    reg         bready;
    reg  [4:0]  araddr;
    reg         arvalid;
    wire        arready;
    wire [31:0] rdata;
    wire [1:0]  rresp;
    wire        rvalid;
    reg         rready;

    reg  [15:0] s_axis_tdata;
    reg         s_axis_tvalid;
    wire        s_axis_tready;
    wire [33:0] m_axis_tdata;
    wire        m_axis_tvalid;
    reg         m_axis_tready;

    top_axi_fir_ip dut (
        .aclk(aclk), .aresetn(aresetn),
        .s_axi_awaddr(awaddr), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
        .s_axi_wdata(wdata),   .s_axi_wstrb(wstrb),     .s_axi_wvalid(wvalid), .s_axi_wready(wready),
        .s_axi_bresp(bresp),   .s_axi_bvalid(bvalid),   .s_axi_bready(bready),
        .s_axi_araddr(araddr), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
        .s_axi_rdata(rdata),   .s_axi_rresp(rresp),     .s_axi_rvalid(rvalid), .s_axi_rready(rready),
        .s_axis_tdata(s_axis_tdata),   .s_axis_tvalid(s_axis_tvalid), .s_axis_tready(s_axis_tready),
        .m_axis_tdata(m_axis_tdata),   .m_axis_tvalid(m_axis_tvalid), .m_axis_tready(m_axis_tready)
    );

    always #5 aclk = ~aclk;   // 100 MHz

    // ------------------------------------------------------------ bookkeeping
    integer errors;
    integer to_send;      // samples the stream driver may present (set by the main sequence)
    integer sent;         // samples presented so far
    integer tx_count;     // samples accepted by the DUT
    integer rx_count;     // results accepted from the DUT
    reg     bp_on;        // random back-pressure on m_axis_tready
    integer seed;

    // reference model state
    reg signed [15:0] mc0, mc1, mc2, mc3;     // ACTIVE coefficients as the DUT should use them
    reg signed [15:0] x0, x1, x2, x3;         // sample history (newest first)
    reg signed [63:0] exp_mem [0:4095];       // expected results, in order

    // ------------------------------------------------------------ AXI-Lite tasks
    reg [1:0]  t_resp;
    reg [31:0] t_data;
    reg        aw_done, w_done, b_done, ar_done, r_done;

    task axi_write;
        input [4:0]  addr;
        input [31:0] data;
        input [3:0]  strb;
        begin
            @(posedge aclk);
            awaddr <= addr;  awvalid <= 1'b1;
            wdata  <= data;  wstrb   <= strb;  wvalid <= 1'b1;
            bready <= 1'b1;
            aw_done = 0; w_done = 0; b_done = 0;
            while (!(aw_done && w_done)) begin
                @(posedge aclk);
                if (awvalid && awready) begin aw_done = 1; awvalid <= 1'b0; end
                if (wvalid  && wready)  begin w_done  = 1; wvalid  <= 1'b0; end
            end
            while (!b_done) begin
                @(posedge aclk);
                if (bvalid && bready) begin b_done = 1; t_resp = bresp; bready <= 1'b0; end
            end
        end
    endtask

    task axi_read;
        input [4:0] addr;
        begin
            @(posedge aclk);
            araddr <= addr; arvalid <= 1'b1; rready <= 1'b1;
            ar_done = 0; r_done = 0;
            while (!ar_done) begin
                @(posedge aclk);
                if (arvalid && arready) begin ar_done = 1; arvalid <= 1'b0; end
            end
            while (!r_done) begin
                @(posedge aclk);
                if (rvalid && rready) begin r_done = 1; t_data = rdata; t_resp = rresp; rready <= 1'b0; end
            end
        end
    endtask

    task check_read;      // read addr and compare data + response
        input [4:0]  addr;
        input [31:0] exp_data;
        input [1:0]  exp_resp;
        begin
            axi_read(addr);
            if (t_data !== exp_data || t_resp !== exp_resp) begin
                errors = errors + 1;
                $display("ERROR read  @%0t addr=%h data=%h resp=%b (expected %h / %b)",
                         $time, addr, t_data, t_resp, exp_data, exp_resp);
            end
        end
    endtask

    task check_write;     // write and compare the response
        input [4:0]  addr;
        input [31:0] data;
        input [3:0]  strb;
        input [1:0]  exp_resp;
        begin
            axi_write(addr, data, strb);
            if (t_resp !== exp_resp) begin
                errors = errors + 1;
                $display("ERROR write @%0t addr=%h bresp=%b (expected %b)", $time, addr, t_resp, exp_resp);
            end
        end
    endtask

    task write_shadows;   // write 4 coefficients (full strobes), no commit
        input [15:0] a0, a1, a2, a3;
        begin
            check_write(5'h00, {16'd0, a0}, 4'hF, 2'b00);
            check_write(5'h04, {16'd0, a1}, 4'hF, 2'b00);
            check_write(5'h08, {16'd0, a2}, 4'hF, 2'b00);
            check_write(5'h0C, {16'd0, a3}, 4'hF, 2'b00);
        end
    endtask

    task wait_drained;    // all requested samples sent, accepted and their results received
        begin
            while (!(sent == to_send && !s_axis_tvalid && rx_count == tx_count))
                @(posedge aclk);
            repeat (5) @(posedge aclk);
        end
    endtask

    // ------------------------------------------------------------ stream driver + reference model
    always @(posedge aclk) begin
        if (!aresetn) begin
            s_axis_tvalid <= 1'b0;
            s_axis_tdata  <= 16'd0;
        end else begin
            // 1) a sample is transferred on this edge -> update the model
            if (s_axis_tvalid && s_axis_tready) begin
                x3 = x2; x2 = x1; x1 = x0; x0 = s_axis_tdata;
                exp_mem[tx_count] = mc0 * x0 + mc1 * x1 + mc2 * x2 + mc3 * x3;
                tx_count = tx_count + 1;
            end
            // 2) choose what to present next (only if the current beat is finished)
            if (!s_axis_tvalid || s_axis_tready) begin
                if (sent < to_send && ($random(seed) % 4 != 0)) begin
                    s_axis_tvalid <= 1'b1;
                    s_axis_tdata  <= $random(seed);
                    sent = sent + 1;
                end else begin
                    s_axis_tvalid <= 1'b0;
                end
            end
        end
    end

    // ------------------------------------------------------------ sink: random back-pressure
    always @(posedge aclk) begin
        if (!aresetn) m_axis_tready <= 1'b1;
        else          m_axis_tready <= bp_on ? ($random(seed) % 3 != 0) : 1'b1;
    end

    // ------------------------------------------------------------ output checker
    reg        prev_stall;
    reg [33:0] prev_data;
    always @(posedge aclk) begin
        if (!aresetn) begin
            prev_stall <= 1'b0;
        end else begin
            // AXI-Stream rule: once TVALID is high it must stay high with stable data until TREADY
            if (prev_stall && (!m_axis_tvalid || m_axis_tdata !== prev_data)) begin
                errors = errors + 1;
                $display("ERROR @%0t output changed while stalled", $time);
            end
            prev_stall <= m_axis_tvalid && !m_axis_tready;
            prev_data  <= m_axis_tdata;

            if (m_axis_tvalid && m_axis_tready) begin
                if (rx_count >= tx_count) begin
                    errors = errors + 1;
                    $display("ERROR @%0t output without a matching input", $time);
                end else if (m_axis_tdata !== exp_mem[rx_count][33:0]) begin
                    errors = errors + 1;
                    $display("ERROR @%0t result #%0d = %0d, expected %0d", $time, rx_count,
                             $signed(m_axis_tdata), exp_mem[rx_count]);
                end
                rx_count = rx_count + 1;
            end
        end
    end

    // ------------------------------------------------------------ watchdog
    initial begin
        #2_000_000;
        $display("ERROR: timeout");
        $display("STATUS: SIMULATION FAIL (timeout)");
        $finish;
    end

    // ------------------------------------------------------------ main sequence
    initial begin
        $dumpfile("top_axi_fir.vcd");
        $dumpvars(0, tb_top);

        seed = 1;
        errors = 0; to_send = 0; sent = 0; tx_count = 0; rx_count = 0; bp_on = 1'b0;
        mc0 = 0; mc1 = 0; mc2 = 0; mc3 = 0;
        x0 = 0; x1 = 0; x2 = 0; x3 = 0;

        aclk = 0;
        aresetn = 0;
        awaddr = 0; awvalid = 0; wdata = 0; wstrb = 0; wvalid = 0; bready = 0;
        araddr = 0; arvalid = 0; rready = 0;
        s_axis_tvalid = 0; s_axis_tdata = 0; m_axis_tready = 1;
        repeat (4) @(posedge aclk);
        aresetn <= 1'b1;
        repeat (2) @(posedge aclk);

        // ---------- 1. register tests ----------
        $display("[%0t] register tests", $time);
        write_shadows(16'h1000, 16'h2000, 16'h2000, 16'h1000);
        check_read(5'h00, 32'h0000_1000, 2'b00);
        check_read(5'h04, 32'h0000_2000, 2'b00);
        check_read(5'h08, 32'h0000_2000, 2'b00);
        check_read(5'h0C, 32'h0000_1000, 2'b00);
        check_read(5'h10, 32'h0000_0000, 2'b00);       // CTRL reads 0
        // byte strobes: change only byte 0, then only byte 1 of C0
        check_write(5'h00, 32'h0000_00AB, 4'b0001, 2'b00);
        check_read (5'h00, 32'h0000_10AB, 2'b00);
        check_write(5'h00, 32'h0000_CD00, 4'b0010, 2'b00);
        check_read (5'h00, 32'h0000_CDAB, 2'b00);
        // unmapped addresses -> SLVERR, and nothing changes
        check_write(5'h14, 32'hDEAD_BEEF, 4'hF, 2'b10);
        check_read (5'h14, 32'h0000_0000, 2'b10);
        check_write(5'h02, 32'h0000_FFFF, 4'hF, 2'b10);   // misaligned
        check_read (5'h00, 32'h0000_CDAB, 2'b00);

        // ---------- commit coefficient set A ----------
        write_shadows(16'h1000, 16'h2000, 16'h2000, 16'h1000);
        check_write(5'h10, 32'h1, 4'hF, 2'b00);
        mc0 = 16'h1000; mc1 = 16'h2000; mc2 = 16'h2000; mc3 = 16'h1000;

        // ---------- 2. stream A with back-pressure + concurrent CPU writes ----------
        $display("[%0t] stream A: gaps + back-pressure + shadow writes during streaming", $time);
        bp_on = 1'b1;
        to_send = to_send + 300;
        repeat (50) @(posedge aclk);
        // CPU writes a NEW set into the shadows while data flows: outputs must not change
        write_shadows(16'hF800, 16'h4000, 16'h7FFF, 16'h8000);
        check_read(5'h08, 32'h0000_7FFF, 2'b00);
        wait_drained;

        // ---------- 3. commit set B (pipeline drained) ----------
        check_write(5'h10, 32'h1, 4'hF, 2'b00);
        mc0 = 16'hF800; mc1 = 16'h4000; mc2 = 16'h7FFF; mc3 = 16'h8000;   // -2048, 16384, 32767, -32768

        // ---------- 4. stream B ----------
        $display("[%0t] stream B: negative coefficients", $time);
        to_send = to_send + 300;
        wait_drained;

        // ---------- report ----------
        if (tx_count != 600 || rx_count != 600) begin
            errors = errors + 1;
            $display("ERROR: sent %0d, accepted %0d, received %0d (expected 600)", sent, tx_count, rx_count);
        end
        $display("samples in: %0d, results out: %0d", tx_count, rx_count);
        if (errors == 0)
            $display("\n===================================\nSTATUS: SIMULATION PASS\n===================================\n");
        else
            $display("\n===================================\nSTATUS: SIMULATION FAIL (%0d ERRORS)\n===================================\n", errors);
        $finish;
    end
endmodule
