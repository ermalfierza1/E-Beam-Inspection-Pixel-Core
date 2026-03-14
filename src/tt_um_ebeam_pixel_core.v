// TinyTapeout user module: E-Beam Inspection Pixel Core
// Author: Ermal Fierza
// Description:
//  - 8-bit pixel-in per cycle with pix_valid
//  - Running mean (IIR) and local contrast |pixel - mean|
//  - Edge magnitude |pixel - prev_pixel|
//  - Config via tiny SPI (uio_in[2:4]=CS,SCLK,MOSI; uio_out[5]=MISO)
//  - Outputs flags + 4-bit magnitude nibble on uo_out.
//
// Harness pins (TinyTapeout standard):
//   input  clk, rst_n, ena
//   input  [7:0] ui_in            // pixel input
//   output [7:0] uo_out           // flags + mag nibble
//   input  [7:0] uio_in           // [0]=pix_valid, [1]=frame_start, [2]=CSn, [3]=SCLK, [4]=MOSI
//   output [7:0] uio_out          // [5]=MISO
//   output [7:0] uio_oe           // drive OE for MISO only

module tt_um_ebeam_pixel_core (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        ena,

    input  wire [7:0]  ui_in,
    output wire [7:0]  uo_out,

    input  wire [7:0]  uio_in,
    output wire [7:0]  uio_out,
    output wire [7:0]  uio_oe
);

    // -----------------------------
    // IO mapping
    // -----------------------------
    wire [7:0] pixel_in   = ui_in;
    wire       pix_valid  = uio_in[0];
    wire       frame_start= uio_in[1];
    wire       cfg_cs_n   = uio_in[2];
    wire       cfg_sclk   = uio_in[3];
    wire       cfg_mosi   = uio_in[4];

    reg        cfg_miso;
    assign uio_out = {2'b00, cfg_miso, 5'b0_0000}; // bit[5]=MISO, concatinate/pad with 0s
    assign uio_oe  = 8'b0010_0000; // drive only uio_out[5]

    // -----------------------------
    // Config registers (reset defaults)
    // -----------------------------
    reg  [7:0] reg_thresh;
    reg  [7:0] reg_contrast_thr;
    reg  [7:0] reg_edge_thr;
    reg  [2:0] reg_alpha_shift;
    reg  [7:0] reg_mode;

    // Readback/status
    reg  [7:0] reg_status;      // {def_any,bright,dark,edge, 4'h0}
    reg  [7:0] reg_mean_rdback;
    reg  [7:0] reg_absdiff_rdback;

    // Defaults
    localparam [7:0] DEF_THRESH        = 8'd128;
    localparam [7:0] DEF_CONTRAST_THR  = 8'd12;   // ~5% of 255
    localparam [7:0] DEF_EDGE_THR      = 8'd12;
    localparam [2:0] DEF_ALPHA_SHIFT   = 3'd3;    // 1/8 IIR

    // SPI clocked by cfg_sclk) uses a CDC handshake to request writes into
    // the clk domain. This keeps the datapath and configuration registers synchronous
    // to clk while allowing SPI to run as fast as cfg_sclk without missing edges.
    reg        spi_wr_toggle;
    reg  [2:0] spi_wr_addr;
    reg  [7:0] spi_wr_data;
    reg        cfg_wr_toggle_ff1, cfg_wr_toggle_ff2, cfg_wr_toggle_ff2_d;
    reg  [2:0] cfg_wr_addr_ff1,   cfg_wr_addr_ff2;
    reg  [7:0] cfg_wr_data_ff1,   cfg_wr_data_ff2;
    wire       cfg_wr_pulse = cfg_wr_toggle_ff2 ^ cfg_wr_toggle_ff2_d; // transition detector

    // SPI domain completes a write → toggles spi_wr_toggle (could be anytime)
    // Toggle propagates through synchronizers (2 clk cycles)
    // XOR detects the change → generates one pulse
    // Nothing happens until next SPI write (could be seconds later)

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cfg_wr_toggle_ff1   <= 1'b0;
            cfg_wr_toggle_ff2   <= 1'b0;
            cfg_wr_toggle_ff2_d <= 1'b0;
            cfg_wr_addr_ff1     <= 3'd0;
            cfg_wr_addr_ff2     <= 3'd0;
            cfg_wr_data_ff1     <= 8'h00;
            cfg_wr_data_ff2     <= 8'h00;

            reg_thresh       <= DEF_THRESH;
            reg_contrast_thr <= DEF_CONTRAST_THR;
            reg_edge_thr     <= DEF_EDGE_THR;
            reg_alpha_shift  <= DEF_ALPHA_SHIFT;
            reg_mode         <= 8'h00;
        end else begin
            cfg_wr_toggle_ff1   <= spi_wr_toggle;       // latched
            cfg_wr_toggle_ff2   <= cfg_wr_toggle_ff1;   // dobule FF methode to avoid metastability
            cfg_wr_toggle_ff2_d <= cfg_wr_toggle_ff2;

            cfg_wr_addr_ff1     <= spi_wr_addr;
            cfg_wr_addr_ff2     <= cfg_wr_addr_ff1;
            cfg_wr_data_ff1     <= spi_wr_data;
            cfg_wr_data_ff2     <= cfg_wr_data_ff1;

            if (cfg_wr_pulse) begin
                case (cfg_wr_addr_ff2)
                    3'h0: reg_thresh       <= cfg_wr_data_ff2;
                    3'h1: reg_contrast_thr <= cfg_wr_data_ff2;
                    3'h2: reg_edge_thr     <= cfg_wr_data_ff2;
                    3'h3: reg_alpha_shift  <= cfg_wr_data_ff2[2:0];
                    3'h4: reg_mode         <= cfg_wr_data_ff2;
                    default: ;
                endcase
            end
        end
    end

    // -----------------------------
    // Tiny SPI slave (CPOL=0, CPHA=0) clock idles low, data sampled on rising edge
    //   cmd[7]=1 write, 0 read
    //   cmd[6:4]=addr (0..7)
    //   cmd[3:0]=0
    // -----------------------------
    reg  [7:0] spi_shift_in;
    reg  [7:0] spi_shift_out;
    reg  [3:0] spi_bitcnt;
    reg        spi_cmd_phase;     // 1=shifting command byte, 0=data phase
    reg        spi_cmd_wr;        // latched write/read
    reg  [2:0] spi_cmd_addr;

    // Readback preload uses a separate load register so spi_shift_out is only driven
    // from one always block (negedge cfg_sclk) for clean synthesis.
    reg  [7:0] spi_shift_out_load;
    reg        spi_shift_out_load_pending;

    wire [63:0] spi_rd_bus_clk = {
        reg_absdiff_rdback,
        reg_mean_rdback,
        reg_status,
        reg_mode,
        {5'b0, reg_alpha_shift},
        reg_edge_thr,
        reg_contrast_thr,
        reg_thresh
    };
    reg  [63:0] spi_rd_bus_ff1;
    reg  [63:0] spi_rd_bus_ff2;

    always @(posedge cfg_sclk) begin
        if (!rst_n_sclk) begin
            spi_rd_bus_ff1 <= 64'h0;
            spi_rd_bus_ff2 <= 64'h0;
        end else begin
            spi_rd_bus_ff1 <= spi_rd_bus_clk;
            spi_rd_bus_ff2 <= spi_rd_bus_ff1;
        end
    end

    wire [7:0] spi_rd_reg0 = spi_rd_bus_ff2[7:0];
    wire [7:0] spi_rd_reg1 = spi_rd_bus_ff2[15:8];
    wire [7:0] spi_rd_reg2 = spi_rd_bus_ff2[23:16];
    wire [7:0] spi_rd_reg3 = spi_rd_bus_ff2[31:24];
    wire [7:0] spi_rd_reg4 = spi_rd_bus_ff2[39:32];
    wire [7:0] spi_rd_reg5 = spi_rd_bus_ff2[47:40];
    wire [7:0] spi_rd_reg6 = spi_rd_bus_ff2[55:48];
    wire [7:0] spi_rd_reg7 = spi_rd_bus_ff2[63:56];

    // -----------------------------
    // Reset synchronizer for cfg_sclk domain (2-FF chain)
    // -----------------------------
    // Purpose: Safely cross rst_n from the clk domain into the cfg_sclk domain
    // to avoid metastability and SYNCASYNCNET warnings in synthesis.
    //
    // Operation:
    //  - rst_n asserts asynchronously (negedge rst_n in sensitivity list)
    //  - rst_n deassertion is synchronized through 2 FFs clocked by cfg_sclk
    //  - FF1 may go metastable, but FF2 will be stable by the time it's used
    //  - SPI state machine blocks use rst_n_sclk for their async reset
    //
    // Note: All cfg_sclk domain blocks use synchronous reset with rst_n_sclk.
    // This is Yosys-friendly and avoids "unclocked register" warnings.
    (* async_reg = "true" *) reg rst_n_sclk_ff1;
    (* async_reg = "true" *) reg rst_n_sclk;
    
    always @(posedge cfg_sclk) begin
        rst_n_sclk_ff1 <= rst_n;
        rst_n_sclk     <= rst_n_sclk_ff1;
    end

    wire [7:0] spi_shift_in_next = {spi_shift_in[6:0], cfg_mosi};

    reg cfg_cs_n_prev;

    always @(posedge cfg_sclk) begin
        if (!rst_n_sclk) begin
            cfg_cs_n_prev <= 1'b1;
            spi_shift_in  <= 8'h00;
            spi_bitcnt    <= 4'd0;
            spi_cmd_phase <= 1'b1;
            spi_cmd_wr    <= 1'b0;
            spi_cmd_addr  <= 3'd0;
            spi_wr_toggle <= 1'b0;
            spi_wr_addr   <= 3'd0;
            spi_wr_data   <= 8'h00;
            spi_shift_out_load         <= 8'h00;
            spi_shift_out_load_pending <= 1'b0;
        end else begin
            spi_shift_out_load_pending <= 1'b0;

            if (cfg_cs_n) begin
                spi_bitcnt    <= 4'd0;
                spi_cmd_phase <= 1'b1;
                spi_cmd_wr    <= 1'b0;
                spi_cmd_addr  <= 3'd0;
            end else if (cfg_cs_n_prev) begin
                spi_shift_in  <= {7'b0, cfg_mosi};
                spi_bitcnt    <= 4'd1;
                spi_cmd_phase <= 1'b1;
                spi_cmd_wr    <= 1'b0;
                spi_cmd_addr  <= 3'd0;
            end else begin
                spi_shift_in <= spi_shift_in_next;

                if (spi_bitcnt == 4'd7) begin
                    spi_bitcnt <= 4'd0;
                    if (spi_cmd_phase) begin
                        spi_cmd_phase <= 1'b0;
                        spi_cmd_wr    <= spi_shift_in_next[7];
                        spi_cmd_addr  <= spi_shift_in_next[6:4];

                        if (spi_shift_in_next[7] == 1'b0) begin
                            case (spi_shift_in_next[6:4])
                                3'h0: spi_shift_out_load <= spi_rd_reg0;
                                3'h1: spi_shift_out_load <= spi_rd_reg1;
                                3'h2: spi_shift_out_load <= spi_rd_reg2;
                                3'h3: spi_shift_out_load <= spi_rd_reg3;
                                3'h4: spi_shift_out_load <= spi_rd_reg4;
                                3'h5: spi_shift_out_load <= spi_rd_reg5;
                                3'h6: spi_shift_out_load <= spi_rd_reg6;
                                3'h7: spi_shift_out_load <= spi_rd_reg7;
                                default: spi_shift_out_load <= 8'h00;
                            endcase
                            spi_shift_out_load_pending <= 1'b1;
                        end
                    end else begin
                        spi_cmd_phase <= 1'b1;
                        if (spi_cmd_wr) begin
                            spi_wr_addr   <= spi_cmd_addr;
                            spi_wr_data   <= spi_shift_in_next;
                            spi_wr_toggle <= ~spi_wr_toggle;
                        end
                    end
                end else begin
                    spi_bitcnt <= spi_bitcnt + 1'b1;
                end
            end

            cfg_cs_n_prev <= cfg_cs_n;
        end
    end

    // Mode-0 MISO timing: update/shift on falling edge so it is stable at the next
    // rising edge when the master samples.
    always @(negedge cfg_sclk) begin
        if (!rst_n_sclk) begin
            cfg_miso      <= 1'b0;
            spi_shift_out <= 8'h00;
        end else if (cfg_cs_n) begin
            cfg_miso      <= 1'b0;
            spi_shift_out <= 8'h00;
        end else begin
            if (spi_shift_out_load_pending) begin
                cfg_miso                   <= spi_shift_out_load[7];
                spi_shift_out              <= {spi_shift_out_load[6:0], 1'b0};
            end else begin
                cfg_miso      <= spi_shift_out[7];
                spi_shift_out <= {spi_shift_out[6:0], 1'b0};
            end
        end
    end

    // -----------------------------
    // Core datapath
    // -----------------------------
    reg  [7:0] pixel_q;
    reg  [7:0] prev_pixel;
    reg  [7:0] mean_q;

    // Signed temp for (pixel - mean)
    wire signed [8:0] diff_pm = {1'b0, pixel_q} - {1'b0, mean_q};
    wire [7:0] abs_diff = diff_pm[8] ? (~diff_pm[7:0] + 8'd1) : diff_pm[7:0];

    // Edge = |pixel - prev_pixel|
    wire signed [8:0] diff_edge = {1'b0, pixel_q} - {1'b0, prev_pixel};
    wire [7:0] edge_mag = diff_edge[8] ? (~diff_edge[7:0] + 8'd1) : diff_edge[7:0];

    // Classifiers
    wire bright_defect = (diff_pm[8] == 1'b0) && (abs_diff >= reg_contrast_thr);
    wire dark_defect   = (diff_pm[8] == 1'b1) && (abs_diff >= reg_contrast_thr);
    wire edge_strong   = (edge_mag >= reg_edge_thr);
    wire defect_any    = bright_defect | dark_defect | edge_strong;

    // Magnitude nibble: max(abs_diff, edge)[3:0]
    wire [7:0] mag_max = (abs_diff >= edge_mag) ? abs_diff : edge_mag;
    wire [3:0] mag_nib = mag_max[3:0];

    // Optional threshold OUT or passthrough
    wire thresh_bit = (pixel_q >= reg_thresh);

    // Outputs pack
    reg [7:0] uo_out_r;
    assign uo_out = uo_out_r;

    // Mean update (IIR). When frame_start, re-seed mean to current pixel.
    // Only update on ena && pix_valid.
    wire signed [8:0] mean_delta = diff_pm >>> reg_alpha_shift; // arithmetic shift
    // NOTE: Option A behavior (intentional): we update the 8-bit mean using only the
    // lower 8 bits of the signed delta. This implies:
    //  - The add is modulo-256 (wraps on overflow/underflow), not saturating.
    //  - mean_delta[8] (sign/extra magnitude bit) is discarded, so large deltas are
    //    effectively truncated before being applied.
    wire [7:0] mean_next = frame_start ? pixel_in
                           : (mean_q + mean_delta[7:0]);

    // Sequential logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pixel_q           <= 8'd0;
            prev_pixel        <= 8'd0;
            mean_q            <= 8'd0;

            reg_status        <= 8'd0;
            reg_mean_rdback   <= 8'd0;
            reg_absdiff_rdback<= 8'd0;
            uo_out_r          <= 8'd0;
        end else begin
            if (ena) begin
                // Latch pixel on valid
                if (pix_valid) begin
                    pixel_q    <= pixel_in;

                    // prev_pixel update
                    if (frame_start) begin
                        prev_pixel <= pixel_in;
                    end else begin
                        prev_pixel <= pixel_q;
                    end

                    // Mean update
                    mean_q <= mean_next;

                    // Update readbacks
                    reg_mean_rdback    <= mean_next;
                    reg_absdiff_rdback <= abs_diff;

                    // Status (RO)
                    reg_status <= {defect_any, bright_defect, dark_defect, edge_strong, 4'h0};

                    // Output muxing
                    if (reg_mode[0]) begin
                        // debug_passthrough: show pixel on output
                        uo_out_r <= pixel_in;
                    end else if (reg_mode[1]) begin
                        // use_thresh_out: binarize
                        uo_out_r <= {thresh_bit, 7'd0};
                    end else begin
                        // Flags + magnitude nibble
                        uo_out_r <= {defect_any, bright_defect, dark_defect, edge_strong, mag_nib};
                    end
                end
            end
        end
    end

endmodule
