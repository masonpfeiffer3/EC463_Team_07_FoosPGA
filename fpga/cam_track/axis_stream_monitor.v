`timescale 1ns / 1ps
`default_nettype none
// -----------------------------------------------------------------------------
// axis_stream_monitor
//
// Passive checker for an AXI4-Stream video link. Watches the handshake only
// (never drives tready), so it can sit on any stream: the CSI-2 RX output,
// after the broadcaster, or on the test source.
//
// Reports, updated every frame:
//   line_len        pixels in the most recent complete line   (expect 640)
//   lines_per_frame lines in the most recent complete frame   (expect 400)
//   frame_period    clk cycles from one SOF to the next
//                   -> fps = f_clk / frame_period             (expect ~210)
//   frame_count     SOFs seen since reset
//   stats_valid     1 once a full frame has been measured
// Sticky errors (cleared by clear_err):
//   err_line_len    a line's length differed from the first line's length
//   err_sof_midline SOF arrived before the previous line finished
//
// Put these on an ILA first; later, expose them as AXI-Lite registers so
// Linux can read the frame rate.
// -----------------------------------------------------------------------------
module axis_stream_monitor #(
    parameter CNT_W = 32
) (
    input  wire             clk,
    input  wire             rst,
    input  wire             tvalid,
    input  wire             tready,
    input  wire             tlast,
    input  wire             tuser,
    input  wire             clear_err,

    output reg  [15:0]      line_len,
    output reg  [15:0]      lines_per_frame,
    output reg  [CNT_W-1:0] frame_period,
    output reg  [CNT_W-1:0] frame_count,
    output reg              stats_valid,
    output reg              err_line_len,
    output reg              err_sof_midline
);
    wire beat = tvalid & tready;

    reg [15:0]      pix_cnt;    // pixels so far in the current line
    reg [15:0]      line_cnt;   // complete lines so far in the current frame
    reg [CNT_W-1:0] cyc_cnt;    // cycles since the last SOF
    reg [15:0]      ref_len;    // first line length seen
    reg             ref_set;
    reg             seen_sof;

    // index of the pixel on this beat (SOF always starts a new line)
    wire [15:0] pidx      = tuser ? 16'd0 : pix_cnt;
    wire [15:0] line_base = tuser ? 16'd0 : line_cnt;

    always @(posedge clk) begin
        if (rst) begin
            pix_cnt         <= 0;
            line_cnt        <= 0;
            cyc_cnt         <= 0;
            ref_len         <= 0;
            ref_set         <= 0;
            seen_sof        <= 0;
            line_len        <= 0;
            lines_per_frame <= 0;
            frame_period    <= 0;
            frame_count     <= 0;
            stats_valid     <= 0;
            err_line_len    <= 0;
            err_sof_midline <= 0;
        end else begin
            cyc_cnt <= cyc_cnt + 1'b1;

            if (clear_err) begin
                err_line_len    <= 0;
                err_sof_midline <= 0;
                ref_set         <= 0;
            end

            if (beat) begin
                // ---- frame boundary ----
                if (tuser) begin
                    if (seen_sof) begin
                        lines_per_frame <= line_cnt;
                        frame_period    <= cyc_cnt;
                        stats_valid     <= 1'b1;
                    end
                    seen_sof    <= 1'b1;
                    frame_count <= frame_count + 1'b1;
                    cyc_cnt     <= 1;
                    if (pix_cnt != 0)
                        err_sof_midline <= 1'b1;
                end

                // ---- line boundary ----
                if (tlast) begin
                    line_len <= pidx + 1'b1;
                    pix_cnt  <= 0;
                    line_cnt <= line_base + 1'b1;
                    if (!ref_set || clear_err) begin
                        ref_len <= pidx + 1'b1;
                        ref_set <= 1'b1;
                    end else if (pidx + 1'b1 != ref_len) begin
                        err_line_len <= 1'b1;
                    end
                end else begin
                    pix_cnt  <= pidx + 1'b1;
                    line_cnt <= line_base;
                end
            end
        end
    end
endmodule
`default_nettype wire
