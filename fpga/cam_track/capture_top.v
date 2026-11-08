// =============================================================================
// capture_top.v  --  the Stage 2 capture block that goes in the block design
// =============================================================================
//
// WHAT IT DOES
//   Takes the video stream from AMD's MIPI CSI-2 receiver and feeds it to the
//   two blocks you already wrote and tested:
//
//                               +--> axis_to_pixel ------> pix_* outputs
//   CSI-2 receiver video_out ---+
//                               +--> axis_stream_monitor -> line_len, ...
//
//   All outputs go to the System ILA so you can see them on the board.
//
// HOW TO USE IT IN VIVADO
//   Add this file plus axis_to_pixel.v and axis_stream_monitor.v as design
//   sources. In the block design: right-click > Add Module > capture_top.
//   Because the input ports are named s_axis_video_*, Vivado recognises them
//   as one AXI4-Stream port, so you connect the receiver's video_out to it
//   with a single wire.
//
// NOTE
//   tready comes from axis_to_pixel, which always holds it at 1. The monitor
//   watches the same stream, so it sees exactly what axis_to_pixel sees.
// =============================================================================

`timescale 1ns / 1ps

module capture_top #    (
    parameter HEIGHT = 400                     // lines per frame
) (
    // Clock and reset: use the receiver's video clock (200 MHz) and an
    // active-LOW reset from a Processor System Reset block on that clock.
    input             clk,
    input             resetn,

    // AXI4-Stream video from the CSI-2 receiver (its video_out port)
    input      [15:0] s_axis_video_tdata,      // 10-bit pixel in bits [9:0]
    input             s_axis_video_tvalid,
    output            s_axis_video_tready,
    input       [0:0] s_axis_video_tuser,      // 1 = first pixel of a frame
    input             s_axis_video_tlast,      // 1 = last pixel of a line

    // Pixel stream with x/y (to the ILA now, to the trackers later)
    output            pix_valid,
    output      [7:0] pix_data,
    output     [10:0] pix_x,
    output     [10:0] pix_y,
    output            pix_sof,
    output            pix_eol,
    output            pix_eof,
    output            locked,                  // 1 once the first frame start is seen

    // Measurements of the stream (to the ILA)
    output     [15:0] line_len,                // pixels in the last line      (expect 640)
    output     [15:0] lines_per_frame,         // lines in the last frame      (expect 400)
    output     [31:0] frame_period,            // clock cycles per frame: fps = 200 MHz / this
    output     [31:0] frame_count,             // frames since reset
    output            stats_valid,             // 1 once a whole frame has been measured
    output            err_line_len,            // sticky: a line had a different length
    output            err_sof_midline          // sticky: a frame started in the middle of a line
);

    // Our blocks use an active-HIGH reset, the block design gives active-LOW.
    wire rst = ~resetn;

    // ---- add x/y to every pixel ----
    // Defaults in axis_to_pixel already match: 16-bit tdata, 10-bit pixel at
    // bit 0, 8-bit output, 11-bit coordinates. Only the frame height is set here.
    axis_to_pixel #(
        .V_ACTIVE(HEIGHT)
    ) adapter (
        .clk(clk), .rst(rst),
        .s_axis_tdata(s_axis_video_tdata),
        .s_axis_tvalid(s_axis_video_tvalid),
        .s_axis_tready(s_axis_video_tready),
        .s_axis_tlast(s_axis_video_tlast),
        .s_axis_tuser(s_axis_video_tuser[0]),
        .pix_valid(pix_valid), .pix_data(pix_data),
        .pix_x(pix_x), .pix_y(pix_y),
        .pix_sof(pix_sof), .pix_eol(pix_eol), .pix_eof(pix_eof),
        .locked(locked)
    );

    // ---- measure the stream ----
    // clear_err is tied to 0: the error flags stay set until the next reset,
    // so a single bad line can't hide between ILA captures.
    axis_stream_monitor monitor (
        .clk(clk), .rst(rst),
        .tvalid(s_axis_video_tvalid),
        .tready(s_axis_video_tready),
        .tlast(s_axis_video_tlast),
        .tuser(s_axis_video_tuser[0]),
        .clear_err(1'b0),
        .line_len(line_len),
        .lines_per_frame(lines_per_frame),
        .frame_period(frame_period),
        .frame_count(frame_count),
        .stats_valid(stats_valid),
        .err_line_len(err_line_len),
        .err_sof_midline(err_sof_midline)
    );

endmodule