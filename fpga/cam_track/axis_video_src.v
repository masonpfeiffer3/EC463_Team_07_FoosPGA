`timescale 1ns / 1ps
`default_nettype none
// -----------------------------------------------------------------------------
// axis_video_src
//
// Stand-in for the MIPI CSI-2 RX Subsystem's AXI4-Stream video output, so the
// rest of the pipeline can be built and tested before the camera works.
// Synthesizable: on the board, mux it in place of the CSI-2 RX output to test
// downstream logic without the sensor.
//
// Stream contract (same as CSI-2 RX Subsystem, 1 pixel per clock):
//   tuser = 1 on the first pixel of a frame (SOF)
//   tlast = 1 on the last pixel of each line (EOL)
//   tdata = pixel, LSB-aligned, zero-padded to DATA_W
//
// mode 0: diagonal ramp, pixel = (x + y) mod 2^PIX_W  -> checks x/y counting
// mode 1: bright disc ("ball") on a dark background, moving and bouncing
//         BALL_VX/BALL_VY pixels per frame -> gives the tracker something to track
// -----------------------------------------------------------------------------
module axis_video_src #(
    parameter H_ACTIVE   = 640,
    parameter V_ACTIVE   = 400,
    parameter H_BLANK    = 64,    // idle cycles after each line
    parameter V_BLANK    = 10,    // idle lines after each frame
    parameter PIX_W      = 10,
    parameter DATA_W     = 16,
    parameter BALL_R     = 8,     // radius in pixels (~16 px ball at 640 wide)
    parameter BALL_X0    = 100,
    parameter BALL_Y0    = 80,
    parameter BALL_VX    = 23,    // ~10 m/s at 210 fps, ~2.1 mm/px
    parameter BALL_VY    = 9,
    parameter BG_LEVEL   = 40,
    parameter BALL_LEVEL = 900
) (
    input  wire              clk,
    input  wire              rst,
    input  wire              mode,

    output wire [DATA_W-1:0] m_axis_tdata,
    output wire              m_axis_tvalid,
    input  wire              m_axis_tready,
    output wire              m_axis_tlast,
    output wire              m_axis_tuser,

    // ground truth for testbenches: ball position used for the current frame
    output wire signed [12:0] ball_x,
    output wire signed [12:0] ball_y
);
    localparam H_TOTAL = H_ACTIVE + H_BLANK;
    localparam V_TOTAL = V_ACTIVE + V_BLANK;

    reg [12:0] hcnt, vcnt;
    reg        run;   // keeps tvalid low during and right after reset (AXIS rule)
    reg signed [12:0] bx, by, vx, vy;

    wire active  = run && (hcnt < H_ACTIVE) && (vcnt < V_ACTIVE);
    wire advance = active ? m_axis_tready : 1'b1;
    wire h_wrap  = (hcnt == H_TOTAL - 1);
    wire v_wrap  = (vcnt == V_TOTAL - 1);

    // ball geometry
    wire signed [13:0] dx = $signed({1'b0, hcnt}) - bx;
    wire signed [13:0] dy = $signed({1'b0, vcnt}) - by;
    wire        [27:0] d2 = dx * dx + dy * dy;
    wire               in_ball = (d2 <= BALL_R * BALL_R);

    wire [PIX_W-1:0] ramp_pix = hcnt[PIX_W-1:0] + vcnt[PIX_W-1:0];
    wire [PIX_W-1:0] ball_pix = in_ball ? BALL_LEVEL[PIX_W-1:0] : BG_LEVEL[PIX_W-1:0];
    wire [PIX_W-1:0] pix      = mode ? ball_pix : ramp_pix;

    assign m_axis_tdata  = {{(DATA_W - PIX_W){1'b0}}, pix};
    assign m_axis_tvalid = active;
    assign m_axis_tuser  = active && (hcnt == 0) && (vcnt == 0);
    assign m_axis_tlast  = active && (hcnt == H_ACTIVE - 1);
    assign ball_x = bx;
    assign ball_y = by;

    // next ball position, with bounce off the edges
    wire signed [12:0] nbx = bx + vx;
    wire signed [12:0] nby = by + vy;

    always @(posedge clk) begin
        if (rst) begin
            run  <= 0;
            hcnt <= 0;
            vcnt <= 0;
            bx   <= BALL_X0;
            by   <= BALL_Y0;
            vx   <= BALL_VX;
            vy   <= BALL_VY;
        end else if (!run) begin
            run <= 1'b1;
        end else if (advance) begin
            hcnt <= h_wrap ? 13'd0 : hcnt + 1'b1;
            if (h_wrap) begin
                vcnt <= v_wrap ? 13'd0 : vcnt + 1'b1;
                if (v_wrap) begin
                    // move the ball once per frame, during blanking
                    if (nbx < BALL_R || nbx > H_ACTIVE - 1 - BALL_R) begin
                        vx <= -vx;
                        bx <= bx - vx;
                    end else begin
                        bx <= nbx;
                    end
                    if (nby < BALL_R || nby > V_ACTIVE - 1 - BALL_R) begin
                        vy <= -vy;
                        by <= by - vy;
                    end else begin
                        by <= nby;
                    end
                end
            end
        end
    end
endmodule
`default_nettype wire
