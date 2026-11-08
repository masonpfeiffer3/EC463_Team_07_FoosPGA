`timescale 1ns / 1ps
`default_nettype none
// -----------------------------------------------------------------------------
// axis_to_pixel
//
// Converts the AXI4-Stream video from the CSI-2 RX (via an AXI4-Stream
// Broadcaster) into the plain pixel interface the Verilog ball tracker uses:
//
//   pix_valid  1 when pix_* holds a pixel
//   pix_data   top OUT_W bits of the pixel (RAW10 -> 8-bit by default)
//   pix_x/y    coordinates, (0,0) at top-left
//   pix_sof    first pixel of a frame
//   pix_eol    last pixel of a line
//   pix_eof    last pixel of a frame
//
// One register stage, so outputs are 1 cycle behind the stream. tready is
// held high: the tracker must take one pixel per clock (it's a streaming
// design). Output stays quiet until the first SOF, so starting mid-frame after
// reset never produces wrong coordinates.
//
// PIX_LSB: bit position of the pixel's LSB inside tdata. 0 means LSB-aligned.
// Confirm against the CSI-2 RX Subsystem product guide / an ILA capture of
// the test pattern, and change it here if the IP packs it differently.
// -----------------------------------------------------------------------------
module axis_to_pixel #(
    parameter DATA_W   = 16,
    parameter PIX_W    = 10,
    parameter PIX_LSB  = 0,
    parameter OUT_W    = 8,
    parameter V_ACTIVE = 400,
    parameter COORD_W  = 11
) (
    input  wire               clk,
    input  wire               rst,

    input  wire [DATA_W-1:0]  s_axis_tdata,
    input  wire               s_axis_tvalid,
    output wire               s_axis_tready,
    input  wire               s_axis_tlast,
    input  wire               s_axis_tuser,

    output reg                pix_valid,
    output reg  [OUT_W-1:0]   pix_data,
    output reg  [COORD_W-1:0] pix_x,
    output reg  [COORD_W-1:0] pix_y,
    output reg                pix_sof,
    output reg                pix_eol,
    output reg                pix_eof,
    output reg                locked     // 1 after the first SOF
);
    assign s_axis_tready = 1'b1;
    wire beat = s_axis_tvalid;

    reg [COORD_W-1:0] x_cnt, y_cnt;   // coordinates of the next pixel

    wire [COORD_W-1:0] cx = s_axis_tuser ? {COORD_W{1'b0}} : x_cnt;
    wire [COORD_W-1:0] cy = s_axis_tuser ? {COORD_W{1'b0}} : y_cnt;
    wire               go = beat && (locked || s_axis_tuser);

    always @(posedge clk) begin
        if (rst) begin
            x_cnt     <= 0;
            y_cnt     <= 0;
            locked    <= 0;
            pix_valid <= 0;
            pix_data  <= 0;
            pix_x     <= 0;
            pix_y     <= 0;
            pix_sof   <= 0;
            pix_eol   <= 0;
            pix_eof   <= 0;
        end else begin
            pix_valid <= go;
            pix_sof   <= go && s_axis_tuser;
            pix_eol   <= go && s_axis_tlast;
            pix_eof   <= go && s_axis_tlast && (cy == V_ACTIVE - 1);

            if (go) begin
                locked   <= 1'b1;
                pix_data <= s_axis_tdata[PIX_LSB + PIX_W - 1 -: OUT_W];
                pix_x    <= cx;
                pix_y    <= cy;
                if (s_axis_tlast) begin
                    x_cnt <= 0;
                    y_cnt <= cy + 1'b1;
                end else begin
                    x_cnt <= cx + 1'b1;
                    y_cnt <= cy;
                end
            end
        end
    end
endmodule
`default_nettype wire
