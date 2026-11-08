// =============================================================================
// frame_to_bram.v  --  copy ONE whole camera frame into block RAM when asked
// =============================================================================
//
// WHAT IT DOES
//   The ARM (Python) wants to look at a frame. It can't keep up with the
//   camera's 260 frames per second, so we don't stream to it. Instead:
//
//     1. Python sets capture_req = 1                 (through an AXI GPIO)
//     2. We wait for the start of the next frame     (pix_sof)
//     3. We write every pixel of that frame into BRAM, 4 pixels per 32-bit word
//     4. We set capture_done = 1 and STOP writing    (so the picture can't change
//                                                     while Python reads it)
//     5. Python reads the BRAM, then sets capture_req = 0
//     6. We see that, clear capture_done, and wait for the next request
//
//   That back-and-forth (req up, done up, req down, done down) is called a
//   4-phase handshake. It is safe even though Python's GPIO runs on a
//   different clock (100 MHz) than we do (200 MHz).
//
// MEMORY LAYOUT (what Python sees)
//   One byte per pixel, row after row:  byte address = y * 640 + x
//   So the BRAM is just the picture, 640 x 400 = 256,000 bytes.
//   Each 32-bit word holds 4 neighbouring pixels, the LEFT pixel in the
//   lowest byte (that's what the ARM, which is little-endian, expects).
//
// HOW TO CONNECT IT (block design)
//   pix_* inputs      <- capture_top's pix_* outputs (they can ALSO go to the ILA)
//   capture_req       <- AXI GPIO channel 1 output (gpio_io_o)
//   capture_done      -> AXI GPIO channel 2 input  (gpio2_io_i)
//   bram_*            -> Block Memory Generator port B (addrb, dinb, web, enb, rstb)
//   The BRAM's clkb gets the same 200 MHz clock as this block.
// =============================================================================

`timescale 1ns / 1ps

module frame_to_bram #(
    parameter WIDTH  = 640,                 // pixels per line
    parameter HEIGHT = 400                  // lines per frame
) (
    input             clk,                  // 200 MHz video clock (pl_clk1)
    input             resetn,               // active-low reset

    // Pixel stream from capture_top
    input             pix_valid,
    input       [7:0] pix_data,
    input      [10:0] pix_x,
    input      [10:0] pix_y,
    input             pix_sof,              // 1 on the first pixel of a frame

    // Handshake with Python (through AXI GPIO)
    input             capture_req,          // comes from the 100 MHz side!
    output reg        capture_done,

    // Write port to the Block Memory Generator (port B)
    output reg [31:0] bram_addr,            // BYTE address of the word
    output reg [31:0] bram_din,             // 4 pixels
    output reg  [3:0] bram_we,              // 4'b1111 = write all 4 bytes
    output            bram_en,
    output            bram_rst
);

    wire rst = ~resetn;

    assign bram_en  = 1'b1;                 // port always enabled; bram_we decides
    assign bram_rst = 1'b0;                 // never reset the RAM output

    // -------------------------------------------------------------------------
    // 1. Bring capture_req into our clock domain
    // -------------------------------------------------------------------------
    // It changes on a different clock, so we pass it through two flip-flops.
    // The first one may go "metastable" for a moment; the second gives us a
    // clean 0 or 1. (Standard 2-flop synchronizer.)
    reg req_meta, req_sync;
    always @(posedge clk) begin
        req_meta <= capture_req;
        req_sync <= req_meta;
    end

    // -------------------------------------------------------------------------
    // 2. A small state machine
    // -------------------------------------------------------------------------
    localparam IDLE     = 2'd0;             // waiting for Python to ask
    localparam WAIT_SOF = 2'd1;             // asked: wait for a frame to start
    localparam WRITING  = 2'd2;             // copying this frame into BRAM
    localparam DONE     = 2'd3;             // frame is in BRAM, hands off

    reg [1:0] state;

    // The frame starts right now (this pixel is x = 0, y = 0)
    wire start_now = (state == WAIT_SOF) && pix_valid && pix_sof;

    // Should this pixel go into the BRAM?
    wire take = pix_valid && ((state == WRITING) || start_now);

    // Is this the very last pixel of the frame?
    wire last_pixel = (pix_x == WIDTH - 1) && (pix_y == HEIGHT - 1);

    // Where this pixel lives in memory: y * 640 + x (one byte per pixel)
    wire [31:0] pixel_index = pix_y * WIDTH + pix_x;

    // -------------------------------------------------------------------------
    // 3. Pack 4 pixels into one 32-bit word
    // -------------------------------------------------------------------------
    // 'held' keeps the first 3 pixels of the current group of 4. Every new
    // pixel shifts in at the top, so after x = 0, 1, 2 it holds
    // {p2, p1, p0} and p0 sits in the lowest byte. When the 4th pixel
    // (x = 3, 7, 11, ...) arrives, we write {p3, p2, p1, p0}.
    // This works because 640 is a multiple of 4, so every line starts a
    // new group.
    reg [23:0] held;

    always @(posedge clk) begin
        if (rst) begin
            state        <= IDLE;
            capture_done <= 1'b0;
            held         <= 24'd0;
            bram_addr    <= 32'd0;
            bram_din     <= 32'd0;
            bram_we      <= 4'b0000;
        end else begin
            // Default: no write this clock
            bram_we <= 4'b0000;

            // ---- pixel packing and BRAM writes ----
            if (take) begin
                held <= {pix_data, held[23:8]};
                if (pix_x[1:0] == 2'd3) begin
                    bram_addr <= {pixel_index[31:2], 2'b00};   // address of pixel x-3
                    bram_din  <= {pix_data, held};
                    bram_we   <= 4'b1111;
                end
            end

            // ---- state machine ----
            case (state)
                IDLE: begin
                    if (req_sync)
                        state <= WAIT_SOF;
                end

                WAIT_SOF: begin
                    if (start_now)
                        state <= WRITING;
                end

                WRITING: begin
                    if (pix_valid && last_pixel) begin
                        state        <= DONE;
                        capture_done <= 1'b1;    // tell Python: go ahead, read it
                    end
                end

                DONE: begin
                    // Nothing is written in this state, so the frame stays put.
                    if (!req_sync) begin         // Python finished reading
                        state        <= IDLE;
                        capture_done <= 1'b0;
                    end
                end
            endcase
        end
    end

endmodule
