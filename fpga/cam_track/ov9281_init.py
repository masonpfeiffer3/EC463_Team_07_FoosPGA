#!/usr/bin/env python3
# =============================================================================
# ov9281_init.py  --  set up the OV9281 for 640x400 RAW10 and start streaming
# =============================================================================
#
# WHAT IT DOES
#   Talks to the camera over I2C (through the PCA9546 switch) and writes the
#   same registers the Linux ov9282 driver writes, in the same order:
#
#     1. select switch channel 2      (camera is behind the switch)
#     2. soft reset                   (0x0103 = 1)
#     3. check the chip ID            (0x300A/0x300B = 0x92 0x81)
#     4. common settings              (from the driver's common_regs)
#     5. RAW10 output                 (0x030D, 0x3662)
#     6. 640x400 mode                 (from the driver's mode_640x400_regs)
#     7. continuous MIPI clock        (0x4800 = 0x00)
#     8. line length, frame length, exposure, gain
#     9. test pattern on/off          (0x5E00)
#    10. start streaming              (0x0100 = 1)
#
#   Register values come from drivers/media/i2c/ov9282.c in the Linux kernel
#   (the OV9282 driver also covers the OV9281).
#
# HOW TO RUN (on the Kria, after "xmutil loadapp cam_stream")
#   sudo python3 ov9281_init.py --pattern        # color-bar test pattern
#   sudo python3 ov9281_init.py                  # real image
#   sudo python3 ov9281_init.py --vts 422        # faster frames (see below)
#   sudo python3 ov9281_init.py --stop           # stop streaming
#
#   Find the bus number with "i2cdetect -l" (the AXI IIC bus). Default is 3.
#
# NO EXTRA PACKAGES: it uses the Linux /dev/i2c-N device directly.
# =============================================================================

import argparse
import fcntl
import os
import sys
import time

I2C_SLAVE = 0x0703          # Linux ioctl number: "the next reads/writes go to this address"

SWITCH_ADDR = 0x74          # PCA9546 I2C switch
SWITCH_CH2  = 0x04          # bit 2 = channel 2 = camera
CAM_ADDR    = 0x60          # OV9281 (7-bit address)


# -----------------------------------------------------------------------------
# Register lists from the Linux driver
# -----------------------------------------------------------------------------

# common_regs (the driver's names replaced by their numbers)
COMMON_REGS = [
    (0x0302, 0x32), (0x030e, 0x02), (0x3001, 0x00),
    (0x3004, 0x00), (0x3005, 0x00), (0x3006, 0x04),   # output enables (ILPWM on)
    (0x3011, 0x0a), (0x3013, 0x18), (0x301c, 0xf0), (0x3022, 0x01),
    (0x3030, 0x10), (0x3039, 0x32), (0x303a, 0x00),
    (0x3503, 0x08),                                   # manual exposure/gain, gain in 1/16 steps
    (0x3505, 0x8c), (0x3507, 0x03), (0x3508, 0x00),
    (0x3610, 0x80), (0x3611, 0xa0), (0x3620, 0x6e), (0x3632, 0x56),
    (0x3633, 0x78), (0x3666, 0x00), (0x366f, 0x5a), (0x3680, 0x84),
    (0x3712, 0x80), (0x372d, 0x22), (0x3731, 0x80), (0x3732, 0x30),
    (0x377d, 0x22), (0x3788, 0x02), (0x3789, 0xa4), (0x378a, 0x00),
    (0x378b, 0x4a), (0x3799, 0x20), (0x3881, 0x42), (0x38a8, 0x02),
    (0x38a9, 0x80), (0x38b1, 0x00), (0x38c4, 0x00), (0x38c5, 0xc0),
    (0x38c6, 0x04), (0x38c7, 0x80), (0x3920, 0xff), (0x4010, 0x40),
    (0x4043, 0x40), (0x4307, 0x30), (0x4317, 0x00), (0x4501, 0x00),
    (0x450a, 0x08), (0x4601, 0x04), (0x470f, 0x00), (0x4f07, 0x00),
    (0x5000, 0x9f), (0x5001, 0x00), (0x5e00, 0x00), (0x5d00, 0x07),
    (0x5d01, 0x00), (0x0101, 0x01), (0x1000, 0x03), (0x5a08, 0x84),
]

# RAW10 output (driver's bitdepth_regs for 10-bit)
RAW10_REGS = [
    (0x030d, 0x50),     # PLL setting for 10-bit
    (0x3662, 0x05),     # analog core setting for 10-bit
]

# mode_640x400_regs: full 1280x800 sensor area, read at half size (2:1)
MODE_640x400_REGS = [
    (0x3778, 0x10),
    (0x3800, 0x00), (0x3801, 0x00), (0x3802, 0x00), (0x3803, 0x00),   # start x, y
    (0x3804, 0x05), (0x3805, 0x0f), (0x3806, 0x03), (0x3807, 0x2f),   # end x, y
    (0x3808, 0x02), (0x3809, 0x80),                                   # output width  = 640
    (0x380a, 0x01), (0x380b, 0x90),                                   # output height = 400
    (0x3810, 0x00), (0x3811, 0x04), (0x3812, 0x00), (0x3813, 0x04),   # offsets
    (0x3814, 0x31), (0x3815, 0x22),                                   # skipping
    (0x3820, 0x60), (0x3821, 0x01),                                   # format / binning
    (0x4008, 0x02), (0x4009, 0x05), (0x400c, 0x00), (0x400d, 0x03),
    (0x4507, 0x03), (0x4509, 0x80),
]

# Timing (from the driver's 640x400 mode, continuous MIPI clock):
#   line length  = 640 + 816 = 1456 "pixel times"; register HTS = 1456 / 2 = 728
#   frame length = 400 + vblank lines; the driver's default vblank is 1022,
#                  the smallest allowed is 22 (so VTS can go from 422 up).
HTS_DEFAULT = 728
VTS_DEFAULT = 1422          # safe and slow (driver default)
VTS_MIN     = 422           # fastest the driver allows
EXPOSURE_OFFSET = 25        # exposure must be at most VTS - 25 lines


# -----------------------------------------------------------------------------
# Small I2C helpers
# -----------------------------------------------------------------------------
class Bus:
    """One open /dev/i2c-N, with the slave address set per call."""

    def __init__(self, bus_number):
        self.fd = os.open(f"/dev/i2c-{bus_number}", os.O_RDWR)

    def talk_to(self, addr):
        fcntl.ioctl(self.fd, I2C_SLAVE, addr)

    def write_bytes(self, data):
        os.write(self.fd, bytes(data))

    def read_bytes(self, n):
        return os.read(self.fd, n)


def cam_write(bus, reg, value):
    """Write one camera register: [reg high, reg low, value]."""
    bus.write_bytes([reg >> 8, reg & 0xFF, value & 0xFF])


def cam_read(bus, reg):
    """Read one camera register: send the 2-byte address, then read 1 byte."""
    bus.write_bytes([reg >> 8, reg & 0xFF])
    return bus.read_bytes(1)[0]


def write_list(bus, regs):
    for reg, value in regs:
        cam_write(bus, reg, value)


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(description="Set up the OV9281 and start streaming")
    p.add_argument("--bus", type=int, default=3, help="I2C bus number (from i2cdetect -l)")
    p.add_argument("--pattern", action="store_true", help="send the color-bar test pattern")
    p.add_argument("--vts", type=int, default=VTS_DEFAULT,
                   help=f"frame length in lines ({VTS_MIN} = fastest, {VTS_DEFAULT} = default)")
    p.add_argument("--exposure", type=int, default=0x282,
                   help="exposure in lines (clamped to VTS - 25)")
    p.add_argument("--gain", type=int, default=0x10, help="gain, 0x10 = 1x")
    p.add_argument("--stop", action="store_true", help="just stop streaming and exit")
    args = p.parse_args()

    bus = Bus(args.bus)

    # 1. Point the switch at the camera. The switch forgets this on every
    #    power-down or bitstream reload, so we always do it.
    bus.talk_to(SWITCH_ADDR)
    bus.write_bytes([SWITCH_CH2])
    bus.talk_to(CAM_ADDR)

    if args.stop:
        cam_write(bus, 0x0100, 0x00)
        print("streaming stopped")
        return

    # 2. Soft reset, then give the sensor a moment to wake up.
    cam_write(bus, 0x0103, 0x01)
    time.sleep(0.02)

    # 3. Is it really the OV9281?
    chip_id = (cam_read(bus, 0x300A) << 8) | cam_read(bus, 0x300B)
    print(f"chip ID: 0x{chip_id:04X}  (expect 0x9281)")
    if chip_id != 0x9281:
        sys.exit("wrong chip ID: check the switch channel and the bus number")

    # 4-6. Settings, RAW10, 640x400 mode (same order as the driver)
    write_list(bus, COMMON_REGS)
    write_list(bus, RAW10_REGS)
    write_list(bus, MODE_640x400_REGS)

    # 7. Continuous MIPI clock (0 = continuous, 0x20 = gated between packets).
    #    Continuous is the easier case for the receiver.
    cam_write(bus, 0x4800, 0x00)

    # 8. Line length (HTS) and frame length (VTS), 16 bits each
    vts = max(VTS_MIN, args.vts)
    cam_write(bus, 0x380C, HTS_DEFAULT >> 8)
    cam_write(bus, 0x380D, HTS_DEFAULT & 0xFF)
    cam_write(bus, 0x380E, vts >> 8)
    cam_write(bus, 0x380F, vts & 0xFF)

    #    Exposure: lines x 16 (the low 4 bits are fractions), 20 bits over
    #    three registers. Gain: 0x10 = 1x.
    lines = min(args.exposure, vts - EXPOSURE_OFFSET)
    expo = lines << 4
    cam_write(bus, 0x3500, (expo >> 16) & 0x0F)
    cam_write(bus, 0x3501, (expo >> 8) & 0xFF)
    cam_write(bus, 0x3502, expo & 0xFF)
    cam_write(bus, 0x3509, args.gain)

    # 9. Test pattern: bit 7 of 0x5E00 turns on the color bars
    cam_write(bus, 0x5E00, 0x80 if args.pattern else 0x00)

    # Read a few back to prove the writes landed
    width  = (cam_read(bus, 0x3808) << 8) | cam_read(bus, 0x3809)
    height = (cam_read(bus, 0x380A) << 8) | cam_read(bus, 0x380B)
    print(f"output size: {width} x {height}  (expect 640 x 400)")
    print(f"HTS = {HTS_DEFAULT}, VTS = {vts}, exposure = {lines} lines, gain = 0x{args.gain:02X}")
    print(f"test pattern: {'ON' if args.pattern else 'off'}")

    # 10. Go
    cam_write(bus, 0x0100, 0x01)
    print(f"streaming: 0x0100 = {cam_read(bus, 0x0100)}  (expect 1)")
    print("Trust the real frame rate from frame_period on the ILA: "
          "fps = 200,000,000 / frame_period")


if __name__ == "__main__":
    main()
