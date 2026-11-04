#!/usr/bin/env python3
# =============================================================================
# csi_status.py  --  read the MIPI CSI-2 receiver's status registers
# =============================================================================
#
# WHAT IT DOES
#   The CSI-2 receiver IP keeps counters and error flags in registers on its
#   AXI-Lite port. Reading them tells you WHERE the link is failing, even when
#   nothing reaches the ILA:
#
#     packet count = 0          -> nothing is arriving at all
#     packets, but errors       -> data arrives garbled (line rate, lanes)
#     line count / byte count   -> the size of what arrives
#                                  (expect 400 lines, 800 bytes per line,
#                                   because 640 RAW10 pixels = 640*10/8 bytes)
#     data type                 -> expect 0x2B (RAW10)
#
#   Register offsets and bit meanings are from AMD's MIPI CSI-2 Receiver
#   Subsystem guide (PG232). If a value looks like nonsense, compare the
#   offsets with the register table for YOUR IP version in PG232.
#
# HOW TO RUN (on the Kria)
#   sudo python3 csi_status.py                 # read once
#   sudo python3 csi_status.py --watch         # read every second
#   sudo python3 csi_status.py --clear         # also clear the error flags after reading
#   sudo python3 csi_status.py --base 0xB0010000   # if you used another address
#
#   The base address is the receiver's address in Vivado's Address Editor.
# =============================================================================

import argparse
import mmap
import os
import struct
import time

# ---- register offsets (PG232) ----
CORE_CONFIG   = 0x00   # bit 0 = core enable, bit 1 = soft reset
PROTOCOL_CFG  = 0x04   # bits [1:0] = active lanes - 1
CORE_STATUS   = 0x10   # bits [31:16] = packet count
INT_STATUS    = 0x24   # error / event flags (write 1 to clear)
CLK_LANE_INFO = 0x3C   # bit 1 = clock lane in stop state
LANE0_INFO    = 0x40   # bit 5 = stop state, bit 2 = SoT error, bit 1 = SoT sync error
LANE1_INFO    = 0x44
IMG_INFO1_VC0 = 0x60   # bits [31:16] = line count, [15:0] = byte count (virtual channel 0)
IMG_INFO2_VC0 = 0x64   # bits [5:0] = data type

# Interrupt status bits worth knowing about (PG232)
INT_BITS = {
    31: "frame received",
    30: "virtual channel frame error",
    20: "stream line buffer full (video clock too slow?)",
    19: "stop state",
    13: "SoT error (lane start-of-transmission)",
    12: "SoT sync error",
    11: "ECC 2-bit error (header corrupted)",
    10: "ECC 1-bit error (corrected)",
    9:  "CRC error (payload corrupted)",
    8:  "unsupported data type",
}


class Regs:
    """Read/write 32-bit registers through /dev/mem."""

    def __init__(self, base, size=0x10000):
        fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
        self.mem = mmap.mmap(fd, size, mmap.MAP_SHARED,
                             mmap.PROT_READ | mmap.PROT_WRITE, offset=base)
        os.close(fd)

    def read(self, offset):
        return struct.unpack_from("<I", self.mem, offset)[0]

    def write(self, offset, value):
        struct.pack_into("<I", self.mem, offset, value)


def report(r):
    core   = r.read(CORE_CONFIG)
    proto  = r.read(PROTOCOL_CFG)
    status = r.read(CORE_STATUS)
    isr    = r.read(INT_STATUS)
    clk    = r.read(CLK_LANE_INFO)
    l0     = r.read(LANE0_INFO)
    l1     = r.read(LANE1_INFO)
    img1   = r.read(IMG_INFO1_VC0)
    img2   = r.read(IMG_INFO2_VC0)

    print(f"core enabled      : {core & 1}   (CORE_CONFIG = 0x{core:08X}, expect bit 0 = 1)")
    print(f"active lanes      : {(proto & 3) + 1}   (expect 2)")
    print(f"packet count      : {status >> 16}   (should keep going up while streaming)")
    print(f"lines / bytes     : {img1 >> 16} lines, {img1 & 0xFFFF} bytes per line"
          f"   (expect 400, 800)")
    print(f"data type         : 0x{img2 & 0x3F:02X}   (expect 0x2B = RAW10)")
    print(f"clock lane info   : 0x{clk:08X}")
    print(f"lane 0 / 1 info   : 0x{l0:08X} / 0x{l1:08X}")
    print(f"interrupt status  : 0x{isr:08X}")
    for bit, name in INT_BITS.items():
        if isr & (1 << bit):
            print(f"    bit {bit:2d}: {name}")
    return isr


def main():
    p = argparse.ArgumentParser(description="Read the CSI-2 receiver status")
    p.add_argument("--base", type=lambda s: int(s, 0), default=0xB0010000,
                   help="receiver base address from the Address Editor")
    p.add_argument("--watch", action="store_true", help="repeat every second")
    p.add_argument("--clear", action="store_true", help="clear the flags after reading")
    p.add_argument("--enable", action="store_true", help="set the core-enable bit first")
    args = p.parse_args()

    r = Regs(args.base)

    if args.enable:
        r.write(CORE_CONFIG, r.read(CORE_CONFIG) | 1)

    while True:
        isr = report(r)
        if args.clear and isr:
            r.write(INT_STATUS, isr)      # write 1s back to clear those flags
        if not args.watch:
            break
        print("-" * 60)
        time.sleep(1)


if __name__ == "__main__":
    main()
