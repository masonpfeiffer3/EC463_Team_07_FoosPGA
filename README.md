# 07-Team Repo
Template for team repo

<p align="center">
<img src="./images/thisismyteam.png" width="50%">
</p>
<p align="center">
</p>

## Team links
- [Team Google Drive](https://drive.google.com/drive/u/1/folders/1HSWW8e8HxwHPLZcLo6FfCO6m2fV2Ahi7)
- [Team Board](https://app.notion.com/p/Team07_ProjectBoard-3da2b21b7a028070acded2745a7e6e01?source=copy_link)

## Organization of this repo

Target hardware is the AMD/Xilinx Kria KV260, which has three programmable
tiles plus a display-only GPU. Each computer engineering folder below maps to
one tile, with its own toolchain and build output. The PL (FPGA fabric) is
the dependency root — its Vivado output (bitstream + XSA) is what the RPU
firmware build and the Linux device-tree overlay build against — so changes
there should land before RPU/APU integration follows.

- **`Computer_VerilogRTL`** — PL tile (FPGA fabric). Verilog/RTL and HLS
  sources built in Vivado/Vitis, producing the bitstream and hardware
  description (XSA).
- **`Computer_RTOSControl`** — RPU tile (Cortex-R5F, bare-metal/RTOS).
  Hard-real-time control code built in Vitis, loaded onto the R5 cores by
  Linux via remoteproc. Used for timing-critical control loops (e.g. rod
  actuation) that can't tolerate Linux scheduling jitter.
- **`Computer_LinuxCore`** — APU tile (Cortex-A53, Linux). Application logic,
  orchestration, and pre/post-processing running as normal userspace
  software under Ubuntu on the KV260.
- **`Electrical_SystemArchitecture`** — system-level wiring, power
  distribution, sensor/motor driver schematics, and pinout documentation
  tying the KV260 to the rest of the electronics.
- **`Mechanical_RodActuation`** — CAD, drawings, and design files for the
  rod actuation mechanism (motor/linkage mounts, rod guides, enclosure).

The rest of the layout (packaging of the Kria "accelerated application"
bundle, shared docs, scripts) is still TBD.

