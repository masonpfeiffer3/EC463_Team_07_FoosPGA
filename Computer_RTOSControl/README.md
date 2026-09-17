# Computer_RTOSControl

RPU tile (Cortex-R5F, bare-metal/RTOS) — hard-real-time control code built in Vitis, loaded onto the R5 cores by Linux via remoteproc. Used for timing-critical control loops (e.g. rod actuation) that can't tolerate Linux scheduling jitter.
