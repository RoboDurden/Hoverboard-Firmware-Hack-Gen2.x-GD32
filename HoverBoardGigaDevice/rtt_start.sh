#!/bin/bash
# Start OpenOCD with RTT server on port 9090.
# Run rtt_read.py in another terminal to see the output.
# Kill this before flashing (pkill openocd).
/Users/alex/.platformio/packages/tool-openocd-gd32/bin/openocd \
  -f interface/stlink.cfg \
  -c "transport select hla_swd" \
  -c "set CPUTAPID 0" \
  -f target/stm32f1x.cfg \
  -c "rtt setup 0x20000000 0x2000 \"SEGGER RTT\"" \
  -c "init" \
  -c "rtt start" \
  -c "rtt server start 9090 0"
