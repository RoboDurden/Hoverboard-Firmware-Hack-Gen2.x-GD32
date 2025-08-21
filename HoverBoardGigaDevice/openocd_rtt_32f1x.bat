;@echo off
"tool-openocd/bin/openocd.exe" ^
  -s tool-openocd\scripts ^
  -f interface/stlink.cfg ^
  -c "transport select hla_swd" ^
  -f target/stm32f1x.cfg ^
  -c init ^
  -c "rtt setup 0x20000000 10000 \"SEGGER RTT\"" ^
  -c "rtt start" ^
  -c "rtt server start 9090 0"
