;@echo off
"tool-openocd/bin/openocd.exe" ^
  -s tool-openocd\scripts ^
  -f board/gd32e230-start.cfg ^
  -c init ^
  -c "rtt setup 0x20000000 10000 \"SEGGER RTT\"" ^
  -c "rtt start" ^
  -c "rtt server start 9090 0"
