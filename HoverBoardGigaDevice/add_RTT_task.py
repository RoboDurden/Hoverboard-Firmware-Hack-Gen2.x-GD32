Import("env")
env.AddCustomTarget(
    "RTT Start",
    None,
    "\"$PROJECT_PACKAGES_DIR/tool-openocd/bin/openocd\"" + 
    " -f interface/stlink.cfg" +
    " -c \"transport select hla_swd\"" + 
    " -f target/stm32f1x.cfg" +
    " -c init" +
    " -c \"rtt setup 0x20000000 10000 \\\"SEGGER RTT\\\"\"" +
    " -c \"rtt start\"" +
    " -c \"rtt server start 9090 0\""
)
