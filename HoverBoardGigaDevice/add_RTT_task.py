import shutil
import os
Import("env")

openocd = os.path.join(env.subst("$PROJECT_PACKAGES_DIR"),
                       "tool-openocd", "bin", "openocd")
if not os.path.isfile(openocd):
    # fall back to system install, e.g. Homebrew
    openocd = shutil.which("openocd") or openocd

env.AddCustomTarget(
    "RTT Start",
    None,
    "\"" + openocd + "\"" +
    " -f interface/stlink.cfg" +
    " -c \"transport select hla_swd\"" +
    " -f target/stm32f1x.cfg" +
    " -c init" +
    " -c \"rtt setup 0x20000000 10000 \\\"SEGGER RTT\\\"\"" +
    " -c \"rtt start\"" +
    " -c \"rtt server start 9090 0\""
)
