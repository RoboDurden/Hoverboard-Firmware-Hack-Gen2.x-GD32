#!/bin/bash
# Start OpenOCD with RTT and read output, auto-reconnecting on MCU reset.
# Ctrl-C to exit. Run `pkill openocd` if stuck.

cleanup() {
    pkill -P $$ openocd 2>/dev/null
    pkill openocd 2>/dev/null
    exit 0
}
trap cleanup INT TERM

start_openocd() {
    /Users/alex/.platformio/packages/tool-openocd-gd32/bin/openocd \
      -f interface/stlink.cfg \
      -c "transport select hla_swd" \
      -c "set CPUTAPID 0" \
      -f target/stm32f1x.cfg \
      -c "rtt setup 0x20000000 0x2000 \"SEGGER RTT\"" \
      -c "init" \
      -c "rtt start" \
      -c "rtt server start 9090 0" > /tmp/openocd_rtt.log 2>&1 &
    OPENOCD_PID=$!
}

pkill openocd 2>/dev/null
sleep 1
start_openocd
sleep 3

python3 -c "
import socket, signal, sys, time, subprocess
signal.signal(signal.SIGINT, lambda *a: sys.exit(0))

def try_reset_rtt():
    '''Tell openocd to re-scan for the RTT control block (needed after MCU reset)'''
    try:
        t = socket.socket()
        t.settimeout(2.0)
        t.connect(('localhost', 4444))
        try: t.recv(2048)
        except: pass
        t.sendall(b'rtt stop\n')
        time.sleep(0.2)
        t.sendall(b'rtt start\n')
        time.sleep(0.2)
        t.close()
    except: pass

def connect():
    s = socket.socket()
    s.settimeout(2.0)
    for i in range(10):
        try:
            s.connect(('localhost', 9090))
            s.settimeout(1.0)
            return s
        except (ConnectionRefusedError, OSError):
            time.sleep(0.5)
    return None

print('RTT reader. Ctrl-C to quit.\n')
while True:
    s = connect()
    if s is None:
        print('\n[rtt] cannot connect, retrying openocd...')
        subprocess.run(['pkill', 'openocd'], check=False)
        time.sleep(1)
        subprocess.Popen(['sh', '-c', '''/Users/alex/.platformio/packages/tool-openocd-gd32/bin/openocd \\
            -f interface/stlink.cfg \\
            -c \"transport select hla_swd\" \\
            -c \"set CPUTAPID 0\" \\
            -f target/stm32f1x.cfg \\
            -c \"rtt setup 0x20000000 0x2000 \\\"SEGGER RTT\\\"\" \\
            -c init -c \"rtt start\" \\
            -c \"rtt server start 9090 0\" > /tmp/openocd_rtt.log 2>&1'''])
        time.sleep(3)
        continue

    last_data_time = time.time()
    while True:
        try:
            d = s.recv(4096)
            if d:
                print(d.decode('utf-8', errors='replace'), end='', flush=True)
                last_data_time = time.time()
            else:
                print('\n[rtt] connection closed')
                break
        except socket.timeout:
            # No data for too long? MCU might have reset
            if time.time() - last_data_time > 3:
                try_reset_rtt()
                last_data_time = time.time()
        except (ConnectionResetError, BrokenPipeError, OSError) as e:
            print(f'\n[rtt] socket error: {e}')
            break
    s.close()
    time.sleep(0.5)
"

cleanup
