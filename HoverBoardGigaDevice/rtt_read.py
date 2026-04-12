#!/usr/bin/env python3
"""Read RTT output from OpenOCD on port 9090. Run in a separate terminal."""
import socket, sys

port = int(sys.argv[1]) if len(sys.argv) > 1 else 9090
s = socket.socket()
try:
    s.connect(('localhost', port))
except ConnectionRefusedError:
    print(f"Cannot connect to localhost:{port} — is OpenOCD running?")
    sys.exit(1)
print(f"Connected to RTT on port {port}. Ctrl-C to quit.\n")
s.settimeout(1.0)
try:
    while True:
        try:
            d = s.recv(4096)
            if d:
                print(d.decode('utf-8', errors='replace'), end='', flush=True)
        except socket.timeout:
            pass
except KeyboardInterrupt:
    print("\nDisconnected.")
finally:
    s.close()
