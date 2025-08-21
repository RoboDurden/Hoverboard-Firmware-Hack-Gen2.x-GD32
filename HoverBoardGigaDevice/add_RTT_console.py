Import("env")
import sys
import socket
import threading

def rtt_console(*args, **kwargs):
    HOST, PORT = "127.0.0.1", 9090

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST, PORT))
        print("✅ Connected to RTT at %s:%d" % (HOST, PORT))
    except Exception as e:
        print("❌ Could not connect to RTT:", e)
        return

    # Thread for receiving data from RTT
    def recv_thread():
        while True:
            try:
                data = s.recv(1024)
                if not data:
                    break
                sys.stdout.write(data.decode(errors="ignore"))
                sys.stdout.flush()
            except Exception:
                break

    t = threading.Thread(target=recv_thread, daemon=True)
    t.start()

    # Forward stdin → RTT
    try:
        for line in sys.stdin:
            s.sendall(line.encode())
    except KeyboardInterrupt:
        pass
    finally:
        s.close()

# Add as custom target
env.AddCustomTarget(
    "rtt_console",
    None,
    rtt_console,
    title="RTT Console",
    description="Open a poor-man's RTT console inside VS Code terminal"
)
