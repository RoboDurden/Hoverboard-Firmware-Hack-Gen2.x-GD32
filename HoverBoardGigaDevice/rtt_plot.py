#!/usr/bin/env python3
"""Live plot of FOC currents from the RTT log.

One figure, four panels (ADCs, αβ, dq + averages, Iα-Iβ Lissajous).
Reader thread auto-reconnects to openocd's RTT server on port 9090,
so `pio run -t upload` (which kills openocd) doesn't end the session.

openocd's RTT port is single-consumer, so a second reader (e.g. `nc
localhost 9090`) would steal bytes from this plotter. Instead, every
received line is also teed to /tmp/rtt.log; `tail -f /tmp/rtt.log`
from another terminal gives a live text view alongside the plot.

Close the window or Ctrl-C to quit.
"""
import os
import re
import socket
import sys
import threading
import time
from collections import deque

LOG_PATH = os.environ.get("RTT_TEE", "/tmp/rtt.log")

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation

PORT = 9090
LOG_HZ = 100
WINDOW = 2 * LOG_HZ         # 2 s
UPDATE_MS = 80              # plot refresh interval
STALL_TIMEOUT = 3.0         # seconds of silence before we force-reconnect

LINE_RE = re.compile(
    r"adc_a=\s*(-?\d+)\s+adc_b=\s*(-?\d+).*?"
    r"Ia:\s*([+-]?\d+)\s+Ib:\s*([+-]?\d+).*?"
    r"Ialpha:\s*([+-]?\d+)\s+Ibeta:\s*([+-]?\d+).*?"
    r"Id:\s*([+-]?\d+)\s+Iq:\s*([+-]?\d+).*?"
    r"Idavg:\s*([+-]?\d+)\s+Iqavg:\s*([+-]?\d+).*?"
    r"ang:\s*(\d+)"
)
RPM_RE = re.compile(r"rpm:\s*([+-]?\d+)")

adc_a  = deque([2000] * WINDOW, maxlen=WINDOW)
adc_b  = deque([2000] * WINDOW, maxlen=WINDOW)
ialpha = deque([0] * WINDOW, maxlen=WINDOW)
ibeta  = deque([0] * WINDOW, maxlen=WINDOW)
id_    = deque([0] * WINDOW, maxlen=WINDOW)
iq_    = deque([0] * WINDOW, maxlen=WINDOW)
id_avg = deque([0] * WINDOW, maxlen=WINDOW)
iq_avg = deque([0] * WINDOW, maxlen=WINDOW)
rpm    = deque([0] * WINDOW, maxlen=WINDOW)
last_rx = [time.monotonic()]   # mutable container so the reader thread can mutate


def rtt_reader():
    """Connect, read, parse, tee every line to LOG_PATH. On any error/stall,
    close + retry every 0.5s."""
    try:
        tee = open(LOG_PATH, "wb", buffering=0)
    except OSError as e:
        print(f"rtt_plot: could not open {LOG_PATH}: {e}", file=sys.stderr)
        tee = None

    while True:
        try:
            s = socket.socket()
            s.settimeout(1.0)
            s.connect(("localhost", PORT))
        except (ConnectionRefusedError, OSError):
            time.sleep(0.5)
            continue

        buf = b""
        try:
            while True:
                try:
                    data = s.recv(4096)
                except socket.timeout:
                    if time.monotonic() - last_rx[0] > STALL_TIMEOUT:
                        break  # stream is dead, force reconnect
                    continue
                if not data:
                    break
                last_rx[0] = time.monotonic()
                buf += data
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    if tee is not None:
                        try:
                            tee.write(raw + b"\n")
                        except OSError:
                            pass
                    line = raw.decode("utf-8", errors="replace")
                    m = LINE_RE.search(line)
                    if not m:
                        continue
                    try:
                        adc_a.append(int(m.group(1)))
                        adc_b.append(int(m.group(2)))
                        # ia/ib (phase) no longer plotted in main figure —
                        # skip groups 3,4
                        ialpha.append(int(m.group(5)))
                        ibeta.append(int(m.group(6)))
                        id_.append(int(m.group(7)))
                        iq_.append(int(m.group(8)))
                        id_avg.append(int(m.group(9)))
                        iq_avg.append(int(m.group(10)))
                    except (ValueError, IndexError):
                        continue
                    m_rpm = RPM_RE.search(line)
                    if m_rpm:
                        try:
                            rpm.append(int(m_rpm.group(1)))
                        except ValueError:
                            pass
        except (ConnectionResetError, OSError):
            pass
        finally:
            try:
                s.close()
            except OSError:
                pass
        time.sleep(0.5)


threading.Thread(target=rtt_reader, daemon=True).start()

# ----- figure layout --------------------------------------------------------
# Three stacked time-series on the left (dq on top: the most important
# diagnostic), tall Lissajous spanning the full right column.
fig = plt.figure(figsize=(13, 8))
gs  = fig.add_gridspec(3, 2, width_ratios=[2, 1], hspace=0.35, wspace=0.25)
ax_dq   = fig.add_subplot(gs[0, 0])
ax_ab   = fig.add_subplot(gs[1, 0], sharex=ax_dq)
ax_adc  = fig.add_subplot(gs[2, 0], sharex=ax_dq)
ax_liss = fig.add_subplot(gs[:, 1])

fig.suptitle("FOC current-sense live")

# Park (dq) — top-left, prime real estate.
l_id,     = ax_dq.plot([], [], label="Id", color="tab:red",   lw=0.6, alpha=0.5)
l_iq,     = ax_dq.plot([], [], label="Iq", color="tab:green", lw=0.6, alpha=0.5)
l_id_avg, = ax_dq.plot([], [], label="Id avg", color="tab:red",   lw=1.8)
l_iq_avg, = ax_dq.plot([], [], label="Iq avg", color="tab:green", lw=1.8)
ax_dq.axhline(0, color="gray", lw=0.5, ls="--")
ax_dq.set_ylim(-500, 500)
ax_dq.set_ylabel("Park (rotor)  d/q")
ax_dq.legend(loc="upper left", ncol=2, fontsize=9)
ax_dq.grid(True, alpha=0.3)

# Clarke (αβ) — middle-left.
l_alpha, = ax_ab.plot([], [], label="Iα", color="tab:cyan")
l_beta,  = ax_ab.plot([], [], label="Iβ", color="tab:olive")
ax_ab.axhline(0, color="gray", lw=0.5, ls="--")
ax_ab.set_ylim(-300, 300)
ax_ab.set_ylabel("Clarke (stator)  α/β")
ax_ab.legend(loc="upper left", ncol=2, fontsize=9)
ax_ab.grid(True, alpha=0.3)

# Raw ADCs — bottom-left, smallest mental priority.
l_a, = ax_adc.plot([], [], label="adc_a (phase A)", color="tab:blue")
l_b, = ax_adc.plot([], [], label="adc_b (phase B)", color="tab:orange")
ax_adc.axhline(2000, color="gray", lw=0.5, ls="--")
ax_adc.set_ylim(1900, 2100)
ax_adc.set_ylabel("ADC counts\n(1 A ≈ 128)")
ax_adc.set_xlabel(f"samples (rolling {WINDOW/LOG_HZ:.0f} s)")
ax_adc.legend(loc="upper left", ncol=2, fontsize=9)
ax_adc.grid(True, alpha=0.3)

# Fix the x-axis once (sharex propagates).
ax_dq.set_xlim(0, WINDOW)

# Iα/Iβ Lissajous — right column, full height, square.
l_liss, = ax_liss.plot([], [], color="tab:purple", lw=0.6, alpha=0.7)
ax_liss.set_xlim(-300, 300)
ax_liss.set_ylim(-300, 300)
ax_liss.set_aspect("equal", adjustable="box")
ax_liss.axhline(0, color="gray", lw=0.5)
ax_liss.axvline(0, color="gray", lw=0.5)
ax_liss.set_xlabel("Iα")
ax_liss.set_ylabel("Iβ")
ax_liss.set_title("Iα vs Iβ  (→ circle)")
ax_liss.grid(True, alpha=0.3)

rpm_text = ax_dq.text(
    0.985, 0.92, "", transform=ax_dq.transAxes, ha="right", va="top",
    fontsize=12, family="monospace",
    bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="gray", alpha=0.85),
)

x = list(range(WINDOW))


def update(_):
    try:
        l_a.set_data(x, list(adc_a))
        l_b.set_data(x, list(adc_b))
        l_alpha.set_data(x, list(ialpha))
        l_beta.set_data(x, list(ibeta))
        l_id.set_data(x, list(id_))
        l_iq.set_data(x, list(iq_))
        l_id_avg.set_data(x, list(id_avg))
        l_iq_avg.set_data(x, list(iq_avg))
        l_liss.set_data(list(ialpha), list(ibeta))
        rpm_val = rpm[-1] if rpm else 0
        age = time.monotonic() - last_rx[0]
        suffix = "  (no data)" if age > 2 else ""
        rpm_text.set_text(f"{rpm_val:+5d} RPM{suffix}")
    except Exception as e:
        print(f"update error: {e}", file=sys.stderr)
    return l_a, l_b, l_alpha, l_beta, l_id, l_iq, l_id_avg, l_iq_avg, l_liss, rpm_text


ani = animation.FuncAnimation(
    fig, update, interval=UPDATE_MS, blit=False, cache_frame_data=False,
)
plt.tight_layout()
try:
    plt.show()
except KeyboardInterrupt:
    pass
