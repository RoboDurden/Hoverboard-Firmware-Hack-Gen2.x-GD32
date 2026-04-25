#!/usr/bin/env bash
# rtt_supervisor.sh — keep openocd + plotter alive across MCU resets.
#
# The GD32 wipes RAM (and with it the SEGGER_RTT control block) on every
# reset/brown-out, which leaves openocd holding a stale handle and the
# plotter starving silently. This watches /tmp/rtt.log's mtime (rtt_plot
# tees every line there) and, if it goes stale for STALL_SECS, kills
# openocd and restarts it — the plotter's own auto-reconnect picks
# up the fresh stream.
#
# Usage: just run it. Ctrl-C to stop (kills the plotter window too).

set -u

PROJ=/Users/alex/dev/c/Hoverboard-Firmware-Hack-Gen2.x-GD32/HoverBoardGigaDevice
LOG=/tmp/rtt.log
STALL_SECS=3
ENV=genericGD32F130C8

start_openocd() {
    # Kill any stragglers first so we own the usb/stlink cleanly.
    pkill -f "tool-openocd/bin/openocd" 2>/dev/null || true
    sleep 0.3
    pio run -e "$ENV" -t "RTT Start" -d "$PROJ" >/tmp/rtt_openocd.log 2>&1 &
    echo $!
}

cleanup() {
    echo ""
    echo "[supervisor] shutting down..."
    [ -n "${PLOT_PID:-}" ] && kill "$PLOT_PID" 2>/dev/null || true
    [ -n "${OOCD_PID:-}" ] && kill "$OOCD_PID" 2>/dev/null || true
    pkill -f "tool-openocd/bin/openocd" 2>/dev/null || true
    pkill -f "rtt_plot.py" 2>/dev/null || true
    exit 0
}
trap cleanup INT TERM

# --- boot ------------------------------------------------------------------
rm -f "$LOG"
touch "$LOG"

echo "[supervisor] starting openocd..."
OOCD_PID=$(start_openocd)

# Wait for port 9090 to listen before launching the plotter
for _ in $(seq 1 30); do
    if nc -z localhost 9090 2>/dev/null; then break; fi
    sleep 0.3
done
echo "[supervisor] openocd up (pid $OOCD_PID)"

echo "[supervisor] starting plotter..."
"$PROJ/.venv/bin/python" "$PROJ/rtt_plot.py" &
PLOT_PID=$!

# --- watch loop ------------------------------------------------------------
echo "[supervisor] watching $LOG — restart when stalled > ${STALL_SECS}s"
touch "$LOG"  # reset baseline now that plotter is running

while true; do
    sleep 1

    # if plotter died (user closed window), shut everything down
    if ! kill -0 "$PLOT_PID" 2>/dev/null; then
        echo "[supervisor] plotter exited — shutting down"
        cleanup
    fi

    MTIME=$(stat -f %m "$LOG" 2>/dev/null || echo 0)
    NOW=$(date +%s)
    AGE=$((NOW - MTIME))

    if [ "$AGE" -gt "$STALL_SECS" ]; then
        echo "[supervisor] RTT stall (${AGE}s) — restarting openocd"
        kill "$OOCD_PID" 2>/dev/null || true
        pkill -f "tool-openocd/bin/openocd" 2>/dev/null || true
        sleep 0.5
        OOCD_PID=$(start_openocd)
        for _ in $(seq 1 30); do
            if nc -z localhost 9090 2>/dev/null; then break; fi
            sleep 0.3
        done
        echo "[supervisor] openocd restarted (pid $OOCD_PID)"
        touch "$LOG"  # reset mtime baseline
    fi
done
