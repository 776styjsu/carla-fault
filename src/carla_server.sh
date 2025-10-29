#!/bin/bash
#
# carla_server.sh
#
# Usage:
#   ./carla_server.sh start [PORT] [interactive|headless]
#   ./carla_server.sh stop
#
# Examples:
#   ./carla_server.sh start               # port 2000, interactive window
#   ./carla_server.sh start 2100 headless # port 2100, offscreen/headless
#   ./carla_server.sh stop                # kill the saved PID
#
# This script:
#   - launches CarlaUE4 in server mode
#   - records its PID in .carla_server.pid
#   - can later kill that PID cleanly
#
# NOTE:
#   Make sure CARLA_ROOT points at the CARLA install that has CarlaUE4.sh

set -euo pipefail

# >>>> EDIT THIS <<<<
CARLA_ROOT="${CARLA_ROOT:?Please export CARLA_ROOT (e.g., "path/to/carla-0.9.15") before running this script}"
UE_BIN="$CARLA_ROOT/CarlaUE4.sh"
PIDFILE=".carla_server.pid"

start_carla() {
    local PORT="${1:-2000}"
    local MODE="${2:-interactive}"

    if [ ! -x "$UE_BIN" ]; then
        echo "[error] Can't find CarlaUE4.sh at $UE_BIN"
        exit 1
    fi

    if [ -f "$PIDFILE" ]; then
        if kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
            echo "[warn] CARLA already running with PID $(cat "$PIDFILE"). Stop it first."
            exit 1
        else
            echo "[warn] Stale $PIDFILE found. Removing."
            rm -f "$PIDFILE"
        fi
    fi

    echo "[info] Launching CARLA on port $PORT in mode '$MODE'..."

    if [ "$MODE" = "headless" ]; then
        # Headless / offscreen mode for servers or Slurm nodes with no display.
        # -RenderOffScreen and -nosound are common flags to avoid X/audio deps.
        SDL_VIDEODRIVER=offscreen \
        "$UE_BIN" \
          -carla-server \
          -carla-port="$PORT" \
          -RenderOffScreen \
          -nosound \
          -quality-level=Low \
          -fps=20 \
          > carla_server.out 2>&1 &
    else
        # Interactive mode with a visible UE4 window for local debugging.
        "$UE_BIN" \
          -carla-server \
          -carla-port="$PORT" \
          -quality-level=Epic \
          -fps=30 \
          > carla_server.out 2>&1 &
    fi

    CARLA_PID=$!
    echo "$CARLA_PID" > "$PIDFILE"
    echo "[info] CARLA PID = $CARLA_PID"
    echo "[info] Waiting ~5s for server to finish booting..."
    sleep 5
    echo "[info] Done. You can now run main.py (the collector) on port $PORT."
}

stop_carla() {
    if [ -f "$PIDFILE" ]; then
        local PID
        PID="$(cat "$PIDFILE")"
        if kill -0 "$PID" 2>/dev/null; then
            echo "[info] Stopping CARLA PID $PID"
            kill "$PID"
            # give it a moment to exit gracefully
            sleep 2
            if kill -0 "$PID" 2>/dev/null; then
                echo "[warn] PID still alive, sending SIGKILL"
                kill -9 "$PID" || true
            fi
        else
            echo "[warn] PID $PID not running. Cleaning up stale pidfile."
        fi
        rm -f "$PIDFILE"
    else
        echo "[warn] No $PIDFILE found. Attempting fallback pkill..."
        pkill -f CarlaUE4 || true
    fi
    echo "[info] CARLA stopped."
}

cmd="${1:-}"
case "$cmd" in
    start)
        # shift off "start" and pass the rest to start_carla
        shift
        start_carla "$@"
        ;;
    stop)
        stop_carla
        ;;
    *)
        echo "Usage:"
        echo "  $0 start [PORT] [interactive|headless]"
        echo "  $0 stop"
        exit 1
        ;;
esac
