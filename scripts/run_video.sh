#!/usr/bin/env bash
#
# run_video.sh - run the firmware's UDP luma stream and verify frames on the host.
#
# The Renode process must be killed by BOTH names: the wrapper is
# "/Applications/renode/renode" but the real listener is
# "dotnet .../Renode.dll". Killing only the wrapper leaves a stale instance
# still bound to the port, and the client then silently talks to the OLD
# simulation instead of the one just built.
#
# Usage: ./scripts/run_video.sh [port] [width] [height] [frames] [fps]
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PORT="${1:-18080}"
W="${2:-320}"
H="${3:-240}"
FRAMES="${4:-3}"
FPS="${5:-24}"

# Stale receivers from an aborted run keep the UDP port bound, and a stale
# Renode keeps answering on it - both silently target the wrong simulation.
pkill -f "Renode.dll" 2>/dev/null || true
pkill -f "renode --disable-xwt" 2>/dev/null || true
pkill -f "video_client.py" 2>/dev/null || true
sleep 2
if lsof -nP -iUDP:"$PORT" >/dev/null 2>&1; then
    echo "ERROR: udp/$PORT already in use by:" >&2
    lsof -nP -iUDP:"$PORT" >&2
    exit 1
fi

# Makefile variable changes do not trigger recompiles on their own; the
# flag-dependent translation units must be touched or the ELF stays stale.
touch "$ROOT/firmware/Core/Src/main.c" "$ROOT/firmware/video_udp.c" "$ROOT/firmware/ov7670.c" \
      "$ROOT/firmware/i2c1.c" "$ROOT/firmware/Core/Src/spi.c" \
      "$ROOT/firmware/ov7670_dvp.c"
make -C "$ROOT/firmware" NCAP_VIDEO_UDP=1 NCAP_DVP="${DVP:-0}" NCAP_OV7670="${OV7670:-0}" \
     VIDEO_PORT="$PORT" VIDEO_HOST_IP=127.0.0.1 VIDEO_WIDTH="$W" VIDEO_HEIGHT="$H" \
     VIDEO_FPS="$FPS" DVP_FRAMES="${DVP_FRAMES:-2}" DVP_YUV420="${DVP_YUV420:-0}" \
     MICRO_AGENT_IP=127.0.0.1 MICRO_AGENT_PORT=8888 MICRO_CLIENT_PORT=5000 >/dev/null

UART="$(mktemp -t f4video)"
RSCR="$(mktemp -t f4resc)"
OUT="$(mktemp -t f4vid)"
cat > "$RSCR" <<EOF
include @renode_configs/peripherals/W5500.cs
include @renode_configs/peripherals/OV7670.cs
include @renode_configs/peripherals/Dcmi.cs
mach create "f4"
machine LoadPlatformDescription @renode_configs/platforms/boards/stm32f4_w5500.repl
sysbus LoadELF @firmware/build/ncap_f4_w5500.elf
logLevel 3
sysbus.usart2 CreateFileBackend @$UART true
start
sleep 600
quit
EOF

# Receiver first, so no early datagrams are lost.
python3 "$ROOT/scripts/video_client.py" "$PORT" "$FRAMES" > "$OUT" 2>&1 &
CLIENT=$!

cd "$ROOT"
/Applications/renode/renode --disable-xwt --console -e "i @$RSCR" >/dev/null 2>&1 &
RENODE=$!

wait $CLIENT || true

kill $RENODE 2>/dev/null || true
pkill -f "Renode.dll" 2>/dev/null || true

echo "=== receiver ==="
cat "$OUT"
echo "=== firmware ==="
tr -d '\000' < "$UART" | sed 's/\r/\n/g' | grep -a "F4: video" | tail -n 6
rm -f "$UART" "$OUT"