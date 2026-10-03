#!/usr/bin/env bash
#
# run_5node_video.sh - run N simulated STM32F4+W5500 nodes streaming video in
# one Renode emulation. No micro-ROS, no agent: each node streams grayscale
# luma over UDP to its own host port.
#
# Usage: ./scripts/run_5node_video.sh [--gui] [count] [base_port] [ov7670]
#   --gui      run Renode with UI (per-node analyzers); default headless
#   count      number of nodes (default 5)
#   base_port  first host UDP port; node i uses base_port+i (default 18080)
#   ov7670     1 = sensor-fed frames, 0 = synthetic (default 1)
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

GUI=0
if [ "${1:-}" = "--gui" ]; then GUI=1; shift; fi
COUNT="${1:-5}"
BASE_PORT="${2:-18080}"
# Default to synthetic for multi-node: 5x concurrent I2C bulk fetch of 76.8 KB
# per frame exceeds what the host simulates in reasonable time (sensor-fed is
# proven solo via OV7670=1 with count=1). Same bytes either way.
OV7670="${3:-0}"

# Kill stale Renode (both names) and stale receivers; a leftover holding a
# port silently captures that node's stream.
pkill -f "Renode.dll" 2>/dev/null || true
pkill -f "renode --disable-xwt" 2>/dev/null || true
pkill -f "video_client.py" 2>/dev/null || true
sleep 2
for i in $(seq 0 $((COUNT-1))); do
    P=$((BASE_PORT+i))
    if lsof -nP -iUDP:"$P" >/dev/null 2>&1; then
        echo "ERROR: udp/$P already in use" >&2; exit 1
    fi
done

echo "=== building $COUNT video firmware images (OV7670=$OV7670) ==="
for i in $(seq 0 $((COUNT-1))); do
    # Isolated build dir per node: rebuilding one tree with different -D flags
    # back-to-back silently reuses stale objects.
    # Fresh object dir every time: back-to-back builds with different -D
    # flags in one tree silently reuse stale objects.
    rm -rf "$ROOT/firmware/build_vn${i}"
    make -C "$ROOT/firmware" BUILD="build_vn${i}" TARGET="ncap_vid_n${i}" \
        NODE_ID="$i" NCAP_VIDEO_UDP=1 NCAP_OV7670="$OV7670" \
        VIDEO_PORT=$((BASE_PORT+i)) VIDEO_SRC_PORT=$((5000+i)) \
        MICRO_AGENT_IP=127.0.0.1 MICRO_AGENT_PORT=8888 MICRO_CLIENT_PORT=$((5000+i)) >/dev/null
    echo "  node $i: build_vn${i}/ncap_vid_n${i}.elf (-> 127.0.0.1:$((BASE_PORT+i)))"
done

RSCR="$(mktemp -t f4_5vid)"
UARTDIR="$(mktemp -d -t f4_5vid_uart)"
{
echo 'include @renode_configs/peripherals/W5500.cs'
echo 'include @renode_configs/peripherals/OV7670.cs'
for i in $(seq 0 $((COUNT-1))); do
    echo "mach create \"v${i}\""
    echo "machine LoadPlatformDescription @renode_configs/platforms/boards/stm32f4_w5500.repl"
    echo "sysbus LoadELF @firmware/build_vn${i}/ncap_vid_n${i}.elf"
    echo "logLevel 1"
    echo "sysbus.usart2 CreateFileBackend @${UARTDIR}/v${i}.log true"
    if [ "$GUI" = 1 ]; then
        echo "showAnalyzer usart2"
    fi
done
for i in $(seq 0 $((COUNT-1))); do
    echo "mach set \"v${i}\""
    echo "start"
done
echo "sleep 1200"
echo "quit"
} > "$RSCR"

# Receivers first so no early datagrams are lost.
VIDOUT="$(mktemp -d -t f4_5vid_out)"
for i in $(seq 0 $((COUNT-1))); do
    python3 "$ROOT/scripts/video_client.py" $((BASE_PORT+i)) 2 > "$VIDOUT/v${i}.log" 2>&1 &
done

echo "=== launching Renode ($([ "$GUI" = 1 ] && echo GUI || echo headless)) ==="
cd "$ROOT"
if [ "$GUI" = 1 ]; then
    /Applications/renode/renode -e "i @$RSCR" >/dev/null 2>&1 &
else
    /Applications/renode/renode --disable-xwt --console -e "i @$RSCR" >/dev/null 2>&1 &
fi
RENODE=$!
echo "uart logs: $UARTDIR | receivers: $VIDOUT | renode pid $RENODE"
echo "watch with: tail -f $UARTDIR/v0.log"
