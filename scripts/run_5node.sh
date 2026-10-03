#!/usr/bin/env bash
#
# run_5node.sh - run N simulated STM32F4+W5500 nodes in one Renode emulation.
#
# Each node gets its own firmware ELF (NODE_ID), MAC/IP, client key, source
# port and ROS topic names, so a single micro-ROS agent multiplexes them all
# without collisions. Usage:
#   ./scripts/run_5node.sh [--gui] [count] [agent_port]
#
# --gui runs Renode with its UI (analyzers per node); default is headless with
# per-node UART logs. Needs the agent image locally; starts it fresh.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

GUI=0
if [ "${1:-}" = "--gui" ]; then GUI=1; shift; fi
COUNT="${1:-5}"
AGENT_PORT="${2:-8888}"

# Stale Renode (both process names) or stale clients silently answer on the
# same ports, so kill everything first.
pkill -f "Renode.dll" 2>/dev/null || true
pkill -f "renode --disable-xwt" 2>/dev/null || true
pkill -f "video_client.py" 2>/dev/null || true
sleep 2

# Distinct host UDP source ports per node (model binds Sn_PORT; equal ports
# would self-send or clash).
for i in $(seq 0 $((COUNT-1))); do
    P=$((5000+i))
    if lsof -nP -iUDP:"$P" >/dev/null 2>&1; then
        echo "ERROR: udp/$P already in use" >&2; exit 1
    fi
done

echo "=== building $COUNT firmware images ==="
for i in $(seq 0 $((COUNT-1))); do
    # Isolated build dir per node (same pattern as Distributed/mesh_five_node):
    # rebuilding one tree with different -D flags back-to-back silently reuses
    # stale objects, producing byte-identical ELFs for every node.
    make -C "$ROOT/firmware" BUILD="build_n${i}" TARGET="ncap_5node_n${i}" \
        NODE_ID="$i" NCAP_VIDEO_UDP=0 \
        MICRO_AGENT_IP=127.0.0.1 MICRO_AGENT_PORT="$AGENT_PORT" \
        MICRO_CLIENT_PORT=$((5000+i)) >/dev/null
    echo "  node $i: build_n${i}/ncap_5node_n${i}.elf (client port $((5000+i)))"
done

echo "=== starting agent ==="
docker rm -f ncap-agent-5node >/dev/null 2>&1 || true
docker run -d --name ncap-agent-5node -p "$AGENT_PORT:$AGENT_PORT/udp" \
    microros/micro-ros-agent:humble udp4 -p "$AGENT_PORT" -v 4 -d 0 >/dev/null 2>&1
sleep 3

RSCR="$(mktemp -t f4_5node)"
UARTDIR="$(mktemp -d -t f4_5node_uart)"
{
echo 'include @renode_configs/peripherals/W5500.cs'
echo 'include @renode_configs/peripherals/OV7670.cs'
for i in $(seq 0 $((COUNT-1))); do
    echo "mach create \"n${i}\""
    echo "machine LoadPlatformDescription @renode_configs/platforms/boards/stm32f4_w5500.repl"
    echo "sysbus LoadELF @firmware/build_n${i}/ncap_5node_n${i}.elf"
    echo "logLevel 1"
    echo "sysbus.usart2 CreateFileBackend @${UARTDIR}/n${i}.log true"
    # Distinct IMU stimuli per node so the data is attributable (small values;
    # the model scales by sensitivity and large ones overflow int16).
    AZ=$(python3 -c "print(1.0+0.2*$i)")
    GX=$(python3 -c "print(float($i))")
    echo "i2c1.lsm9ds1_imu FeedAccelerationSample 0 0 $AZ 100000"
    echo "i2c1.lsm9ds1_imu FeedAngularRateSample $GX 0 0 100000"
    if [ "$GUI" = 1 ]; then
        echo "showAnalyzer usart2"
    fi
done
for i in $(seq 0 $((COUNT-1))); do
    echo "mach set \"n${i}\""
    echo "start"
done
echo "sleep 1200"
echo "quit"
} > "$RSCR"

echo "=== launching Renode ($([ "$GUI" = 1 ] && echo GUI || echo headless)) ==="
cd "$ROOT"
if [ "$GUI" = 1 ]; then
    /Applications/renode/renode -e "i @$RSCR" >/dev/null 2>&1 &
else
    /Applications/renode/renode --disable-xwt --console -e "i @$RSCR" >/dev/null 2>&1 &
fi
RENODE=$!
echo "uart logs: $UARTDIR (resc: $RSCR, renode pid $RENODE)"
echo "watch with: tail -f $UARTDIR/n0.log"
