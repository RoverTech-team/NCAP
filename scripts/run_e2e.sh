#!/usr/bin/env bash
#
# run_e2e.sh — NCAP end-to-end: F4 firmware -> W5500 model -> micro-ROS agent
#
# Usage:  ./scripts/run_e2e.sh
#
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

echo "=== Building F4 firmware ==="
make -C firmware -j4

echo ""
echo "=== Ensuring micro-ROS agent is up ==="
if ! docker ps --filter name=ncap-agent --format '{{.Names}}' | grep -q ncap-agent; then
    docker rm -f ncap-agent >/dev/null 2>&1 || true
    docker run -d --name ncap-agent -p 8888:8888/udp \
        microros/micro-ros-agent:humble udp4 --port 8888 >/dev/null
    echo "agent container started"
else
    echo "agent container already running"
fi

echo ""
echo "=== Running Renode (F4 + W5500) ==="
echo "UART output is on usart2; agent logs: docker logs -f ncap-agent"
dotnet /Applications/renode/output/bin/Release/Renode.dll --ui \
    renode_configs/scripts/project/f4_w5500_e2e.resc
