#!/usr/bin/env bash
#
# test_w5500_model.sh — W5500 model datagram-fidelity tests (no Renode needed)
#
# Drives renode_configs/peripherals/W5500.cs directly with real SPI frames:
#   VERSIONR, socket lifecycle, TX_FSR/RX_RSR accounting,
#   TCP/UDP loopback, UDP RX header (ioLibrary recvfrom format),
#   host-bridge UDP echo via real localhost sockets.
#
# Usage:
#   ./scripts/test_w5500_model.sh
#
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

dotnet run --project "$PROJECT_ROOT/renode_configs/tests/w5500_model/w5500_model.csproj"
