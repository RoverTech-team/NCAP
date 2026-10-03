# STM32F4 + W5500 Renode Simulation

Simulate an STM32F4 (Cortex-M4) board with a WIZnet W5500 SPI Ethernet
controller (hardwired TCP/IP, 8 sockets) in Renode 1.16. Upstream Renode has
no W5500 model, so this project ships one as a C# `ISPIPeripheral`.

## Layout

| Path | Purpose |
|------|---------|
| `renode_configs/peripherals/W5500.cs` | W5500 model (SPI frame parsing, common + socket regs, 16 KB TX/RX per socket, `INTn` IRQ) |
| `renode_configs/platforms/boards/stm32f4_w5500.repl` | Board: F4 (`platforms/cpus/stm32f4.repl` from the Renode install) + `w5500 @ spi1`, `INTn -> gpioPortB@1` |
| `renode_configs/scripts/project/f4_w5500_single.resc` | Single-node simulation script |
| `renode_configs/tests/project/test_f4_w5500.robot` | Smoke + datagram-fidelity tests (`renode-test`) |
| `renode_configs/tests/w5500_model/` | Standalone model test project (`dotnet run`, no Renode needed; incl. `EthernetTests.cs` for the ARP/IPv4/UDP path) |
| `renode_configs/peripherals/OV7670.cs` | OV7670 SCCB camera model (QVGA/YUV422, synthetic scene) |
| `renode_configs/scripts/project/f4_w5500_tap.resc` | Ethernet-mode run via switch + host TAP (needs TUNTAP kext) |
| `scripts/test_w5500_model.sh` | Runs the model test suite |
| `scripts/run_video.sh` | Builds + runs the UDP video stream, verifies frames (`OV7670=1` for sensor) |
| `scripts/video_client.py` | Host UDP receiver / frame verifier |
| `scripts/run_5node.sh` | N parallel micro-ROS nodes, one agent (`--gui` supported) |
| `scripts/run_5node_video.sh` | N parallel video streams, one port per node |
| `firmware/i2c1.c`, `firmware/imu.c` | Minimal I2C1 master + LSM9DS1 driver (`sensor_msgs/Imu` on `/imu/data`) |
| `firmware/ov7670.c` | OV7670 SCCB init + luma-row fetch |
| `firmware/video_udp.c` | Framed mono8 UDP video stream (standalone or alongside micro-ROS) |

## Run

From this directory:

```bash
dotnet /Applications/renode/output/bin/Release/Renode.dll --ui renode_configs/scripts/project/f4_w5500_single.resc
# in the monitor, with F4 + ioLibrary firmware:
sysbus LoadELF @/path/to/f4_w5500.elf
start
```

## Test

```bash
./scripts/test_w5500_model.sh   # 47 checks: SPI framing, socket lifecycle, UDP headers, host-bridge echo, Ethernet ARP/IPv4/UDP
renode-test renode_configs/tests/project/test_f4_w5500.robot   # needs renode-test on PATH
```

## Modes

The W5500 is hardwired TCP/IP with three modes:
- `LoopbackMode=true` (default): `SEND` loops TX→RX inside the model (UDP packets
  get the `DIPR/DPORT/LEN` header ioLibrary `recvfrom` expects).
- `EnableHostSockets=true` (`spi1.w5500 EnableHostSockets true` in the monitor):
  TCP/UDP sockets bridge to real host sockets (e.g. micro-ROS agent on UDP :8888).
- `UseEthernet=true`: the model is a real Renode MAC (`IMACInterface`,
  ARP/IPv4/UDP terminated internally) — attach with
  `connector Connect spi1.w5500 switch`. Needs a host TAP (TUNTAP kext);
  see `f4_w5500_tap.resc`.

## F4 micro-ROS firmware notes

Reference transport: `microroseth/.../extra_sources/microros_transports/w5500_transport.c`
(same 4-function shape as `udp_transport.c`, ioLibrary calls instead of LwIP).
Checklist: vendor `ioLibrary_Driver`, `wizchip_init()` + netinfo before rclc init,
add the transport to `C_SOURCES`, register via `rmw_uros_set_custom_transport()`
(reserve socket 0 for the XRCE session), rebuild `libmicroros` for
`-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=soft`.

---

## End-to-end status

**Working (verified):** STM32F4 firmware -> SPI1 -> W5500 model -> host UDP -> peer,
with a full round trip.

```
NCAP F4+W5500 booting
W5500 VERSIONR=04          <- W5500 read over SPI (CSn-framed VDM frames)
W5500 SPI OK
socket(0,UDP)=00
sendto=04
recvfrom=08 from port B8   <- reply came back into the W5500 RX buffer
AGENT REACHABLE
```

The datagram really leaves the model — a UDP echo server on `127.0.0.1:8888`
logged `GOT 4 bytes from ('127.0.0.1', 5000): deadbeef`.

Run it: `./scripts/run_e2e.sh` (builds firmware, ensures the agent container, starts Renode).

### Gotchas found while getting here

* **Client port must differ from the agent port.** With host-socket bridging the model
  binds the socket's local port, so a client port of 8888 would send to itself. Firmware
  uses `CLIENT_PORT 5000` -> `AGENT_PORT 8888`.
* **`STM32SPI` never calls `FinishTransmission()`**, so the model delimits SPI frames by
  the real `SCSn` GPIO (`w5500@0`) instead. `RSTn` is `w5500@1`.
* **HAL timebase must be SysTick.** The CubeMX project used TIM14; with it removed and
  `SysTick_Handler` calling `HAL_IncTick`, `HAL_Delay` works in Renode.
* **ioLibrary master (v3.2.0) has no `wizchip_init()`** — it uses the `WIZCHIP` callback
  struct, and device registers are programmed directly by `wizchip_port.c`.

### micro-ROS client history: transport proven, handshake was blocked (resolved below)

`firmware/Core/Src/main.c` builds a real micro-ROS client against the bundled
`libmicroros.a` (84 KB image): transport -> `rclc_support_init_with_options()` ->
node -> `heartbeat` publisher -> executor spin.

**Fixed along the way:** XRCE framing was the first blocker. With
`rmw_uros_set_custom_transport(true, ...)` the client emits a 31-byte *framed*
message that the agent silently drops. With `framing=false` it emits the correct
24-byte packet-mode `GET_INFO`, and the agent accepts it:

```
recv_message | client_key: 0x00000000, len: 24, data:
0000: 80 00 00 00 00 01 10 00 58 52 43 45 01 00 01 0F 34 29 40 76 81 00 FC 01
create_client     | create  | client_key: 0x34294076, session_id: 0x81
establish_session | session established
send_message      | 81 00 00 00 04 01 0B 00 00 00 58 52 43 45 01 00 01 0F 00
```

The reply is a well-formed `STATUS_AGENT` (id 0x04; this XRCE numbering has
`GET_INFO=0x00`, `STATUS_AGENT=0x04`, `FRAGMENT=0x0D`, matching the client's
`cmp r3, #13` in `session.c.obj`).

## Root cause (resolved)

The blocker was **not** the network, the transport, XRCE framing, or a
client/agent version mismatch. Every one of those hypotheses was disproved by
instrumenting the transport.

All six `rclc_*` entry points used here return **`rcl_ret_t`**, and
`RCL_RET_OK` is `RMW_RET_OK`, i.e. **`0`**:

```c
rcl_ret_t rclc_support_init_with_options(...);   // rclc/init.h
rcl_ret_t rclc_node_init_default(...);            // rclc/node.h
rcl_ret_t rclc_publisher_init_default(...);       // rclc/publisher.h
rcl_ret_t rclc_executor_init(...);                // rclc/executor.h
rcl_ret_t rclc_executor_add_timer(...);           // rclc/executor.h
rcl_ret_t rclc_timer_init_default(...);           // rclc/timer.h
```

The firmware tested them with `if (!ret)`, which inverts the result: **every
successful init printed `FAILED` and the firmware parked in `while (1) { }`**.
`support-init FAILED` therefore reported success. Each check must be
`if (ret != RCL_RET_OK)`.

Two related bugs fixed at the same time:

- `rclc_executor_add_timer()` was called *before* `rclc_timer_init_default()`.
  The executor stores a handle into the timer struct, so registering a
  zero-initialised `rcl_timer_t` left the executor with a stale handle.
  Initialise the timer first, then add it.
- The static RMW pool was not declared 8-byte aligned, which made allocation
  results depend on unrelated code changes.

## Working end-to-end

```
STM32F446 firmware (libmicroros.a)
  -> SPI1 -> W5500 Renode model (UDP, host-socket bridge)
  -> micro-ROS agent (microros/micro-ros-agent:humble)
  -> ROS 2  std_msgs/msg/Int32  on /heartbeat
```

Agent log:

```
create_client      | create          | client_key: 0x34294076, session_id: 0x81
create_participant | participant created | participant_id: 0x000(1)
create_topic       | topic created   | topic_id: 0x000(2)
create_publisher   | publisher created | publisher_id: 0x000(3)
create_datawriter  | datawriter created | datawriter_id: 0x000(5)
```

ROS 2 side (run a `ros2` CLI in a container sharing the agent's network
namespace, because DDS discovery is multicast and does not cross Docker's
bridge to the macOS host):

```bash
docker run --rm --network container:ncap-agent \
  --entrypoint bash microros/micro-ros-agent:humble -c \
  "source /opt/ros/humble/setup.bash; ros2 topic echo /heartbeat std_msgs/msg/Int32"
```
```
data: 1
data: 2
```

The topic is `/heartbeat` on the ROS 2 side: the micro-ROS agent strips the
`rt/` prefix that XRCE uses.

Note on timing: Renode's simulated clock advances far slower than wall time for
this firmware, so a 1 Hz publish timer produces roughly one message per minute of
wall clock. Let a simulation run for several minutes before concluding that
publishing is dead.

### Tracing datagrams

The transport hex-dumps every XRCE datagram when built with `make W5500_TDEBUG=1`
(`firmware/w5500_transport.c`). It is **off by default**: a live session sends
several datagrams per second and the UART cannot keep up, which perturbs timing
and makes runs appear to hang.

### Host UDP relay (scripts/udp_relay.py)

`--network host` (what production `docker-compose.hil-host.yml` uses) is **not
available on Docker Desktop for macOS**: the container starts and logs
`running... port: 8888`, but the agent is unreachable from the host. Reaching it
via `-p 8888:8888/udp` works, but docker-proxy rewrites the UDP source port on
every datagram, so the agent sees `192.168.65.1:<random>` each time and
re-creates the session instead of resuming it.

`scripts/udp_relay.py` relays datagrams both ways through a single stable socket,
so it is useful for tracing and for decoupling the client from the agent. It does
**not** remove the churn, though: the relay's own datagrams still pass through
docker-proxy, so the agent still sees a new proxy port per datagram. Eliminating
that needs a real host network (Linux host or VM).

### Client port must differ from the agent port on a single host

This bit us and is worth stating plainly. In production the client firmware and
the agent are different hosts, so both can use port 8888 - which is why the
working production firmware defines explicit client IP, agent IP and gateway. On
one host, the model's host socket binds the W5500 source port; if that equals the
agent port, the socket receives its own datagram back and the agent is never
contacted (the firmware just waits forever). Hence `MICRO_CLIENT_PORT=5000` while
`MICRO_AGENT_PORT=8888`.

Rebuild with:
```bash
make -C firmware MICRO_AGENT_IP=127.0.0.1 MICRO_AGENT_PORT=8888 MICRO_CLIENT_PORT=5000
```
Changing these needs a clean or touched rebuild of `main.c` and
`w5500_transport.c` - Makefile variable changes alone do not trigger recompiles.

## Grayscale luma video stream (UDP)

`make NCAP_VIDEO_UDP=1` builds a firmware that streams synthetic grayscale luma
over UDP instead of speaking micro-ROS. This sidesteps DDS fragmentation,
needs no host-side `rmw_microxrcedds`, and involves no Docker agent at all.

```
firmware -> W5500 UDP datagrams (127.0.0.1:<port>) -> host receiver
```

Wire format: one self-describing datagram per row block
(`'V''F''R''0' | u32 seq | u16 w | u16 h | u16 row | u16 rows | u32 len | luma`),
preceded by a meta datagram with an HTTP-style preamble. Frames stream
line-block by line-block, so peak RAM is one row block - no full-frame buffer.

```bash
./scripts/run_video.sh 18080        # builds, runs Renode, verifies 3 frames
```

### Parallel micro-ROS + video (same run)

`NCAP_VIDEO_UDP` selects the firmware personality:

| Value | Behaviour |
|---|---|
| `0` | micro-ROS only (`/heartbeat` on DDS) |
| `1` | video UDP stream only |
| `2` | **both in parallel** |

In mode 2 the micro-ROS stack owns socket 0 and the video stream owns socket 1
(`VIDEO_SOCK`), with distinct source ports. The blocking
`rclc_executor_spin()` is replaced by bounded `rclc_executor_spin_some()`
slices with one video row block emitted per slice, so a single bare-metal loop
drives both. Verified in one simulation: `std_msgs/Int32` on `/heartbeat`
(data 10, 11, 12) **and** pixel-perfect 320x240 mono8 frames on the UDP
receiver, simultaneously.

Verified: `320x240 mono8, 76800 B/frame, min=0 max=255 mean~138` reassembled
pixel-perfect on the host.

### The 256-byte SPI burst trap

`firmware/wizchip_port.c` drives all W5500 SPI traffic through
`w5500_spi_write_burst()` / `w5500_spi_read_burst()`, which used a 256-byte
stack buffer and **silently returned without transferring anything larger**.
Small micro-ROS datagrams always fit, so the link looked healthy, but any payload
over 256 bytes was discarded while `TX_WR` still advanced - the peer faithfully
received zeros. Both helpers now loop in 64-byte chunks (CS stays asserted for
the whole burst, so framing is preserved). This was also the cause of the
earlier TCP video payload failure, not the `Sn_TX_W` path that was suspected.

## LSM9DS1 IMU over I2C1

The board description wires Renode's stock `Sensors.LSM9DS1_IMU` (0x6B) and
`Sensors.LSM9DS1_Magnetic` (0x1E) to `i2c1` - no custom model needed. The
firmware drives them with a minimal register-level I2C master (`i2c1.c`,
`imu.c`) and publishes `sensor_msgs/Imu` on `/imu/data` alongside the
heartbeat, from the same executor timer. IMU bring-up is non-fatal: a missing
sensor degrades to heartbeat-only.

Verified live values: fed 1 g on Z reads `linear_acceleration.z = 9.801 m/s^2`
in ROS 2; gyro tracks the fed rotation.

Two Renode/model quirks worth knowing:

- `STM32F1_I2C` forwards queued transmit bytes to the peripheral **on STOP**.
  Waiting BTF before STOP stalls, because BTF for trailing bytes is only
  produced by that flush. `i2c1_write_reg()` therefore stages on TXE and goes
  straight to STOP. Multi-byte master *reads* hit an unhandled register-read
  path that aborts the emulator, so `imu_read()` fetches each output byte with
  its own single-byte transaction.
- `FeedAccelerationSample` values are scaled by the configured sensitivity, so
  they must fit in `int16` after scaling (feed accel in g, gyro in dps, and
  keep them small - 9.8 g overflows). Use a large `repeat` count or the FIFO
  drains after one read.

## OV7670 camera model (SCCB)

`renode_configs/peripherals/OV7670.cs` is a register-faithful SCCB model
defaulting to QVGA YUV422: PID `0x76` / VER `0x73`, COM7 reset/format/resolution,
CLKRC/COM3/COM14/COM15/COM17/TSLB storage with side effects, and a deterministic
synthetic scene whose luma matches `video_udp.c`'s generator pixel-for-pixel.
It sits on `i2c1` at `0x21` alongside the IMU. Firmware (`ov7670.c`) runs a
real SCCB init sequence (reset, QVGA/YUV422, readback check) and fetches luma
rows for the streaming pipeline; `run_video.sh` takes `OV7670=1` to use the
sensor instead of the synthetic generator, with automatic fallback.

Pixel readout is via test registers `0xF0-0xF3`, documented in the model as a
stand-in for the PCLK/VSYNC/HREF/D0-D7 parallel bus. The bytes are identical
to bus luma (Y of YUYV); only the transfer differs. A parallel-bus capture
path is the natural next step and does not change anything downstream.

## Five nodes in one simulation

`./scripts/run_5node.sh [--gui] [count] [agent_port]` builds per-node firmware
and runs them simultaneously in a single Renode emulation, all against one
micro-ROS agent:

```bash
./scripts/run_5node.sh        # 5 nodes, headless, agent :8888
./scripts/run_5node.sh --gui 2 8889   # 2 nodes with Renode UI, agent :8889
```

Each node gets a `NODE_ID` baked in at build time: distinct MAC/IP
(`...:01+i` / `192.168.0.1x`), host source port (`5000+i`), ROS names
(`ncap_f4_i`, `heartbeat_i`, `imu/data_i`), and — critically — a unique DDS
client key via `rmw_uros_options_set_client_key()`, without which the agent
conflates all nodes into one. `NODE_ID=0` preserves the exact legacy
single-node personality (names, ports, default key).

Verified: 5 distinct agent clients, 5 `/heartbeat_i` and up to 5 `/imu/data_i`
in ROS 2 concurrently. IMU bring-up retries under load and degrades to
heartbeat-only rather than blocking; under 5-machine load some nodes'
time-sensitive I2C polling can still lose the race (proven load-related: the
same ELF passes solo).

Two structural lessons, both now in the scripts:

- **Isolated `BUILD=` dir per node** (same pattern as
  `Distributed/.../mesh_five_node.resc` + `BUILD_DIR=build_20`). Rebuilding one
  tree with different `-D` flags back-to-back silently reuses stale objects
  (same-second timestamps), producing byte-identical ELFs for every node.
- **Kill Renode by both process names** (`Renode.dll` and the wrapper) and
  assert ports are free, or a stale instance answers instead of the new one.

### Five parallel video streams

`./scripts/run_5node_video.sh [--gui] [count] [base_port] [ov7670]` runs N
video nodes in one emulation, each streaming to its own host UDP port
(`base_port+i`, source ports `5000+i`, isolated `build_vn{i}` dirs):

```bash
./scripts/run_5node_video.sh 5 18080 0   # 5x synthetic, verified below
```

Verified: all five receivers reassemble pixel-perfect 320x240 mono8 frames
concurrently. Multi-node defaults to the synthetic generator because 5x
concurrent I2C bulk fetch (76.8 KB/frame/node) exceeds practical sim time;
sensor-fed streaming is proven separately with `count=1, ov7670=1`.

## Image feed in the microk3 web app

The `VFR0` UDP stream ends in microk3's `/images` dashboard. That required a
small backend extension on their side (no fork of existing logic):

- New `microk3/vfr_stream.py`: `VfrUdpViewer` binds a UDP port, reassembles
  row-block datagrams by `(seq, row)` into frames, converts mono8 to JPEG with
  the same helpers as the ROS viewer, and exposes the identical
  `status()`/`snapshot()` shape so the frontend polls it uniformly.
- `socket_stream.py`: `udp` added to `SUPPORTED_SCHEMES`
  (`udp://host:port` canonicalizes to `udp://host:port/`).
- `app.py`: the subscribe endpoint branches on scheme to instantiate the
  right viewer. Unsubscribe, `/latest`, and feed listing work unchanged.
- `templates/images.html`: `udp://` added to the per-tile scheme dropdown
  (the subscribe call already passes `scheme` straight through, so no JS
  changes were needed).
- Tests: VFR parse unit test, full subscribe → stream → JPEG → unsubscribe
  e2e, and the pre-existing assertion that `udp://` raises was updated.
  Suite stays green (48 passed).

Verified live, both with a byte-identical fake sender and with real Renode
firmware (OV7670 sensor model → SCCB → W5500 → UDP → microk3 → JPEG tile):

```bash
# terminal 1: dashboard (no ROS needed for socket feeds)
cd Stm-Micro_ros-eth/microrosWs/microk3 && FLASK_PORT=5050 python3 app.py

# terminal 2: firmware stream (or run_video.sh equivalent on :18090)
# terminal 3: subscribe
curl -X POST 127.0.0.1:5050/api/socket/image/subscribe \
  -H 'Content-Type: application/json' \
  -d '{"scheme":"udp","host":"127.0.0.1","port":18090}'
# dashboard: 127.0.0.1:5050/images -> Socket tile, udp://, 127.0.0.1:18090
```

Operational lessons:

- **One binder per UDP port.** Two viewers cannot bind the same port; the
  second gets `Errno 48` and its tile reports bind failure. Unsubscribe (API
  or Stop in the UI) before reconnecting, and check `/api/socket/image/feeds`
  when a tile won't go live.
- **Kill everything between runs** (Renode by both process names, Flask, fake
  senders): a stale sender or viewer silently captures the port or serves old
  frames.
