# STM32F4 + W5500 Renode Simulation

Simulate an STM32F4 (Cortex-M4) board with a WIZnet W5500 SPI Ethernet
controller (hardwired TCP/IP, 8 sockets) in Renode 1.16. Upstream Renode has
no W5500 model, so this project ships one as a C# `ISPIPeripheral`.

## Architecture diagrams

| Diagram | File | Shows |
|---|---|---|
| Modules | [`docs/architecture-modules.drawio`](docs/architecture-modules.drawio) ([JSON](docs/architecture-modules.drawio.json)) | Every module grouped by role - Renode models, board description, application firmware, CubeMX glue, third-party, host tooling, external consumers - with the include and call edges between them |
| Software architecture | [`docs/architecture-software.drawio`](docs/architecture-software.drawio) ([JSON](docs/architecture-software.drawio.json)) | Runtime data path for all three firmware personalities (`NCAP_DVP`, synthetic, micro-ROS), the numbered per-frame interaction sequence, the timing budget, and the buffer arithmetic that dictates which resolutions fit in RAM |

Both are draw.io diagrams, supplied as native `.drawio` (XML) and `.drawio.json`;
open either directly in <https://app.diagrams.net>.

## Layout

| Path | Purpose |
|------|---------|
| `renode_configs/peripherals/W5500.cs` | W5500 model (SPI frame parsing, common + socket regs, 16 KB TX/RX per socket, `INTn` IRQ) |
| `renode_configs/platforms/boards/stm32f4_w5500.repl` | Board: F4 (`platforms/cpus/stm32f4.repl` from the Renode install) + `w5500 @ spi2`, `ov7670` DVP bus -> `dcmi`, `INTn -> gpioPortB@1` |
| `renode_configs/scripts/project/f4_w5500_single.resc` | Single-node simulation script |
| `renode_configs/tests/project/test_f4_w5500.robot` | Smoke + datagram-fidelity tests (`renode-test`) |
| `docs/architecture-*.drawio[.json]` | Architecture diagrams (modules, and software architecture / runtime interactions) |
| `renode_configs/tests/w5500_model/` | Standalone model test project (`dotnet run`, no Renode needed; incl. `EthernetTests.cs` for the ARP/IPv4/UDP path) |
| `renode_configs/peripherals/OV7670.cs` | OV7670 camera model: SCCB registers + DVP parallel output (PCLK/VSYNC/HREF/D0-D7), COM3/COM14 downscaling, synthetic scene |
| `renode_configs/peripherals/Dcmi.cs` | STM32F4 DCMI model (RM0090 ch.13 register map, no CGR/CCMR), byte capture on PCLK, 32-bit DMA requests, crop/snapshot |
| `renode_configs/scripts/project/f4_w5500_tap.resc` | Ethernet-mode run via switch + host TAP (needs TUNTAP kext) |
| `scripts/test_w5500_model.sh` | Runs the model test suite |
| `scripts/run_video.sh` | Builds + runs the UDP video stream, verifies frames (`DVP=1` for real OV7670 capture) |
| `scripts/video_client.py` | Host UDP receiver / frame verifier |
| `scripts/run_5node.sh` | N parallel micro-ROS nodes, one agent (`--gui` supported) |
| `scripts/run_5node_video.sh` | N parallel video streams, one port per node |
| `scripts/verify_sysusage.sh` | Builds the `NCAP_SYSUSAGE` x `NCAP_FREERTOS` matrix in scratch `BUILD=` dirs, checks RAM budget and the on/off symbol switch (`--clean` removes them) |
| `firmware/sysusage.c`, `.h` | System-usage telemetry on `/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`), sampled by a dedicated FreeRTOS task; off by default |
| `firmware/i2c1.c`, `firmware/imu.c` | Minimal I2C1 master + LSM9DS1 driver (`sensor_msgs/Imu` on `/imu/data`) |
| `firmware/ov7670.c` | OV7670 SCCB init (reset, YUV422, COM3/COM14 output scaling, readback check) |
| `firmware/ov7670_dvp.c`, `.h` | DCMI + DMA2 capture into a frame ring, in-place Y/Cb/Cr de-interleave |
| `firmware/video_udp.c` | Framed mono8 UDP video stream, deadline-paced to `VIDEO_FPS` (synthetic generator or DVP capture) |

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
* **`logLevel` is a disk-space hazard.** Renode's DMA model logs two lines per
  *rejected* transfer request. Whenever the video DMA's `NDTR` reaches 0, a run
  emits millions of lines: capturing Renode's console to a file produced
  **1.6 GB per run** and filled the disk, after which Renode's file backends
  silently fail to write and results look like phantom hardware bugs. The run
  scripts use `logLevel 3` (Error only), which emits none of it. If you capture
  console output, cap it.
* **Stale UDP receivers keep the port.** `pkill -f video_client.py` does not
  match them (the process name is `python3`), and a leftover client bound to
  `127.0.0.1:<port>` silently swallows every datagram so the next run looks
  dead. Check `lsof -nP -iUDP:<port>` before each run and kill by PID.
* **CMSIS names are full of traps.** `DCMI_IRQn` is **78** (not 83),
  `DCMI_BASE` is **0x50050000** (not 0x50000000), the DMA stream registers are
  `PAR`/`M0AR`/`NDTR` (not `CPAR`/`CMAR`/`CNDTR`), and the F4 HAL has no
  `GPIO_PIN_17`. Each of these produced a silent failure.
* **`ClockEntry` must be registered.** Constructing one does not schedule it;
  `machine.ClockSource.AddClockEntry(pclk)` is required or the pixel clock
  never fires.
* **SCCB tops out at 400 kHz.** `i2c1.c` was pinned to 100 kHz standard mode;
  the OV7670 datasheet allows `fSIO_C` = 400 KHz, so `I2C_SCL_HZ` is now a
  tunable defaulting to 400 kHz with the correct fast-mode `TRISE`. This is the
  hard ceiling for register traffic, and the reason pixels cannot come over
  SCCB at any useful rate (a 160x120 YUV422 frame is 691,200 SCL pulses = 1.7 s
  at 400 kHz).
* **DCMI requests DMA per 32-bit word, not per byte** (RM0090 §13.8.11 packs
  received bytes into `DR` before requesting a transfer). Getting this wrong
  quadruples the DMA request rate.

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

This is broader than just `-D` flags. The object rules in `firmware/Makefile`
depend only on the source file and on `Makefile` itself, so **editing any header
- `FreeRTOSConfig.h` included - does not recompile anything that includes it.**
Toggling `NCAP_SYSUSAGE`/`NCAP_FREERTOS` at least changes the source list, so it
is caught; changing `configTOTAL_HEAP_SIZE` or any other config value in place is
silently ignored. `scripts/verify_sysusage.sh` therefore `rm -rf`s each scratch
`BUILD=` dir before building a row, so it cannot report a stale size - the first
version of it did exactly that and hid a config change that had in fact applied.

## Grayscale luma video stream (UDP)

`make NCAP_VIDEO_UDP=1` builds a firmware that streams grayscale luma over UDP
instead of speaking micro-ROS. This sidesteps DDS fragmentation, needs no
host-side `rmw_microxrcedds`, and involves no Docker agent at all.

```
firmware -> W5500 UDP datagrams (127.0.0.1:<port>) -> host receiver
```

Wire format: one self-describing datagram per row block
(`'V''F''R''0' | u32 seq | u16 w | u16 h | u16 row | u16 rows | u32 len | luma`),
preceded by a meta datagram with an HTTP-style preamble.

```bash
./scripts/run_video.sh 18080                  # synthetic, verifies 3 frames
./scripts/run_video.sh 18080 160 120 3 24     # port w h frames fps
DVP=1 ./scripts/run_video.sh 18080 160 120 3 24   # real OV7670 over DVP
```

### Frame rate is a target, not a delay

`VIDEO_FPS` (default 24) is a **rate**. It used to be `VIDEO_FPS_MS`, an idle
delay applied *after* a frame was already sent, so the real period was
`send_time + VIDEO_FPS_MS` and raising the clock silently ate the budget. The
serve loop is now deadline-paced: it accumulates 1000 and drains whole `VIDEO_FPS`
units, giving an exact average period of `1000/VIDEO_FPS` ms (41/42 ms
alternating at 24 fps) with no integer-division drift, and a late frame drops
its debt instead of cascading.

Measured with `VIDEO_FPS_REPORT=1` (prints the achieved rate once per simulated
second), DVP capture at 160x120:

```
F4: video rate 24.82 fps (25 frames / 1007 ms)
F4: video rate 24.00 fps (24 frames / 1000 ms)
F4: video rate 24.00 fps (24 frames / 1000 ms)
```

Note the **host-observed** rate in Renode is far lower (~0.1 fps) because
Renode is peripheral-throughput-bound, not because the firmware is slow. The
emulated figure above is what the firmware actually does.

### Throughput notes that matter at 24 fps

At 320x240 a frame is 60 datagrams of 1300 B, ~81 KB of SPI traffic including
W5500 register overhead, i.e. **~29 ms at 22.5 MHz**. Three things had to give
before that fits a 41 ms period:

- **SPI clock.** `Core/Src/spi.c` had SPI1 at `/256` (351 kHz), which capped
  the link at ~0.54 fps. Now `/4` (22.5 MHz), comfortably inside the W5500's
  80 MHz datasheet maximum and the F446's PCLK2/4 limit.
- **Socket TX buffer.** At the W5500's default 2 KB only one 1300 B datagram
  fits, so `sendto()` blocks in the `Sn_TX_FSR` spin and SPI and Ethernet
  serialise instead of overlapping. `Sn_TXBUF_SIZE` is now 4 KB.
- **Per-frame logging.** One `printf` is ~3.5 ms at 115200 baud, i.e. 8% of the
  budget. Frame logging is gated by `VIDEO_LOG_EVERY` (default 30).

### Memory

With the synthetic generator, peak RAM is one row block - no full-frame buffer.
With `NCAP_DVP=1` a capture ring is unavoidable because DCMI DMA runs
continuously; see the frame-size table in the OV7670 section.


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

## System-usage reporting (sysusage)

`firmware/sysusage.{c,h}` publishes live F4 telemetry to ROS 2 as
`diagnostic_msgs/msg/DiagnosticArray` on `/diagnostics`, sampled either inline or
by a dedicated FreeRTOS task depending on `NCAP_FREERTOS`. It is off by default
and builds with or without FreeRTOS. Design rationale:
[`docs/plan-sysusage.md`](docs/plan-sysusage.md).

### Build toggles

| Macro | Default | Effect |
|---|---|---|
| `NCAP_SYSUSAGE` | `0` | Compile the module at all. `0` leaves `sysusage.c` out of `SRC` entirely: no symbols, no `.bss`, file absent from the ELF. |
| `NCAP_FREERTOS` | `0` | Build the vendored FreeRTOS V10.3.1 kernel and hand the CPU to `vTaskStartScheduler()`. Prerequisite for the sysusage **task** path only. |
| `SYSUSAGE_PERIOD_MS` | `1000` | Sampler task period. Does **not** set the publish rate - that is `PUB_PERIOD` (1000 ms, `main.c:99`). |

All three are `?=` defaults overridden from the command line, like every other
toggle in `firmware/Makefile`.

### Build matrix

| `NCAP_SYSUSAGE` | `NCAP_FREERTOS` | What runs |
|---|---|---|
| `0` | `0` | **Default.** Bare-metal super-loop, `rclc_executor_spin()`, no sysusage code in the image. The historic behaviour. |
| `1` | `0` | Telemetry, no scheduler. `sysusage_publish()` samples inline from `HAL_GetTick()` on the executor's 1 Hz timer. Publishes `/diagnostics` but no CPU load, no heap, no task list - with no kernel there is nothing to read. |
| `0` | `1` | Scheduler, no telemetry. Heartbeat and IMU still publish (the executor runs as a task); no `/diagnostics`. |
| `1` | `1` | **Full feature.** A statically allocated sampler task samples the kernel; the executor task publishes everything, including the task's own report. |

All four rows publish. Row 2 and row 4 both produce a real `/diagnostics` array;
row 4 is the only one with `cpu_load_pct`, heap figures and `f4_tasks`.

**Build-level verified, not yet run in Renode.** All four rows build clean with
zero warnings under `-Wall` and `./scripts/verify_sysusage.sh` passes end to
end. No `/diagnostics` message has been observed in ROS 2 yet, so treat the
runtime behaviour as designed, not proven.

| Row | `data+bss` | sysusage symbols in ELF |
|---|---|---|
| `0/0` default | 63,636 | **none** - `sysusage.c` is not in the image |
| `1/0` | 65,004 | `sysusage_publish`, `sysusage_publisher_init`, `sysusage_set_extra`, `sysusage_publish_failed` |
| `0/1` | 65,712 | none |
| `1/1` | 70,056 | the above plus `sysusage_task_start` |

The full-feature row costs **+6,296 B of `.bss`** over the default, inside the
8 KB budget, leaving 61,216 B of the F446's SRAM. The zero-symbol row is the
real proof that "builds with or without" is true rather than merely intended.

```bash
make -C firmware                                        # default: no sysusage, bare metal
make -C firmware NCAP_SYSUSAGE=1                        # /diagnostics, inline sampling, no scheduler
make -C firmware NCAP_FREERTOS=1                        # scheduler, executor as a task, no telemetry
make -C firmware NCAP_SYSUSAGE=1 NCAP_FREERTOS=1        # full feature: sampler task + executor task
make -C firmware NCAP_SYSUSAGE=1 SYSUSAGE_PERIOD_MS=200 # sample every 200 ms
./scripts/verify_sysusage.sh                            # builds all four rows and checks them
./scripts/verify_sysusage.sh --clean                    # ... and removes the scratch build dirs
```

`NCAP_FREERTOS=1` **and** `NCAP_VIDEO_UDP=2` are mutually exclusive and the
Makefile rejects the pair with `$(error)`, not a runtime surprise - see the
gotchas.

### One executor, one publisher: the task samples only

There is exactly one XRCE/UDP session (socket 0, one client). **Two
`rclc_executor`s spinning that client would interleave XRCE datagrams and
corrupt the session**, so:

- the existing `rclc_executor_t` in `main.c` stays the only rclc caller, and
  the only thing that ever calls it is **one task** (`rcl`, `main.c`);
- the sysusage task **never touches rclc** - it samples RTOS/kernel state into
  a plain-POD snapshot and publishes nothing;
- `timer_callback()` runs on that task and calls `sysusage_publish()`, which
  copies the newest snapshot into a preallocated `DiagnosticArray`.

So under `NCAP_FREERTOS=1` there are exactly two tasks: `rcl` at priority 2,
which owns the executor, and the sysusage sampler at priority 1. The idle task
at priority 0 supplies the CPU-load numerator.

The transport's RX poll is a hard spin on `getSn_RX_RSR()`, so on its own it
would lock out every lower-priority task - including the idle task, which would
pin `cpu_load_pct` at a permanent 100%. `w5500_transport.c` therefore calls a
weak `ncap_cpu_relax()` hook inside that spin, which `main.c` defines as
`vTaskDelay(1)` under `NCAP_FREERTOS=1` and leaves as a no-op on bare metal.
`vTaskDelay` rather than `taskYIELD` because only `vTaskDelay` reopens the CPU
to strictly lower priorities. This is the one place the transport learned about
the RTOS, and it learned it through a hook rather than an `#include`.

The handoff is a struct copy behind a short `taskENTER_CRITICAL()` /
`taskEXIT_CRITICAL()` around a pointer flip only - never around rclc, never
around SPI/W5500. The task is `xTaskCreateStatic`, so it consumes no heap at
all.

`diagnostic_msgs/msg/DiagnosticArray` is used instead of a purpose-built `.msg`
**because `firmware/libmicroros.a` is a prebuilt archive** (34,717,218 B, built
out-of-tree). A new custom message would require regenerating the IDL and
rebuilding that library, which this project does not do. The stock type is
already linked in - `nm` finds 262 `diagnostic_msgs` symbols including
`rosidl_typesupport_microxrcedds_c__get_message_type_support_handle__diagnostic_msgs__msg__DiagnosticArray`
- and it is ROS 2's native representation for arbitrary key/value health data,
so no rebuild is needed at all.

### What is reported

One `DiagnosticArray` per heartbeat on `/diagnostics`, with
`header.frame_id` = `ncap_f4` (or `ncap_f4_<NODE_ID>`) and the stamp taken from
`HAL_GetTick()`. Every `KeyValue` value is a **string**, not an int - the wire
type is fixed by the prebuilt archive, so parse accordingly.

Status 1 of 2, `f4_system`, always present:

| Key | Value | Notes |
|---|---|---|
| `build` | `freertos` or `bare-metal` | which personality produced this |
| `period_ms` | `SYSUSAGE_PERIOD_MS` | the sampler period, not the publish rate |
| `uptime_ms` | `HAL_GetTick()` at publish | |
| `seq` | publish counter | |
| `cpu_load_pct` | `0.00` - `100.00` | **`NCAP_FREERTOS=1` only** |
| `heap_free_bytes` | `xPortGetFreeHeapSize()` | **`NCAP_FREERTOS=1` only** |
| `heap_min_ever_free_bytes` | `xPortGetMinimumEverFreeHeapSize()` | **`NCAP_FREERTOS=1` only** |
| `rcl_pool_used` / `rcl_pool_size` / `rcl_pool_free_pct` | bytes / bytes / `0.00`-`100.00` | from `sysusage_extra_t`, stamped in `main.c` |
| `video_fps_x100` | fps x100 | **only when non-zero**; `main.c` always sends 0, so the key is *absent* in a micro-ROS build |
| `time_sync` | `ntp` or `session` | `rmw_uros_epoch_synchronized()` |
| `publish_fails` | count | non-zero means the agent is not keeping up |

Status 2 of 2, `f4_tasks`, **only in `NCAP_FREERTOS=1` builds** (with no
scheduler there are no tasks, so `status.size` is 1, not 2). The key of each
entry is the task name; the value is `"<state> stk=<hwm> rt=n/a"` where
`<state>` is a one-letter code and `<hwm>` is `usStackHighWaterMark`:

```
IDLE   stk=412 rt=n/a
READY  stk=388 rt=n/a
BLOCKED stk=402 rt=n/a
```

`rt=` is permanently `n/a` - `configGENERATE_RUN_TIME_STATS` is 0 and no
run-time counter is wired up, so per-task CPU share is genuinely unavailable
rather than merely unimplemented. The code that would compute it is compiled out
rather than faked.

`DiagnosticStatus.level` is derived, not reported: `ERROR` on rcl pool >= 90%
used or stack high-water <= 2 words, `WARN` on cpu load > 80% or stack <= 8
words, `STALE` when no tasks were sampled. `hardware_id` mirrors `frame_id`
(`ncap_f4_<NODE_ID>`) so a ROS consumer can tell the boards apart; `frame_id`
is the same value at the array level.

CPU load is measured with `configUSE_IDLE_HOOK 1`: the idle hook bumps a
`volatile` counter and `load% = 100 - idle_picks / tick_delta * 100`. If no
time has elapsed since the previous sample it reports 100% rather than dividing
by zero. There is no cycle counter wired up.

### RAM

The F446 has 128 KB of SRAM. The default build measures `data 388  bss 63248`,
i.e. **63,636 B of `data+bss`** and 67,436 B of headroom. (`docs/plan-sysusage.md`
quotes `data+bss` = 82,296 B and "~48 KB headroom"; that is a **stale
pre-video baseline** and no longer true - the current default build sits 18 KB
lower. The authoritative numbers are in the table above, regenerated by
`./scripts/verify_sysusage.sh`.)

micro-ROS already holds a 40 KB static RMW pool (`POOL_SIZE`, `main.c:105`).
The sysusage module is required to add **zero** heap: every message buffer must
be a `static` array with its sequence `capacity` set to the true length, because
`libmicroros.a` will call `rosidl_string_fini` / `fini` on it and a heap
pointer would be freed. The bound is fixed by design at 2 `DiagnosticStatus`
(<= 24 `KeyValue` each) plus a 12-entry `TaskStatus_t` array clamped by
`uxTaskGetNumberOfTasks()`. **Measured: +6,296 B of `.bss`** for the full-feature
row, hard ceiling 96 KiB of `data+bss` (32 KiB of physical SRAM held back), and
`scripts/verify_sysusage.sh` fails the build if either is exceeded.

The module must never call `rosidl_runtime_c__String__fini` or a sequence
`__fini` on its own buffers: that path reaches
`rcutils_get_default_allocator()` and calls `free()`. Nothing in the publish
path does, but a static buffer is a latent `free()` target if that ever changes.

FreeRTOS is not free either, and the RAM budget is what decided most of these
values. Two of these changes were needed to fit the feature at all:

| `FreeRTOSConfig.h` | Was | Now | Why |
|---|---|---|---|
| `configENABLE_FPU` | `0` | `0` (unchanged) | The plan wanted `1`, but **V10.3.1 has no `configENABLE_FPU` option** - nothing in the vendored kernel reads it, so it is inert CubeMX legacy either way. The real constraint is that `port.c:445,467` assembles `vstmdbeq`/`vldmiaeq` for `s16-s31`, which cannot be assembled under `-mfloat-abi=soft` at any `-mfpu`. The Makefile compiles **only `port.c`** with `-mfloat-abi=softfp`; everything else, including the prebuilt `libmicroros.a`, stays soft. Under the soft ABI GCC emits no FP instructions at all, so the lazy-stacking path that code enables can never be taken. |
| `configUSE_IDLE_HOOK` | `0` | `1` | Required for the CPU-load measurement above. |
| `configMAX_PRIORITIES` | `56` | `8` | This firmware has 3 priorities (idle 0, sampler 1, rcl 2). 56 was sized for the robot-arm project that used to live here and cost 1120 B of `pxReadyTasksLists` used or not. |
| `configUSE_TIMERS` | `1` | `0` | Nothing calls `xTimerCreate` - rclc drives its own `rcl_timer`, sysusage samples from its own task. The daemon cost ~1356 B of `.bss` (`xTimerStack` 1024 + TCB 92 + command queue ~240) for no gain. |
| `INCLUDE_xTimerPendFunctionCall` | `1` | `0` | Forced by the row above: `timers.c:42` hard-errors when `configUSE_TIMERS` is 0 and this is 1. |
| `configTOTAL_HEAP_SIZE` | `15360` | `4096` | 15 KB of `.bss` for a heap nothing allocates from is 15 KB of RAM we do not have. Every task is `xTaskCreateStatic`. |

`configSUPPORT_STATIC_ALLOCATION 1` and `INCLUDE_uxTaskGetStackHighWaterMark 1`
were already on and the module depends on both. `configTOTAL_HEAP_SIZE` is only
free because `--gc-sections` drops `ucHeap` while the tree contains no
`xTaskCreate` / `xQueueCreate` / `xTimerCreate` - the first one puts the full
4096 B back at link time.

### Gotchas found while adding sysusage

* **The sysusage task must not call rclc.** Stated once more because it is the
  whole architecture: one XRCE session means one executor, and publishing from
  the task corrupts it. Sampling and publishing are deliberately split across
  two contexts.
* **The scheduler has to own the executor, not just the CPU.** `NCAP_FREERTOS=1`
  hands the loop to `vTaskStartScheduler()`, so if nothing spins the executor the
  1 Hz timer never fires and *nothing* publishes - heartbeat, IMU and
  `/diagnostics` alike. The fix is the `rcl` task in `main.c`, which is the only
  caller of `rclc_executor_spin_some()`. A first cut of this feature shipped a
  firmware that linked, started a scheduler, and published nothing at all, with
  a comment claiming that was "by design"; it is not, and the build checks now
  exist partly to catch that class of mistake.
* **`ncap_cpu_relax()`: `vTaskDelay`, not `taskYIELD`.** The W5500 RX poll
  (`w5500_transport.c`) is a hard spin on `getSn_RX_RSR()`. Left alone inside a
  task it locks out every lower priority - including the idle task, pinning
  `cpu_load_pct` at a permanent 100%. `taskYIELD` does **not** fix that: it only
  lets a task of equal or higher priority run. Only `vTaskDelay` reopens the CPU
  downward. Also make `main.c`'s definition **strong**, not weak: the transport's
  fallback is weak, and two weak definitions are resolved by link order, i.e. by
  luck.
* **`NCAP_FREERTOS=1` and `NCAP_VIDEO_UDP=2` cannot be combined.** With the
  scheduler running, `main.c`'s cooperative `rclc_executor_spin_some()` +
  `video_udp_step()` multiplexing stops happening - `video_udp_step()` *is* the
  mode-2 main loop - so the video would silently stop streaming. The Makefile
  rejects the pair with `$(error)` (in `firmware/Makefile`) rather than shipping
  firmware that quietly goes mute. Supporting both needs `video_udp_step()`
  driven from a task, which is not implemented.
* **`NCAP_DVP=1` is a RAM budget, not a personality ban - and sysusage fits inside
  it.** The ring costs `VIDEO_WIDTH x stream-height x bytes/px x DVP_FRAMES`
  (stream-height is `DVP_CROP_ROWS` when set, else `VIDEO_HEIGHT`; 2 B/px YUV422,
  1 when `DVP_LUMA_ONLY=1`). Measured on the linked ELFs, micro-ROS + QQVGA:

  | Config | RAM | of 128 KB |
  |---|---|---|
  | `NCAP_DVP=1` 160x120 `DVP_FRAMES=1` | 102,148 | 77.9% |
  | + `NCAP_SYSUSAGE=1` | 103,516 | 79.0% |
  | + `NCAP_FREERTOS=1` | 104,224 | 79.5% |
  | + both (full feature) | 108,568 | 82.8% |
  | `DVP_FRAMES=1`, `DVP_CROP_ROWS=80`, both | 95,768 | 73.1% |
  | `DVP_FRAMES=2`, `DVP_LUMA_ONLY=1`, both | 108,568 | 82.8% |
  | `NCAP_VIDEO_UDP=2` + DVP F1 | 104,908 | 80.0% |

  So no - adding sysusage does **not** break micro-ROS + QQVGA. The full feature
  adds 6,420 B and the tightest fitting combo still has ~22.5 KB free. What does
  not fit is a second full YUV422 frame: `DVP_FRAMES=2` is 76,800 B of ring on
  top of the ~63 KB the 40 KB RMW pool already occupies, ~140 KB against 128 KB
  with no SDRAM on this board.
* **Frame rates: the sensor offers ~31 fps, the firmware takes less, mode 0
  takes nothing.** `renode_configs/peripherals/OV7670.cs` drives PCLK at
  1.2 MHz (`DefaultPclkHz`); QQVGA YUV422 is 38,400 B/frame, so the model
  delivers ~31 fps and the comment in the model says 30. What each personality
  captures from that:

  | Personality | Who steps capture | Achieved rate |
  |---|---|---|
  | `NCAP_VIDEO_UDP=1` | deadline-paced serve loop | **24 fps sim-time** (measured `video rate 24.00 fps` with `VIDEO_FPS_REPORT=1`); ~0.1 fps host-observed, Renode is peripheral-throughput-bound |
  | `NCAP_VIDEO_UDP=2` | one `video_udp_step()` per 5 ms executor slice, 15 slices per 160x120 frame | ~13 fps sim-time ceiling, derived from `ROWS_PER_DGRAM`, not measured |
  | `NCAP_VIDEO_UDP=0` | **nobody** | **0 fps - the ring is dead RAM** |

  The last row is the one that matters for the table above: `ov7670_dvp_init()`
  is called only from `video_udp_init()` (`video_udp.c:263`), which runs only in
  modes 1 and 2 (`main.c:401,575`). In mode 0 the DCMI is never enabled, the DMA
  never runs, and the 38,400 B ring is allocated but never filled. The RAM fit
  is real; the photons are not. QQVGA capture alongside micro-ROS means mode 2
  (104,908 B / 80.0% without sysusage, ~106 KB / ~81% with it - both fit).
* **The Makefile enforces the budget arithmetically.** An earlier version of the
  `CHECK_DVP_*` guard banned `NCAP_DVP=1` next to any micro-ROS personality
  outright, which was wrong - it rejected the fitting rows in the table above.
  It now computes `DVP_RING` from the actual `VIDEO_WIDTH` / height / `DVP_BPP` /
  `DVP_FRAMES` and adds the measured baseline for the exact personality, and
  `$(error)`s only when the sum exceeds 128 KB minus 1 KB of alignment margin.
  The estimate was verified within ~112 B of the linked size on every row, and
  `VIDEO_WIDTH`/`HEIGHT` were verified to not move `.bss` in any personality,
  so the baselines are exact, not conservative. A fitting config can never be
  rejected; an overflowing one fails at parse time with the arithmetic printed,
  instead of `ld` reporting `region 'RAM' overflowed` from 240 lines away.
* **`SVC_Handler` and `PendSV_Handler` did not exist.** `Core/Src/stm32f4xx_it.c`
  only had `SysTick_Handler`, and `startup_stm32f446xx.s:270,276` provides both
  of the others as `.weak` aliases to `Default_Handler`. Without strong
  definitions the first task switch lands in the fault-loop and the scheduler
  never starts. Both are now defined under `#if NCAP_FREERTOS == 1`.
* **`SysTick_Handler` must call `xPortSysTickHandler()` *and* `HAL_IncTick()`.**
  They look redundant and are not: `xPortSysTickHandler()` only advances the
  FreeRTOS tick and never touches `uwTick`. HAL's tick is load-bearing -
  `w5500_transport.c` escapes its socket-read spin through
  `(int32_t)(HAL_GetTick() - start) >= timeout`, so a frozen `uwTick` makes every
  XRCE read report an instant timeout and the node goes quiet with no error
  message at all. `video_udp.c` paces its frame deadline the same way.
* **`CMSIS_RTOS_V2/cmsis_os2.c` is deliberately not linked.** The module uses
  the native kernel API (`xTaskCreateStatic`, `uxTaskGetSystemState`) and
  `main.c` keeps calling rclc bare-metal, so nothing needs the wrapper - and it
  brings an idle thread of its own. `event_groups.c`, `croutine.c` and
  `stream_buffer.c` are excluded for the same reason (unused /
  `configUSE_CO_ROUTINES` 0). `FREERTOS_SRC` enumerates the six files it does
  link; dropping any is a link error, never a silent behaviour change.
  `Core/Src/freertos.c` is CubeMX robot-arm code and is not built at all - do
  not "fix" it into the build.
* **`vApplicationIdleHook` must be defined exactly once.** The idle hook is the
  CPU-load mechanism, and it is a FreeRTOS *application* callback: the kernel
  references it unconditionally, so something in the application must define it.
  Two agents each provided one (`main.c` and `sysusage.c`) and the full-feature
  link failed with `multiple definition of vApplicationIdleHook`. Both are
  `__attribute__((weak))` so all four rows link whoever wins; `sysusage.c`'s is
  strong and is what runs in the full-feature build. Same for
  `vApplicationGetIdleTaskMemory`, which `configSUPPORT_STATIC_ALLOCATION 1`
  makes mandatory and which `sysusage.c` **cannot** own - it is absent from the
  `NCAP_SYSUSAGE=0 NCAP_FREERTOS=1` row, so the callback has to live in a file
  that row always compiles. `vApplicationGetTimerTaskMemory` is deliberately
  absent, because `configUSE_TIMERS` is 0.
* **`hardware_id` mirrors `frame_id`.** `su_hw_id` is bound once and was left
  unwritten, which made every node look like the same anonymous board; it is now
  set from `header.frame_id` (`ncap_f4_<NODE_ID>`). `su_status[1]` shares the
  same static buffer, so the two statuses stay consistent.
* **`su_str_setn()` takes a `char *`, not a `rosidl_runtime_c__String *`.**
  Passing `&su_msg.header.frame_id` compiles to an incompatible-pointer error
  under `-Wall`, not a warning someone will miss.
* **`video_fps_x100` is omitted, not zero.** `main.c` always passes 0 and the
  formatter only emits the key when it is non-zero, so a micro-ROS build simply
  has no such key. Do not treat a missing key as a fault.
* **`main.c` includes `sysusage.h` unconditionally** (`main.c:48`). That is
  harmless (the header's whole body is `#if NCAP_SYSUSAGE == 1`) but it does
  mean the *default* build still needs the header to exist on disk, so
  `NCAP_SYSUSAGE=0` is not a hermetic compile without it.
* **One `BUILD=` dir per matrix row, or the matrix lies.** As with the five-node
  builds, rebuilding one tree with different `-D` flags back-to-back reuses
  stale objects and produces byte-identical ELFs. `scripts/verify_sysusage.sh`
  uses one scratch dir per row (`build_su_s0f0`, `build_su_s1f0`,
  `build_su_s0f1`, `build_su_s1f1`) for exactly this reason, and `--clean`
  removes them.

## OV7670 camera: SCCB control + DVP parallel capture

`renode_configs/peripherals/OV7670.cs` is a register-faithful SCCB model
defaulting to QVGA YUV422: PID `0x76` / VER `0x73`, COM7 reset/format/resolution,
CLKRC/COM3/COM14/COM15/COM17/TSLB storage with side effects, and a deterministic
synthetic scene whose luma matches `video_udp.c`'s generator pixel-for-pixel.
It sits on `i2c1` at `0x21` alongside the IMU.

**Pixels travel over the DVP parallel bus, not SCCB.** The model drives
`PCLK`/`VSYNC`/`HREF`/`D0-D7` from a `ClockEntry`, so
`OV7670 -> DCMI -> DMA2 -> ring buffer -> UDP` is the real path
(`firmware/ov7670_dvp.c`).

### The `0xF0-0xF3` registers do not exist

Earlier revisions of this file described pixel readout over "test registers
`0xF0-0xF3`". **That was wrong and has been removed.** The OV7670 datasheet
Table 5 ends at register `0xC9`; there is nothing at `0xF0-0xF3`. Those
addresses were a fiction invented by `renode_configs/peripherals/OV7670.cs`, so
`OV7670=1` only ever produced a Renode-generated test pattern and could never
have worked on silicon. The registers are still in the model so the old
firmware path can be diffed, but `ov7670.c` no longer uses them for pixels.

### Output scaling: COM3 / COM14

Per the datasheet, a pre-defined output mode is downscaled with
`COM3[3]` scale enable, `COM3[2]` DCW enable (horizontal /2) and `COM14[3]`
manual scaling with `COM14[2:0]` as the vertical divider. `ov7670_init()`
programs these from the build's target geometry:

```
F4: ov7670 QVGA YUV422 configured, out 160x120 (COM3=0C COM14=09)
```

`320x240` -> `160x240` (DCW) -> `160x120` (COM14), which is what makes a full
YUV422 frame fit the F446's 128 KB of RAM.

### No hardware chroma drop on STM32F4

AN5020's "Y only data capture" recipe uses `DCMI_BSM_OTHER` + `DCMI_OEBS_EVEN`,
but **RM0090 marks `DCMI_CR` bits 31:15 as "Reserved, must be kept at reset
value"** for STM32F4, even though `stm32f446xx.h` defines `DCMI_CR_BSM_0/1`,
`OEBS`, `LSM` and `OELS` at bits 16-20. There is no `CGR` or `CCMR` on this
part at all. Chroma is therefore stripped **on the CPU**, in place.

### Byte select: the only route to luma-only capture

Since the OV7670 cannot be asked for less than 2 bytes per pixel (its 8-bit
output formats are YUV422, RGB565/555/444, GRB 4:2:2 and Raw RGB - **no
YUV420**), the only way to stop chroma entering RAM is DCMI byte select.
`DVP_LUMA_ONLY=1` sets `CR.BSM = 01` (every other byte) with `CR.OEBS = 0`,
which for YUYV keeps exactly the luma bytes. A frame then costs **1 byte per
pixel**, so QVGA becomes 76,800 B instead of 153,600 B and fits with room to
spare - no crop needed, and because the slot equals a full sensor frame it is
self-aligning.

Measured (320x240, `DVP_LUMA_ONLY=1`, bss 82,176 B): **still produces a wrong
image.** Two problems are outstanding:

- The model's `CR` readback only reconstructs bits 0-14, so the `BSM`/`OEBS` bits
  written by firmware are invisible and cannot be confirmed set.
- Frames show only row 0 populated with the remainder stale, i.e. the DMA is
  completing on a **partially filled slot** rather than transferring the whole
  frame. That is a different fault from the misalignment seen with cropping and
  is the more promising thing to chase.

`DVP_LUMA_ONLY` defaults to **off**; the verified QQVGA configuration does not
use it. Note also that the reference manual marks these bits reserved, so this
must be validated on real silicon before being trusted: if the bits are ignored
the stream is byte-aligned garbage.

### Frame size decides the resolution

YUV422 puts 2 bytes per pixel on the bus, and 128 KB of RAM is the hard limit:

| Capture | Bytes/frame | Ring that fits |
|---|---|---|
| **QQVGA 160x120 YUV422** | 38,400 | **2-3 frames - verified working** |
| QVGA 320x240 YUV422 | 153,600 | **impossible** - exceeds all of RAM |

An impossible configuration fails loudly at link time rather than silently:
`region 'RAM' overflowed by 28016 bytes`.

```bash
# QQVGA, double buffered (verified end to end)
make -C firmware NCAP_VIDEO_UDP=1 NCAP_DVP=1 VIDEO_WIDTH=160 VIDEO_HEIGHT=120 DVP_FRAMES=2
```

**Full-width QVGA is experimental** (see below), for two independent reasons - both of
which were implemented and then measured before being backed out:

- *YUV420 is not an OV7670 mode.* An earlier revision of this file offered a
  `DVP_YUV420` build for QVGA. YUV420 appears **nowhere** in the datasheet, and
  `COM7[2]/[0]` only selects YUV / RGB / Bayer RAW / processed Bayer RAW. That
  build reported `OK: verified 3 frames` while producing a geometrically
  corrupt image.
- *A DCMI crop window was implemented, measured, and backed out.* Cropping to
  320x160 would fit (102,400 B/frame). It produced a reproducible but wrong
  image (row means non-monotonic, chroma leaking into luma).

  Two distinct bugs were found. First, a real model bug: the crop test compared
  a **byte** index against a **pixel** width, capturing only the left half of
  each line (hence exactly `nonzero=50%`). Second, the frame-alignment gate
  added to fix the remainder was a **no-op** - it was only cleared when
  `CR.CAPTURE` went low, which never happens in continuous capture mode, so it
  never actually constrained anything.

  What is established is the structural difference: a DMA slot equal to one
  sensor frame (QQVGA) is self-aligning, because the DMA completes exactly at
  the frame boundary. A slot smaller than a sensor frame (any crop) must be
  re-armed exactly on each boundary.

* **Second attempt: crop works, but not reliably.* `DVP_CROP_ROWS` now captures
  a top-of-frame window in DCMI snapshot mode (CR.CM), and the model honours
  `CR.CAPTURE` only at the next frame start, so an arming write can never begin
  capture mid-frame. Raw-byte dumps show this **does** produce a pixel-perfect
  image when it lands - one frame in four had `evenMean=15`, against a predicted
  row-0 mean of 15.1, with chroma correctly on odd bytes throughout. The other
  frames start at a varying offset.

  Measured with a clean build (sensor 320x240, ring 102400 B, DCMI_CR=0x40B7 with
  CROP+snapshot set, zero CPU aborts), 4 consecutive frames:

  | frame | monotonic rows | row-0 mean | |
  |---|---|---|---|
  | 1-3 | 78/160 | ~78-79 | misaligned (consistent offset) |
  | 4 | **158/160** | **15.1** | **pixel-perfect** |

  Row 0 of a correct frame is predicted to mean 15.1, so the mechanism does
  produce exactly the right image - it just fails to lock onto the frame
  boundary on most frames. Adding realistic vertical blanking to the sensor
  model (VSYNC held for 8 lines rather than one pixel clock, which is both more
  faithful and gives the frame interrupt room to be taken) did not fix it.

  So this remains **incomplete**: correct roughly one frame in four. It is left
  in the tree as experimental and **must not be treated as working** - the
  default is `DVP_CROP_ROWS=0` (no crop, self-aligning), and only the
  configuration below has been verified.

* **Per-frame validation was implemented and measured - it does not work.** The
  idea was to reject any frame whose row 0 differs sharply from the previous
  accepted frame's row 0, since a misaligned capture holds an arbitrary sensor
  line there. Measured row-0 means across six consecutive frames were
  `78.3, 79.1, 79.1, 79.1, 79.1, 79.1` - **near-identical**. The re-arm slip is
  *consistent*, so a misaligned frame is just as temporally coherent as an
  aligned one and the check passes all of them (0/6 rejected). The approach can
  only detect a frame whose offset differs from its predecessor. It is retained
  behind `DVP_ALIGN_CHECK` (default **off**) because it would catch a *varying*
  offset.

  Also observed: on the frames that do come out aligned, only row 0 carries fresh
  data and the remainder is stale, which suggests the DMA is completing on a
  partially filled slot rather than a full one.

  The remaining fault is that the firmware re-arms the DMA in software from the
  frame interrupt, and Renode does not reliably schedule the CPU at the instant
  that matters. The most promising untried fix is to move the wrap into the DCMI
  model - emit a "slot N complete" event the DMA can key off - rather than
  re-arming from software. Continuous/circular DMA is the textbook answer but is
  unavailable because Renode's `STM32DMA` implements `CIRC` as a tagged, ignored
  bit.

Full-width 320x240 therefore requires external SDRAM on the FMC bus, as
AN5020's own examples do.

### Verify the pixels, not just the byte count

`video_client.py` checks reassembled size and that min/max/mean are plausible.
That is **not** enough - a format mismatch produces garbage that still passes.
The synthetic scene is a vertical ramp, so a correct capture must have
row means rising monotonically:

```python
# good capture: 118/120 rows non-decreasing, last row ~252
# corrupt one : 78/160 non-decreasing, row means clustered ~105
```

### Hardware rewiring is required

STM32F446 DCMI pins are AF13 with no remap (only `DCMI_D0` can move, PC6<->PC13),
and two of them collide with the original W5500 SPI1 pins:

| DCMI | Pin | Was |
|---|---|---|
| `DCMI_PIXCLK` | PA6 | SPI1_MISO |
| `DCMI_VSYNC` | PA4 | W5500 SCSn |

**The W5500 must move to SPI2.** Simulation-only users need no change; real
hardware must be re-soldered:

```
W5500: PB13=SCK, PB14=MISO, PB15=MOSI, PB12=SCSn, PB11=RSTn  (was PA5/6/7, PA4, PA3)
DCMI : PA6=PIXCLK, PA4=VSYNC, PA17=HREF, PC6..PC11=D0..D5, PC12=D6, PD6=D7
```


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
