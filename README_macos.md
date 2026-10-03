# NCAP — macOS Setup (ROS 2 + micro-ROS + Renode)

Host-side notes for running the NCAP STM32F4 + W5500 simulation end-to-end on this
machine (Apple Silicon `arm64`, macOS 27.0, Renode 1.16.1 at `/Applications/renode`).

## 1. ROS 2 environment

ROS 2 **Humble** is installed from conda/RoboStack and lives **outside any repo**:

```bash
source ~/Documents/Polispace/ros2_humble/local_setup.zsh
export ROS_LOCALHOST_ONLY=1
```

- Use `local_setup.zsh` (not `.bash`) — this shell is `zsh`.
- `source ~/Documents/Polispace/ros2_humble/setup.zsh` also exists; `local_setup.*`
  is preferred because it does not re-resolve the conda prefix.
- Verified: `ROS_DISTRO=humble`, `ros2` and `rclpy` both work after sourcing.
- `ROS_LOCALHOST_ONLY=1` is **required** — DDS discovery fails without it on macOS.

Then source this project's own workspace (if you build the custom interface package):

```bash
source install/setup.zsh    # after: colcon build --symlink-install
```

## 2. micro-ROS Agent

Runs in **Docker** using the same image as the `microk3` project. See
`README_microros_agent.md` for the full write-up (including why building it from
source fails on this machine).

```bash
open -a Docker                # must be running first
docker run -d --name ncap-agent -p 8888:8888/udp \
  microros/micro-ros-agent:humble udp4 --port 8888
```

Do **not** try to build it from source here: ROS 2 Humble on this machine ships
`fmt` v12, which is incompatible with Micro-XRCE-DDS-Agent v2.4.2's formatter API.

## 3. Agent IP the firmware must use

Your existing `microroseth` project hardcodes the agent at `192.168.0.1`
(`MICROROS_AGENT_IP` in `Core/Inc/microros_sim_network.h`). For the NCAP W5500 sim
the W5500 model bridges to **host** sockets, and the agent is published on the host
at `127.0.0.1:8888` by Docker, so:

```c
// w5500_transport.c
static uint8_t w5500_agent_ip[4] = {127, 0, 0, 1};
```

and in the Renode monitor:

```bash
w5500 EnableHostSockets true
```

## 4. End-to-end run order

Terminal 1 — agent:

```bash
docker run -d --name ncap-agent -p 8888:8888/udp microros/micro-ros-agent:humble udp4 --port 8888
```

Terminal 2 — simulation (from `NCAP/`):

```bash
source ~/Documents/Polispace/ros2_humble/local_setup.zsh
export ROS_LOCALHOST_ONLY=1
dotnet /Applications/renode/output/bin/Release/Renode.dll --ui renode_configs/scripts/project/f4_w5500_single.resc
# then in the monitor:
(w5500) w5500 EnableHostSockets true
(w5500) sysbus LoadELF @/path/to/f4_w5500.elf
(w5500) start
```

Terminal 3 — verify:

```bash
source ~/Documents/Polispace/ros2_humble/local_setup.zsh
export ROS_LOCALHOST_ONLY=1
ros2 topic list
ros2 topic echo /heartbeat        # std_msgs/msg/Int32 in the reference app
```

## 5. Firmware build flags (F4)

The reference `CM7_arm_e2e` Makefile is patched for **soft-float** (M7 lib ABI).
The STM32F4 (Cortex-M4F) build must match its own static library:

```
-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
```

Rebuild `libmicroros` with these flags before linking, or the ABI will mismatch.

## 6. Troubleshooting

| Symptom | Cause / fix |
|---------|-------------|
| `ros2` command not found | `source ~/Documents/Polispace/ros2_humble/local_setup.zsh` first |
| DDS nodes don't discover each other | `export ROS_LOCALHOST_ONLY=1` |
| Agent not receiving XRCE | Check `w5500 EnableHostSockets true` and agent IP = `127.0.0.1` |
| `micro_ros_agent: command not found` | Expected — use the Docker agent, see `README_microros_agent.md` |
| Agent build fails on `fmt` errors | Expected on this machine — ROS 2's fmt v12 is incompatible; use Docker |
| Link error on float ABI | Static lib built for a different `-mfloat-abi`; rebuild it |
| Docker agent unreachable from host sockets | Use `-p 8888:8888/udp`, not `--net=host` |
