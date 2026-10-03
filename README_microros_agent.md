# micro-ROS Agent on macOS (NCAP)

The micro-ROS agent runs in Docker, exactly like the `microk3` project does
(`microros/micro-ros-agent:humble` + `udp4 --port 8888`).

> Building it from source on this machine does **not** work: ROS 2 Humble here ships
> `fmt` v12, and Micro-XRCE-DDS-Agent v2.4.2 uses the pre-v11 formatter API
> (`error: implicit instantiation of undefined template
> 'fmt::detail::type_is_unformattable_for<eprosima::uxr::IPv4EndPoint, char>'`).
> Building a private spdlog/fmt does not help because Fast-DDS pulls ROS's
> `include/` in transitively. Use Docker.

## One-time setup

```bash
open -a Docker                 # Docker Desktop must be running
docker pull microros/micro-ros-agent:humble
```

## Run

```bash
docker rm -f ncap-agent 2>/dev/null
docker run -d --name ncap-agent -p 8888:8888/udp \
  microros/micro-ros-agent:humble udp4 --port 8888
```

Check:

```bash
docker logs ncap-agent        # "running... port: 8888"
docker ps --filter name=ncap-agent
```

## Stop / clean

```bash
docker stop ncap-agent
docker rm ncap-agent
```

## Verified on this machine

- Docker 29.2.1, image `microros/micro-ros-agent:humble` pulled OK.
- Agent container up, UDP `0.0.0.0:8888 -> 8888/udp`.
- Agent socket confirmed bound inside the container (`/proc/net/udp` local `:22B8`).
- Host → container UDP path verified independently with a throwaway UDP echo
  container on 8899 (round-trip `PONG:HELLO` succeeded).

## Why the firmware talks to `127.0.0.1`

The W5500 model in `renode_configs/peripherals/W5500.cs` bridges sockets to the
**host** stack (`w5500 EnableHostSockets true`), so the firmware's agent IP must be
the Docker-published port on the host:

```c
// w5500_transport.c
static uint8_t w5500_agent_ip[4] = {127, 0, 0, 1};   // was {192,168,0,1}
```

With `-p 8888:8888/udp` the agent is reachable at `127.0.0.1:8888` from the host.

## E2E run order

1. `docker run -d --name ncap-agent -p 8888:8888/udp microros/micro-ros-agent:humble udp4 --port 8888`
2. Start Renode from `NCAP/`:
   ```bash
   dotnet /Applications/renode/output/bin/Release/Renode.dll --ui renode_configs/scripts/project/f4_w5500_single.resc
   ```
   then in the monitor: `w5500 EnableHostSockets true`, `sysbus LoadELF @<fw>.elf`, `start`
3. Verify:
   ```bash
   source ~/Documents/Polispace/ros2_humble/local_setup.zsh
   export ROS_LOCALHOST_ONLY=1
   ros2 topic list
   ```

## Alternative: the microk3 stack

If you want the full dashboard (ROS bridge, telemetry, Caddy), `microk3` already
wires the same agent image together:

```bash
cd ~/Documents/Polispace/Stm-Micro_ros-eth/microrosWs/microk3
docker compose up -d          # uros-agent on 8888/udp + dashboard on :5050
```

Reuse that by pointing NCAP at the same `uros-agent` container, or just run the
single `ncap-agent` container as above when you only need the agent.
