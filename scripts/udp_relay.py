#!/usr/bin/env python3
"""
Host UDP relay for the NCAP micro-ROS agent.

Why this exists
---------------
The micro-ROS agent normally runs with `network_mode: host` in production, so it
sees one stable client address (IP:port) across retransmits. On Docker Desktop for
macOS host networking is unavailable, so the agent must be reached through a
published port (`-p 8888:8888/udp`). docker-proxy rewrites the UDP source port on
every datagram, so the agent instead sees `192.168.65.1:<random>` each time and
re-creates the session rather than resuming it.

This relay fixes that: it owns ONE socket toward the agent, so the agent always
sees the same peer address, and it relays datagrams both ways for the W5500 model.

    w5500 model --UDP--> relay :8890 --UDP--> agent :8888
    w5500 model <--UDP-- relay :8890 <--UDP-- agent :8888

Usage:
    ./scripts/udp_relay.py [--listen-port 8890] [--agent 127.0.0.1:8888]
"""

import argparse
import socket
import sys
import threading


def parse_endpoint(text):
    host, _, port = text.rpartition(":")
    if not host or not port.isdigit():
        raise argparse.ArgumentTypeError(f"expected HOST:PORT, got {text!r}")
    return (host, int(port))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--listen-port", type=int, default=8890,
                    help="UDP port clients (the W5500 model) send to")
    ap.add_argument("--listen-host", default="0.0.0.0",
                    help="address to bind for clients")
    ap.add_argument("--agent", type=parse_endpoint, default=("127.0.0.1", 8888),
                    help="micro-ROS agent endpoint")
    args = ap.parse_args()

    # Single socket toward the agent: its source address:port is what the agent
    # sees, and it must not change between datagrams.
    uplink = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    uplink.bind(("127.0.0.1", 0))
    uplink.settimeout(0.2)
    print(f"relay -> agent {args.agent} via {uplink.getsockname()}", flush=True)

    downlink = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    downlink.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    downlink.bind((args.listen_host, args.listen_port))
    downlink.settimeout(0.2)
    print(f"relay listening for clients on {args.listen_host}:{args.listen_port}", flush=True)

    peer_lock = threading.Lock()
    peer = {"addr": None}

    def client_to_agent():
        while True:
            try:
                data, addr = downlink.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                return
            with peer_lock:
                peer["addr"] = addr
            try:
                uplink.sendto(data, args.agent)
                print(f"-> agent {len(data):4d}B from client {addr[0]}:{addr[1]}",
                      flush=True)
            except OSError as e:
                print(f"send to agent failed: {e}", flush=True)

    def agent_to_client():
        while True:
            try:
                data, _ = uplink.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                return
            with peer_lock:
                addr = peer["addr"]
            if addr is None:
                print("<- agent: no client yet, dropping", flush=True)
                continue
            try:
                downlink.sendto(data, addr)
                print(f"<- client {len(data):4d}B to {addr[0]}:{addr[1]}", flush=True)
            except OSError as e:
                print(f"send to client failed: {e}", flush=True)

    threads = [threading.Thread(target=client_to_agent, daemon=True),
               threading.Thread(target=agent_to_client, daemon=True)]
    for t in threads:
        t.start()
    try:
        for t in threads:
            t.join()
    except KeyboardInterrupt:
        print("relay stopped", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
