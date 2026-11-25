#!/usr/bin/env python3
"""Bench test for the OV7670 DVP wiring on a NUCLEO-F446RE.

Flashes nothing: it reads the UART output of the PIN TEST firmware and turns it
into a pass/fail exit code.

Build and flash the self-test first:

    make -C firmware clean
    make -C firmware NCAP_PIN_TEST=1
    STM32_Programmer_CLI -c port=SWD -w firmware/build/ncap_f4_w5500.hex -v -rst

Then run:

    python3 scripts/test_camera_pins.py              # auto-detect the port
    python3 scripts/test_camera_pins.py -p /dev/cu.usbmodem113303

The firmware probes each data pin with the internal pull-up and pull-down:

    OK          driven by the sensor (same duty both ways, or toggling)
    OPEN        follows the pull -> wire not connected to the sensor
    STUCK LOW   ~0% both ways   -> wire on GND / sensor not driving
    STUCK HIGH  ~100% both ways -> wire on VCC

HS, VS and PLK are checked for edges (NO SIGNAL means the line is dead).

Exit codes: 0 PASS, 1 FAIL, 2 no result (wrong port / firmware not running).
"""

import argparse
import glob
import re
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is required: python3 -m pip install pyserial")

PIN_RE = re.compile(
    r"PIN\s+(\S+)\s+P([ABC])(\d+)\s+up=\s*(\d+)%\s+dn=\s*(\d+)%\s+edges=(\d+)\s+hz=(\d+)\s+(.+?)\s*$"
)
RESULT_RE = re.compile(r"PIN_TEST RESULT:\s*(PASS|FAIL)\s*\((\d+)\s+bad\)")


def find_port(explicit):
    if explicit:
        return explicit
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    if len(ports) == 1:
        return ports[0]
    if not ports:
        return None
    return ports[0]


def main():
    ap = argparse.ArgumentParser(description="OV7670 DVP wiring self-test")
    ap.add_argument("-p", "--port", help="serial port (default: first /dev/cu.usbmodem*)")
    ap.add_argument("-t", "--timeout", type=float, default=12.0,
                    help="seconds to wait for a result (default 12)")
    args = ap.parse_args()

    port = find_port(args.port)
    if not port:
        sys.exit("no /dev/cu.usbmodem* found; connect the Nucleo or pass --port")

    print(f"listening on {port} for {args.timeout:.0f}s ...")
    ser = serial.Serial(port, 115200, timeout=0.2)

    deadline = time.time() + args.timeout
    pins = []
    result = None
    try:
        while time.time() < deadline:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", "replace").strip()
            if line:
                print("  " + line)
            m = PIN_RE.search(line)
            if m:
                name, port_char, pin_no, up, dn, edges, hz, verdict = m.groups()
                pins.append((name, f"P{port_char}{pin_no}", int(up), int(dn),
                             int(edges), int(hz), verdict))
            m = RESULT_RE.search(line)
            if m:
                result = (m.group(1), int(m.group(2)))
                break
    finally:
        ser.close()

    if not pins and result is None:
        sys.exit(2)

    print("\nDVP pin summary")
    print(f"{'signal':6} {'pin':5} {'pull-up':>7} {'pull-dn':>7} {'edges':>7} {'Hz':>8}  verdict")
    for name, pin, up, dn, edges, hz, verdict in pins:
        print(f"{name:6} {pin:5} {up:6}% {dn:6}% {edges:7} {hz:8}  {verdict}")

    if result is None:
        print("\nFAIL: no PIN_TEST RESULT line (firmware not running?)")
        sys.exit(2)

    ok, bad = result
    print(f"\nresult: {ok} ({bad} bad)")
    sys.exit(0 if ok == "PASS" else 1)


if __name__ == "__main__":
    main()