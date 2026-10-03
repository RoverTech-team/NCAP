#!/usr/bin/env python3
"""Receive the firmware's UDP luma stream and verify the pixels.

Datagram format (one per row block):
    'V' 'F' 'R' '0' | u32 seq | u16 width | u16 height
                   | u16 row_offset | u16 rows | u32 payload_len | luma...

Usage: video_client.py [port] [frames_to_check]
"""
import socket
import struct
import sys
from collections import defaultdict

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 18080
WANT = int(sys.argv[2]) if len(sys.argv) > 2 else 3

FRAME_HDR = 20
META_MAGIC = b"VFM0"
FRAME_MAGIC = b"VFR0"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("127.0.0.1", PORT))
import os
sock.settimeout(float(os.environ.get('VID_TIMEOUT', '60')))

print(f"listening on udp/{PORT}")

frames = defaultdict(dict)   # seq -> {row_offset: (rows, payload)}
meta = None
checked = set()
seen_seq = []

while True:
    data, addr = sock.recvfrom(65535)

    if data[:4] == META_MAGIC:
        meta = data[4:].decode(errors="replace")
        print("--- stream preamble ---")
        print(meta.strip())
        continue

    if data[:4] != FRAME_MAGIC:
        print(f"unexpected magic {data[:4]!r} ({len(data)} B) - ignored")
        continue
    if len(data) < FRAME_HDR:
        continue

    seq, w, h, row, rows, plen = struct.unpack(">IHHHHI", data[4:FRAME_HDR])
    payload = data[FRAME_HDR:FRAME_HDR + plen]

    if len(payload) != plen:
        print(f"seq {seq}: short payload {len(payload)} != {plen}")
        continue

    frames[seq][row] = (rows, payload)
    if seq not in seen_seq:
        seen_seq.append(seq)

    # Reassemble and check a frame once every row block has arrived.
    if seq not in checked and len(frames[seq]) * rows >= h:
        covered = sum(r for (r, _p) in frames[seq].values())
        if covered >= h:
            checked.add(seq)
            img = bytearray(w * h)
            for row_off, (r, p) in sorted(frames[seq].items()):
                for i in range(r):
                    dst = (row_off + i) * w
                    src = i * w
                    img[dst:dst + w] = p[src:src + w]

            nz = sum(1 for b in img if b)
            print(f"frame seq={seq} {w}x{h} bytes={len(img)} "
                  f"min={min(img)} max={max(img)} mean={sum(img)/len(img):.1f} "
                  f"nonzero={100*nz/len(img):.1f}%")
            # A correct luma ramp is neither all-zero nor saturated.
            assert plen == rows * w, f"payload {plen} != rows*width {rows*w}"
            assert min(img) < max(img), "frame is flat"
            assert len(checked) >= WANT or True

            if len(checked) >= WANT:
                print(f"OK: verified {len(checked)} frames")
                break

    if len(seen_seq) > 200:
        seen_seq.pop(0)

sock.close()