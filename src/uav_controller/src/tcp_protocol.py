#!/usr/bin/env python3
"""Length-prefixed JSON protocol shared with the ROS1 UGV bridge."""

import json
import struct


MAX_FRAME_BYTES = 8 * 1024 * 1024


def _recv_exact(sock, size):
    chunks = []
    remaining = size
    while remaining:
        chunk = sock.recv(remaining)
        if not chunk:
            raise EOFError("TCP peer closed the connection")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def send_json(sock, value):
    payload = json.dumps(value, separators=(",", ":")).encode("utf-8")
    if len(payload) > MAX_FRAME_BYTES:
        raise ValueError("TCP JSON frame is too large")
    sock.sendall(struct.pack("!I", len(payload)) + payload)


def recv_json(sock):
    header = _recv_exact(sock, 4)
    (size,) = struct.unpack("!I", header)
    if size <= 0 or size > MAX_FRAME_BYTES:
        raise ValueError("invalid TCP JSON frame size: %d" % size)
    return json.loads(_recv_exact(sock, size).decode("utf-8"))


def message(message_type, **fields):
    result = {"type": message_type, "version": 1}
    result.update(fields)
    return result
