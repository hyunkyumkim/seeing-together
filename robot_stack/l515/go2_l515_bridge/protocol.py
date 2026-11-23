"""Network protocol shared with pi/sender.cpp; no ROS dependency."""
from dataclasses import dataclass
import socket
import struct
import zlib
import numpy as np

HEADER = struct.Struct('!4sIQII5fI5fII4fI5f12fII')  # 172 bytes
MAX_FRAME = 20_000_000

@dataclass
class Frame:
    timestamp_ns: int
    depth: np.ndarray           # uint16 raw sensor units
    color_jpeg: bytes
    depth_intr: tuple           # fx fy ppx ppy
    color_intr: tuple
    depth_scale: float          # meters per raw uint16
    depth_model: int
    color_model: int
    depth_coeffs: tuple
    color_coeffs: tuple
    extrinsics: tuple           # depth -> color: column-major R[9] + t[3]
    nbytes: int = 0             # packet size on the wire

def read_exact(sock: socket.socket, size: int) -> bytes:
    parts = []
    remaining = size
    while remaining:
        part = sock.recv(min(remaining, 1024 * 1024))
        if not part:
            raise ConnectionError('TCP connection closed')
        parts.append(part)
        remaining -= len(part)
    return b''.join(parts)

def recv_frame(sock: socket.socket) -> Frame:
    (length,) = struct.unpack('!I', read_exact(sock, 4))
    if length < HEADER.size or length > MAX_FRAME:
        raise ValueError(f'Invalid frame length {length}')
    return decode(read_exact(sock, length))

def decode(data: bytes) -> Frame:
    if len(data) < HEADER.size:
        raise ValueError('Truncated header')
    v = HEADER.unpack_from(data)
    (magic, version, ts, dw, dh, dfx, dfy, dcx, dcy, scale, dmodel,
     *rest) = v
    if magic != b'L515' or version != 1:
        raise ValueError('Wrong magic or protocol version')
    dcoeff = tuple(rest[:5])
    cw, ch = rest[5:7]
    cfx, cfy, ccx, ccy = rest[7:11]
    cmodel = rest[11]
    ccoeff = tuple(rest[12:17])
    extr = tuple(rest[17:29])
    dlen, clen = rest[29:31]
    if not (0 < dw <= 4096 and 0 < dh <= 4096 and 0 < cw <= 4096 and 0 < ch <= 4096):
        raise ValueError('Invalid image dimensions')
    if scale <= 0 or dfx <= 0 or dfy <= 0 or cfx <= 0 or cfy <= 0:
        raise ValueError('Invalid camera calibration')
    if HEADER.size + dlen + clen != len(data):
        raise ValueError('Payload lengths do not match')
    raw = zlib.decompress(data[HEADER.size:HEADER.size + dlen])
    if len(raw) != dw * dh * 2:
        raise ValueError('Depth buffer has incorrect size')
    depth = np.frombuffer(raw, dtype='<u2').reshape(dh, dw)
    jpeg = data[HEADER.size + dlen:]
    return Frame(ts, depth, jpeg, (dfx, dfy, dcx, dcy), (cfx, cfy, ccx, ccy),
                 scale, dmodel, cmodel, dcoeff, ccoeff, extr, len(data))
