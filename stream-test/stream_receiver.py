"""Laptop side of the frame streaming test (pairs with FrameStreamActivity.kt).

Receives JPEG frames over TCP, shows them, and measures:
  - delivered fps, frame loss (gaps in the glasses' sequence numbers), bitrate
  - end-to-end latency with a "tag clock": a window shows an AprilTag whose ID changes every
    --step-ms milliseconds. Point the glasses at that window. When a received frame contains a
    clock tag, latency = (time the frame arrived) - (time that tag ID was first drawn).
    This covers camera exposure + encode + Wi-Fi/USB + receive. It also includes the monitor's
    own display delay, so it slightly overestimates, and it has a resolution of about one step.

Usage:
  python stream_receiver.py --port 5006 --duration 60 --out wifi_640
  python stream_receiver.py --headless ...      (no windows; no latency measurement)
Keys: q = quit.
Outputs: <out>_frames.csv, <out>_latency.csv, <out>_summary.txt
"""
import argparse
import csv
import json
import socket
import statistics
import struct
import threading
import time

import cv2
import numpy as np

HDR = struct.Struct(">4sqqqiiii")  # magic, seq, captureTsNs, encodedNs, devicePipeUs, w, h, jpegLen
CLOCK_BASE, CLOCK_COUNT = 100, 487  # clock tags use IDs 100..586; real tags (0..99) are left alone


def recv_exact(conn, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("closed")
        buf += chunk
    return bytes(buf)


class Receiver(threading.Thread):
    """Accepts one sender at a time; timestamps each frame on arrival."""

    def __init__(self, port):
        super().__init__(daemon=True)
        self.port = port
        self.frames = []          # (seq, capture_ts_ns, encoded_ns, device_pipe_us, w, h, nbytes, t_recv)
        self.latest = None        # (seq, t_recv, jpeg, capture_ts_ns)
        self.lock = threading.Lock()
        self.connections = 0
        self.first_recv = None
        self.stop = False

    def run(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", self.port))
        srv.listen(1)
        srv.settimeout(1.0)
        print(f"Listening on TCP :{self.port} ... (waiting for the glasses)")
        while not self.stop:
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            self.connections += 1
            print(f"Connected from {addr[0]} (connection #{self.connections})")
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            try:
                while not self.stop:
                    magic, seq, cap, enc, pipe, w, h, n = HDR.unpack(recv_exact(conn, HDR.size))
                    if magic != b"FRM1":
                        raise ValueError(f"bad magic {magic!r}")
                    jpeg = recv_exact(conn, n)
                    t = time.perf_counter()
                    with self.lock:
                        if self.first_recv is None:
                            self.first_recv = t
                        self.frames.append((seq, cap, enc, pipe, w, h, n, t))
                        self.latest = (seq, t, jpeg, cap)
            except (ConnectionError, OSError, ValueError) as e:
                print(f"Connection ended: {e}")
            finally:
                conn.close()
        srv.close()


def tag_image(dictionary, tag_id, px):
    m = cv2.aruco.generateImageMarker(dictionary, tag_id, px)
    return np.pad(m, px // 6, constant_values=255)


def pct(values, p):
    if not values:
        return float("nan")
    s = sorted(values)
    return s[min(len(s) - 1, int(round(p / 100 * (len(s) - 1))))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5006)
    ap.add_argument("--duration", type=float, default=0, help="seconds after the first frame; 0 = until q")
    ap.add_argument("--out", default="stream", help="output file prefix")
    ap.add_argument("--rotate", choices=["cw", "ccw", "180", "none"], default="cw",
                    help="rotate decoded frames so the view is upright (the camera sensor is mounted "
                         "rotated 90 deg); default cw. Does not change what is logged or measured.")
    ap.add_argument("--step-ms", type=int, default=50, help="tag clock step")
    ap.add_argument("--clock-px", type=int, default=480, help="tag clock size in pixels")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--obs-interval", type=float, default=0.0, metavar="SEC",
                    help="write at most one observation per tag every SEC seconds (0 = every frame)")
    ap.add_argument("--no-clock", action="store_true",
                    help="do not show the tag clock window (use when pointing the glasses at real tags)")
    ap.add_argument("--obs-out", metavar="PATH",
                    help="write each real tag detection (IDs < 100) as a schema-v1 'tag' observation "
                         "(JSONL, see memory/schema.md)")
    args = ap.parse_args()

    rx = Receiver(args.port)
    rx.start()

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    shown = {}            # clock tag id -> perf_counter time it was first drawn
    latency_rows = []     # (seq, t_recv, tag_id, latency_ms)
    real_tags = 0
    checked = 0           # frames that were decoded and searched for tags
    tag_frames = {}       # real tag id -> number of checked frames it was seen in
    obs_f = open(args.obs_out, "w", encoding="utf-8") if args.obs_out else None
    last_written = {}     # real tag id -> t_recv of the last observation written
    obs_written = 0
    unix_off = time.time() - time.perf_counter()  # perf_counter -> laptop Unix time
    ghost_since, ghost_last = {}, {}   # static clock-tag IDs (e.g. a stale window) are ignored
    last_k, last_seq_shown = None, None
    t0 = time.perf_counter()

    try:
        while True:
            now = time.perf_counter()
            if args.duration and rx.first_recv and now - rx.first_recv >= args.duration:
                break
            if args.headless:
                time.sleep(0.05)
                continue

            # tag clock
            k = int((now - t0) * 1000 // args.step_ms)
            if k != last_k and not args.no_clock:
                tid = CLOCK_BASE + k % CLOCK_COUNT
                img = cv2.cvtColor(tag_image(dictionary, tid, args.clock_px), cv2.COLOR_GRAY2BGR)
                cv2.putText(img, f"{(now - t0) * 1000:10.0f} ms", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.imshow("clock (point the glasses here)", img)
                last_k = k
                pending_tid = tid
            else:
                pending_tid = None

            # newest received frame
            with rx.lock:
                latest = rx.latest
            if latest and latest[0] != last_seq_shown:
                seq, t_recv, jpeg, cap_ns = latest
                last_seq_shown = seq
                frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    rot = {"cw": cv2.ROTATE_90_CLOCKWISE, "ccw": cv2.ROTATE_90_COUNTERCLOCKWISE,
                           "180": cv2.ROTATE_180}.get(args.rotate)
                    if rot is not None:
                        frame = cv2.rotate(frame, rot)
                    corners, ids, _ = detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
                    checked += 1
                    if ids is not None:
                        for k, i in enumerate(ids.flatten().tolist()):
                            if i >= CLOCK_BASE:
                                # a live clock tag changes every step; one that stays in view
                                # for > 0.5 s is a static ghost (stale window / reflection)
                                if t_recv - ghost_last.get(i, -9) > 0.3:
                                    ghost_since[i] = t_recv
                                ghost_last[i] = t_recv
                                if t_recv - ghost_since[i] > 0.5:
                                    continue
                            if i >= CLOCK_BASE and i in shown and 0 <= t_recv - shown[i] <= 3.0:
                                latency_rows.append((seq, t_recv, i, (t_recv - shown[i]) * 1000))
                            elif i < CLOCK_BASE:
                                real_tags += 1
                                tag_frames[i] = tag_frames.get(i, 0) + 1
                                if obs_f and t_recv - last_written.get(i, -1e9) >= args.obs_interval:
                                    last_written[i] = t_recv
                                    obs_written += 1
                                    c = corners[k].reshape(4, 2)
                                    obs_f.write(json.dumps({
                                        "v": 1, "id": f"g-{seq:06d}-{i}", "src": "glasses", "type": "tag",
                                        "t": round(unix_off + t_recv, 3), "t_dev": cap_ns / 1e9, "frame": "image",
                                        "tag_id": i,
                                        "center": [round(float(c[:, 0].mean()), 1), round(float(c[:, 1].mean()), 1)],
                                        "corners": [[round(float(x), 1), round(float(y), 1)] for x, y in c],
                                        "img": [frame.shape[1], frame.shape[0]], "frame_seq": seq}) + "\n")
                    lat = [r[3] for r in latency_rows[-30:]]
                    n = len(rx.frames)
                    el = (t_recv - rx.first_recv) if rx.first_recv else 0
                    info = f"seq {seq}  frames {n}  fps {n / el if el > 0 else 0:.1f}"
                    if lat:
                        info += f"  latency med {statistics.median(lat):.0f} ms (last 30)"
                    view = frame
                    if view.shape[1] > 960:
                        view = cv2.resize(view, (960, int(view.shape[0] * 960 / view.shape[1])))
                    cv2.putText(view, info, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.imshow("stream", view)

            key = cv2.waitKey(1) & 0xFF
            if pending_tid is not None:
                shown[pending_tid] = time.perf_counter()  # just after the window was refreshed
            if key == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        rx.stop = True
        cv2.destroyAllWindows()

    with rx.lock:
        frames = list(rx.frames)
    if obs_f:
        obs_f.close()
    write_outputs(args, frames, latency_rows, real_tags, rx.connections, checked, tag_frames, obs_written)


def write_outputs(args, frames, latency_rows, real_tags, connections, checked=0, tag_frames=None,
                  obs_written=0):
    with open(f"{args.out}_frames.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seq", "capture_ts_ns", "encoded_ns", "device_pipe_us", "width", "height", "jpeg_bytes", "pc_recv_s"])
        w.writerows(frames)
    with open(f"{args.out}_latency.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seq", "pc_recv_s", "clock_tag_id", "latency_ms"])
        w.writerows(latency_rows)

    # seq restarts at 0 on every new connection: report the longest connection only
    segs = [[]]
    for i, r in enumerate(frames):
        if i and r[0] < frames[i - 1][0]:
            segs.append([])
        segs[-1].append(r)
    all_frames = frames
    frames = max(segs, key=len)
    lines = [f"connections: {connections}", f"frames received (all connections): {len(all_frames)}"]
    if len(segs) > 1:
        lines.append(f"NOTE: {len(segs)} connections; stats below use the longest one "
                     f"({len(frames)} frames; others: {[len(x) for x in segs if x is not frames]})")
    lines.append(f"frames received (this connection): {len(frames)}")
    if len(frames) >= 2:
        seqs = [r[0] for r in frames]
        t = [r[7] for r in frames]
        dur = t[-1] - t[0]
        expected = seqs[-1] - seqs[0] + 1
        lost = expected - len(set(seqs))
        nbytes = [r[6] for r in frames]
        gaps = [(b - a) * 1000 for a, b in zip(t, t[1:])]
        pipe = [r[3] / 1000 for r in frames if r[3] >= 0]
        cam_dt = [(b[1] - a[1]) / 1e6 for a, b in zip(frames, frames[1:])]
        lines += [
            f"resolution: {frames[0][4]}x{frames[0][5]}",
            f"duration (PC clock): {dur:.1f} s",
            f"delivered fps: {(len(frames) - 1) / dur:.1f}",
            f"camera frames (seq range): {expected}  -> not delivered: {lost} ({100 * lost / expected:.1f} %)",
            f"camera fps (seq / sensor clock): {(expected - 1) / ((frames[-1][1] - frames[0][1]) / 1e9):.1f}",
            f"JPEG size: mean {statistics.mean(nbytes) / 1024:.1f} KB  -> {8 * sum(nbytes) / dur / 1e6:.2f} Mbit/s",
            f"arrival gaps: median {statistics.median(gaps):.1f} ms, p95 {pct(gaps, 95):.1f} ms, max {max(gaps):.1f} ms",
            f"sensor time between delivered frames: median {statistics.median(cam_dt):.1f} ms",
        ]
        if pipe:
            lines.append(f"on-glasses capture->JPEG: median {statistics.median(pipe):.1f} ms, p95 {pct(pipe, 95):.1f} ms")
        else:
            lines.append("on-glasses capture->JPEG: n/a (sensor timestamps are not in the realtime clock)")
    lat = [r[3] for r in latency_rows]
    if lat:
        lines.append(f"end-to-end latency (tag clock, step {args.step_ms} ms): n={len(lat)}, "
                     f"median {statistics.median(lat):.0f} ms, p95 {pct(lat, 95):.0f} ms, "
                     f"min {min(lat):.0f} ms, max {max(lat):.0f} ms")
    else:
        lines.append("end-to-end latency: no clock tags seen in received frames")
    lines.append(f"real tag detections (IDs < {CLOCK_BASE}): {real_tags}")
    lines.append(f"frames checked for tags: {checked} (only the newest frame is checked, so this can be "
                 f"below frames received)")
    for i in sorted(tag_frames or {}):
        lines.append(f"  tag {i}: seen in {tag_frames[i]} of {checked} checked frames "
                     f"({100 * tag_frames[i] / checked:.1f} %)")
    if args.obs_out:
        every = f"at most 1 per tag every {args.obs_interval:g} s" if args.obs_interval > 0 else "every detection"
        lines.append(f"tag observations written to {args.obs_out}: {obs_written} ({every})")
    text = "\n".join(lines)
    print("\n" + text)
    with open(f"{args.out}_summary.txt", "w") as f:
        f.write(text + "\n")
    print(f"\nSaved {args.out}_frames.csv, {args.out}_latency.csv, {args.out}_summary.txt")


if __name__ == "__main__":
    main()
