"""UDP IMU receiver for the RayNeo X3 Pro feasibility test.

Usage:  python imu_receiver.py --port 5005 --duration 300 --out imu_log.csv

Packet format (one or more text lines per UDP packet):
    <tag>,<seq>,<device_timestamp_ns>,<x>,<y>,<z>
    tag: A = accelerometer, G = gyroscope
seq increments per sensor on the glasses, so gaps in seq = lost or dropped samples.
"""
import argparse, csv, socket, statistics, time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--duration", type=float, default=60.0, help="seconds to record after first packet")
    ap.add_argument("--out", default="imu_log.csv")
    a = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", a.port))
    sock.settimeout(1.0)
    print(f"Listening on UDP :{a.port} ... (waiting for first packet)")

    rows = []            # (tag, seq, dev_ts_ns, pc_time_s, x, y, z)
    t_start = None
    while True:
        try:
            data, _ = sock.recvfrom(65535)
        except socket.timeout:
            if t_start is not None and time.perf_counter() - t_start > a.duration:
                break
            continue
        now = time.perf_counter()
        if t_start is None:
            t_start = now
            print("First packet received, recording...")
        for line in data.decode("ascii", "ignore").splitlines():
            p = line.split(",")
            if len(p) != 6:
                continue
            try:
                rows.append((p[0], int(p[1]), int(p[2]), now, float(p[3]), float(p[4]), float(p[5])))
            except ValueError:
                continue
        if now - t_start > a.duration:
            break

    with open(a.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["tag", "seq", "device_ts_ns", "pc_time_s", "x", "y", "z"])
        w.writerows(rows)

    print(f"\nSaved {len(rows)} rows to {a.out}")
    for tag, name in (("A", "accelerometer"), ("G", "gyroscope")):
        r = [x for x in rows if x[0] == tag]
        if not r:
            print(f"{name}: no samples")
            continue
        seqs = sorted({x[1] for x in r})
        expected = seqs[-1] - seqs[0] + 1
        lost = expected - len(seqs)
        ts = sorted(x[2] for x in r)
        dev_dur = (ts[-1] - ts[0]) / 1e9
        pc = [x[3] for x in r]
        pc_dur = max(pc) - min(pc)
        dts = [(b - a_) / 1e6 for a_, b in zip(ts, ts[1:])]  # ms
        rate_dev = (expected - 1) / dev_dur if dev_dur > 0 else float("nan")  # generated rate (lost samples counted)
        rate_pc = (len(seqs) - 1) / pc_dur if pc_dur > 0 else float("nan")  # delivered rate
        p95 = sorted(dts)[int(0.95 * (len(dts) - 1))] if dts else float("nan")
        print(f"\n[{name}]")
        print(f"  samples received : {len(seqs)} (unique), duplicates: {len(r) - len(seqs)}")
        print(f"  expected (seq)   : {expected}  -> lost: {lost} ({100.0 * lost / expected:.2f} %)")
        print(f"  duration         : {dev_dur:.1f} s (device clock), {pc_dur:.1f} s (PC clock)")
        print(f"  generated rate   : {rate_dev:.1f} Hz (from seq / device clock)")
        print(f"  delivered rate   : {rate_pc:.1f} Hz (received / PC clock)")
        print(f"  dt device ts     : mean {statistics.mean(dts):.2f} ms, p95 {p95:.2f} ms, max {max(dts):.2f} ms")


if __name__ == "__main__":
    main()
