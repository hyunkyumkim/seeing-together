"""Save 640x480 frames from the glasses stream for calibration and the tag pose test.

Runs the same receiver as stream-test (FrameStreamActivity.kt sends JPEG over TCP) and rotates frames
the same way, so calibration and pose use exactly the images the live system sees.

Calibration:  python grab.py calib            -> calib/calib_###.jpg
  Hold the checkerboard still in view and press c. A frame is saved only if all 9x6 corners are found.
  Collect about 20 frames: near and far, tilted left/right/up/down, board in every part of the image.

Pose test:    python grab.py pose             -> poses/<label>_r<k>/frame_##.jpg
  Walks through the positions in POSITIONS (3 repeats each). Put the glasses on the mark shown on screen,
  wait until the image is still, press g: the next 20 frames are saved, then it moves to the next item.
  Keys: g = grab, n = skip, b = back, q = quit.

Options: --port 5006  --rotate cw  --frames 20  --repeats 3
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "stream-test"))
from stream_receiver import Receiver  # noqa: E402

# distance in cm, angle in degrees (camera to the right of the board's normal)
POSITIONS = [(d, a) for d in (50, 100, 200, 300) for a in (0, 30, 60)]
ROT = {"cw": cv2.ROTATE_90_CLOCKWISE, "ccw": cv2.ROTATE_90_COUNTERCLOCKWISE, "180": cv2.ROTATE_180}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["calib", "pose"])
    ap.add_argument("--port", type=int, default=5006)
    ap.add_argument("--rotate", choices=["cw", "ccw", "180", "none"], default="cw")
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--repeats", type=int, default=3)
    args = ap.parse_args()

    rx = Receiver(args.port)
    rx.start()
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
    items = [(d, a, r) for d, a in POSITIONS for r in range(1, args.repeats + 1)]
    idx, n_calib, grabbing, saved, last_seq = 0, 0, None, 0, None
    os.makedirs("calib" if args.mode == "calib" else "poses", exist_ok=True)
    if args.mode == "calib":
        n_calib = len([f for f in os.listdir("calib") if f.endswith(".jpg")])

    while True:
        with rx.lock:
            latest = rx.latest
        if latest is None or latest[0] == last_seq:
            if cv2.waitKey(5) & 0xFF == ord("q"):
                break
            continue
        last_seq = latest[0]
        frame = cv2.imdecode(np.frombuffer(latest[2], np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            continue
        if args.rotate in ROT:
            frame = cv2.rotate(frame, ROT[args.rotate])
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        view = frame.copy()

        if args.mode == "pose" and grabbing:
            cv2.imwrite(os.path.join(grabbing, f"frame_{saved:02d}.jpg"), frame)
            saved += 1
            if saved >= args.frames:
                print(f"saved {saved} frames to {grabbing}")
                grabbing, idx = None, idx + 1

        if args.mode == "calib":
            ok, corners = cv2.findChessboardCorners(gray, (9, 6), cv2.CALIB_CB_FAST_CHECK)
            if ok:
                cv2.drawChessboardCorners(view, (9, 6), corners, ok)
            msg = f"calib frames: {n_calib}   board {'FOUND' if ok else 'not found'}   c=save q=quit"
        else:
            corners, ids, _ = det.detectMarkers(gray)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(view, corners, ids)
            seen = [] if ids is None else sorted(ids.flatten().tolist())
            if idx >= len(items):
                msg = "all positions done   q=quit"
            else:
                d, a, r = items[idx]
                state = f"saving {saved}/{args.frames}" if grabbing else "g=grab n=skip b=back q=quit"
                msg = f"[{idx + 1}/{len(items)}] d={d} cm a={a} deg repeat {r}   tags {seen}   {state}"
        cv2.putText(view, msg, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.imshow("grab", view)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if args.mode == "calib" and key == ord("c") and ok:
            cv2.imwrite(os.path.join("calib", f"calib_{n_calib:03d}.jpg"), frame)
            n_calib += 1
            print(f"saved calib_{n_calib - 1:03d}.jpg")
        if args.mode == "pose" and not grabbing and idx < len(items):
            if key == ord("g"):
                d, a, r = items[idx]
                grabbing = os.path.join("poses", f"d{d}_a{a}_r{r}")
                os.makedirs(grabbing, exist_ok=True)
                saved = 0
            elif key == ord("n"):
                idx += 1
            elif key == ord("b") and idx > 0:
                idx -= 1
    rx.stop = True
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
