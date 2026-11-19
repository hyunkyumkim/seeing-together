"""Camera calibration from checkerboard frames saved by grab.py.

Usage:  python calibrate.py --square-mm 25.0          (use the square size you measured on the print)
Reads calib/*.jpg (9x6 inner corners), writes camera.json and prints the reprojection error.
Good result: RMS reprojection error below about 0.5 px; redo frames if it is above 1 px.
"""
import argparse
import glob
import json

import cv2
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--images", default="calib/*.jpg")
ap.add_argument("--square-mm", type=float, required=True)
ap.add_argument("--cols", type=int, default=9)
ap.add_argument("--rows", type=int, default=6)
ap.add_argument("--out", default="camera.json")
args = ap.parse_args()

pattern = (args.cols, args.rows)
objp = np.zeros((args.cols * args.rows, 3), np.float32)
objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square_mm / 1000.0  # meters

obj_pts, img_pts, size, used = [], [], None, []
crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
for f in sorted(glob.glob(args.images)):
    gray = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
    if gray is None:
        continue
    if size is None:
        size = gray.shape[::-1]
    elif gray.shape[::-1] != size:
        raise SystemExit(f"{f}: size {gray.shape[::-1]} differs from {size}; use one resolution only")
    ok, c = cv2.findChessboardCorners(gray, pattern)
    if not ok:
        print(f"skip {f} (board not found)")
        continue
    c = cv2.cornerSubPix(gray, c, (5, 5), (-1, -1), crit)
    obj_pts.append(objp)
    img_pts.append(c)
    used.append(f)

if len(used) < 10:
    raise SystemExit(f"only {len(used)} usable frames; collect at least 15")

rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj_pts, img_pts, size, None, None)
per = []
for o, i, r, t in zip(obj_pts, img_pts, rvecs, tvecs):
    p, _ = cv2.projectPoints(o, r, t, K, dist)
    per.append(float(np.sqrt(np.mean(np.sum((p.reshape(-1, 2) - i.reshape(-1, 2)) ** 2, axis=1)))))
for f, e in sorted(zip(used, per), key=lambda x: -x[1])[:5]:
    print(f"worst frame: {f}  {e:.2f} px")
print(f"frames used: {len(used)}   image size: {size[0]}x{size[1]}   RMS reprojection error: {rms:.3f} px")
print(f"fx={K[0,0]:.1f} fy={K[1,1]:.1f} cx={K[0,2]:.1f} cy={K[1,2]:.1f}")
json.dump({"image_size": list(size), "K": K.tolist(), "dist": dist.ravel().tolist(), "rms_px": rms,
           "frames": len(used), "square_mm": args.square_mm}, open(args.out, "w"), indent=2)
print(f"wrote {args.out}")
