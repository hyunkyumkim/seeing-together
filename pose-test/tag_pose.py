"""Tag pose accuracy: estimated glasses position vs tape-measured position, one tag vs two tags.

Board: tag 0 (left) and tag 1 (right) on one flat vertical board, centers --spacing-cm apart, same height.
Reference point = midpoint between the two tag centers, at the height of the camera lens.
Ground truth for a folder named d<cm>_a<deg>_r<k>: the camera lens is d cm from the reference point,
a degrees to the right of the board's normal (viewer's right when facing the board).

For every frame it estimates the camera position in the board frame
  - one tag : tag 0 only (IPPE_SQUARE), shifted by the known offset to the reference point
  - two tags: all 8 corners of tags 0 and 1 together
and compares distance, angle and position with the ground truth.

Usage:  python tag_pose.py --tag-mm 150 --spacing-cm 20
Outputs: pose_frames.csv (every frame) and pose_summary.csv (per position), summary printed.
"""
import argparse
import csv
import glob
import json
import math
import os
import re
from collections import defaultdict

import cv2
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--camera", default="camera.json")
ap.add_argument("--poses", default="poses")
ap.add_argument("--tag-mm", type=float, required=True, help="measured black-square side")
ap.add_argument("--spacing-cm", type=float, required=True, help="measured center-to-center spacing")
args = ap.parse_args()

cam = json.load(open(args.camera))
K, dist = np.array(cam["K"]), np.array(cam["dist"])
s, S = args.tag_mm / 1000.0, args.spacing_cm / 100.0
# tag corners in the tag frame (x right, y up, z toward the viewer); aruco order TL, TR, BR, BL
SQ = np.array([[-s / 2, s / 2, 0], [s / 2, s / 2, 0], [s / 2, -s / 2, 0], [-s / 2, -s / 2, 0]])
OFFSET = {0: np.array([-S / 2, 0, 0]), 1: np.array([S / 2, 0, 0])}
det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))


def cam_pos(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    return (-R.T @ tvec.reshape(3)).reshape(3)   # camera center in the object frame


def metrics(p, gt):
    d = float(np.linalg.norm(p))
    ang = math.degrees(math.atan2(p[0], p[2]))   # horizontal angle from the normal, + = right
    return d, ang, float(np.linalg.norm(p - gt))


rows = []
for folder in sorted(glob.glob(os.path.join(args.poses, "*"))):
    m = re.match(r"d(\d+)_a(-?\d+)_r(\d+)$", os.path.basename(folder))
    if not m:
        continue
    d_gt, a_gt, rep = int(m[1]) / 100.0, int(m[2]), int(m[3])
    gt = np.array([d_gt * math.sin(math.radians(a_gt)), 0.0, d_gt * math.cos(math.radians(a_gt))])
    for f in sorted(glob.glob(os.path.join(folder, "*.jpg"))):
        gray = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        corners, ids, _ = det.detectMarkers(gray)
        found = {} if ids is None else {int(i): c.reshape(4, 2) for i, c in zip(ids.flatten(), corners)}
        row = {"file": f, "d_cm": int(m[1]), "a_deg": a_gt, "rep": rep,
               "tag0": int(0 in found), "tag1": int(1 in found)}
        if 0 in found:
            ok, rv, tv = cv2.solvePnP(SQ, found[0], K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                p = cam_pos(rv, tv) + OFFSET[0]
                row["one_d"], row["one_a"], row["one_err"] = metrics(p, gt)
        if 0 in found and 1 in found:
            obj = np.vstack([SQ + OFFSET[0], SQ + OFFSET[1]])
            img = np.vstack([found[0], found[1]])
            ok, rv, tv = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE)
            if ok:
                rv, tv = cv2.solvePnPRefineLM(obj, img, K, dist, rv, tv)
                p = cam_pos(rv, tv)
                row["two_d"], row["two_a"], row["two_err"] = metrics(p, gt)
        rows.append(row)

if not rows:
    raise SystemExit(f"no frames under {args.poses}/d<cm>_a<deg>_r<k>/")
keys = ["file", "d_cm", "a_deg", "rep", "tag0", "tag1", "one_d", "one_a", "one_err", "two_d", "two_a", "two_err"]
with open("pose_frames.csv", "w", newline="") as fh:
    w = csv.DictWriter(fh, keys)
    w.writeheader()
    w.writerows(rows)

groups = defaultdict(list)
for r in rows:
    groups[(r["d_cm"], r["a_deg"])].append(r)


def stat(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return "", ""
    return round(float(np.mean(vals)), 3), round(float(np.std(vals)), 3)


summary = []
print(f"{'pos':>10} {'n':>3} {'det0':>5} {'det2':>5} | {'one: dist err cm':>17} {'ang err':>8} {'pos err cm':>11} |"
      f" {'two: dist err cm':>17} {'ang err':>8} {'pos err cm':>11}")
for (d, a), rs in sorted(groups.items()):
    out = {"d_cm": d, "a_deg": a, "frames": len(rs),
           "det_tag0": round(sum(r["tag0"] for r in rs) / len(rs), 3),
           "det_both": round(sum(r["tag0"] and r["tag1"] for r in rs) / len(rs), 3)}
    for k in ("one", "two"):
        de = [(r[f"{k}_d"] - d / 100) * 100 for r in rs if f"{k}_d" in r]
        ae = [r[f"{k}_a"] - a for r in rs if f"{k}_a" in r]
        pe = [r[f"{k}_err"] * 100 for r in rs if f"{k}_err" in r]
        out[f"{k}_dist_err_cm_mean"], out[f"{k}_dist_err_cm_sd"] = stat(de)
        out[f"{k}_ang_err_deg_mean"], out[f"{k}_ang_err_deg_sd"] = stat(ae)
        out[f"{k}_pos_err_cm_mean"], out[f"{k}_pos_err_cm_sd"] = stat(pe)
    summary.append(out)
    f = lambda k, q: f"{out[f'{k}_{q}_mean']}±{out[f'{k}_{q}_sd']}" if out[f"{k}_{q}_mean"] != "" else "-"
    print(f"{f'{d}cm/{a}deg':>10} {len(rs):>3} {out['det_tag0']:>5} {out['det_both']:>5} |"
          f" {f('one','dist_err_cm'):>17} {f('one','ang_err_deg'):>8} {f('one','pos_err_cm'):>11} |"
          f" {f('two','dist_err_cm'):>17} {f('two','ang_err_deg'):>8} {f('two','pos_err_cm'):>11}")
with open("pose_summary.csv", "w", newline="") as fh:
    w = csv.DictWriter(fh, list(summary[0].keys()))
    w.writeheader()
    w.writerows(summary)
print("wrote pose_frames.csv and pose_summary.csv")
