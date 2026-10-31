"""Detect AprilTags (tag36h11) in photos and print the IDs found in each one.

Usage:  python tag_test.py "tagphotos\*\*.jpg"      (quote the pattern)
Output: one line per image: <path> [ids]  or  <path> None
"""
import cv2, sys, glob
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
files = sorted(glob.glob(sys.argv[1]))
if not files:
    sys.exit(f"no files match {sys.argv[1]!r}")
for f in files:
    img = cv2.imread(f)
    corners, ids, _ = det.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
    print(f, None if ids is None else ids.flatten().tolist())
