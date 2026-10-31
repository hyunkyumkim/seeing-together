"""Generate printable AprilTag images (tag36h11, IDs 0-2) with a white border.

Usage:  python make_tags.py      -> tag_0.png, tag_1.png, tag_2.png (600 px tag + 150 px border)
Show one full screen or print it at 100% scale, then measure the black square's side length.
"""
import cv2, numpy as np
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
for i in range(3):
    m = cv2.aruco.generateImageMarker(d, i, 600)
    cv2.imwrite(f"tag_{i}.png", np.pad(m, 150, constant_values=255))
