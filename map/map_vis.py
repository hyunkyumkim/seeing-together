import math
import numpy as np
import cv2

RES, OCC = 0.05, 2
g = np.load("lab.grid.npz")
score, seen, origin = g["score"], g["seen"], g["origin"]  # origin = [-25, -25]

img = np.full(score.shape, 205, np.uint8)   # grey = unseen
img[seen] = 254                             # white = free
img[score >= OCC] = 0                       # black = occupied
img = np.ascontiguousarray(img[::-1])       # row 0 at the top, +y up; OpenCV rejects the flipped view
img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)    # a color arrow needs a 3-channel image

def project(x, y, yaw):
    """Map-frame meters -> pixel column, row-from-top, and a screen angle (y down)."""
    col = (x - origin[0]) / RES
    row = img.shape[0] - (y - origin[1]) / RES
    return col, row, -yaw

# example: a pose already in the map frame
col, row, ang = project(-0.44, 2.33, math.radians(136))
tip = (int(col + 40 * math.cos(ang)), int(row + 40 * math.sin(ang)))
cv2.arrowedLine(img, (int(col), int(row)), tip, (0, 0, 255), 2, tipLength=0.3)
cv2.imwrite("lab_map.png", img)