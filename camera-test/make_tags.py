import cv2, numpy as np
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
for i in range(3):
    m = cv2.aruco.generateImageMarker(d, i, 600)
    cv2.imwrite(f"tag_{i}.png", np.pad(m, 150, constant_values=255))
