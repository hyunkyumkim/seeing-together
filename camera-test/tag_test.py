import cv2, sys, glob
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
for f in sorted(glob.glob(sys.argv[1])):
    img = cv2.imread(f)
    corners, ids, _ = det.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
    print(f, None if ids is None else ids.flatten().tolist())
