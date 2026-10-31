# Glasses camera AprilTag test (2026-10-30)

- Device: RayNeo X3 Pro, camera id 0 (back-facing), JPEG 1920x1080 still capture via Camera2 (`CameraTestActivity.kt`)
- Camera 1 (also back-facing) offers only up to 640x480 JPEG; not used in this test
- Tag: AprilTag tag36h11, ID 0, shown full screen on a PC monitor; black square side approx. 20 cm (estimated, not measured precisely)
- Distances: 0.5, 1, 2, 3 m (estimated by eye, not measured with a tape)
- Angles at 1 m: 0 deg (head-on), approx. 30 and 60 deg; the tester wore the glasses and moved to the right of the screen's front axis (angles are rough estimates)
- 3 stills per condition, about 1 s apart; glasses worn (not on a fixed mount), so slight hand/head motion is possible
- Detector: OpenCV 5.0.0 aruco, DICT_APRILTAG_36h11, default parameters (`tag_test.py`)
- Result: tag 0 detected in 18/18 images, no false IDs (`detection_results.txt`)
- Photos are not published (they show the tester's screen); raw files are kept locally

## Limitations
- Tag shown on a screen (brightness/glare dependent), not a printed tag
- Still images only; no real-time or moving-camera test
- Detection only; pose/position accuracy not measured
- Single tag (ID 0); no occlusion or multi-tag test
- Frames within a condition are near-duplicates, not independent samples
- Distances/angles are approximate; the failure limit (farther than 3 m, beyond 60 deg) was not found
- Initial attempt failed with CAMERA_DISABLED when the screen was off (app counted as background); it worked once the screen was on and the app was in the foreground
