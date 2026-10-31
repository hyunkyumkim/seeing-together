# Camera access and AprilTag detection test

Checks that our own app can open the RayNeo X3 Pro camera and that AprilTags are detectable in its
images (for aligning the wearer's observations with predefined locations).

- `CameraTestActivity.kt` (glasses): lists the cameras, opens one with Camera2, takes 3 JPEG stills
  (1920x1080), saves them in the app's folder, and logs under the tag `CAMTEST`.
- `shoot.ps1` (laptop): runs one capture and copies the 3 stills to `tagphotos\<label>\`.
- `make_tags.py`: generates tag36h11 images `tag_0.png` to `tag_2.png`.
- `tag_test.py`: prints the tag IDs found in each photo.
- `detection_results.txt`: output of `tag_test.py` for the run below.

## Camera facts (from `dumpsys media.camera` and the CAMTEST log)

- 2 cameras, both back-facing (looking where the wearer looks).
- Camera 0: JPEG up to 4032x3024; fixed focus (no autofocus modes); used for this test.
- Camera 1: JPEG only up to 640x480.
- Stills are saved in sensor orientation (rotated 90 degrees); tag detection is not affected.
- The camera is blocked (`CAMERA_DISABLED ... from background`) when the screen is off or the app
  is not in the foreground. Wake the screen before starting the activity.

## How to run

1. Add `CameraTestActivity.kt` to the ImuStream Android project and declare it with the CAMERA
   permission (see `../android/AndroidManifest.xml`). Build and install.
2. Grant the permission once:
   ```powershell
   $env:ADB_LIBUSB="1"
   adb shell pm grant com.example.imustream android.permission.CAMERA
   ```
3. Laptop environment (from this folder):
   ```powershell
   python -m venv .venv
   .\.venv\Scripts\Activate.ps1        # if blocked: Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
   pip install -r ..\requirements.txt
   python make_tags.py
   ```
4. Show `tag_0.png` full screen (or print it), place the glasses at each distance/angle, and run:
   ```powershell
   .\shoot.ps1 d100_a0
   ```
   Check the log if a capture fails: `adb logcat -d -s CAMTEST`
5. Detect tags in all photos:
   ```powershell
   python tag_test.py "tagphotos\*\*.jpg"
   ```
Stop the app: `adb shell am force-stop com.example.imustream`

## Test run (2026-10-30)

- Device: RayNeo X3 Pro, camera id 0 (back-facing), JPEG 1920x1080 still capture via Camera2 (`CameraTestActivity.kt`)
- Tag: AprilTag tag36h11, ID 0, shown full screen on a PC monitor; black square side approx. 20 cm (estimated, not measured precisely)
- Distances: 0.5, 1, 2, 3 m (estimated by eye, not measured with a tape)
- Angles at 1 m: 0 deg (head-on), approx. 30 and 60 deg; the tester wore the glasses and moved to the right of the screen's front axis (angles are rough estimates)
- 3 stills per condition, about 1 s apart; glasses worn (not on a fixed mount), so slight hand/head motion is possible
- Detector: OpenCV 5.0.0 aruco, DICT_APRILTAG_36h11, default parameters (`tag_test.py`)
- Result: tag 0 detected in 18/18 images, no false IDs (`detection_results.txt`)
- Photos are not published (they show the tester's screen); raw files are kept locally

| Distance | Angle | Detected |
|---|---|---|
| 0.5 m | head-on | 3 / 3 |
| 1 m | head-on | 3 / 3 |
| 2 m | head-on | 3 / 3 |
| 3 m | head-on | 3 / 3 |
| 1 m | ~30 deg | 3 / 3 |
| 1 m | ~60 deg | 3 / 3 |

## Limitations
- Tag shown on a screen (brightness/glare dependent), not a printed tag
- Still images only; no real-time or moving-camera test
- Detection only; pose/position accuracy not measured
- Single tag (ID 0); no occlusion or multi-tag test
- Frames within a condition are near-duplicates, not independent samples
- Distances/angles are approximate; the failure limit (farther than 3 m, beyond 60 deg) was not found
- Initial attempt failed with CAMERA_DISABLED when the screen was off (app counted as background); it worked once the screen was on and the app was in the foreground
