# Tag pose accuracy test (home test room)

Measures how accurately the glasses' position is estimated from AprilTags, against tape-measured positions,
and compares one tag with two tags (P14 suggestion). Uses the same 640x480 stream as the live system.

## Setup
- Print `checkerboard_9x6_25mm_letter.pdf` and two pages of `apriltag36h11_id0-3_150mm_letter.pdf` (ID 0, ID 1)
  at 100%. Check the 100 mm scale line. Measure the checkerboard square and the tags' black-square side.
- Tape tag 0 (left) and tag 1 (right) flat on a wall, upright, centers 20 cm (7 7/8 in) apart, same height.
  Mark the midpoint between the two tag centers: this is the reference point.
- Put the glasses on a box so the camera lens is at the height of the reference point. Wear nothing.
- Even indoor light, no glare on the paper, no direct sun.

## Positions (12 x 3 repeats x 20 frames)
Distance d from the reference point to the lens, angle a to the right of the wall's normal (viewer's right).
On the floor: forward = d cos a (from the wall), sideways = d sin a (along the wall, to the right).
Distances 50, 100, 200, 300 cm; angles 0, 30, 60 deg. Re-place the glasses for each repeat.

## Run
```powershell
# 1. calibration: receiver first, then the stream app (640x480)
python grab.py calib
adb shell am start -n com.example.imustream/.FrameStreamActivity --es ip <laptop IP> --ei port 5006 --ei w 640 --ei h 480 --ei q 80
#    hold the checkerboard still, press c; about 20 frames, near/far, tilted, in all image areas; q to quit
python calibrate.py --square-mm <measured>          # RMS below ~0.5 px is good

# 2. pose frames: put the glasses on the mark shown, press g (20 frames), next mark ...
python grab.py pose

# 3. analysis
python tag_pose.py --tag-mm <measured> --spacing-cm <measured>
```
Outputs: `camera.json`, `pose_frames.csv` (every frame), `pose_summary.csv` (per position: detection rate,
distance / angle / position error for one tag and two tags, mean and SD).

Note: with nearly head-on views at 2-3 m, a flat target can give two similar poses (tilted up or down), so
the position error can be much larger than the distance error. Report both.
