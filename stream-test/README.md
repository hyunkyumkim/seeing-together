# Camera frame streaming test

Checks the Milestone 1 key assumption: frames from the RayNeo X3 Pro camera reach the laptop
with measurable latency. Wireless (Wi-Fi) first; if it is too slow, the same app runs over USB.

- `FrameStreamActivity.kt` (glasses): Camera2 YUV frames -> JPEG -> TCP. Every camera frame gets a
  sequence number; frames dropped on the glasses (encoder or network busy) show up as seq gaps.
- `stream_receiver.py` (laptop): receives frames, shows them, and writes per-frame logs, latency
  samples and a summary.

Status: built and run on the glasses; all six runs below are done (results in `results/`).

## Setup (once)

1. Copy `FrameStreamActivity.kt` into the ImuStream Android project
   (`app/src/main/java/com/example/imustream/`).
2. Add to `AndroidManifest.xml`, inside `<application>`:
   ```xml
   <activity android:name=".FrameStreamActivity" android:exported="true" />
   ```
   (CAMERA and INTERNET permissions are already declared.)
3. Build, install, and grant the camera permission:
   ```powershell
   $env:ADB_LIBUSB="1"
   adb install -r "C:\Users\admin\AndroidStudioProjects\ImuStream\app\build\outputs\apk\debug\app-debug.apk"
   adb shell pm grant com.example.imustream android.permission.CAMERA
   ```
4. Wireless only: allow inbound TCP 5006 (PowerShell as administrator):
   ```powershell
   New-NetFirewallRule -DisplayName "Frame stream TCP 5006" -Direction Inbound -Protocol TCP -LocalPort 5006 -Action Allow
   ```
5. Laptop: `pip install opencv-contrib-python numpy` (the camera-test venv already has them).

## Run

Order: stop the app, start the receiver, then start the app (`adb shell am force-stop com.example.imustream`
first; if the app is already running, `am start` only prints "Activity not started" and the old
settings stay). Keep the glasses screen on: the camera is blocked when the app is in the background.
The receiver rotates the view upright by default (`--rotate cw|ccw|180|none`; the sensor is mounted
rotated 90 deg). Keep the "stream" window out of the glasses' view.

```powershell
# laptop
python stream_receiver.py --port 5006 --duration 60 --out wifi_640

# glasses, wireless (use the laptop's IPv4 address from ipconfig)
adb shell input keyevent KEYCODE_WAKEUP
adb shell am start -n com.example.imustream/.FrameStreamActivity --es ip 192.168.1.209 --ei port 5006 --ei w 640 --ei h 480 --ei q 80
```

USB instead of Wi-Fi (same app, traffic goes through the cable):

```powershell
adb reverse tcp:5006 tcp:5006
adb shell am start -n com.example.imustream/.FrameStreamActivity --es ip 127.0.0.1 --ei port 5006 --ei w 640 --ei h 480 --ei q 80
```

Stop the app: `adb shell am force-stop com.example.imustream`. Glasses log: `adb logcat -s STREAMTEST`.

App options (intent extras): `ip`, `port` (5006), `w`/`h` (640x480; nearest supported size is used),
`q` JPEG quality (80), `fps` camera frame rate (30), `id` camera ID ("0").

## Measuring latency (tag clock)

The receiver opens a "clock" window that shows an AprilTag whose ID changes every 50 ms
(`--step-ms`). Point the glasses at that window. When a received frame contains a clock tag, the
receiver logs latency = arrival time - time that tag was first drawn. This covers camera exposure,
JPEG encoding, transfer and receive. It also includes the monitor's own display delay (so it is a
slight overestimate) and has a resolution of about one step.

Suggested runs, about 60 s each, glasses pointed at the clock window:

| Run | Link | Size | `--out` |
|---|---|---|---|
| 1 | Wi-Fi | 640x480 | `wifi_640` |
| 2 | Wi-Fi | 1280x720 | `wifi_720` |
| 3 | Wi-Fi | 1920x1080 | `wifi_1080` |
| 4 | USB | 640x480 | `usb_640` |
| 5 | USB | 1280x720 | `usb_720` |
| 6 | USB | 1920x1080 | `usb_1080` |

Optional: one run while walking (motion blur), and one with printed or on-screen tags 0-2 in view.

## Outputs

- `<out>_frames.csv`: seq, capture timestamp, encode time, on-glasses capture->JPEG time, size, PC arrival time
- `<out>_latency.csv`: one row per received frame that contained a clock tag
- `<out>_summary.txt`: delivered fps, camera fps, frames not delivered (%), Mbit/s, arrival gaps,
  on-glasses capture->JPEG time, end-to-end latency (median / p95 / min / max)

The receiver ignores clock tags that stay in view for more than 0.5 s (static ghosts) and latencies
above 3 s. If a run has several connections (seq restarts at 0) the summary uses the longest one.

Raw camera frames are not saved, so no images of the room end up in the repository.

## Results (40 s per run, glasses pointed at the clock window)

| Run | Link | Size | Delivered fps | Not delivered | Latency median / p95 | n |
|---|---|---|---|---|---|---|
| wifi_640 | Wi-Fi | 640x480 | 29.8 | 0.8 % | 106 / 133 ms | 590 |
| wifi_640_b (repeat) | Wi-Fi | 640x480 | 29.8 | 0.7 % | 100 / 122 ms | 563 |
| wifi_720 | Wi-Fi | 1280x720 | 18.6 | 38.0 % | 135 / 168 ms | 403 |
| wifi_720_q50 (q 50) | Wi-Fi | 1280x720 | 19.6 | 34.7 % | 129 / 161 ms | 419 |
| wifi_1080 | Wi-Fi | 1920x1080 | 9.5 | 68.3 % | 206 / 244 ms | 178 |
| usb_640 | USB | 640x480 | 29.5 | 1.5 % | 96 / 118 ms | 540 |
| usb_720 | USB | 1280x720 | 18.5 | 38.4 % | 127 / 159 ms | 382 |
| usb_1080 | USB | 1920x1080 | 9.7 | 67.6 % | 183 / 212 ms | 127 |

- At 640x480 Wi-Fi and USB are equivalent (about 30 fps, about 100 ms end to end), so wireless is enough.
- At the same size Wi-Fi and USB give the same fps; the limit at 720p and above is YUV->NV21 conversion
  plus JPEG encoding on the glasses (conv 4-6 ms / 13 ms / 20-36 ms and encode 4-6 / 18 / 60-69 ms at
  480p / 720p / 1080p), not the network.
- Latency includes the monitor's own display delay (slight overestimate) and has a resolution of about one
  step (50 ms). Only 640x480 Wi-Fi was repeated. Frame count and Mbit/s depend on the scene.
- An earlier build converted chroma pixel by pixel and delivered about 14 fps at 640x480; the numbers
  above are from the fixed build (row-wise copy).
- `*_frames.csv` / `*_latency.csv` hold only timestamps and sizes, no images.
