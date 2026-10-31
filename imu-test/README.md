# IMU streaming test

Checks that our own app can read the RayNeo X3 Pro accelerometer and gyroscope and stream them over
Wi-Fi to a laptop at a usable rate.

- `MainActivity.kt` (glasses): registers the accelerometer and gyroscope and sends text lines
  `tag,seq,device_ts_ns,x,y,z` over UDP (A = accelerometer, G = gyroscope; up to 8 lines per packet).
  `seq` counts per sensor, so gaps mean lost samples. A failed send is counted (`sendErr` on the
  glasses screen) instead of crashing the app.
- `imu_receiver.py` (laptop): logs the samples to CSV and prints samples, loss, generated and
  delivered rate, device timestamp spacing, and PC arrival gaps.
- `test.csv`, `test2.csv`: 20 s runs (columns: tag, seq, device_ts_ns, pc_time_s, x, y, z).

Sensors reported by `adb shell dumpsys sensorservice`: lsm6dsr accelerometer and gyroscope (maximum
415.97 Hz).

## How to run

1. Add `MainActivity.kt` to the ImuStream Android project (INTERNET permission, see
   `../android/AndroidManifest.xml`). Build and install:
   ```powershell
   $env:ADB_LIBUSB="1"      # Windows: without this, adb did not list the glasses
   adb kill-server
   adb devices
   adb install -r "C:\Users\admin\AndroidStudioProjects\ImuStream\app\build\outputs\apk\debug\app-debug.apk"
   ```
2. Allow inbound UDP 5005 on the laptop (PowerShell as administrator, once):
   ```powershell
   New-NetFirewallRule -DisplayName "IMU UDP 5005" -Direction Inbound -Protocol UDP -LocalPort 5005 -Action Allow
   ```
3. Start the receiver, then the app (glasses and laptop on the same Wi-Fi; laptop IPv4 from `ipconfig`):
   ```powershell
   python imu_receiver.py --port 5005 --duration 20 --out test.csv
   adb shell am start -n com.example.imustream/.MainActivity --es ip 192.168.1.209 --ei port 5005 --ei us 10000
   ```
   `us` is the requested sampling period in microseconds (10000 = 100 Hz).
4. Stop the app: `adb shell am force-stop com.example.imustream`

## Results (home Wi-Fi, glasses worn, 100 Hz requested)

| Run | App version | Samples (accel / gyro) | Lost | Generated rate | Timestamp spacing |
|---|---|---|---|---|---|
| `test.csv`, 20 s | before the send-error fix | 2,193 / 2,193 | 0 (0.0%) | 109.5 Hz | 9.13 ms (mean = max) |
| `test2.csv`, 20 s | current `MainActivity.kt` | 2,198 / 2,199 | 0 (0.0%) | 109.5 Hz | 9.13 ms |

`test.csv` was recorded with the earlier app, which crashed when Wi-Fi was down; the only change
since then is the try/catch around the UDP send and the `sendErr` counter.

Wi-Fi drop test (90 s run, current app; raw CSV not included): the glasses' Wi-Fi was turned off
about 19 s into the run for ~17 s (estimated) with `adb shell svc wifi disable`, then turned back on.
- The app stayed alive (same process ID before and after) and resumed streaming by itself.
- Data gap: 39.2 s (19.4 s to 58.6 s), which includes the Wi-Fi reconnect time.
- 4,292 of 9,866 accelerometer samples (43.5%) were lost, all inside the gap; none outside it.
  There is no buffering, so samples during an outage are dropped.
- The glasses screen showed `sendErr=4703` (failed UDP send calls, each up to 8 lines) and `queueFull=0`.

## Limitations
- Short runs on home Wi-Fi only; no 5-minute run yet.
- Only 100 Hz was requested (the sensor delivers ~109.5 Hz); 50 and 200 Hz not tested.
- Running without USB needs the laptop IP as the app's default (it is passed by adb today).
