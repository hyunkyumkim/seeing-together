# Take 3 stills with the glasses camera (CameraTestActivity) and copy them to tagphotos\<label>\.
# Usage (from camera-test, glasses connected over USB, glasses held still for ~8 s):
#   .\shoot.ps1 d100_a0            # camera 0
#   .\shoot.ps1 d100_a0 -CamId 1   # camera 1
# If scripts are blocked: Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
param(
    [Parameter(Mandatory = $true)][string]$Label,
    [string]$CamId = "0"
)
$env:ADB_LIBUSB = "1"
$remote = "/sdcard/Android/data/com.example.imustream/files"
adb shell "rm -f $remote/*.jpg"
adb shell input keyevent KEYCODE_WAKEUP          # the camera is blocked while the screen is off
adb shell am force-stop com.example.imustream
adb shell am start -n com.example.imustream/.CameraTestActivity --es id $CamId | Out-Null
Start-Sleep -Seconds 8
New-Item -ItemType Directory -Force -Path "tagphotos\$Label" | Out-Null
adb pull "$remote/." "tagphotos\$Label"
