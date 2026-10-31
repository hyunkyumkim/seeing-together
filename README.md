# Seeing Together: Context-Aware Human-Robot Collaboration with Smart Glasses

Khang Nguyen and Hyun Kyum Kim 


UCLA ECE M202A, Fall 2026 - Seeing Together: shared evidence memory for human-robot object search with smart glasses and a Unitree Go2.

## Overview

Seeing Together explores whether a robot can find everyday objects more efficiently by combining observations from human smart glasses and a robot in a shared, temporally aware memory. Instead of relying only on language instructions or a long, unstructured multimodal history, the robot maintains object-level evidence and decides what information to acquire next.

For example, a person may have seen a screwdriver on a workbench while the robot was elsewhere. Later, the robot might find that workbench empty. The system should preserve both observations, recognize that the earlier evidence may be stale, and decide whether to inspect another location or ask the person a targeted question.

## Research Question

Can a robot improve object-search success and efficiency by maintaining shared, time-aware object-centric memory from human smart glasses and robot observations, and actively deciding when to inspect, ask, or report?

## Repository layout

| Path | Contents |
|---|---|
| `imu-test/` | Glasses IMU (accelerometer + gyroscope) streaming over Wi-Fi: Kotlin app, laptop receiver, 20 s logs |
| `camera-test/` | Glasses camera access and AprilTag detection: Kotlin capture activity, tag scripts, results |
| `map/` | Go2 lab occupancy grid (`lab.grid.npz`, 5 cm cells) and `map_vis.py` to draw it |
| `android/` | `AndroidManifest.xml` of the glasses app (Android Studio project `ImuStream`, package `com.example.imustream`) |
| `requirements.txt` | Python packages for the laptop scripts |

All Kotlin files are activities of one Android app; copy them into the `ImuStream` project and declare
them in its manifest. The glasses are driven over adb (on Windows, set `$env:ADB_LIBUSB="1"` first).

## Feasibility tests

| Test | Status | Result so far |
|---|---|---|
| IMU streaming over Wi-Fi (`imu-test/`) | Done | 20 s runs: 0% loss, ~109.5 Hz; app survives a Wi-Fi drop and resumes |
| Camera access + AprilTag detection (`camera-test/`) | Done | Tag detected in 18/18 stills at 0.5-3 m and ~30/60 deg (tag on a screen) |
| Camera frame streaming latency (Wi-Fi / USB) | Next | - |
| Shared-memory schema on mock observations | Next | - |

## License