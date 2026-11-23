# Seeing Together: Smart Glasses-Augmented Spatial Memory for Human-Robot Collaboration

Khang Nguyen and Hyun Kyum Kim

UCLA ECE M202A, Fall 2026 (Project 1). Shared, time-stamped object memory for human-robot object search with RayNeo X3 Pro smart glasses and a Unitree Go2.

**Project page:** [hyunkyumkim.github.io/seeing-together](https://hyunkyumkim.github.io/seeing-together/) (`index.html` in this repo)

**Checkpoint slides:** [P01_Checkpoint.pdf](P01_Checkpoint.pdf) (November 10, 2026)

## Overview

Seeing Together explores whether a robot can find everyday objects more efficiently by combining observations from human smart glasses and a robot in a shared, temporally aware memory. Instead of relying only on language instructions or a long, unstructured multimodal history, the robot maintains object-level evidence and decides what information to acquire next.

For example, a person may have seen a screwdriver on a workbench while the robot was elsewhere. Later, the robot might find that workbench empty. The system should preserve both observations, recognize that the earlier evidence may be stale, and decide whether to inspect another location or ask the person a targeted question.

## Research Question

Can a robot improve object-search success and efficiency by maintaining shared, time-aware object-centric memory from human smart glasses and robot observations, and actively deciding when to inspect, ask, or report?

## Final scope

**Setting.** One indoor lab, the RayNeo X3 Pro glasses, a laptop hub, and the Go2 with ROS2. AprilTags at known positions in the Go2 map are the shared anchor between the glasses and the robot.

**In scope**
- A small predefined set of everyday objects, chosen for reliable detection and localization.
- The robot first explores the lab, builds its map and an object-location memory, and initializes the shared memory.
- The glasses stream camera frames and IMU to the laptop; tags seen by the glasses place the wearer, and observed object moves update the object's location and timestamp in the shared memory, before the user asks.
- An agent reads the shared memory and sends navigation goals to the Go2.

**Out of scope**
- Open-vocabulary detection of arbitrary objects.
- Localization without tags (no visual SLAM on the glasses).
- Multiple rooms, multiple users, or outdoor use.

## Minimum deliverable

The Go2 reaches an object that the glasses saw being moved while it was out of the robot's view.

## Evaluation plan

**Task.** An object from the predefined set is moved in the lab while the robot is not looking; the robot is then asked to find it.

| Condition | What the robot uses |
|---|---|
| Robot-Only (baseline) | Only its own last observation of the object |
| Naive Human+Robot | Glasses observations without timestamps or staleness handling |
| Seeing Together | Shared, time-stamped memory from glasses and robot |

- **Primary metric:** task success rate. **Secondary:** search time, unnecessary actions, clarification turns, final position error.
- **Trials:** at least 10 trials per condition, with the same object placements across conditions.
- **Stress tests:** the object is moved out of the robot's view; Wi-Fi is dropped in the middle of a task.
- **Component test (tag pose, home test room):** after camera calibration, estimate the glasses' pose from tags at 0.5, 1, 2 and 3 m and 0, 30 and 60 degrees, and compare with tape-measured positions; compare one tag against two tags seen together.

## Plan

| Task | Owner | Target |
|---|---|---|
| Camera calibration; tag pose error vs tape-measured positions (one vs two tags) | Hyun Kyum | Done (Nov 18) |
| Live glasses tag and IMU observations into the shared memory | Hyun Kyum | Nov 24 |
| Resolve the head-on tilt ambiguity with the IMU gravity direction | Hyun Kyum | Nov 24 |
| Clearer map and memory construction; LLM-controlled navigation | Khang | Done (Nov 24) |
| End-to-end integration; Robot-Only and Naive comparison runs | Both | Dec 4 |
| Final presentation and report | Both | Dec 5-11 |

## Remaining risks

| Risk | Fallback |
|---|---|
| Tag pose error is too large to place observations in the map. Measured: small up to 1 m, but head-on views at 2-3 m flip the estimated tilt | Use two or more tags per area and prefer oblique views; resolve the tilt ambiguity with the IMU's gravity direction; otherwise snap each observation to the nearest saved place in the lab map |
| Live tag detection is unreliable at distance while worn (15 cm tags found in 18-93% of frames at 2 m in the Nov 2 live test; 95-100% up to 3 m with the glasses static) | Larger tags, more tags per area, or closer viewing distances |
| Robot side is only tested offline on a saved map so far | Run the comparison with the robot driven to saved places |
| Wi-Fi drops during a task | Buffer observations and resume; the IMU stream already recovers after a drop |

## Response to checkpoint feedback

- **Which objects are recorded before the user asks? (P14 question)** We narrowed the scope to a predefined object set; the robot builds the initial memory, and the glasses update tracked objects when they move.
- **Compare one tag with two tags (P14 suggestion).** Added to the tag pose test. Two tags reduced the position error in the hardest cases, for example from 120 cm to 6 cm at 2 m / 30 deg and from 17 cm to 5.5 cm at 1 m head-on, so we will place at least two tags per area.
- **Can we validate in a home test room first? (our question to the instructor)** The instructor approved it, so the tag pose test runs first in a home test room with tape-measured tags, and the lab is used for the end-to-end comparison.

## Repository layout

| Path | Contents |
|---|---|
| `imu-test/` | Glasses IMU (accelerometer + gyroscope) streaming over Wi-Fi: Kotlin app, laptop receiver, 20 s logs |
| `camera-test/` | Glasses camera access and AprilTag detection: Kotlin capture activity, tag scripts, results |
| `stream-test/` | Glasses camera streaming (JPEG over TCP) and latency test with an AprilTag clock: Kotlin app, receiver, results |
| `pose-test/` | Camera calibration and tag pose accuracy test (one vs two tags) with tape-measured ground truth |
| `memory/` | Shared-memory schema, store and tests (prototype; tag observations from the glasses load, robot observations are mocked) |
| `robot_stack/` | Go2 stack: driver, dashboard, mapper, navigation and agent (Khang) |
| `evidence_milestone2/` | Feasibility-check evidence PDFs (November 2) |
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
| Camera frame streaming latency (Wi-Fi / USB) | Done | 640x480 over Wi-Fi: 29.8 fps, 0.8% dropped, median/p95 latency 106/133 ms; larger frames are limited by JPEG encoding on the glasses |
| Shared-memory schema on mock observations | Prototype | Tag observations from the glasses load; robot observations are mocked |
| Camera calibration and tag pose accuracy (`pose-test/`) | Done | 640x480 calibration RMS 0.27 px. Home test room, 15 cm tags, 9 positions x 3 placements x 20 frames: distance error under 6 cm up to 3 m; position error 2-6 cm up to 1 m with two tags; head-on views at 2-3 m give 23-108 cm position error (tilt ambiguity) |

## License