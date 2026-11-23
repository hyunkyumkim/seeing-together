# Go2 stack

Everything needed to run the dog from this folder: the web console, the navigation agent, the lidar driver source, and the current lab map. Links point at the copies in `/home/khang/m202a`, so there is one map and one set of programs.

The dog is `172.17.39.82`. Only one program may connect. Close the Unitree phone app, and stop the other driver if it is up: `docker stop go2_test`.

## 1. Web console

```bash
cd /home/khang/m202a/robot_stack
./start_console.sh
```

That starts the driver container `go2_khang` (image `khang/go2:dev`) and the dashboard at http://localhost:8080. Tick **Enable driving** on the page before the keyboard or the agent can move the dog. Take over or STOP on that page always wins.

The page is `go2_dashboard.py`. Routes and the map pose come from `go2_mapper.py`, `go2_nav.py`, and `go2_agent.py`. `go2_goto.sh sofa` sends the dog to a place without the language model.

## 2. Ask Qwen or ask GPT

Leave the console running. Both look at the map and the camera photos, then call `go_to` one step at a time. The route around furniture is still the planner in `go2_nav.py`; the model only chooses the goal. With no sentence, either script waits for tasks. `/reset` clears the conversation, `/quit` exits.

Local Qwen on GPU 1:

```bash
cd /home/khang/m202a/robot_stack
./ask_qwen.sh "find the red cup"
```

The first time on this machine, install that model once with `./qwen_agent/setup_env.sh`.

OpenAI gpt-4o:

```bash
OPENAI_API_KEY=sk-... ./ask_gpt.sh "find the red cup"
```

Another vision model is `QWEN_MODEL=gpt-4.1 ./ask_gpt.sh "..."`. Either way the agent only talks to the dog through http://localhost:8080.

## 3. L515 depth camera

The RealSense L515 on the Raspberry Pi is a second camera: color, depth, and a point cloud, on top of the dog's own lidar and front camera. It does not replace either. The sender stays on the Pi (`/home/nesl/oliver/l515_go2_pi4_bridge`, `bash scripts/run_pi.sh`). This folder runs the receiver inside `go2_khang`, so the topics sit next to `/odom` and `/point_cloud2`.

Leave the console running, then:

```bash
cd /home/khang/m202a/robot_stack
./start_l515.sh
```

The page is http://localhost:8081: color, depth, a light point-cloud preview, and a bag recorder into `go2_recordings/`. Topics are `/go2/l515/...` in `ROS_DOMAIN_ID=78`. `./start_l515.sh --status`, `--logs`, and `--stop` check or stop it. A driver restart (`./start_console.sh --reset`) stops the receiver; run `./start_l515.sh` again afterwards.

The mount is still unmeasured, so the cloud sits at the origin of `base_link` until you set it, for example `L515_MOUNT_XYZ="0.28 0 0.12" L515_MOUNT_RPY_DEG="0 10 0" ./start_l515.sh`. The Pi login defaults to `khang@172.17.29.72`; if that address changed, `L515_PI=khang@<ip> ./start_l515.sh`.

## The map

The robot reads **`go2_maps/lab.grid.npz`**. That file is the lab map saved 21 Nov 2026 (about 100 m²). Named spots are **`go2_maps/lab.places.json`** (17 places). Photos for those spots are in `go2_maps/lab_places/`.

After a reboot the dog's odometry starts over. On the console, line it up with the map before a `go_to`.

## The controller

`controller/go2_robot_sdk` is the WebRTC driver: it turns the dog's lidar, lidar pose, camera, IMU, feet, and joints into ROS topics. `controller/go2_interfaces` is the messages that driver publishes. `controller/docker` is the image recipe. The running container uses the already built image `khang/go2:dev`, which has this driver installed. Speech, simulation, and the old SLAM maps are not in this folder.

## Left out on purpose

Recordings, old map backups, the 14 Oct SLAM pose graph, trail logs, and the phone-app experiments under `Go2Controller` (speech, people, simulation). Those are not needed to start the console or the agent.
