# Go2 stack

Everything needed to run the dog from this folder: the web console, the Qwen agent, the lidar driver source, and the current lab map. Links point at the copies in `/home/khang/m202a`, so there is one map and one set of programs.

The dog is `172.17.39.82`. Only one program may connect. Close the Unitree phone app, and stop the other driver if it is up: `docker stop go2_test`.

## 1. Web console

```bash
cd /home/khang/m202a/robot_stack
./start_console.sh
```

That starts the driver container `go2_khang` (image `khang/go2:dev`) and the dashboard at http://localhost:8080. Tick **Enable driving** on the page before the keyboard or the agent can move the dog. Take over or STOP on that page always wins.

The page is `go2_dashboard.py`. Routes and the map pose come from `go2_mapper.py`, `go2_nav.py`, and `go2_agent.py`. `go2_goto.sh sofa` sends the dog to a place without the language model.

## 2. Qwen or OpenAI

Leave the console running. In another terminal:

```bash
cd /home/khang/m202a/robot_stack
./ask_qwen.sh "find the red cup"
```

With no sentence, it waits for tasks. `/reset` clears the conversation, `/quit` exits. The first time on this machine, install the local model once:

```bash
./qwen_agent/setup_env.sh
```

By default the agent is local Qwen on GPU 1. OpenAI is the same loop:

```bash
OPENAI_API_KEY=sk-... QWEN_BACKEND=openai \
  QWEN_URL=https://api.openai.com/v1 QWEN_MODEL=gpt-4o \
  ./ask_qwen.sh "find the red cup"
```

`QWEN_MODEL` must be a vision model. A local vLLM server is the same switch with `QWEN_URL=http://127.0.0.1:8000/v1`. Either way the agent only talks to the dog through http://localhost:8080.

## The map

The robot reads **`go2_maps/lab.grid.npz`**. That file is the straightened lab map (23 Oct 2026). Named spots are **`go2_maps/lab.places.json`** (16 places). Photos for those spots are in `go2_maps/lab_places/`.

After a reboot the dog's odometry starts over. On the console, line it up with the map before a `go_to`.

## The controller

`controller/go2_robot_sdk` is the WebRTC driver: it turns the dog's lidar, lidar pose, camera, IMU, feet, and joints into ROS topics. `controller/go2_interfaces` is the messages that driver publishes. `controller/docker` is the image recipe. The running container uses the already built image `khang/go2:dev`, which has this driver installed. Speech, simulation, and the old SLAM maps are not in this folder.

## Left out on purpose

Recordings, old map backups, the 14 Oct SLAM pose graph, trail logs, and the phone-app experiments under `Go2Controller` (speech, people, simulation). Those are not needed to start the console or the agent.
