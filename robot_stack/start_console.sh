#!/usr/bin/env bash
# Web console for this folder. Uses go2_maps/lab.grid.npz (the straightened lab map)
# and the programs linked next to this script. The lidar driver itself runs from the
# image khang/go2:dev; controller/ is the source of that driver.
#   ./start_console.sh          start, or restart the driver only if it has no data
#   ./start_console.sh --reset  always restart the driver first
set -u
STACK=$(dirname "$(readlink -f "$0")")
DOG=172.17.39.82
C=go2_khang
IMAGE=khang/go2:dev
ROS='source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash'

MAP=$STACK/go2_maps/lab.grid.npz
PLACES=$STACK/go2_maps/lab.places.json
[ -f "$MAP" ] || { echo "No map at $MAP"; exit 1; }
[ -f "$PLACES" ] || { echo "No places at $PLACES"; exit 1; }
echo "Map: $(readlink -f "$MAP")"
echo "     $(date -r "$MAP" '+%Y-%m-%d %H:%M:%S')"

driver_ok() {
    docker exec $C bash -c "$ROS && timeout 4 ros2 topic hz /imu 2>/dev/null | grep -q average"
}

wait_for() {
    local secs=$1 what=$2; shift 2
    printf '%s' "$what"
    for _ in $(seq "$secs"); do
        if "$@" >/dev/null 2>&1; then echo " ok"; return 0; fi
        printf '.'; sleep 1
    done
    echo " timed out"; return 1
}

wait_for 90 "Waiting for the dog on the network ($DOG)" ping -c1 -W1 $DOG \
    || { echo "Dog not reachable. Is it on and on the lab Wi-Fi?"; exit 1; }
wait_for 60 "Waiting for the dog's WebRTC port" nc -z -w1 $DOG 9991 \
    || { echo "Dog is up but not accepting connections yet. Try again in a minute."; exit 1; }

if docker inspect go2_test >/dev/null 2>&1 && [ "$(docker inspect -f '{{.State.Running}}' go2_test 2>/dev/null)" = "true" ]; then
    echo "The shared driver go2_test is still running (image rosrobot)."
    echo "Stop it before starting $C. The dog allows only one connection:"
    echo "  docker stop go2_test"
    exit 1
fi

if ! docker inspect $C >/dev/null 2>&1; then
    echo "Creating driver container from $IMAGE..."
    mkdir -p "$STACK/go2_recordings" "$STACK/go2_maps"
    docker run -d --name $C --net=host --entrypoint bash \
        -v /home/khang:/home/khang \
        -v "$STACK/go2_recordings:/recordings" -v /etc/localtime:/etc/localtime:ro \
        -v "$STACK/go2_maps:/maps" -e MAP_NAME=lab \
        -e ROBOT_IP=$DOG -e CONN_TYPE=webrtc -e ROS_DOMAIN_ID=78 "$IMAGE" \
        -c "$ROS && ros2 launch go2_robot_sdk robot.launch.py rviz2:=false nav2:=false slam:=false foxglove:=false joystick:=false teleop:=false" >/dev/null
elif [ "$(docker inspect -f '{{.State.Running}}' $C 2>/dev/null)" != "true" ]; then
    echo "Starting driver..."; docker start $C >/dev/null
elif [ "${1:-}" = "--reset" ]; then
    echo "Restarting driver..."; docker restart $C >/dev/null
elif ! driver_ok; then
    echo "Driver not receiving data; restarting it..."; docker restart $C >/dev/null
else
    echo "Driver already connected."
fi

wait_for 45 "Waiting for sensor data" driver_ok || {
    echo "Driver still has no data. Close the Unitree app on all phones (only one connection"
    echo "is allowed), then run this again with --reset."; exit 1; }

for f in go2_keys.py go2_dashboard.py go2_agent.py go2_mapper.py go2_trail.py go2_nav.py; do
    docker cp -q "$(readlink -f "$STACK/$f")" $C:/root/$f
done
if ! docker exec $C pgrep -f go2_dashboard.py >/dev/null; then
    docker exec -d $C bash -c "$ROS && exec python3 /root/go2_dashboard.py 8080 > /tmp/dash.log 2>&1"
fi
URL=http://localhost:8080
wait_for 15 "Starting dashboard" curl -sf -o /dev/null $URL
if [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ] && command -v xdg-open >/dev/null; then
    xdg-open $URL >/dev/null 2>&1 &
    echo "Opened the dashboard in your browser: $URL (tick Enable driving there to drive)"
else
    echo "Dashboard: $URL (no desktop session here, so open it yourself)"
fi
echo "Terminal keyboard control also works below. Press x to quit."

docker exec -it $C bash -c "$ROS && python3 /root/go2_keys.py"
