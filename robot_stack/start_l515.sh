#!/usr/bin/env bash
# L515 depth camera for this folder. The Pi sender is unchanged (l515_go2_pi4_bridge on the Pi).
# This opens the SSH tunnel and runs the receiver inside go2_khang, next to the console.
#   ./start_l515.sh            start, or restart the receiver with the current settings
#   ./start_l515.sh --stop     stop the receiver (finishing any recording) and the tunnel
#   ./start_l515.sh --status   tunnel, receiver, and depth rate
#   ./start_l515.sh --logs     follow the receiver log
# The console must already be up (./start_console.sh). The page is http://localhost:8081.
set -u
STACK=$(dirname "$(readlink -f "$0")")
PI="${L515_PI:-khang@172.17.29.72}"
LOCAL_PORT="${L515_LOCAL_PORT:-18765}"   # 8765 on this PC is already used by go2_remote
WEB_PORT="${L515_WEB_PORT:-8081}"
C="${GO2_CONTAINER:-go2_khang}"
PARENT="${L515_PARENT_FRAME:-base_link}"
MOUNT_XYZ="${L515_MOUNT_XYZ:-0 0 0}"
MOUNT_RPY="${L515_MOUNT_RPY_DEG:-0 0 0}"
STRIDE="${L515_STRIDE:-2}"
HOST="${L515_HOST:-}"                    # set to the Pi IP for direct LAN mode (no tunnel)
ROS='source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash'
TUNNEL_MATCH="${LOCAL_PORT}:127.0.0.1:8765"
PKG="$STACK/l515/go2_l515_bridge"

tunnel_up() { ss -ltn "sport = :$LOCAL_PORT" | grep -q LISTEN; }

stop_receiver() {
    docker exec "$C" pkill -INT -f 'ros2 bag record -o /recordings/l515_' 2>/dev/null && sleep 2
    docker exec "$C" pkill -TERM -f 'go2_l515_bridge.receiver' 2>/dev/null && sleep 1
    docker exec "$C" pkill -KILL -f 'go2_l515_bridge.receiver' 2>/dev/null
    true
}

stop_tunnel() {
    pkill -f 'L515_TUNNEL=1' 2>/dev/null
    pkill -f -- "$TUNNEL_MATCH" 2>/dev/null
    true
}

start_tunnel() {
    if tunnel_up; then echo "Tunnel port $LOCAL_PORT already open."; return 0; fi
    local probe
    probe=$(ssh -o BatchMode=yes -o ConnectTimeout=5 "$PI" true 2>&1)
    local rc=$?
    if [ $rc -eq 0 ] || ! grep -qi 'permission denied' <<<"$probe"; then
        # Key login works (or the Pi is just not up yet): keep a tunnel that reconnects by itself.
        pgrep -f 'L515_TUNNEL=1' >/dev/null || nohup setsid bash -c "L515_TUNNEL=1; while true; do ssh -N -o BatchMode=yes \
            -o ConnectTimeout=5 -o ExitOnForwardFailure=yes -o ServerAliveInterval=5 -o ServerAliveCountMax=3 \
            -L $TUNNEL_MATCH $PI; sleep 3; done" >/tmp/l515_tunnel.log 2>&1 < /dev/null &
        if [ $rc -ne 0 ]; then
            echo "Pi $PI not reachable yet (${probe##*: }). The tunnel keeps retrying; the page shows"
            echo "'camera live' once the Pi is up and l515-sender is running."
            return 0
        fi
    else
        echo "No SSH key login to $PI (run: ssh-copy-id $PI). Asking for the password once;"
        echo "this tunnel will not reconnect by itself if Wi-Fi drops."
        ssh -f -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=5 -o ServerAliveCountMax=3 \
            -L "$TUNNEL_MATCH" "$PI" || { echo "SSH tunnel to $PI failed."; exit 1; }
    fi
    for _ in $(seq 10); do tunnel_up && { echo "Tunnel localhost:$LOCAL_PORT -> $PI:8765 ok"; return 0; }; sleep 1; done
    echo "Tunnel did not come up; see /tmp/l515_tunnel.log"; exit 1
}

case "${1:-}" in
    --stop) stop_receiver; stop_tunnel; echo "Stopped."; exit 0 ;;
    --logs) exec docker exec "$C" tail -n 50 -f /tmp/l515.log ;;
    --status)
        tunnel_up && echo "tunnel: localhost:$LOCAL_PORT open" || echo "tunnel: down"
        docker exec "$C" pgrep -f go2_l515_bridge.receiver >/dev/null && echo "receiver: running" || echo "receiver: stopped"
        docker exec "$C" bash -c "$ROS && ros2 topic list | grep /go2/l515 && timeout 6 ros2 topic hz /go2/l515/depth/image_raw 2>/dev/null | grep -m1 average"
        exit 0 ;;
esac

if [ "$(docker inspect -f '{{.State.Running}}' "$C" 2>/dev/null)" != "true" ]; then
    echo "Container $C is not running. Start the console first: ./start_console.sh"
    exit 1
fi
[ -f "$PKG/receiver.py" ] || { echo "No receiver at $PKG"; exit 1; }

if [ -n "$HOST" ]; then
    CONNECT=(--host "$HOST" --port 8765)
else
    start_tunnel
    CONNECT=(--host 127.0.0.1 --port "$LOCAL_PORT")
fi

TF=()
if [ -n "$PARENT" ]; then
    TF=(--parent-frame "$PARENT" --mount-xyz $MOUNT_XYZ --mount-rpy-deg $MOUNT_RPY)
    [ "$MOUNT_XYZ" = "0 0 0" ] && echo "Note: L515_MOUNT_XYZ is 0 0 0, so the cloud sits at $PARENT's origin until you measure the mount."
fi

stop_receiver
docker exec "$C" rm -rf /root/l515
docker exec "$C" mkdir -p /root/l515
docker cp -q "$PKG" "$C":/root/l515/
ARGS="${CONNECT[*]} --stride $STRIDE ${TF[*]} --web-port $WEB_PORT"
docker exec -d "$C" bash -c "$ROS && cd /root/l515 && exec python3 -m go2_l515_bridge.receiver $ARGS > /tmp/l515.log 2>&1"

printf 'Starting L515 page'
for _ in $(seq 15); do
    if curl -sf -o /dev/null "http://localhost:$WEB_PORT/status.json"; then
        echo " ok"
        IP=$(hostname -I | tr ' ' '\n' | grep -m1 '^172\.17\.')
        echo "L515 page: http://localhost:$WEB_PORT  (lab LAN: http://${IP:-<this-pc>}:$WEB_PORT)"
        echo "Topics: /go2/l515/... in ROS_DOMAIN_ID=78. Log: ./start_l515.sh --logs"
        exit 0
    fi
    printf '.'; sleep 1
done
echo " failed. Last log lines:"; docker exec "$C" tail -n 20 /tmp/l515.log
exit 1
