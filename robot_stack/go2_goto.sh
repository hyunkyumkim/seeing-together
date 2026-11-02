#!/usr/bin/env bash
# Send the dog to a saved place or a map point. It plans a route on the map and drives there.
#   ./go2_goto.sh                      list the places
#   ./go2_goto.sh conference_table     go to a place (its stand spot, facing the thing)
#   ./go2_goto.sh 1.5 -2.0 [yaw_deg]   go to a map point (metres), optionally face yaw_deg after
#   PLAN=1 ./go2_goto.sh sofa          only show the route, don't move
# Press "Hand back" on the console first. Take over / STOP there always wins.
set -euo pipefail
URL=http://localhost:8080
CLIENT=${GO2_AGENT_NAME:-oliver-goto}
curl -sf -o /dev/null $URL/ || { echo "The console isn't running: start it with run_go2_keys.sh"; exit 1; }
post() { curl -s -H 'Content-Type: application/json' -d "$2" "$URL/agent/$1"; }

if [ $# -eq 0 ]; then
    curl -s $URL/places | python3 -c '
import json, sys
for p in json.load(sys.stdin).values():
    print("  %-18s stand (%.1f, %.1f) facing %.0f°  %s" % (p["id"], p["x"], p["y"], p["yaw_deg"], p.get("description") or ""))'
    exit 0
fi
if [ $# -eq 1 ]; then
    q="place=$1"; body="{\"client\":\"$CLIENT\",\"place\":\"$1\",\"timeout_s\":120}"
else
    q="x=$1&y=$2"; body="{\"client\":\"$CLIENT\",\"x\":$1,\"y\":$2${3:+,\"yaw_deg\":$3},\"timeout_s\":120}"
fi

if [ -n "${PLAN:-}" ]; then
    curl -s "$URL/nav/plan?$q"; echo; exit 0
fi
trap 'post stop "{\"client\":\"$CLIENT\"}" >/dev/null; post release "{\"client\":\"$CLIENT\"}" >/dev/null' EXIT
echo "Driving to $* (Ctrl-C stops the dog)..."
post go_to "$body" | python3 -c '
import json, math, sys
d = json.load(sys.stdin); mp = d.get("map_pose")
print("result:", d.get("result"), "-", d.get("reason", ""), "| moved", d.get("moved_m"), "m, replans", d.get("replans"))
if mp: print("now at x=%.2f y=%.2f facing %.0f°" % (mp["x"], mp["y"], math.degrees(mp["yaw"])))'
