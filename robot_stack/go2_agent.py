"""Blocking movement tools for AI agents, served by go2_dashboard.py under /agent/*.

Agents claim the dog, then call one tool at a time; each call returns when the motion has
finished, was blocked, timed out or was aborted. A person at the console always wins:
driving from the console or pressing Take over aborts the agent's tool and ends its claim.
Every photo an agent takes is kept so the console can show what the agent saw.
"""
import base64
import json
import math
import os
import re
import threading
import time
from collections import deque

import cv2
import numpy as np

AGENT_LEASE_S = 60.0       # an agent's claim lapses after this long with no call
HUMAN_HOLD_S = 300.0       # a takeover lasts until Hand back, or this long with no console command
MOVE_SPEED, MIN_SPEED = 0.25, 0.15          # m/s; slower and the legs may not step
TURN_SPEED = 0.6                           # rad/s; the legs barely step below this (tested by cheetah, board #8)
TURN_OVERSHOOT = math.radians(6)           # the dog keeps turning about this much after stop
MAX_MOVE_M, MAX_TURN_DEG = 3.0, 360.0      # per call
FRONT_CLEAR = (0.35, 0.75, 0.25)           # obstacle box ahead of base_link: x from, x to, half width (m)
BUMP_H = (0.18, 0.60)                      # heights above the floor the dog can bump into (m). The lidar
                                           # reports the floor itself in ~10 cm steps, and the dog fits under
                                           # desk and table tops, so neither counts.
LIVE_R = 3.0                               # go_to: within this of the dog, plan on what the lidar sees now
STALL_S, STALL_M = 2.0, 0.03               # blocked if moved less than this in this long
TILT_LIMIT = 0.6                           # rad of roll/pitch that counts as fallen
MIN_BODY_H = 0.18                          # m; lower means lying down
STALE_S = 1.0                              # sensors older than this: refuse to move
STILL_S = 1.0                              # photos are taken after standing still this long
GOAL_TOL_M = 0.20                          # go_to: close enough to the goal
WAYPOINT_TOL_M = 0.15                      # go_to: move on to the next leg this close to a corner
LOOKAHEAD_M = 0.5                          # go_to: steer toward the point this far ahead on the leg
STEER_GAIN, STEER_MAX = 1.5, 0.5           # go_to: turn rate (rad/s) per rad of heading error, and its cap
TURN_IN_PLACE = math.radians(30)           # go_to: heading error above this: stop and turn in place
MAX_REPLANS = 3                            # go_to: plan around new obstacles this many times
MAPS_DIR = "/maps"
MAP_NAME = os.environ.get("MAP_NAME", "lab")
HOST_UID = 1000


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Aborted(Exception):
    pass


class Agents:
    def __init__(self, node, latest, lock, full):
        self.node, self.latest, self.lock, self.full = node, latest, lock, full
        self.owner = None              # {"kind": "agent"|"human", "client", "ip", "t"}
        self.busy = threading.Lock()   # one tool at a time
        self.abort = threading.Event()
        self.abort_reason = ""
        self.current = None            # {"client", "call", "args", "t"}
        self.notes = {}                # client -> latest free-text note ("what I'm doing")
        self.log = deque(maxlen=40)
        self.photos = deque(maxlen=60)  # {"id", "jpeg", "meta"}
        self.photo_n = 0
        self.last_motion = 0.0
        self.places_lock = threading.Lock()
        self.mapper = None             # set by the console: map frame (go2_mapper.py)
        self.planner = None            # and route planning (go2_nav.py)

    # ---------- ownership ----------
    def owner_now(self):
        o, now = self.owner, time.monotonic()
        if o and now - o["t"] > (AGENT_LEASE_S if o["kind"] == "agent" else HUMAN_HOLD_S):
            self.owner = o = None
        return o

    def agent_may_drive(self, client, ip):
        """Claim or refresh for an agent. Returns None if OK, else the reason it may not."""
        o = self.owner_now()
        if o and o["kind"] == "human":
            return f"A person at {o['ip']} has taken over"
        if o and o["client"] != client:
            return f"Agent {o['client']} has control"
        self.owner = {"kind": "agent", "client": client, "ip": ip, "t": time.monotonic()}
        return None

    def human_drives(self, client, ip):
        """Console activity: a person always wins over an agent."""
        o = self.owner_now()
        if o and o["kind"] == "agent":
            self.interrupt(f"a person at {ip} took over")
        self.owner = {"kind": "human", "client": client, "ip": ip, "t": time.monotonic()}

    def hand_back(self):
        if self.owner and self.owner["kind"] == "human":
            self.owner = None

    def release(self, client):
        if self.owner and self.owner["kind"] == "agent" and self.owner["client"] == client:
            self.owner = None
            return True
        return False

    def interrupt(self, reason):
        if self.current:
            self.abort_reason = reason
            self.abort.set()
        self.node.moving = False
        self.node.stop()

    def status(self):
        o = self.owner_now()
        cur = self.current
        return {"owner": dict(o, idle_s=round(time.monotonic() - o["t"], 1)) if o else None,
                "current": dict(cur, secs=round(time.time() - cur["t"], 1)) if cur else None,
                "notes": self.notes, "log": list(self.log)[-12:],
                "photos": [p["meta"] for p in list(self.photos)[-8:]]}

    # ---------- sensor helpers ----------
    def snap(self):
        with self.lock:
            now = time.time()
            ages = {k: now - t for k, t in self.latest["stamps"].items()}
            return (self.latest["odom"], self.latest["imu"], self.latest["state"], self.latest["cloud"],
                    self.latest["map_pose"], ages)

    def check_safe(self):
        odom, imu, state, _, _, ages = self.snap()
        if odom is None or ages.get("odom", 99) > STALE_S or ages.get("imu", 99) > STALE_S:
            return "stale_sensors"
        if abs(imu["rpy"][0]) > TILT_LIMIT or abs(imu["rpy"][1]) > TILT_LIMIT:
            return "fallen"
        if state and state["body_height"] < MIN_BODY_H:
            return "not_standing"
        return None

    def obstacle(self, direction):
        """Nearest obstacle distance in a box ahead (direction=+1) or behind (-1), from the lidar grid.
        None while the dog's own obstacle avoidance is on: it slows and stops the dog itself, and a
        stall then counts as blocked."""
        if self.node.avoid:
            return None
        odom, _, _, cloud, _, ages = self.snap()
        if not cloud or not cloud["obs"] or ages.get("cloud", 99) > 3.0 or cloud["frame"] == "base_link":
            return None
        pts = np.asarray(cloud["obs"], dtype=np.float64)
        dx, dy = pts[:, 0] - odom["pos"][0], pts[:, 1] - odom["pos"][1]
        c, s = math.cos(odom["yaw"]), math.sin(odom["yaw"])
        fx, fy = (c * dx + s * dy) * direction, -s * dx + c * dy
        x0, x1, half = FRONT_CLEAR
        hit = (fx > x0) & (fx < x1) & (np.abs(fy) < half) & (pts[:, 2] > BUMP_H[0]) & (pts[:, 2] < BUMP_H[1])
        return float(fx[hit].min()) if hit.any() else None

    def pose_out(self):
        odom, _, _, _, mp, _ = self.snap()
        return {"odom": {"x": odom["pos"][0], "y": odom["pos"][1], "yaw": odom["yaw"]} if odom else None,
                "map_pose": mp}

    def tick(self, deadline):
        if self.abort.is_set():
            raise Aborted(self.abort_reason or "stopped")
        if time.monotonic() > deadline:
            raise TimeoutError
        bad = self.check_safe()
        if bad:
            raise RuntimeError(bad)

    # ---------- tool runner ----------
    def run(self, client, call, args, fn):
        if not self.busy.acquire(blocking=False):
            return 409, {"result": "error", "reason": "busy: another tool is running"}
        self.abort.clear()
        self.abort_reason = ""
        self.current = {"client": client, "call": call, "args": args, "t": time.time()}
        if args.get("note"):
            self.notes[client] = {"text": str(args["note"])[:300], "t": time.time()}
        try:
            out = fn()
        except Aborted as e:
            out = {"result": "aborted", "reason": str(e)}
        except TimeoutError:
            out = {"result": "timeout", "reason": "timeout_s reached"}
        except RuntimeError as e:
            out = {"result": "error", "reason": str(e)}
        finally:
            self.node.moving = False
            self.node.stop()
            self.current = None
            self.busy.release()
            if self.owner and self.owner["kind"] == "agent" and self.owner["client"] == client:
                self.owner["t"] = time.monotonic()
        out.update(self.pose_out(), t=round(time.time(), 2))
        brief = {k: v for k, v in out.items() if k in ("result", "reason", "side", "moved_m", "turned_deg", "look_deg")}
        if "photos" in out:
            brief["photos"] = [p["id"] for p in out["photos"]]
        elif "photo" in out:
            brief["photo"] = out["photo"]["id"]
        self.log.append({"t": round(time.time(), 2), "client": client, "call": call,
                         "args": {k: v for k, v in args.items() if k not in ("client", "note")}, **brief})
        return 200, out

    # ---------- motions ----------
    def _move(self, m, timeout_s):
        m = max(-MAX_MOVE_M, min(MAX_MOVE_M, float(m)))
        sign = 1 if m >= 0 else -1
        deadline = time.monotonic() + timeout_s
        odom0 = self.snap()[0]
        if odom0 is None:
            raise RuntimeError("stale_sensors")
        x0, y0, yaw0 = odom0["pos"][0], odom0["pos"][1], odom0["yaw"]
        hist, done = deque(), 0.0
        while True:
            self.tick(deadline)
            odom = self.snap()[0]
            done = ((odom["pos"][0] - x0) * math.cos(yaw0) + (odom["pos"][1] - y0) * math.sin(yaw0)) * sign
            left = abs(m) - done
            if left <= 0.03:
                return {"result": "done", "moved_m": round(done * sign, 3)}
            ob = self.obstacle(sign)
            if ob is not None:
                return {"result": "blocked", "side": "front" if sign > 0 else "back",
                        "reason": f"obstacle {ob:.2f} m from the robot centre", "moved_m": round(done * sign, 3)}
            now = time.monotonic()
            hist.append((now, done))
            while hist and now - hist[0][0] > STALL_S:
                hist.popleft()
            if now - (deadline - timeout_s) > STALL_S + 0.5 and hist and done - hist[0][1] < STALL_M:
                return {"result": "blocked", "side": "front" if sign > 0 else "back",
                        "reason": "not making progress (avoidance stopped it, or something is in the way)",
                        "moved_m": round(done * sign, 3)}
            v = max(MIN_SPEED, min(MOVE_SPEED, left * 1.2)) * sign
            self.node.move(v, 0.0, 0.0)
            self.last_motion = time.monotonic()
            time.sleep(0.1)

    def _turn(self, deg, timeout_s):
        """Turn at full rate, stop early for the overshoot, settle, then correct once if off by > 5 degrees."""
        deg = max(-MAX_TURN_DEG, min(MAX_TURN_DEG, float(deg)))
        deadline = time.monotonic() + timeout_s
        total = 0.0
        want = math.radians(deg)
        for attempt in range(2):
            err = want - total
            if attempt and abs(err) <= math.radians(5):
                break
            r = self._turn_once(err, deadline)
            self.node.moving = False
            self.node.stop()
            self._settle(0.8)
            total += r["turned"] + wrap(self.snap()[0]["yaw"] - r["yaw"])   # includes the overshoot
            if r["result"] != "done":
                return {"result": r["result"], "side": "turn", "reason": r["reason"],
                        "turned_deg": round(math.degrees(total), 1)}
        return {"result": "done", "turned_deg": round(math.degrees(total), 1)}

    def _settle(self, secs):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            if self.abort.is_set():
                raise Aborted(self.abort_reason or "stopped")
            time.sleep(0.05)

    def _turn_once(self, rad, deadline):
        """One turn of `rad` (signed) at full rate. Returns how far it turned (signed) and the yaw when it stopped."""
        target, sign = abs(rad), 1 if rad >= 0 else -1
        early = min(TURN_OVERSHOOT, target * 0.4)
        start = time.monotonic()
        prev = self.snap()[0]["yaw"]
        turned, hist = 0.0, deque()
        while target - turned > early:
            self.tick(deadline)
            yaw = self.snap()[0]["yaw"]
            turned += wrap(yaw - prev) * sign
            prev = yaw
            now = time.monotonic()
            hist.append((now, turned))
            while hist and now - hist[0][0] > STALL_S:
                hist.popleft()
            if now - start > STALL_S + 0.5 and hist and turned - hist[0][1] < math.radians(3):
                return {"result": "blocked", "reason": "not turning", "turned": turned * sign, "yaw": yaw}
            self.node.move(0.0, 0.0, TURN_SPEED * sign)
            self.last_motion = time.monotonic()
            time.sleep(0.1)
        return {"result": "done", "turned": turned * sign, "yaw": prev}

    def _photo(self, client, timeout_s=5.0):
        """Wait until the dog has stood still ~1 s, then keep one full-size frame."""
        self.node.moving = False
        self.node.stop()
        deadline = time.monotonic() + timeout_s
        still_since = None
        while True:
            if self.abort.is_set():
                raise Aborted(self.abort_reason or "stopped")
            odom = self.snap()[0]
            moving = odom is None or abs(odom["vel"][0]) > 0.03 or abs(odom["vel"][1]) > 0.03 or abs(odom["vel"][2]) > 0.05
            now = time.monotonic()
            still_since = None if moving else (still_since or now)
            if still_since and now - still_since >= STILL_S and now - self.last_motion >= STILL_S:
                break
            if now > deadline:
                break     # take it anyway and say so
            time.sleep(0.05)
        with self.lock:
            img, ft = self.full["img"], self.full["t"]
        if img is None or time.time() - ft > 2.0:
            raise RuntimeError("no_camera_frame")
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 90])
        self.photo_n += 1
        pid = f"p{int(time.time())}_{self.photo_n}"
        meta = {"id": pid, "url": f"/agent/photos/{pid}.jpg", "client": client, "t": round(ft, 2),
                "width": int(img.shape[1]), "height": int(img.shape[0]),
                "still": bool(still_since and time.monotonic() - still_since >= STILL_S),
                "look_deg": round(math.degrees(self.node.head)), **self.pose_out()}
        self.photos.append({"id": pid, "jpeg": buf.tobytes(), "meta": meta})
        return meta

    def photo_bytes(self, pid):
        for p in self.photos:
            if p["id"] == pid:
                return p["jpeg"]
        return None

    # ---------- tools (called by the HTTP handler) ----------
    def tool(self, call, client, args):
        timeout_s = float(args.get("timeout_s", 30))
        inline = bool(args.get("inline"))

        def with_b64(meta):
            if inline:
                meta = dict(meta, jpeg_b64=base64.b64encode(self.photo_bytes(meta["id"])).decode())
            return meta

        if call == "move":
            return self.run(client, call, args, lambda: self._move(args["m"], timeout_s))
        if call == "turn":
            return self.run(client, call, args, lambda: self._turn(args["deg"], timeout_s))
        if call == "photo":
            return self.run(client, call, args, lambda: {"result": "done", "photo": with_b64(self._photo(client))})
        if call == "look_around":
            n = max(1, min(12, int(args.get("n", 4))))

            def look():
                shots, deadline = [], time.monotonic() + timeout_s
                for i in range(n):
                    shots.append(with_b64(self._photo(client)))
                    if n > 1:
                        r = self._turn(360.0 / n, max(1.0, deadline - time.monotonic()))
                        if r["result"] != "done":
                            return {"result": r["result"], "reason": r.get("reason"), "photos": shots}
                return {"result": "done", "photos": shots}
            return self.run(client, call, args, look)
        if call == "go_to":
            return self.run(client, call, args, lambda: self._go_to(args, timeout_s))
        if call == "look":
            return self.run(client, call, args, lambda: self._look(float(args["deg"])))
        raise KeyError(call)

    def _look(self, deg):
        """Tilt the camera up (+) or down (-) by pitching the body while standing; 0 levels it.
        Moving levels it again by itself."""
        self.node.look(math.radians(deg))
        time.sleep(1.5)
        self.tick(time.monotonic() + 1)
        imu = self.snap()[1]
        return {"result": "done", "look_deg": round(math.degrees(self.node.head)),
                "body_pitch_deg": round(math.degrees(imu["rpy"][1]), 1) if imu else None}

    def map_pose_now(self):
        """Robot pose on the map from the latest odometry (faster than the 5 Hz map_pose)."""
        odom = self.snap()[0]
        if odom is None or self.mapper is None or self.mapper.T is None:
            return None
        x, y, yaw = self.mapper.to_map(odom["pos"][0], odom["pos"][1], odom["yaw"])
        return {"x": x, "y": y, "yaw": yaw}

    def update_live(self):
        """Give the planner what the lidar sees right now around the dog: it overrides the saved map
        there (things moved, or the map was wrong). The saved map itself isn't changed."""
        odom, _, _, cloud, _, ages = self.snap()
        if (cloud is None or odom is None or cloud.get("frame") == "base_link" or ages.get("cloud", 99) > 3.0
                or self.mapper is None or self.mapper.T is None):
            return
        obs, free = [], []
        for x, y, h in cloud["obs"]:
            (obs if BUMP_H[0] < h < BUMP_H[1] else free).append(self.mapper.to_map(x, y)[:2])
        free += [self.mapper.to_map(x, y)[:2] for x, y in cloud["flat"]]
        mp = self.mapper.to_map(odom["pos"][0], odom["pos"][1])
        self.planner.set_live(obs, free, mp[0], mp[1], LIVE_R)

    def _goal(self, args):
        """(x, y, yaw or None, place or None) for a go_to: a saved place, or x/y[/yaw_deg] on the map."""
        if "place" in args:
            p = self.places().get(str(args["place"]))
            if not p:
                raise RuntimeError(f"unknown place {args['place']}")
            yaw = math.radians(p["yaw_deg"])
            t = p.get("target")
            if t:      # face the thing itself
                yaw = math.atan2(t["y"] - p["y"], t["x"] - p["x"])
            return p["x"], p["y"], yaw, p
        yaw = math.radians(float(args["yaw_deg"])) if args.get("yaw_deg") is not None else None
        return float(args["x"]), float(args["y"]), yaw, None

    def _go_to(self, args, timeout_s):
        """Drive to a place or map point along a planned route, around furniture; re-plan around
        obstacles that aren't on the map; finally face the place's target (or yaw_deg)."""
        gx, gy, gyaw, place = self._goal(args)
        if self.planner is None:
            raise RuntimeError("no planner")
        deadline = time.monotonic() + timeout_s
        moved, replans, first_plan = 0.0, 0, None
        while True:
            mp = self.map_pose_now()
            if mp is None:
                raise RuntimeError("no_map_pose: the console is not lined up with the map")
            if math.hypot(gx - mp["x"], gy - mp["y"]) < GOAL_TOL_M:
                break
            self.update_live()
            plan = self.planner.plan(mp["x"], mp["y"], gx, gy)
            if "error" in plan:
                if replans:
                    return {"result": "blocked", "moved_m": round(moved, 2), "replans": replans,
                            "reason": "something not on the map blocks the way, and there is no other route"}
                return {"result": "no_route", "reason": plan["error"], "moved_m": round(moved, 2), "replans": replans}
            first_plan = first_plan or plan
            gx, gy = plan["goal"]          # may have moved off furniture onto open floor
            r = self._follow(plan["path"][1:], deadline)
            moved += r.pop("moved_m", 0.0)
            if r["result"] == "done":
                continue
            if r["result"] == "blocked" and r.get("at") and replans < MAX_REPLANS:
                replans += 1
                self.planner.block(*r["at"], r=0.15)
                self._settle(0.5)
                continue
            return dict(r, moved_m=round(moved, 2), replans=replans)
        if gyaw is not None:
            mp = self.map_pose_now()
            d = math.degrees(wrap(gyaw - mp["yaw"]))
            if abs(d) > 5:
                r = self._turn(d, max(1.0, deadline - time.monotonic()))
                if r["result"] != "done":
                    return dict(r, moved_m=round(moved, 2), replans=replans)
        out = {"result": "done", "moved_m": round(moved, 2), "replans": replans,
               "route_m": first_plan["length_m"] if first_plan else 0.0}
        if place and place.get("target"):
            out["target"] = place["target"]
        return out

    def _follow(self, path, deadline):
        """Drive the legs of a planned route. Returns result, moved_m and, when blocked, `at`: the
        map spot of the obstacle."""
        moved = 0.0
        prev = self.map_pose_now()
        ax, ay = prev["x"], prev["y"]          # start of the current leg
        for i, (wx, wy) in enumerate(path):
            last = i == len(path) - 1
            tol = GOAL_TOL_M * 0.6 if last else WAYPOINT_TOL_M
            hist = deque()
            lx, ly = wx - ax, wy - ay
            leg = math.hypot(lx, ly) or 1e-6
            while True:
                self.tick(deadline)
                mp = self.map_pose_now()
                moved += math.hypot(mp["x"] - prev["x"], mp["y"] - prev["y"])
                prev = mp
                dist = math.hypot(wx - mp["x"], wy - mp["y"])
                along = ((mp["x"] - ax) * lx + (mp["y"] - ay) * ly) / leg
                if dist < tol or (not last and along >= leg):
                    break
                # steer toward a point a little ahead on the leg, so the dog stays on the line
                k = min(leg, max(0.0, along) + LOOKAHEAD_M) / leg
                px, py = ax + lx * k, ay + ly * k
                err = wrap(math.atan2(py - mp["y"], px - mp["x"]) - mp["yaw"])
                if abs(err) > TURN_IN_PLACE:
                    self.node.moving = False
                    self.node.stop()
                    self._settle(0.3)
                    r = self._turn(math.degrees(err), max(1.0, deadline - time.monotonic()))
                    if r["result"] != "done":
                        return dict(r, moved_m=moved)
                    hist.clear()
                    prev = self.map_pose_now()
                    continue
                ob = self.obstacle(1)
                if ob is not None:
                    self.node.moving = False
                    self.node.stop()
                    return {"result": "blocked", "side": "front", "moved_m": moved,
                            "reason": f"obstacle {ob:.2f} m ahead that is not on the map",
                            "at": [mp["x"] + math.cos(mp["yaw"]) * (ob + 0.1), mp["y"] + math.sin(mp["yaw"]) * (ob + 0.1)]}
                now = time.monotonic()
                hist.append((now, dist))
                while hist and now - hist[0][0] > STALL_S:
                    hist.popleft()
                if now - hist[0][0] > STALL_S - 0.3 and hist[0][1] - dist < STALL_M:
                    self.node.moving = False
                    self.node.stop()
                    return {"result": "blocked", "side": "front", "moved_m": moved,
                            "reason": "not making progress (something is in the way, or avoidance stopped it)",
                            "at": [mp["x"] + math.cos(mp["yaw"]) * 0.5, mp["y"] + math.sin(mp["yaw"]) * 0.5]}
                v = MOVE_SPEED if not last else max(MIN_SPEED, min(MOVE_SPEED, dist * 1.2))
                v *= max(0.6, math.cos(err))
                self.node.move(v, 0.0, max(-STEER_MAX, min(STEER_MAX, STEER_GAIN * err)))
                self.last_motion = time.monotonic()
                time.sleep(0.1)
            ax, ay = wx, wy
        self.node.moving = False
        self.node.stop()
        return {"result": "done", "moved_m": moved}

    # ---------- places (stored next to the map) ----------
    def places_path(self):
        return os.path.join(MAPS_DIR, f"{MAP_NAME}.places.json")

    def places(self):
        try:
            with open(self.places_path()) as f:
                return json.load(f)
        except (OSError, ValueError):
            return {}

    def save_place(self, pid, description, by, pose=None, image=None, target=None):
        """Save a place. A place is where the dog stands (x, y, yaw_deg: what go_to drives to) and,
        when known, target: where the thing itself is.
        - pose None: the dog's spot now with its camera view; the target is the first mapped obstacle
          straight ahead, if any within 4 m.
        - pose {x, y, yaw}: a stand spot picked on the map, with target (x, y) or None and image
          (an earlier camera frame) or None."""
        if not re.fullmatch(r"[a-z0-9_]{1,40}", pid or ""):
            raise ValueError("id must be 1-40 characters of a-z, 0-9 or _")
        if pose is not None:
            mp, img = pose, image
        else:
            mp = self.snap()[4]
            if mp is None:
                raise ValueError("no map pose: mapping or localization is not running")
            mp = dict(mp, pitch_deg=round(math.degrees(self.node.head)))
            with self.lock:
                img = self.full["img"]
            if target is None and self.planner is not None:
                target = self.planner.raycast(mp["x"], mp["y"], mp["yaw"])
        prev_dir = os.path.join(MAPS_DIR, f"{MAP_NAME}_places")
        os.makedirs(prev_dir, exist_ok=True)
        preview = None
        stale = os.path.join(prev_dir, f"{pid}.jpg")
        if img is None and os.path.exists(stale):
            os.remove(stale)      # replacing a place: don't keep the old one's picture
        if img is not None:
            preview = f"{pid}.jpg"
            cv2.imwrite(os.path.join(prev_dir, preview), img, [cv2.IMWRITE_JPEG_QUALITY, 85])
            os.chown(os.path.join(prev_dir, preview), HOST_UID, HOST_UID)
        with self.places_lock:
            places = self.places()
            places[pid] = {"id": pid, "x": round(mp["x"], 3), "y": round(mp["y"], 3),
                           "yaw_deg": round(math.degrees(mp["yaw"]), 1),
                           "pitch_deg": float(mp.get("pitch_deg", 0.0)), "description": str(description or "")[:300],
                           "target": {"x": round(target[0], 2), "y": round(target[1], 2)} if target else None,
                           "preview": f"/places/{preview}" if preview else None, "map": MAP_NAME,
                           "saved_by": by, "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                           "marked_on_map": pose is not None}
            self._write(places)
        os.chown(prev_dir, HOST_UID, HOST_UID)
        return places[pid]

    def delete_place(self, pid):
        with self.places_lock:
            places = self.places()
            if places.pop(pid, None) is None:
                return False
            self._write(places)
            return True

    def _write(self, places):
        tmp = self.places_path() + ".tmp"
        with open(tmp, "w") as f:
            json.dump(places, f, indent=1)
        os.chown(tmp, HOST_UID, HOST_UID)
        os.replace(tmp, self.places_path())
