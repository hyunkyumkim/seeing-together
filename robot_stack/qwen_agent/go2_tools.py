#!/usr/bin/env python3
"""Robot tools for the local VLM agent: plain Python calls to the console's agent API
(http://localhost:8080/agent/...). Same rules as go2_claude/go2_tools_mcp.py: a person at the
console always wins, one tool runs at a time, and the dog refuses to move if it has fallen or its
sensors are stale.

Each tool returns (text, images): text the model reads, and PIL images it looks at (each image
comes with a caption in the text). TOOLS lists the schemas the model is shown; run(name, args)
executes one of them.
"""
import base64
import io
import json
import math
import os
import time
import urllib.error
import urllib.request

from PIL import Image, ImageDraw, ImageFont

CONSOLE = os.environ.get("GO2_CONSOLE", "http://127.0.0.1:8080")
CLIENT = os.environ.get("GO2_AGENT_NAME", "qwen-agent")
PHOTO_W = int(os.environ.get("GO2_PHOTO_W", "896"))     # photos are shrunk to this width for the model


# ---------- console I/O ----------
def call(tool, timeout=60, **args):
    """POST /agent/<tool>; returns the JSON reply (also for refusals and errors)."""
    req = urllib.request.Request(f"{CONSOLE}/agent/{tool}", data=json.dumps({"client": CLIENT, **args}).encode(),
                                 headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read() or b"{}")
    except urllib.error.HTTPError as e:
        try:
            return json.loads(e.read())
        except ValueError:
            return {"result": "error", "reason": f"HTTP {e.code}"}
    except (urllib.error.URLError, OSError) as e:
        return {"result": "error", "reason": f"console not reachable: {e}"}


def get(path, timeout=10):
    with urllib.request.urlopen(CONSOLE + path, timeout=timeout) as r:
        return r.read()


def console_up():
    try:
        get("/control", timeout=2)
        return True
    except (urllib.error.URLError, OSError):
        return False


def pose_text(out):
    mp = out.get("map_pose")
    if not mp:
        return "map pose: unknown (not lined up with the map)"
    return f"map pose: x={mp['x']:.2f} y={mp['y']:.2f} heading={math.degrees(mp['yaw']):.0f}°"


def brief(out, skip=("map_pose", "odom", "photo", "photos", "t", "notes", "log")):
    return json.dumps({k: v for k, v in out.items() if k not in skip})


def shrink(raw, width):
    img = Image.open(io.BytesIO(raw)).convert("RGB")
    if img.width > width:
        img = img.resize((width, round(img.height * width / img.width)))
    return img


def photo_images(out):
    """(captions-and-images) for every photo in a tool reply: [(caption, PIL image), ...]."""
    metas = out.get("photos") or ([out["photo"]] if out.get("photo") else [])
    items = []
    for m in metas:
        raw = base64.b64decode(m["jpeg_b64"]) if m.get("jpeg_b64") else get(m["url"])
        mp = m.get("map_pose") or {}
        cap = f"photo {m['id']}" + (f" taken from x={mp['x']:.2f} y={mp['y']:.2f} facing {math.degrees(mp['yaw']):.0f}°"
                                    if mp else "") + (f", camera tilted {m['look_deg']}°" if m.get("look_deg") else "")
        items.append((cap, shrink(raw, PHOTO_W)))
    return items


# ---------- tools ----------
def robot_status():
    s = json.loads(get("/agent/status"))
    owner = s.get("owner")
    return "\n".join([
        f"safe: {s.get('safe')}",
        f"in control: {owner['kind'] + ' ' + owner['client'] if owner else 'nobody (free)'}",
        f"running now: {s['current']['call'] if s.get('current') else 'nothing'}",
        pose_text(s),
    ]), []


def list_places():
    places = json.loads(get("/places"))
    if not places:
        return "No places saved yet.", []
    rows = []
    for p in places.values():
        t = p.get("target")
        rows.append(f"- {p['id']}: {p.get('description') or '(no description)'}; "
                    + (f"thing at ({t['x']:.1f}, {t['y']:.1f}); " if t else "")
                    + f"stand at ({p['x']:.1f}, {p['y']:.1f}) facing {p['yaw_deg']:.0f}°")
    return "\n".join(rows), []


def view_map():
    st = json.loads(get("/state.json"))
    m, pose = st.get("map"), st.get("map_pose")
    if not m:
        return "No map.", []
    places = json.loads(get("/places"))
    img = Image.open(io.BytesIO(get("/map.png"))).convert("RGB")
    S = 4
    img = img.resize((img.width * S, img.height * S), Image.NEAREST)
    d = ImageDraw.Draw(img)
    font = ImageFont.load_default()
    res, (ox, oy), H = m["resolution"], m["origin"], m["height"]

    def px(x, y):
        return (x - ox) / res * S, (H - (y - oy) / res) * S

    for gx in range(math.ceil(ox), math.floor(ox + m["width"] * res) + 1):
        x0, _ = px(gx, 0)
        d.line([(x0, 0), (x0, img.height)], fill=(170, 190, 230) if gx else (90, 120, 220), width=1)
        d.text((x0 + 2, 2), str(gx), fill=(40, 60, 160), font=font)
    for gy in range(math.ceil(oy), math.floor(oy + H * res) + 1):
        _, y0 = px(0, gy)
        d.line([(0, y0), (img.width, y0)], fill=(170, 190, 230) if gy else (90, 120, 220), width=1)
        d.text((2, y0 + 1), str(gy), fill=(40, 60, 160), font=font)
    for p in places.values():
        t = p.get("target") or {"x": p["x"], "y": p["y"]}
        tx, ty = px(t["x"], t["y"])
        d.ellipse([tx - 5, ty - 5, tx + 5, ty + 5], fill=(30, 120, 220))
        d.text((tx + 7, ty - 6), p["id"], fill=(10, 60, 160), font=font)
    if pose:
        x, y = px(pose["x"], pose["y"])
        a = pose["yaw"]
        pts = [(x + 22 * math.cos(a), y - 22 * math.sin(a)),
               (x + 12 * math.cos(a + 2.4), y - 12 * math.sin(a + 2.4)),
               (x + 12 * math.cos(a - 2.4), y - 12 * math.sin(a - 2.4))]
        d.polygon(pts, fill=(220, 30, 30), outline=(120, 0, 0))
        d.text((x + 14, y + 8), "dog", fill=(160, 0, 0), font=font)
    text = (f"Map {m['width'] * res:.0f} x {H * res:.0f} m. White = open floor, black = walls/furniture, "
            f"grey = unmapped. Grid every metre with map coordinates. Blue dots = places, red arrow = the dog. "
            + pose_text({"map_pose": pose}))
    return text, [("lab map", img)]


def take_photo(note=""):
    out = call("photo", timeout=30, inline=True, note=note)
    if out.get("result") != "done":
        return brief(out), []
    return pose_text(out), photo_images(out)


def look_around(n=4, note=""):
    n = max(1, min(12, int(n)))
    out = call("look_around", timeout=150, n=n, inline=True, note=note, timeout_s=120)
    return brief(out) + "\n" + pose_text(out), photo_images(out)


def go_to(place="", x=None, y=None, yaw_deg=None, note=""):
    args = {"timeout_s": 120, "note": note}
    if place:
        args["place"] = place
    elif x is not None and y is not None:
        args.update(x=float(x), y=float(y))
        if yaw_deg is not None:
            args["yaw_deg"] = float(yaw_deg)
    else:
        return "give place, or x and y", []
    out = call("go_to", timeout=140, **args)
    return brief(out) + "\n" + pose_text(out), []


def move(metres, note=""):
    out = call("move", timeout=40, m=float(metres), note=note)
    return brief(out) + "\n" + pose_text(out), []


def turn(degrees, note=""):
    out = call("turn", timeout=40, deg=float(degrees), note=note)
    return brief(out) + "\n" + pose_text(out), []


def tilt_camera(degrees):
    out = call("look", timeout=20, deg=float(degrees))
    return brief(out), []


def save_place(id, description, x=None, y=None):
    body = {"id": id, "description": description, "client": CLIENT}
    if x is not None and y is not None:
        body["target"] = {"x": float(x), "y": float(y)}
    req = urllib.request.Request(f"{CONSOLE}/places", data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.read().decode(), []
    except urllib.error.HTTPError as e:
        return e.read().decode(), []


def note(text):
    return brief(call("say", timeout=10, text=text)), []


def stop():
    return brief(call("stop", timeout=10)), []


def release_control():
    return brief(call("release", timeout=10)), []


def finish(found, summary, place="", x=None, y=None):
    """Terminal tool: the agent loop ends the task when it sees this. Control is released here."""
    call("release", timeout=10)
    where = place or (f"x={x} y={y}" if x is not None and y is not None else "")
    return f"task finished. found={found}" + (f" at {where}" if where else "") + f"\n{summary}", []


# ---------- registry ----------
def _p(**props):
    required = [k for k, v in props.items() if v.pop("required", False)]
    return {"type": "object", "properties": props, "required": required}


TOOLS = [
    {"name": "robot_status", "description": "Who controls the dog, whether it is safe to move, its pose on the map "
     "and what it is doing. Call this first. Map coordinates are metres; heading 0° = +x (east on the map), 90° = +y (north).",
     "parameters": _p()},
    {"name": "list_places", "description": "The named places on the map that people marked: id, description, where the "
     "thing is and where the dog stands to see it. go_to(place=id) drives to the standing spot and faces the thing.",
     "parameters": _p()},
    {"name": "view_map", "description": "The lab map as an image: white = open floor, black = walls and furniture, grey = "
     "not mapped, grid lines every metre with coordinates, blue dots = places, red arrow = the dog. Use it to plan "
     "where to go and to pick x/y for go_to.", "parameters": _p()},
    {"name": "take_photo", "description": "Stop, wait until the dog is still, and take one photo with the front camera "
     "(head height ~0.3 m, about 80° wide). Returns the image and where it was taken.",
     "parameters": _p(note={"type": "string", "description": "short note for the people watching"})},
    {"name": "look_around", "description": "Take n photos (1-12) while turning a full circle in place, in n equal steps. "
     "Good for getting oriented or searching a spot. Returns all the photos.",
     "parameters": _p(n={"type": "integer", "description": "number of photos, default 4"},
                      note={"type": "string"})},
    {"name": "go_to", "description": "Drive to a saved place (place=id) or a map point (x, y in metres, optional yaw_deg "
     "to face after). Plans a route around furniture, re-plans around new obstacles, stops if blocked. Up to ~2 minutes. "
     "Result: done, blocked, no_route, aborted (a person took over) or error, with the final pose.",
     "parameters": _p(place={"type": "string", "description": "place id from list_places"},
                      x={"type": "number"}, y={"type": "number"}, yaw_deg={"type": "number"},
                      note={"type": "string"})},
    {"name": "move", "description": "Walk straight forward (+) or back (-), at most 3 m. Stops early if something is in the way.",
     "parameters": _p(metres={"type": "number", "required": True}, note={"type": "string"})},
    {"name": "turn", "description": "Turn in place: + = left (counter-clockwise), - = right, in degrees.",
     "parameters": _p(degrees={"type": "number", "required": True}, note={"type": "string"})},
    {"name": "tilt_camera", "description": "Tilt the body to point the camera up (+, up to ~15°) or down (-, a few degrees). "
     "0 = level. Walking levels it again. Then take_photo.",
     "parameters": _p(degrees={"type": "number", "required": True})},
    {"name": "save_place", "description": "Name a place so people and other agents can find it later (id: a-z, 0-9, _). "
     "Without x/y: the dog's current spot, with the thing taken to be the first obstacle straight ahead.",
     "parameters": _p(id={"type": "string", "required": True}, description={"type": "string", "required": True},
                      x={"type": "number"}, y={"type": "number"})},
    {"name": "note", "description": "Tell the people watching what you are doing or what you found (shown on the console).",
     "parameters": _p(text={"type": "string", "required": True})},
    {"name": "stop", "description": "Stop the dog now.", "parameters": _p()},
    {"name": "release_control", "description": "Give up control of the dog without finishing the task.", "parameters": _p()},
    {"name": "finish", "description": "End the task and report. Use when the object is found and confirmed, when every "
     "likely place was checked without finding it, or when you cannot continue (blocked twice, person took over).",
     "parameters": _p(found={"type": "boolean", "required": True},
                      summary={"type": "string", "required": True,
                               "description": "what you did, what you saw, where the object is (place or map x/y), problems"},
                      place={"type": "string", "description": "place id where it was found"},
                      x={"type": "number"}, y={"type": "number"})},
]

_FN = {t["name"]: globals()[t["name"]] for t in TOOLS}


def run(name, args):
    """Execute one tool. Unknown names and bad arguments come back as text, so the model can recover."""
    fn = _FN.get(name)
    if fn is None:
        return f"unknown tool {name!r}; available: {', '.join(_FN)}", []
    try:
        return fn(**(args or {}))
    except TypeError as e:
        return f"bad arguments for {name}: {e}", []
    except (urllib.error.URLError, OSError, ValueError) as e:
        return f"{name} failed: {e}", []


if __name__ == "__main__":      # quick check against the console, no motion
    print("console up:", console_up())
    print(robot_status()[0])
    print(list_places()[0])
    t, imgs = view_map()
    print(t, "| map image:", imgs[0][1].size if imgs else None)
