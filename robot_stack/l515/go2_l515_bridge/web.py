"""Browser view of the L515 stream (color, depth, point cloud preview) and a recorder for its
ROS topics. Runs inside the receiver process; no rclpy import."""
import ipaddress
import json
import os
import shutil
import signal
import struct
import subprocess
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

from .geometry import xyz_from_depth

CONTROL_NETS = [ipaddress.ip_network(n) for n in ("127.0.0.0/8", "172.17.0.0/18")]  # same as the Go2 dashboard
REC_DIR = os.environ.get("L515_REC_DIR", "/recordings")
REC_MIN_FREE_GB = 20
HOST_UID = 1000            # finished recordings are handed to the host user
ROS_SETUP = ("source /opt/ros/humble/setup.bash"
             " && { [ ! -f /ros2_ws/install/setup.bash ] || source /ros2_ws/install/setup.bash; }")
REC_TOPICS = {
    "color": ["/go2/l515/color/image_raw/compressed", "/go2/l515/color/camera_info"],
    "depth": ["/go2/l515/depth/image_raw", "/go2/l515/depth/camera_info"],
    "points": ["/go2/l515/points"],
    "lidar": ["/point_cloud2"],
    "pose": ["/tf", "/tf_static", "/odom"],
}
DEPTH_VIEW_MAX_M = 5.0     # depth colormap range on the page
CLOUD_STRIDE = 4           # page preview only; the ROS cloud uses --stride
CLOUD_MAX_POINTS = 30000


def depth_jpeg(raw, scale, max_m=DEPTH_VIEW_MAX_M):
    v = np.clip(raw.astype(np.float32) * (scale * 255.0 / max_m), 0, 255).astype(np.uint8)
    img = cv2.applyColorMap(255 - v, cv2.COLORMAP_TURBO)   # near = red, far = blue
    img[raw == 0] = 0
    return cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()


def point_colors(pts, extr, color_intr, color):
    """RGB for depth-frame points, sampled from the color image (distortion ignored: preview only)."""
    rgb = np.full((len(pts), 3), 140, dtype=np.uint8)
    if color is None or not len(pts):
        return rgb
    r = np.array(extr[:9], dtype=np.float32).reshape(3, 3, order="F")
    pc = pts @ r.T + np.array(extr[9:12], dtype=np.float32)
    fx, fy, cx, cy = color_intr
    z = pc[:, 2]
    ok = z > 0.05
    u = np.full(len(pts), -1, dtype=np.int64)
    v = np.full(len(pts), -1, dtype=np.int64)
    u[ok] = np.rint(fx * pc[ok, 0] / z[ok] + cx)
    v[ok] = np.rint(fy * pc[ok, 1] / z[ok] + cy)
    h, w = color.shape[:2]
    ok &= (u >= 0) & (u < w) & (v >= 0) & (v < h)
    rgb[ok] = color[v[ok], u[ok], ::-1]
    return rgb


def cloud_bin(frame, color, max_range, stride=CLOUD_STRIDE, max_points=CLOUD_MAX_POINTS):
    """uint32 N, float32 xyz[N*3] (depth optical frame, metres), uint8 rgb[N*3]."""
    pts = xyz_from_depth(frame.depth, frame.depth_intr, frame.depth_scale, stride=stride, max_range=max_range)
    if len(pts) > max_points:
        pts = pts[np.linspace(0, len(pts) - 1, max_points).astype(np.int64)]
    rgb = point_colors(pts, frame.extrinsics, frame.color_intr, color)
    return struct.pack("<I", len(pts)) + pts.tobytes() + rgb.tobytes()


class Recorder:
    """Runs `ros2 bag record` for the chosen L515 topic groups."""

    def __init__(self):
        self.proc, self.name, self.groups, self.started, self.error = None, None, [], 0.0, None
        self.handed = True
        self.lock = threading.Lock()

    def running(self):
        return bool(self.proc and self.proc.poll() is None)

    def free_gb(self):
        return shutil.disk_usage(REC_DIR).free / 1e9

    def start(self, groups):
        with self.lock:
            if self.running():
                return "Already recording"
            groups = [g for g in REC_TOPICS if g in groups]
            if not any(g in groups for g in ("color", "depth", "points", "lidar")):
                return "Choose color, depth, points or Go2 lidar"
            if not os.path.isdir(REC_DIR):
                return f"{REC_DIR} does not exist"
            if self.free_gb() < REC_MIN_FREE_GB:
                return f"Less than {REC_MIN_FREE_GB} GB free on disk"
            self.name = time.strftime("l515_%Y%m%d_%H%M%S_") + "_".join(g for g in groups if g != "pose")
            topics = " ".join(t for g in groups for t in REC_TOPICS[g])
            env = dict(os.environ, HOME="/tmp", ROS_LOG_DIR="/tmp/roslog_l515_rec")
            self.proc = subprocess.Popen(
                ["bash", "-c", f"{ROS_SETUP} && exec ros2 bag record -o {REC_DIR}/{self.name} {topics}"],
                env=env, stdout=subprocess.DEVNULL, stderr=open("/tmp/l515_rec.log", "w"))
            self.groups, self.started, self.error, self.handed = groups, time.time(), None, False
            return None

    def stop(self):
        with self.lock:
            if self.running():
                self.proc.send_signal(signal.SIGINT)   # lets rosbag2 finish writing metadata
                try:
                    self.proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
            self.hand_over()

    def hand_over(self):
        if self.handed or not self.name or self.running():
            return
        self.handed = True
        for d, _, files in os.walk(os.path.join(REC_DIR, self.name)):
            for f in [d] + [os.path.join(d, x) for x in files]:
                try:
                    os.chown(f, HOST_UID, HOST_UID)
                except OSError:
                    pass

    @staticmethod
    def dir_mb(path):
        return round(sum(os.path.getsize(os.path.join(d, f)) for d, _, fs in os.walk(path) for f in fs) / 1e6, 1)

    def status(self):
        running = self.running()
        if not running:
            self.hand_over()
            if self.proc and self.error is None and time.time() - self.started < 5:
                self.error = "Recorder exited right away; see /tmp/l515_rec.log in the container"
        past, free = [], None
        if os.path.isdir(REC_DIR):
            free = round(self.free_gb())
            names = sorted((n for n in os.listdir(REC_DIR) if n.startswith("l515_")), reverse=True)[:8]
            past = [{"name": n, "mb": self.dir_mb(os.path.join(REC_DIR, n))} for n in names]
        if running and free is not None and free < REC_MIN_FREE_GB / 2:
            self.error = "Stopped: disk almost full"
            threading.Thread(target=self.stop, daemon=True).start()
        return {"running": running, "name": self.name, "groups": self.groups,
                "secs": round(time.time() - self.started) if running else 0,
                "mb": self.dir_mb(os.path.join(REC_DIR, self.name)) if running else 0,
                "error": self.error, "past": past, "free_gb": free}


class Live:
    """Latest frame from the receiver; page images are rendered on demand, once per frame."""

    def __init__(self, max_range):
        self.max_range = max_range
        self.cond = threading.Condition()
        self.frame, self.color, self.t, self.seq, self.fps = None, None, 0.0, 0, 0.0
        self.cache = {}
        self.ros_points = 0

    def update(self, frame, color, ros_points):
        with self.cond:
            now = time.time()
            if self.t and now > self.t:
                self.fps = 0.8 * self.fps + 0.2 / (now - self.t)
            self.frame, self.color, self.t, self.ros_points = frame, color, now, ros_points
            self.seq += 1
            self.cache = {}
            self.cond.notify_all()

    def get(self, kind, after):
        with self.cond:
            self.cond.wait_for(lambda: self.frame is not None and self.t > after, timeout=5)
            if self.frame is None or self.t <= after:
                return None, 0.0
            frame, color, t, seq = self.frame, self.color, self.t, self.seq
            data = self.cache.get(kind)
        if data is None:
            if kind == "color":
                data = frame.color_jpeg
            elif kind == "depth":
                data = depth_jpeg(frame.depth, frame.depth_scale)
            else:
                data = cloud_bin(frame, color, self.max_range)
            with self.cond:
                if self.seq == seq:
                    self.cache[kind] = data
        return data, t

    def status(self):
        with self.cond:
            f = self.frame
            s = {"live": bool(f is not None and time.time() - self.t < 3), "fps": round(self.fps, 1),
                 "age": round(time.time() - self.t, 1) if self.t else None, "ros_points": self.ros_points,
                 "rx_time": self.t, "pi_time": f.timestamp_ns / 1e9 if f is not None else None}
            if f is not None:
                s.update(depth=f"{f.depth.shape[1]}x{f.depth.shape[0]}",
                         color=f"{self.color.shape[1]}x{self.color.shape[0]}" if self.color is not None else None,
                         depth_scale=f.depth_scale)
            return s


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def can_control(self):
        try:
            ip = ipaddress.ip_address(self.client_address[0])
        except ValueError:
            return False
        return any(ip in n for n in CONTROL_NETS)

    def send_body(self, code, body, ctype, extra=()):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for k, v in extra:
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def send_json(self, code, obj):
        self.send_body(code, json.dumps(obj).encode(), "application/json")

    def do_GET(self):
        app = self.server.app
        url = urllib.parse.urlparse(self.path)
        kind = {"/color.jpg": "color", "/depth.jpg": "depth", "/cloud.bin": "cloud"}.get(url.path)
        if url.path == "/":
            self.send_body(200, PAGE.encode(), "text/html; charset=utf-8")
        elif url.path == "/status.json":
            self.send_json(200, {"camera": app.live.status(), "rec": app.rec.status(),
                                 "can_record": self.can_control()})
        elif kind:
            try:
                after = float(urllib.parse.parse_qs(url.query).get("after", ["0"])[0])
            except ValueError:
                after = 0.0
            data, t = app.live.get(kind, after)
            if data is None:
                self.send_response(204)
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                return
            ctype = "application/octet-stream" if kind == "cloud" else "image/jpeg"
            self.send_body(200, data, ctype, [("X-Frame-T", repr(t))])
        else:
            self.send_error(404)

    def do_POST(self):
        app = self.server.app
        if self.path != "/record":
            return self.send_error(404)
        if not self.can_control():
            return self.send_json(403, {"error": "Recording is allowed from localhost and 172.17.0.0/18 only"})
        try:
            req = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
        except ValueError:
            return self.send_json(400, {"error": "Bad JSON"})
        if req.get("action") == "start":
            err = app.rec.start(req.get("groups", []))
        else:
            app.rec.stop()
            err = None
        self.send_json(400 if err else 200, {"error": err, "rec": app.rec.status()})


class WebApp:
    def __init__(self, port, bind="0.0.0.0", max_range=9.0):
        self.live = Live(max_range)
        self.rec = Recorder()
        self.server = ThreadingHTTPServer((bind, port), Handler)
        self.server.daemon_threads = True
        self.server.app = self
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def update(self, frame, color, ros_points):
        self.live.update(frame, color, ros_points)

    def close(self):
        self.rec.stop()
        self.server.shutdown()


PAGE = r"""<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Go2 L515</title>
<style>
:root{--bg:#111418;--panel:#1a1f25;--line:#2a313a;--fg:#e6e8eb;--dim:#8b949e;--accent:#4cc38a;--bad:#e5534b}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:14px system-ui,sans-serif}
header{display:flex;gap:14px;align-items:center;padding:10px 16px;border-bottom:1px solid var(--line);flex-wrap:wrap}
header h1{font-size:16px;margin:0}header a{color:var(--dim)}.chip{padding:2px 8px;border-radius:10px;background:var(--line);font-size:12px}
.chip.live{background:#1f4d36;color:var(--accent)}.chip.dead{background:#4d1f1f;color:var(--bad)}#stat{color:var(--dim);font:12px monospace}
main{display:grid;grid-template-columns:repeat(auto-fit,minmax(420px,1fr));gap:12px;padding:12px}
section{background:var(--panel);border:1px solid var(--line);border-radius:6px;padding:10px}
section h2{font-size:13px;margin:0 0 8px;color:var(--dim);font-weight:600;display:flex;justify-content:space-between;align-items:center}
img,canvas{width:100%;display:block;background:#000;border-radius:4px}canvas{aspect-ratio:4/3;cursor:grab}
label{margin-right:12px;white-space:nowrap}button{background:var(--line);color:var(--fg);border:1px solid #3a434e;border-radius:4px;padding:5px 12px;cursor:pointer}
button.rec{background:#5a1f1f}.err{color:var(--bad)}.dim{color:var(--dim)}table{border-collapse:collapse;font:12px monospace}td{padding:1px 10px 1px 0}
</style></head><body>
<header><h1>Go2 L515</h1><span id="chip" class="chip">camera …</span><span id="stat"></span>
<a id="dash" href="#">Go2 dashboard (8080)</a></header>
<main>
<section><h2>Color</h2><img id="color" alt=""></section>
<section><h2>Depth <span class="dim">red = near, blue = 5 m+, black = no return</span></h2><img id="depth" alt=""></section>
<section><h2><label><input type="checkbox" id="cloudOn"> Point cloud preview</label>
<span class="dim" id="cloudInfo">drag to rotate, click then wheel to zoom, double-click to reset</span></h2>
<canvas id="cloud" width="640" height="480"></canvas></section>
<section><h2>Record L515 topics (ROS 2 bag in go2_recordings/)</h2>
<div><label><input type="checkbox" name="g" value="color" checked> color (JPEG, ~2 GB/h)</label>
<label><input type="checkbox" name="g" value="depth" checked> depth (16-bit, ~20 GB/h)</label></div>
<div><label><input type="checkbox" name="g" value="points"> point cloud (heavy, ~30 GB/h; it can be rebuilt from depth)</label></div>
<div><label><input type="checkbox" name="g" value="lidar" checked> Go2 lidar (/point_cloud2, ~8 GB/h)</label></div>
<div><label><input type="checkbox" name="g" value="pose" checked> robot pose (/tf, /tf_static, /odom)</label></div>
<p><button id="recBtn">Start recording</button> <span id="recStat"></span></p>
<table id="past"></table></section>
</main>
<script>
const $=id=>document.getElementById(id), sleep=ms=>new Promise(r=>setTimeout(r,ms));
$('dash').href=location.protocol+'//'+location.hostname+':8080/';
async function imgLoop(kind){const img=$(kind);let after=0;
 for(;;){try{const r=await fetch('/'+kind+'.jpg?after='+after,{cache:'no-store'});
  if(r.status===200){after=parseFloat(r.headers.get('X-Frame-T'));const u=URL.createObjectURL(await r.blob());
   await new Promise(res=>{img.onload=img.onerror=res;img.src=u});URL.revokeObjectURL(u);}
  else if(r.status!==204)await sleep(1000);}catch(e){await sleep(1000);}}}
imgLoop('color');imgLoop('depth');

let cloud=null,yaw=0,pitch=0,dist=2.5,drag=null;const cv=$('cloud'),ctx=cv.getContext('2d');
function draw(){const W=cv.width,H=cv.height,im=ctx.createImageData(W,H),px=im.data,zb=new Float32Array(W*H).fill(1e9);
 for(let i=3;i<px.length;i+=4)px[i]=255;
 if(cloud){const {n,xyz,rgb}=cloud,cy=Math.cos(yaw),sy=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch),f=H*0.9,C=2.0;
  for(let i=0;i<n;i++){const x=xyz[3*i],y=xyz[3*i+1],z=xyz[3*i+2]-C;
   const x1=cy*x+sy*z,z1=-sy*x+cy*z,y2=cp*y-sp*z1,z2=sp*y+cp*z1+dist;if(z2<0.05)continue;
   const u=Math.round(W/2+f*x1/z2),v=Math.round(H/2+f*y2/z2);
   for(let dv=0;dv<2;dv++)for(let du=0;du<2;du++){const uu=u+du,vv=v+dv;if(uu<0||vv<0||uu>=W||vv>=H)continue;
    const k=vv*W+uu;if(z2>=zb[k])continue;zb[k]=z2;px[4*k]=rgb[3*i];px[4*k+1]=rgb[3*i+1];px[4*k+2]=rgb[3*i+2];}}}
 ctx.putImageData(im,0,0);}
cv.onmousedown=e=>{drag=[e.clientX,e.clientY];cv.style.cursor='grabbing'};
window.onmouseup=()=>{drag=null;cv.style.cursor='grab'};
window.onmousemove=e=>{if(!drag)return;yaw+=(e.clientX-drag[0])*0.01;pitch=Math.max(-1.5,Math.min(1.5,pitch-(e.clientY-drag[1])*0.01));drag=[e.clientX,e.clientY];draw()};
cv.tabIndex=0;cv.onwheel=e=>{if(document.activeElement!==cv)return;e.preventDefault();dist=Math.max(0.2,Math.min(15,dist*(e.deltaY>0?1.1:0.9)));draw()};
cv.ondblclick=()=>{yaw=0;pitch=0;dist=2.5;draw()};
async function cloudLoop(){let after=0;draw();
 for(;;){if(!$('cloudOn').checked){await sleep(300);continue;}
  try{const r=await fetch('/cloud.bin?after='+after,{cache:'no-store'});
   if(r.status===200){after=parseFloat(r.headers.get('X-Frame-T'));const b=await r.arrayBuffer(),n=new DataView(b).getUint32(0,true);
    cloud={n,xyz:new Float32Array(b,4,n*3),rgb:new Uint8Array(b,4+n*12,n*3)};$('cloudInfo').textContent=n+' preview points';draw();}
  }catch(e){await sleep(1000);}await sleep(300);}}
cloudLoop();

let rec=null;
function groups(){return [...document.querySelectorAll('input[name=g]:checked')].map(e=>e.value)}
$('recBtn').onclick=async()=>{const body=rec&&rec.running?{action:'stop'}:{action:'start',groups:groups()};
 const r=await fetch('/record',{method:'POST',body:JSON.stringify(body)}),j=await r.json();
 if(j.error)$('recStat').innerHTML='<span class="err">'+j.error+'</span>';if(j.rec)showRec(j.rec);};
function showRec(s){rec=s;$('recBtn').textContent=s.running?'Stop recording':'Start recording';$('recBtn').className=s.running?'rec':'';
 if(s.running)$('recStat').textContent='● '+s.name+'  '+s.secs+' s  '+s.mb+' MB';
 else if(s.error)$('recStat').innerHTML='<span class="err">'+s.error+'</span>';
 else $('recStat').textContent=s.free_gb!=null?s.free_gb+' GB free':'';
 $('past').innerHTML=s.past.map(p=>'<tr><td>'+p.name+'</td><td>'+p.mb+' MB</td></tr>').join('');}
async function statusLoop(){for(;;){try{const j=await(await fetch('/status.json',{cache:'no-store'})).json(),c=j.camera;
  $('chip').textContent=c.live?'camera live':'camera dead';$('chip').className='chip '+(c.live?'live':'dead');
  $('stat').textContent=c.depth?`${c.fps} fps · depth ${c.depth} · color ${c.color} · ${c.ros_points} pts/frame on /go2/l515/points · scale ${c.depth_scale.toPrecision(3)} m`:'waiting for the Pi sender';
  showRec(j.rec);$('recBtn').disabled=!j.can_record;if(!j.can_record)$('recStat').textContent='view only from this network';
 }catch(e){$('chip').textContent='page offline';$('chip').className='chip dead';}await sleep(1000);}}
statusLoop();
</script></body></html>
"""
