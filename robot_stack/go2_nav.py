"""Path planning on the lab map, for go_to.

Plans on a 10 cm grid built from the mapper's 5 cm cells: walls and furniture (and anything not
mapped yet) are blocked, grown by the dog's half width, and cells near obstacles cost more, so routes
keep to the middle of aisles. A* finds the route; it is then cut down to a few straight legs.

Obstacles the dog runs into that are not on the map (a chair moved, a person) can be added for a
while with block(), so the next plan goes around them.
"""
import heapq
import math
import threading
import time

import cv2
import numpy as np

CELL = 0.10                # m per planning cell
ROBOT_R = 0.28             # m: no part of the route closer than this to an obstacle (the Go2 is ~0.31 m wide)
ROBOT_R_TIGHT = 0.20       # fall back to this if nothing fits with ROBOT_R
CLEAR_PREF = 0.60          # m: prefer staying at least this far from obstacles
NEAR_COST = 6.0            # extra cost per cell at ROBOT_R, falling to 0 at CLEAR_PREF
GOAL_SNAP_M = 1.2          # a goal on an obstacle moves to the nearest reachable cell within this
ESCAPE_M = 0.8             # a start the map shows as blocked steps out to open floor within this
LEG_MAX_M = 2.5            # longest straight leg after shortcutting
BLOCK_S = 120.0            # temporary obstacles last this long
SQ2 = math.sqrt(2)


class Planner:
    def __init__(self, mapper):
        self.mapper = mapper
        self.lock = threading.Lock()
        self.key = None
        self.blocks = []           # (expiry, x, y, r) temporary obstacles in map frame
        self.last = None           # last plan, for the console: {"goal", "path", "t"}
        self.live = None           # what the lidar sees now: (obs xy, free xy, robot x, y, radius, t)

    # ---------- grid ----------
    def _grid(self):
        """Rebuild the planning grid when the map or the temporary obstacles changed."""
        now = time.time()
        self.blocks = [b for b in self.blocks if b[0] > now]
        key = (self.mapper.version, self.mapper.saved_at, tuple(self.blocks), self.live and self.live[5])
        if key == self.key:
            return
        from go2_mapper import OCC_SCORE, RES
        with self.mapper.lock:
            score, seen = self.mapper.score.copy(), self.mapper.seen.copy()
        k = int(round(CELL / RES))
        rows, cols = np.nonzero(seen)
        if len(rows) == 0:
            self.occ = None
            self.key = key
            return
        r0, r1 = (rows.min() // k - 2) * k, (rows.max() // k + 3) * k
        c0, c1 = (cols.min() // k - 2) * k, (cols.max() // k + 3) * k
        r0, c0 = max(r0, 0), max(c0, 0)
        occ5 = score[r0:r1, c0:c1] >= OCC_SCORE
        seen5 = seen[r0:r1, c0:c1]
        h, w = occ5.shape[0] // k, occ5.shape[1] // k
        occ = occ5[:h * k, :w * k].reshape(h, k, w, k).any(axis=(1, 3))
        known = seen5[:h * k, :w * k].reshape(h, k, w, k).any(axis=(1, 3))
        # single stray obstacle cells with no neighbours are lidar noise, not furniture
        nb = cv2.filter2D(occ.astype(np.uint8), -1, np.ones((3, 3), np.float32), borderType=cv2.BORDER_CONSTANT)
        occ &= nb > 1
        self.origin = (self.mapper.origin[0] + c0 * RES, self.mapper.origin[1] + r0 * RES)
        if self.live is not None:
            # near the dog, the current lidar view wins over the saved map: open floor it sees now
            # clears old obstacles, and new obstacles are added
            lo, lf, rx, ry, rad, _ = self.live
            for pts, val in ((lf, False), (lo, True)):
                if not len(pts):
                    continue
                a = np.asarray(pts, dtype=np.float64)
                a = a[np.hypot(a[:, 0] - rx, a[:, 1] - ry) <= rad]
                cc = ((a[:, 0] - self.origin[0]) / CELL).astype(int)
                rr = ((a[:, 1] - self.origin[1]) / CELL).astype(int)
                ok = (rr >= 0) & (rr < h) & (cc >= 0) & (cc < w)
                occ[rr[ok], cc[ok]] = val
                known[rr[ok], cc[ok]] = True
        for _, bx, by, br in self.blocks:
            yy, xx = np.mgrid[0:h, 0:w]
            occ |= np.hypot((xx + 0.5) * CELL + self.origin[0] - bx, (yy + 0.5) * CELL + self.origin[1] - by) < br
        blocked = occ | ~known
        # distance (m) from each cell to the nearest blocked cell
        self.dist = cv2.distanceTransform((~blocked).astype(np.uint8), cv2.DIST_L2, 5) * CELL
        self.occ, self.known = occ, known
        self.key = key

    def cell(self, x, y):
        return int((y - self.origin[1]) / CELL), int((x - self.origin[0]) / CELL)

    def xy(self, r, c):
        return self.origin[0] + (c + 0.5) * CELL, self.origin[1] + (r + 0.5) * CELL

    def inside(self, r, c):
        return 0 <= r < self.dist.shape[0] and 0 <= c < self.dist.shape[1]

    # ---------- queries ----------
    def clearance(self, x, y):
        """Distance (m) from (x, y) to the nearest obstacle or unmapped area; 0 off the map."""
        with self.lock:
            self._grid()
            if self.occ is None:
                return 0.0
            r, c = self.cell(x, y)
            return float(self.dist[r, c]) if self.inside(r, c) else 0.0

    def nearest_free(self, x, y, radius=GOAL_SNAP_M, rr=ROBOT_R):
        """The reachable cell closest to (x, y) within radius, as (x, y), or None."""
        with self.lock:
            self._grid()
            return self._nearest_free(x, y, radius, rr)

    def _nearest_free(self, x, y, radius, rr):
        if self.occ is None:
            return None
        r, c = self.cell(x, y)
        n = int(radius / CELL) + 1
        best = None
        for dr in range(-n, n + 1):
            for dc in range(-n, n + 1):
                rr_, cc_ = r + dr, c + dc
                if not self.inside(rr_, cc_) or self.dist[rr_, cc_] < rr:
                    continue
                d = math.hypot(dr, dc) * CELL
                if d <= radius and (best is None or d < best[0]):
                    best = (d, rr_, cc_)
        return self.xy(best[1], best[2]) if best else None

    def stand_for(self, tx, ty, dist=0.8):
        """Where the dog should stand to look at a target point: open floor about `dist` away, facing it.
        Returns (x, y, yaw) or None."""
        with self.lock:
            self._grid()
            if self.occ is None:
                return None
            best = None
            for k in range(36):
                a = k * math.pi / 18
                for d in (dist, dist + 0.3, dist + 0.6, max(0.4, dist - 0.3)):
                    x, y = tx + d * math.cos(a), ty + d * math.sin(a)
                    r, c = self.cell(x, y)
                    if not self.inside(r, c) or self.dist[r, c] < ROBOT_R:
                        continue
                    cost = abs(d - dist) - 0.2 * min(self.dist[r, c], 1.0)
                    if best is None or cost < best[0]:
                        best = (cost, x, y)
            if best is None:
                return None
            _, x, y = best
            return x, y, math.atan2(ty - y, tx - x)

    def set_live(self, obs, free, rx, ry, radius):
        """The lidar's current view (map-frame xy of obstacle and open-floor cells around the robot)."""
        with self.lock:
            self.live = (obs, free, rx, ry, radius, time.time())

    def raycast(self, x, y, yaw, max_m=4.0, min_m=0.3):
        """First mapped obstacle straight ahead of (x, y) facing yaw, as (x, y), or None."""
        from go2_mapper import OCC_SCORE, RES
        m = self.mapper
        for i in range(int(min_m / RES), int(max_m / RES)):
            px, py = x + math.cos(yaw) * i * RES, y + math.sin(yaw) * i * RES
            c, r = int((px - m.origin[0]) / RES), int((py - m.origin[1]) / RES)
            if not (0 <= r < m.score.shape[0] and 0 <= c < m.score.shape[1]):
                return None
            if m.score[r, c] >= OCC_SCORE:
                return round(px, 2), round(py, 2)
        return None

    def block(self, x, y, r=0.25):
        """Treat a spot as an obstacle for a while (something the dog ran into that isn't on the map)."""
        with self.lock:
            self.blocks.append((time.time() + BLOCK_S, round(x, 2), round(y, 2), r))

    def plan(self, sx, sy, gx, gy):
        """Route from (sx, sy) to (gx, gy) in map metres. Returns {"path": [[x, y], ...], "length_m",
        "goal": [x, y] actually used} or {"error": reason}."""
        with self.lock:
            self._grid()
            if self.occ is None:
                return {"error": "no map"}
            for rr in (ROBOT_R, ROBOT_R_TIGHT):
                out = self._plan(sx, sy, gx, gy, rr)
                if "path" in out:
                    out["tight"] = rr < ROBOT_R
                    break
            self.last = dict(out, t=time.time(), start=[sx, sy], want=[gx, gy])
            return out

    def _plan(self, sx, sy, gx, gy, rr):
        g = self._nearest_free(gx, gy, GOAL_SNAP_M, rr)
        if g is None:
            return {"error": f"no open floor within {GOAL_SNAP_M} m of the goal"}
        gr, gc = self.cell(*g)
        sr, sc = self.cell(sx, sy)
        if not self.inside(sr, sc):
            return {"error": "the robot is off the map"}
        lead = []
        if self.dist[sr, sc] < min(rr, 0.12):
            # The map shows the dog's own spot as blocked (it's close to furniture, or lidar noise):
            # step straight out to the nearest open floor and plan from there. The lidar check while
            # driving still stops it if something is really in the way.
            out = self._nearest_free(sx, sy, ESCAPE_M, rr)
            if out is None:
                return {"error": f"the map shows the dog boxed in: no open floor within {ESCAPE_M} m"}
            lead = [[round(sx, 2), round(sy, 2)]]
            sx, sy = out
            sr, sc = self.cell(sx, sy)
        free = self.dist >= rr
        # the dog may be standing close to something: let it leave through any cell it can reach
        # within half a metre, even if that cell is tighter than rr
        start_ok = self.dist >= min(rr, 0.12)
        cost_near = NEAR_COST * np.clip((CLEAR_PREF - self.dist) / (CLEAR_PREF - rr), 0, 1)
        H, W = free.shape
        INF = float("inf")
        best = np.full((H, W), INF)
        prev = -np.ones((H, W), np.int64)
        best[sr, sc] = 0.0
        heap = [(math.hypot(gr - sr, gc - sc), 0.0, sr, sc)]
        steps = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1), (-1, -1, SQ2), (-1, 1, SQ2), (1, -1, SQ2), (1, 1, SQ2)]
        found = False
        while heap:
            _, d, r, c = heapq.heappop(heap)
            if d > best[r, c]:
                continue
            if r == gr and c == gc:
                found = True
                break
            near_start = math.hypot(r - sr, c - sc) * CELL < 0.5
            for dr, dc, w in steps:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < H and 0 <= nc < W):
                    continue
                if not (free[nr, nc] or (near_start and start_ok[nr, nc])):
                    continue
                nd = d + w * (1.0 + cost_near[nr, nc])
                if nd < best[nr, nc]:
                    best[nr, nc] = nd
                    prev[nr, nc] = r * W + c
                    heapq.heappush(heap, (nd + math.hypot(gr - nr, gc - nc), nd, nr, nc))
        if not found:
            return {"error": "no route: the goal is not connected to where the robot is on the map"}
        cells = [(gr, gc)]
        while cells[-1] != (sr, sc):
            k = prev[cells[-1]]
            cells.append((int(k // W), int(k % W)))
        cells.reverse()
        legs = self._shortcut(cells, rr)
        pts = lead + [[round(sx, 2), round(sy, 2)]] + [[round(v, 2) for v in self.xy(r, c)] for r, c in legs[1:]]
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
        return {"path": pts, "length_m": round(length, 2), "goal": pts[-1]}

    def _line_free(self, a, b, rr, skip=0.0):
        """Straight line between cells a and b stays rr clear (ignoring its first `skip` metres)."""
        (r0, c0), (r1, c1) = a, b
        n = int(max(abs(r1 - r0), abs(c1 - c0)) * 2) + 1
        length = math.hypot(r1 - r0, c1 - c0) * CELL
        for i in range(n + 1):
            if length * i / n < skip:
                continue
            r = int(round(r0 + (r1 - r0) * i / n))
            c = int(round(c0 + (c1 - c0) * i / n))
            if self.dist[r, c] < rr:
                return False
        return True

    def _shortcut(self, cells, rr):
        """Fewest straight legs (each at most LEG_MAX_M) that stay rr clear; the first leg may pass
        tighter spots in its first half metre, where the dog may be starting close to something."""
        out = [cells[0]]
        i = 0
        max_cells = LEG_MAX_M / CELL
        while i < len(cells) - 1:
            j = len(cells) - 1
            while j > i + 1:
                a, b = cells[i], cells[j]
                if (math.hypot(b[0] - a[0], b[1] - a[1]) <= max_cells
                        and self._line_free(a, b, rr, skip=0.5 if i == 0 else 0.0)):
                    break
                j -= 1
            out.append(cells[j])
            i = j
        return out
