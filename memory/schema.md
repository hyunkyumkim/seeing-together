# Shared memory schema (draft v1)

Status: draft for Q4 Task 2. Time and coordinate conventions still need to be confirmed with the
robot side.

Glasses and robot both write **observations** (one JSON object per line, JSONL). The shared
memory reads them and keeps one **memory entry** per entity (latest state plus a short history),
which the agent queries.

```
glasses stream -> laptop receiver -> tag detection -> observation --+
robot (mock for now) -----------------------------> observation --+--> shared memory --> queries
```

## 1. Observation

### Common fields (every observation)

| Field | Type | Meaning |
|---|---|---|
| `v` | int | Schema version (1) |
| `id` | string | Unique observation ID, e.g. `g-000412` (prefix: `g` glasses, `r` robot, `m` mock) |
| `src` | `glasses` / `robot` / `mock` | Producer |
| `type` | see below | Kind of observation |
| `t` | float | **Laptop clock, Unix time in seconds**, when the observation was received |
| `t_dev` | float, optional | Device clock (e.g. glasses sensor time), for latency analysis only |
| `frame` | `image` / `map` / `device` | Coordinate frame: `image` = pixels, `map` = Go2 map in meters, `device` = sensor axes |

### Types

| `type` | Producer | Fields |
|---|---|---|
| `tag` | glasses | `tag_id`, `center` [x, y] px, `corners` 4 x [x, y] px, `img` [w, h], `frame_seq` |
| `imu` | glasses | `acc` [x, y, z] m/s^2, `gyr` [x, y, z] rad/s (downsampled to 10 Hz) |
| `object` | robot | `label`, `pos` [x, y] m, `conf` 0..1 |
| `robot_pose` | robot | `pos` [x, y] m, `yaw` rad |
| `person` | derived | `pos` [x, y] m, `conf` 0..1, `from` (list of observation IDs used) |

`tag` observations are in pixels. A `person` observation in map coordinates is derived from them
with the tag position table (section 4).

## 2. Memory entry (one per entity)

Entity keys: `obj:<label>` (e.g. `obj:mug`), `person:wearer`, `robot:go2`, `tag:<id>`.

| Field | Meaning |
|---|---|
| `entity`, `kind`, `label` | Entity key, kind (`object` / `person` / `robot` / `tag`), label |
| `pos`, `frame` | Last known position and its frame |
| `last_seen`, `src`, `conf` | Time of the last observation, its producer, confidence |
| `obs_id` | ID of the observation behind the current state |
| `history` | Previous states, newest first, at most 20 |

Update rules:

- A newer observation (larger `t`) replaces the current state; the old state moves to `history`.
- A late observation (smaller `t` than `last_seen`) only goes into `history`, in time order.

## 3. Queries (first version)

| Query | Returns |
|---|---|
| `last_seen(entity)` | Current state of one entity |
| `where(label)` | Entities with that label, newest first |
| `near(pos, r, since)` | Entities seen within `r` meters of `pos` after time `since` |
| `timeline(entity)` | Current state plus history, newest first |

## 4. Tag position table (`tags.json`, to be filled per test site)

Maps each AprilTag ID to its position in the map, so a glasses `tag` observation can become a
`person` position:

```json
{"frame": "map", "tags": {"0": {"pos": [0.0, 0.0], "height_m": 1.2}, "1": {"pos": [2.0, 0.0], "height_m": 1.2}}}
```

## 5. Open questions

- Time base: laptop clock in Unix seconds for `t` (assumed). Does the robot side use the same clock?
- Coordinates: Go2 map frame (`lab.grid.npz`, meters, origin (-25, -25)) for `pos` (assumed).
- Who maintains `tags.json` for each test site.

Example observations: `examples/obs_example.jsonl`.
