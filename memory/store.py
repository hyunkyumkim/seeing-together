"""Shared memory for Seeing Together (draft v1, see schema.md).

Reads observations (JSONL, one object per line) from the glasses and the robot and keeps one
memory entry per entity: the latest state plus a short history. Standard library only.

Usage:
  python store.py examples/obs_example.jsonl                    # print all entities
  python store.py examples/obs_example.jsonl --where mug
  python store.py examples/obs_example.jsonl --timeline obj:mug
  python store.py examples/obs_example.jsonl --near 1.5 0.0 1.0 --since 1793434975
  python store.py examples/obs_example.jsonl --save memory.json
"""
import argparse
import json
import math

SCHEMA_VERSION = 1
HISTORY_MAX = 20
COMMON = ("v", "id", "src", "type", "t", "frame")
SOURCES = {"glasses", "robot", "mock"}
FRAMES = {"image", "map", "device"}
REQUIRED = {
    "tag": ("tag_id", "center", "img"),
    "imu": ("acc", "gyr"),
    "object": ("label", "pos"),
    "robot_pose": ("pos", "yaw"),
    "person": ("pos",),
}


class SchemaError(ValueError):
    pass


def validate(obs):
    """Raise SchemaError if an observation does not follow schema v1."""
    for f in COMMON:
        if f not in obs:
            raise SchemaError(f"{obs.get('id', '?')}: missing field '{f}'")
    if obs["v"] != SCHEMA_VERSION:
        raise SchemaError(f"{obs['id']}: unsupported version {obs['v']}")
    if obs["src"] not in SOURCES:
        raise SchemaError(f"{obs['id']}: unknown src '{obs['src']}'")
    if obs["frame"] not in FRAMES:
        raise SchemaError(f"{obs['id']}: unknown frame '{obs['frame']}'")
    if obs["type"] not in REQUIRED:
        raise SchemaError(f"{obs['id']}: unknown type '{obs['type']}'")
    for f in REQUIRED[obs["type"]]:
        if f not in obs:
            raise SchemaError(f"{obs['id']}: type '{obs['type']}' needs field '{f}'")
    if not isinstance(obs["t"], (int, float)):
        raise SchemaError(f"{obs['id']}: 't' must be a number (Unix seconds)")


def entity_of(obs):
    """Entity key and kind for an observation, or (None, None) if it is not stored as an entity."""
    typ = obs["type"]
    if typ == "object":
        return f"obj:{obs['label']}", "object"
    if typ == "person":
        return "person:wearer", "person"
    if typ == "robot_pose":
        return "robot:go2", "robot"
    if typ == "tag":
        return f"tag:{obs['tag_id']}", "tag"
    return None, None  # imu: a raw stream, not an entity


def _state(obs):
    pos = obs.get("pos", obs.get("center"))
    s = {"pos": pos, "frame": obs["frame"], "last_seen": obs["t"], "src": obs["src"],
         "conf": obs.get("conf"), "obs_id": obs["id"]}
    if obs["type"] == "robot_pose":
        s["yaw"] = obs["yaw"]
    return s


class Memory:
    def __init__(self):
        self.entities = {}
        self.ingested = 0
        self.skipped = 0  # valid observations that are not entities (imu)

    def ingest(self, obs):
        """Add one observation. Returns the entity key it updated, or None."""
        validate(obs)
        key, kind = entity_of(obs)
        if key is None:
            self.skipped += 1
            return None
        self.ingested += 1
        new = _state(obs)
        e = self.entities.get(key)
        if e is None:
            label = obs.get("label", key.split(":", 1)[1])
            self.entities[key] = {"entity": key, "kind": kind, "label": label, **new, "history": []}
            return key
        if new["last_seen"] >= e["last_seen"]:
            # newer: current state moves into history
            old = {k: e[k] for k in new if k in e}
            e["history"].insert(0, old)
            e.update(new)
        else:
            # late arrival: history only, kept newest first
            e["history"].append(new)
            e["history"].sort(key=lambda s: s["last_seen"], reverse=True)
        del e["history"][HISTORY_MAX:]
        return key

    def ingest_file(self, path):
        with open(path, encoding="utf-8") as f:
            for n, line in enumerate(f, 1):
                line = line.strip()
                if not line:
                    continue
                try:
                    obs = json.loads(line)
                except json.JSONDecodeError as ex:
                    raise SchemaError(f"{path}:{n}: not valid JSON ({ex})")
                self.ingest(obs)

    # queries -------------------------------------------------------------
    def last_seen(self, entity):
        e = self.entities.get(entity)
        return None if e is None else {k: v for k, v in e.items() if k != "history"}

    def where(self, label):
        hits = [self.last_seen(k) for k, e in self.entities.items() if e["label"] == label]
        return sorted(hits, key=lambda e: e["last_seen"], reverse=True)

    def near(self, pos, r, since=float("-inf")):
        """Entities in the map frame seen within r meters of pos at or after `since`."""
        out = []
        for k, e in self.entities.items():
            if e["frame"] != "map" or e["last_seen"] < since:
                continue
            d = math.dist(pos, e["pos"])
            if d <= r:
                out.append({**self.last_seen(k), "dist_m": round(d, 3)})
        return sorted(out, key=lambda e: e["dist_m"])

    def timeline(self, entity):
        e = self.entities.get(entity)
        if e is None:
            return []
        cur = {k: e[k] for k in ("pos", "frame", "last_seen", "src", "conf", "obs_id")}
        return [cur] + e["history"]

    def save(self, path):
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"v": SCHEMA_VERSION, "entities": self.entities}, f, indent=1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="+", help="observation JSONL files")
    ap.add_argument("--where", metavar="LABEL")
    ap.add_argument("--timeline", metavar="ENTITY")
    ap.add_argument("--near", nargs=3, type=float, metavar=("X", "Y", "R"))
    ap.add_argument("--since", type=float, default=float("-inf"))
    ap.add_argument("--save", metavar="PATH")
    a = ap.parse_args()

    m = Memory()
    for p in a.files:
        m.ingest_file(p)
    print(f"ingested {m.ingested} entity observations, skipped {m.skipped} non-entity (imu)")
    if a.where:
        result = m.where(a.where)
    elif a.timeline:
        result = m.timeline(a.timeline)
    elif a.near:
        result = m.near(a.near[:2], a.near[2], a.since)
    else:
        result = [m.last_seen(k) for k in sorted(m.entities)]
    for r in result:
        print(json.dumps(r))
    if a.save:
        m.save(a.save)
        print(f"saved {a.save}")


if __name__ == "__main__":
    main()
