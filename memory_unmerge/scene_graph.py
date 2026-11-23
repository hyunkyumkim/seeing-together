"""Load the lab places, keep the scene graph, and hand a goal to the existing planner.

``Planner.plan(sx, sy, gx, gy)`` in ``robot_stack/go2_nav.py`` is not modified.
``goal_for_planner`` returns the place stand pose, which is the same (x, y)
``go2_agent._goal`` already sends. The object's own pose is not a navigation goal:
the dog stops at the desk, not on the object.

Persistence is one JSON document. Saving and loading restores places, objects,
and observations, including ids.
"""

import importlib.util
import json
from pathlib import Path

from .models import FRAME_ID, ObjectRecord, Observation, Place
from .synthetic import build_objects, places_from_file

HERE = Path(__file__).resolve().parent
DEFAULT_PLACES = HERE.parent / "robot_stack" / "go2_maps" / "lab.places.json"
DEFAULT_GRID = HERE.parent / "robot_stack" / "go2_maps" / "lab.grid.npz"
DEFAULT_PGM = HERE.parent / "robot_stack" / "go2_maps" / "lab.pgm"
DEFAULT_YAML = HERE.parent / "robot_stack" / "go2_maps" / "lab.yaml"
FIXTURE = HERE / "fixtures" / "initial_scene.json"

MAP_ORIGIN_M = (-25.0, -25.0)   # cell (0, 0) of lab.grid.npz
MAP_RES_M = 0.05


class SceneGraphError(KeyError):
    pass


def _point_in_polygon(x, y, polygon):
    """Even-odd test. ``polygon`` is a list of [x, y]."""
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1 + 0.0) + x1):
            inside = not inside
    return inside


class SceneGraphManager:
    def __init__(self):
        self.places = {}
        self.objects = {}
        self.observations = {}
        self.maps = {
            "frame_id": FRAME_ID,
            "grid_npz": str(DEFAULT_GRID),
            "occupancy_pgm": str(DEFAULT_PGM),
            "occupancy_yaml": str(DEFAULT_YAML),
            "places_json": str(DEFAULT_PLACES),
            "origin_m": list(MAP_ORIGIN_M),
            "resolution_m": MAP_RES_M,
        }

    # ----- build ----------------------------------------------------------
    def load_places(self, path=None, place_ids=None):
        """Read ``lab.places.json``. By default only the four desks used for this memory."""
        path = Path(path or DEFAULT_PLACES)
        with open(path, encoding="utf-8") as f:
            raw = json.load(f)
        self.maps["places_json"] = str(path)
        if place_ids is None:
            selected = places_from_file(raw)
        else:
            missing = [i for i in place_ids if i not in raw]
            if missing:
                raise SceneGraphError(", ".join(missing))
            selected = _places_for(raw, place_ids)
        for place in selected.values():
            self.places[place.place_id] = place
        return self

    def load_initial(self, places_path=None):
        """Four desks plus the synthetic cup, bottle, and book on each."""
        path = Path(places_path or DEFAULT_PLACES)
        with open(path, encoding="utf-8") as f:
            raw = json.load(f)
        self.maps["places_json"] = str(path)
        self.places = places_from_file(raw)
        objects, observations = build_objects(self.places)
        for obj, obs in zip(objects, observations):
            self._insert(obj, obs)
        return self

    def register_object(self, obj, observation):
        """Add one known instance and the observation it came from."""
        if obj.object_id in self.objects:
            raise SceneGraphError(f"object {obj.object_id} already registered")
        if obj.place_id not in self.places:
            raise SceneGraphError(f"unknown place {obj.place_id}")
        if observation.object_id != obj.object_id:
            raise SceneGraphError("observation.object_id does not match the object")
        self._insert(obj, observation)
        return obj

    def _insert(self, obj, observation):
        self.objects[obj.object_id] = obj
        self.observations[observation.observation_id] = observation
        place = self.places[obj.place_id]
        if obj.object_id not in place.object_ids:
            place.object_ids.append(obj.object_id)

    def add_observation(self, observation):
        """Append evidence. A newer timestamp updates the current estimate.

        This is the hook for a later move or a new sighting. It does not search
        and it does not plan. If ``place_id`` on the object should change, set
        it on the object after resolving the new place yourself, then call
        ``move_object``.
        """
        obj = self.objects.get(observation.object_id)
        if obj is None:
            raise SceneGraphError(f"unknown object {observation.object_id}")
        if observation.observation_id in self.observations:
            raise SceneGraphError(f"observation {observation.observation_id} already stored")
        self.observations[observation.observation_id] = observation
        if observation.observation_id not in obj.observation_ids:
            obj.observation_ids.append(observation.observation_id)
        if observation.timestamp >= obj.last_seen:
            obj.last_seen = observation.timestamp
            obj.pose_3d = observation.observed_pose
            obj.status = "observed"
        return obj

    def move_object(self, object_id, place_id):
        """Point an instance at a different place. Does not change its pose."""
        obj = self.get_object(object_id)
        if place_id not in self.places:
            raise SceneGraphError(f"unknown place {place_id}")
        old = self.places[obj.place_id]
        if object_id in old.object_ids:
            old.object_ids.remove(object_id)
        obj.place_id = place_id
        if object_id not in self.places[place_id].object_ids:
            self.places[place_id].object_ids.append(object_id)
        return obj

    # ----- queries --------------------------------------------------------
    def get_object(self, object_id):
        try:
            return self.objects[object_id]
        except KeyError:
            raise SceneGraphError(object_id) from None

    def get_place(self, place_id):
        try:
            return self.places[place_id]
        except KeyError:
            raise SceneGraphError(place_id) from None

    def objects_in_place(self, place_id):
        place = self.get_place(place_id)
        return [self.objects[i] for i in place.object_ids]

    def find_category(self, category):
        return [o for o in self.objects.values() if o.category == category]

    def place_containing(self, x, y):
        """First place whose region contains (x, y), or None."""
        for place in self.places.values():
            if place.region and _point_in_polygon(x, y, place.region):
                return place
        return None

    def where_is(self, object_id):
        """Object, the place that contains it, and that place's navigation goal."""
        obj = self.get_object(object_id)
        place = self.get_place(obj.place_id)
        return {"object": obj, "place": place, "navigation_goal": place.navigation_goal}

    def goal_for_planner(self, object_id):
        """(x, y, yaw_rad) for the existing planner.

        x, y are the stand pose. Pass them to ``Planner.plan(sx, sy, x, y)``.
        yaw_rad is only for the final turn, the same split ``go2_agent._go_to`` uses.
        """
        goal = self.where_is(object_id)["navigation_goal"]
        return goal.x, goal.y, goal.yaw_rad()

    def plan_to_object(self, planner, start_xy, object_id):
        """Call the existing ``Planner.plan``. This function does not search a grid."""
        gx, gy, _yaw = self.goal_for_planner(object_id)
        return planner.plan(start_xy[0], start_xy[1], gx, gy)

    # ----- persistence ----------------------------------------------------
    def to_dict(self):
        return {
            "v": 1,
            "frame_id": FRAME_ID,
            "maps": self.maps,
            "places": {k: p.to_dict() for k, p in self.places.items()},
            "objects": {k: o.to_dict() for k, o in self.objects.items()},
            "observations": {k: o.to_dict() for k, o in self.observations.items()},
        }

    def save(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(path.suffix + ".tmp")
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, indent=1)
            f.write("\n")
        tmp.replace(path)
        return path

    @classmethod
    def load(cls, path):
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        g = cls()
        g.maps = data.get("maps", g.maps)
        g.places = {k: Place.from_dict(v) for k, v in data["places"].items()}
        g.objects = {k: ObjectRecord.from_dict(v) for k, v in data["objects"].items()}
        g.observations = {k: Observation.from_dict(v) for k, v in data["observations"].items()}
        return g


def _places_for(raw, place_ids):
    """Stand pose = navigation goal, target = place center, for arbitrary place ids."""
    from .synthetic import _region
    out = {}
    for place_id in place_ids:
        p = raw[place_id]
        target = p.get("target") or {"x": p["x"], "y": p["y"]}
        stand = Pose2D(float(p["x"]), float(p["y"]), float(p.get("yaw_deg", 0.0)))
        center = Pose2D(float(target["x"]), float(target["y"]), stand.yaw_deg)
        place = Place(place_id, place_id, center, stand, _region(center.x, center.y), [])
        place.saved_at = p.get("saved_at", "")
        out[place_id] = place
    return out


def load_legacy_memory(jsonl_path):
    """Read a ``memory/`` JSONL file with that package's validator. Does not copy it in.

    Legacy object rows are keyed by category (``obj:mug``), not by instance, so
    they are not inserted into the scene graph.
    """
    store_path = HERE.parent / "memory" / "store.py"
    spec = importlib.util.spec_from_file_location("seeing_memory_store", store_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    memory = mod.Memory()
    memory.ingest_file(str(jsonl_path))
    return memory


def legacy_object_labels(jsonl_path):
    """Category labels in an old memory file. These are not instance ids."""
    memory = load_legacy_memory(jsonl_path)
    return sorted(e["label"] for e in memory.entities.values() if e["kind"] == "object")


def main():
    import argparse
    ap = argparse.ArgumentParser(description="Build or query the initial scene graph")
    ap.add_argument("--places", type=Path, default=DEFAULT_PLACES)
    ap.add_argument("--save", type=Path, default=FIXTURE)
    ap.add_argument("--load", type=Path)
    ap.add_argument("--where", metavar="OBJECT_ID", help="for example object_07")
    a = ap.parse_args()
    if a.load:
        graph = SceneGraphManager.load(a.load)
    else:
        graph = SceneGraphManager().load_initial(a.places)
        graph.save(a.save)
        print(f"saved {a.save}  places={len(graph.places)} objects={len(graph.objects)}")
    if a.where:
        found = graph.where_is(a.where)
        goal = found["navigation_goal"]
        obj = found["object"]
        print(json.dumps({
            "object_id": obj.object_id,
            "category": obj.category,
            "place_id": found["place"].place_id,
            "navigation_goal": goal.to_dict(),
            "object_pose": obj.pose_3d.to_dict(),
        }, indent=1))


if __name__ == "__main__":
    main()
