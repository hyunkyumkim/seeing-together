"""Synthetic object instances laid out on the four recorded desks.

There is no object detector yet, and ``memory/`` has no real object poses in
the lab frame. This fixture does not pretend otherwise. Each object's x, y
is the desk ``target`` already stored in ``lab.places.json``, shifted a fixed
amount across the direction from the stand pose to that target. z is a
stand-in desk height, not a measurement. ``source`` on every observation is
``synthetic``.
"""

import math
from datetime import datetime, timezone

from .models import FRAME_ID, ObjectRecord, Observation, Place, Pose2D, Pose3D

# Same ids the console and Planner.plan already use.
DESKS = (
    ("desk_khang", "Khang's desk"),
    ("desk_oliver", "Oliver's desk"),
    ("desk_kang", "Kang's desk"),
    ("desk_joseph", "Joseph's desk"),
)
CATEGORIES = ("cup", "bottle", "book")
ACROSS_M = (-0.18, 0.0, 0.18)   # meters, across the view, centered on the saved target
REGION_HALF_M = 0.40
FIXTURE_Z_M = 0.75              # not measured


def _stamp(saved_at):
    """Unix seconds from the place's own saved_at string. UTC, so it is stable."""
    if not saved_at:
        return 0.0
    dt = datetime.strptime(saved_at, "%Y-%m-%d %H:%M:%S").replace(tzinfo=timezone.utc)
    return dt.timestamp()


def _region(tx, ty):
    h = REGION_HALF_M
    return [[tx - h, ty - h], [tx + h, ty - h], [tx + h, ty + h], [tx - h, ty + h]]


def _across(stand, target, yaw_deg):
    vx, vy = target[0] - stand[0], target[1] - stand[1]
    n = math.hypot(vx, vy)
    if n < 1e-6:
        yaw = math.radians(yaw_deg)
        vx, vy = math.cos(yaw), math.sin(yaw)
        n = 1.0
    # Perpendicular to the direction the dog looks, in the map plane.
    return -vy / n, vx / n


def places_from_file(raw):
    """The four desks, using the stand pose as the navigation goal and the target as the place pose."""
    places = {}
    for place_id, name in DESKS:
        if place_id not in raw:
            raise KeyError(f"{place_id} is not in the places file")
        p = raw[place_id]
        target = p.get("target") or {"x": p["x"], "y": p["y"]}
        stand = Pose2D(float(p["x"]), float(p["y"]), float(p.get("yaw_deg", 0.0)))
        center = Pose2D(float(target["x"]), float(target["y"]), stand.yaw_deg)
        places[place_id] = Place(
            place_id, name, center, stand, _region(center.x, center.y), [],
        )
        places[place_id].saved_at = p.get("saved_at", "")
    return places


def build_objects(places):
    """Three instances on each desk: cup, bottle, book. Ids are object_01 .. object_12."""
    objects = []
    observations = []
    n = 0
    for place_id, _name in DESKS:
        place = places[place_id]
        ax, ay = _across(
            (place.navigation_goal.x, place.navigation_goal.y),
            (place.pose.x, place.pose.y),
            place.navigation_goal.yaw_deg,
        )
        stamp = _stamp(getattr(place, "saved_at", ""))
        for category, offset in zip(CATEGORIES, ACROSS_M):
            n += 1
            object_id = f"object_{n:02d}"
            pose = Pose3D(
                round(place.pose.x + ax * offset, 3),
                round(place.pose.y + ay * offset, 3),
                FIXTURE_Z_M,
                place.navigation_goal.yaw_deg,
            )
            obs_id = f"obs_{object_id}"
            obj = ObjectRecord(
                object_id, category, f"{category} on {place.name}", pose, place_id, stamp,
                "observed", [obs_id],
            )
            obs = Observation(
                obs_id, object_id, stamp, "go2", place.navigation_goal, pose, FRAME_ID, "synthetic",
            )
            objects.append(obj)
            observations.append(obs)
    return objects, observations
