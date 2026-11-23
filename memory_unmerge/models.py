"""Records in the scene graph. All positions are meters in the Go2 map frame.

The map frame is the one stored in ``lab.grid.npz``: cell (0, 0) is
``origin`` (saved as -25, -25), 0.05 m per cell. Odometry is a different
frame. ``go2_mapper.Mapper.to_map`` applies the saved ``T`` (tx, ty, yaw)
to bring odometry into this frame. Places in ``lab.places.json`` are already
in the map frame, so this graph never applies ``T`` itself.

A place has two poses on purpose. ``pose`` is the desk (the saved ``target``).
``navigation_goal`` is where the dog stands (``x``, ``y``, ``yaw_deg``), which
is the point ``go2_agent`` already passes to ``Planner.plan``.
"""

from dataclasses import dataclass, field


STATUSES = ("observed", "unknown", "missing")
FRAME_ID = "map"


@dataclass
class Pose2D:
    x: float
    y: float
    yaw_deg: float = 0.0

    def yaw_rad(self):
        from math import radians
        return radians(self.yaw_deg)

    def to_dict(self):
        return {"x": self.x, "y": self.y, "yaw_deg": self.yaw_deg}

    @classmethod
    def from_dict(cls, d):
        return cls(float(d["x"]), float(d["y"]), float(d.get("yaw_deg", 0.0)))


@dataclass
class Pose3D:
    x: float
    y: float
    z: float
    yaw_deg: float = 0.0

    def to_dict(self):
        return {"x": self.x, "y": self.y, "z": self.z, "yaw_deg": self.yaw_deg}

    @classmethod
    def from_dict(cls, d):
        return cls(float(d["x"]), float(d["y"]), float(d["z"]), float(d.get("yaw_deg", 0.0)))


@dataclass
class Place:
    place_id: str
    name: str
    pose: Pose2D
    navigation_goal: Pose2D
    region: list = field(default_factory=list)   # polygon, map-frame [x, y] corners
    object_ids: list = field(default_factory=list)

    def to_dict(self):
        return {
            "place_id": self.place_id,
            "name": self.name,
            "pose": self.pose.to_dict(),
            "navigation_goal": self.navigation_goal.to_dict(),
            "region": self.region,
            "object_ids": list(self.object_ids),
        }

    @classmethod
    def from_dict(cls, d):
        return cls(
            d["place_id"], d["name"], Pose2D.from_dict(d["pose"]),
            Pose2D.from_dict(d["navigation_goal"]),
            [list(p) for p in d.get("region", [])],
            list(d.get("object_ids", [])),
        )


@dataclass
class Observation:
    """One piece of evidence. The object record is the current estimate."""

    observation_id: str
    object_id: str
    timestamp: float
    observer_id: str
    observer_pose: Pose2D
    observed_pose: Pose3D
    frame_id: str = FRAME_ID
    source: str = "synthetic"

    def to_dict(self):
        return {
            "observation_id": self.observation_id,
            "object_id": self.object_id,
            "timestamp": self.timestamp,
            "observer_id": self.observer_id,
            "observer_pose": self.observer_pose.to_dict(),
            "observed_pose": self.observed_pose.to_dict(),
            "frame_id": self.frame_id,
            "source": self.source,
        }

    @classmethod
    def from_dict(cls, d):
        return cls(
            d["observation_id"], d["object_id"], float(d["timestamp"]), d["observer_id"],
            Pose2D.from_dict(d["observer_pose"]), Pose3D.from_dict(d["observed_pose"]),
            d.get("frame_id", FRAME_ID), d.get("source", "synthetic"),
        )


@dataclass
class ObjectRecord:
    object_id: str
    category: str
    name: str
    pose_3d: Pose3D
    place_id: str
    last_seen: float
    status: str = "observed"
    observation_ids: list = field(default_factory=list)

    def __post_init__(self):
        if self.status not in STATUSES:
            raise ValueError(f"status must be one of {STATUSES}, got {self.status!r}")

    def to_dict(self):
        return {
            "object_id": self.object_id,
            "category": self.category,
            "name": self.name,
            "pose_3d": self.pose_3d.to_dict(),
            "place_id": self.place_id,
            "last_seen": self.last_seen,
            "status": self.status,
            "observation_ids": list(self.observation_ids),
        }

    @classmethod
    def from_dict(cls, d):
        return cls(
            d["object_id"], d["category"], d["name"], Pose3D.from_dict(d["pose_3d"]),
            d["place_id"], float(d["last_seen"]), d.get("status", "observed"),
            list(d.get("observation_ids", [])),
        )
