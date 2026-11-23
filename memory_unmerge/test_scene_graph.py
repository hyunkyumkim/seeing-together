"""Tests:  python -m unittest memory_unmerge.test_scene_graph"""

import tempfile
import unittest
from pathlib import Path

from memory_unmerge.models import Observation, Pose2D, Pose3D
from memory_unmerge.scene_graph import (
    DEFAULT_PLACES,
    SceneGraphError,
    SceneGraphManager,
    legacy_object_labels,
)

LEGACY = Path(__file__).resolve().parents[1] / "memory" / "examples" / "obs_example.jsonl"


class InitialGraph(unittest.TestCase):
    def setUp(self):
        self.g = SceneGraphManager().load_initial(DEFAULT_PLACES)

    def test_four_desks_twelve_instances(self):
        self.assertEqual(set(self.g.places), {"desk_khang", "desk_oliver", "desk_kang", "desk_joseph"})
        self.assertEqual(len(self.g.objects), 12)
        self.assertEqual(len(self.g.observations), 12)
        for place_id in self.g.places:
            self.assertEqual(len(self.g.objects_in_place(place_id)), 3)

    def test_same_category_different_instances(self):
        cups = self.g.find_category("cup")
        self.assertEqual(len(cups), 4)
        self.assertEqual(len({c.object_id for c in cups}), 4)

    def test_goal_is_the_stand_pose_not_the_object(self):
        # object_07 is the first instance on Kang's desk (desks in order, 3 each).
        found = self.g.where_is("object_07")
        self.assertEqual(found["place"].place_id, "desk_kang")
        goal = found["navigation_goal"]
        obj = found["object"]
        self.assertEqual((goal.x, goal.y), (4.504, 1.92))
        self.assertNotEqual((round(obj.pose_3d.x, 3), round(obj.pose_3d.y, 3)), (goal.x, goal.y))
        x, y, yaw = self.g.goal_for_planner("object_07")
        self.assertEqual((x, y), (goal.x, goal.y))
        self.assertAlmostEqual(yaw, goal.yaw_rad())

    def test_object_lies_in_its_place_region(self):
        for obj in self.g.objects.values():
            place = self.g.place_containing(obj.pose_3d.x, obj.pose_3d.y)
            self.assertIsNotNone(place)
            self.assertEqual(place.place_id, obj.place_id)

    def test_synthetic_source_and_map_frame(self):
        for obs in self.g.observations.values():
            self.assertEqual(obs.source, "synthetic")
            self.assertEqual(obs.frame_id, "map")
            self.assertEqual(obs.observer_id, "go2")

    def test_round_trip(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "scene.json"
            self.g.save(path)
            loaded = SceneGraphManager.load(path)
        self.assertEqual(loaded.to_dict(), self.g.to_dict())
        self.assertEqual(loaded.goal_for_planner("object_01")[:2], self.g.goal_for_planner("object_01")[:2])

    def test_planner_receives_the_stand_pose(self):
        class FakePlanner:
            def plan(self, sx, sy, gx, gy):
                return {"path": [[sx, sy], [gx, gy]], "goal": [gx, gy], "length_m": 0.0}

        out = self.g.plan_to_object(FakePlanner(), (0.0, 0.0), "object_01")
        gx, gy, _yaw = self.g.goal_for_planner("object_01")
        self.assertEqual(out["goal"], [gx, gy])
        self.assertEqual(out["path"][0], [0.0, 0.0])

    def test_newer_observation_updates_pose_only(self):
        obj = self.g.get_object("object_01")
        later = Observation(
            "obs_object_01_b", "object_01", obj.last_seen + 10, "go2",
            Pose2D(obj.pose_3d.x, obj.pose_3d.y, 0.0),
            Pose3D(obj.pose_3d.x + 0.05, obj.pose_3d.y, obj.pose_3d.z, 0.0),
        )
        self.g.add_observation(later)
        self.assertEqual(self.g.get_object("object_01").pose_3d.x, later.observed_pose.x)
        self.assertEqual(len(self.g.get_object("object_01").observation_ids), 2)
        self.assertEqual(self.g.get_object("object_01").place_id, "desk_khang")

    def test_legacy_memory_is_not_copied_in_as_instances(self):
        labels = legacy_object_labels(LEGACY)
        self.assertEqual(labels, ["backpack", "mug"])
        self.assertEqual(self.g.find_category("mug"), [])


class Rejects(unittest.TestCase):
    def test_unknown_object(self):
        g = SceneGraphManager().load_initial()
        with self.assertRaises(SceneGraphError):
            g.where_is("object_99")

    def test_duplicate_registration(self):
        g = SceneGraphManager().load_initial()
        obj = g.get_object("object_01")
        obs = Observation("obs_dup", obj.object_id, obj.last_seen, "go2", Pose2D(0, 0), obj.pose_3d)
        with self.assertRaises(SceneGraphError):
            g.register_object(obj, obs)


if __name__ == "__main__":
    unittest.main()
