"""Tests for store.py:  python -m unittest test_store   (run inside memory/)"""
import os
import unittest

from store import HISTORY_MAX, Memory, SchemaError

HERE = os.path.dirname(os.path.abspath(__file__))
EXAMPLE = os.path.join(HERE, "examples", "obs_example.jsonl")


def obj(i, t, pos, label="mug"):
    return {"v": 1, "id": f"m-{i}", "src": "mock", "type": "object", "t": t, "frame": "map",
            "label": label, "pos": pos, "conf": 0.9}


class ExampleFile(unittest.TestCase):
    def setUp(self):
        self.m = Memory()
        self.m.ingest_file(EXAMPLE)

    def test_entities(self):
        self.assertEqual(sorted(self.m.entities),
                         ["obj:backpack", "obj:mug", "person:wearer", "robot:go2", "tag:1"])
        self.assertEqual(self.m.skipped, 1)  # the imu line

    def test_late_observation_does_not_overwrite(self):
        mug = self.m.last_seen("obj:mug")
        self.assertEqual(mug["obs_id"], "m-000005")  # t=...985.4, not the late one at ...980.0
        self.assertEqual(mug["pos"], [2.10, -0.30])

    def test_timeline_newest_first(self):
        ids = [s["obs_id"] for s in self.m.timeline("obj:mug")]
        self.assertEqual(ids, ["m-000005", "m-000007", "m-000002"])

    def test_where(self):
        self.assertEqual([e["entity"] for e in self.m.where("mug")], ["obj:mug"])
        self.assertEqual(self.m.where("chair"), [])

    def test_near_map_only_and_since(self):
        hits = [e["entity"] for e in self.m.near([2.0, 0.0], 0.5)]
        self.assertEqual(hits, ["obj:mug", "person:wearer"])  # sorted by distance (0.316, 0.335 m); tag:1 is in pixels, excluded
        self.assertEqual(self.m.near([2.0, 0.0], 0.5, since=1793434980.0)[0]["entity"], "obj:mug")

    def test_robot_yaw_kept(self):
        self.assertAlmostEqual(self.m.last_seen("robot:go2")["yaw"], 0.52)


class Rules(unittest.TestCase):
    def test_history_capped(self):
        m = Memory()
        for i in range(HISTORY_MAX + 10):
            m.ingest(obj(i, 100.0 + i, [i, 0]))
        self.assertEqual(len(m.entities["obj:mug"]["history"]), HISTORY_MAX)
        self.assertEqual(m.last_seen("obj:mug")["obs_id"], f"m-{HISTORY_MAX + 9}")

    def test_schema_errors(self):
        m = Memory()
        bad = obj(1, 1.0, [0, 0])
        del bad["t"]
        with self.assertRaises(SchemaError):
            m.ingest(bad)
        with self.assertRaises(SchemaError):
            m.ingest({**obj(2, 1.0, [0, 0]), "type": "teleport"})
        with self.assertRaises(SchemaError):
            m.ingest({**obj(3, 1.0, [0, 0]), "frame": "world"})
        no_label = obj(4, 1.0, [0, 0])
        del no_label["label"]
        with self.assertRaises(SchemaError):
            m.ingest(no_label)


if __name__ == "__main__":
    unittest.main()
