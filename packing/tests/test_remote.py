"""LLM contingency control (packing.remote) against the simulated arm."""

import tempfile
import unittest
from pathlib import Path

import numpy as np

try:
    from packing.remote.server import RemoteRig

    AVAILABLE, WHY = True, ""
except Exception as e:  # noqa: BLE001 — mcp/placo missing
    AVAILABLE, WHY = False, str(e)

CONFIG = Path(__file__).resolve().parents[1] / "cell.example.yaml"


@unittest.skipUnless(AVAILABLE, "remote control deps unavailable")
class RemoteJogTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.rig = RemoteRig(CONFIG, sim=True)
        self.rig.settings.log_file = Path(self._tmp.name) / "log.jsonl"
        self.rig.cfg.poses_file = Path(self._tmp.name) / "poses.json"
        self.jog = self.rig.connect()

    def tearDown(self):
        self.rig.disconnect()
        self._tmp.cleanup()

    def tip(self):
        return np.array(self.jog.status().tip_cm)

    def test_directions_move_the_tip_along_the_robot_axes(self):
        expected = {"back": (-1, 0, 0), "forward": (1, 0, 0), "left": (0, 1, 0), "right": (0, -1, 0),
                    "down": (0, 0, -1), "up": (0, 0, 1)}
        for direction, axis in expected.items():
            before = self.tip()
            result = self.jog.move(direction, 2)
            self.assertTrue(result.ok, result.message)
            np.testing.assert_allclose(self.tip() - before, np.array(axis) * 2.0, atol=0.2)

    def test_refusals_do_not_move(self):
        before = self.tip()
        for result in [self.jog.move("sideways"), self.jog.move("up", 50), self.jog.tilt("left"),
                       self.jog.gripper("smash")]:
            self.assertFalse(result.ok)
        np.testing.assert_allclose(self.tip(), before, atol=1e-6)

    def test_floor_is_enforced(self):
        for _ in range(10):
            result = self.jog.move("down", 5)
            if not result.ok:
                break
        self.assertFalse(result.ok)
        self.assertIn("too low", result.message)
        self.assertGreaterEqual(self.tip()[2], self.rig.settings.limits.min_z_m * 100 - 0.1)

    def test_tilt_keeps_the_tip_in_place(self):
        before = self.tip()
        self.assertTrue(self.jog.tilt("down", 15).ok)
        np.testing.assert_allclose(self.tip(), before, atol=0.3)

    def test_every_command_is_logged(self):
        self.jog.move("up", 1)
        self.jog.move("nowhere")
        lines = self.rig.settings.log_file.read_text().splitlines()
        self.assertEqual(len(lines), 2)

    def test_schematic_image(self):
        images = self.rig.images()
        self.assertEqual(len(images), 1)
        self.assertGreater(len(images[0].data), 1000)


if __name__ == "__main__":
    unittest.main()
