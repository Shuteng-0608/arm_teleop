import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


MODEL_PATH = Path(__file__).resolve().parents[1] / "model" / "pangu_all_right.xml"


def _vector(element, attribute):
    return [float(value) for value in element.attrib[attribute].split()]


class PanguPegHoleModelMetadataTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = ET.parse(MODEL_PATH).getroot()

    def _named(self, tag, name):
        element = self.root.find(f".//{tag}[@name='{name}']")
        self.assertIsNotNone(element, f"missing {tag}: {name}")
        return element

    def test_required_cameras_exist(self):
        camera_names = {
            camera.attrib["name"] for camera in self.root.findall(".//camera")
        }
        self.assertTrue({"cctv_cam", "ee_cam", "base_top_cam"} <= camera_names)

    def test_peg_dimensions(self):
        peg = self._named("geom", "cylindrical_peg")
        radius, half_length = _vector(peg, "size")[:2]
        self.assertAlmostEqual(radius, 0.010, places=9)
        self.assertAlmostEqual(2.0 * half_length, 0.090, places=9)

    def test_hole_dimensions_and_goal_site(self):
        ring = self._named("geom", "wall_hole_ring_00")
        back_stop = self._named("geom", "hole_back_stop")
        goal = self._named("site", "hole_goal_site")

        ring_pos = _vector(ring, "pos")
        ring_size = _vector(ring, "size")
        back_pos = _vector(back_stop, "pos")
        back_size = _vector(back_stop, "size")

        ring_center_radius = math.hypot(ring_pos[0], ring_pos[2])
        inner_radius = ring_center_radius - ring_size[0]
        entrance_surface_y = ring_pos[1] + ring_size[1]
        back_stop_surface_y = back_pos[1] + back_size[1]
        depth = abs(entrance_surface_y - back_stop_surface_y)

        self.assertAlmostEqual(inner_radius, 0.014, places=9)
        self.assertAlmostEqual(depth, 0.023487, places=9)
        self.assertEqual(_vector(goal, "pos"), [0.0, -0.016, 0.0])


if __name__ == "__main__":
    unittest.main()
