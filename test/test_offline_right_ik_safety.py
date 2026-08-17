import os
import unittest


class OfflineRightIKSafetyTest(unittest.TestCase):
    def test_offline_entry_has_no_lower_controller_interfaces(self):
        package_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        script_path = os.path.join(
            package_root, "vptele", "offline_compare_right_ik.py"
        )
        with open(script_path, "r", encoding="utf-8") as script_file:
            source = script_file.read()

        forbidden_tokens = (
            "Publisher(",
            "/aris_node/",
            "DualArmMovej",
            "MovejService",
            "StartDualTeleOP",
        )
        for token in forbidden_tokens:
            self.assertNotIn(token, source)


if __name__ == "__main__":
    unittest.main()
