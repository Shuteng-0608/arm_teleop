import csv
import os
import tempfile
import unittest

from core.right_joint_playback import load_redundancy_trajectory


FIELDS = [
    "frame_index",
    "source_timestamp",
    "source_sha256",
    "redundancy_success",
    "redundancy_held_previous",
    "redundancy_status",
] + ["redundancy_q{}".format(index) for index in range(1, 8)]


def valid_row(index, timestamp, q1=0.0):
    row = {
        "frame_index": index,
        "source_timestamp": timestamp,
        "source_sha256": "source",
        "redundancy_success": "True",
        "redundancy_held_previous": "False",
        "redundancy_status": "redundancy_selector:selected",
    }
    for joint_index in range(1, 8):
        row["redundancy_q{}".format(joint_index)] = q1 if joint_index == 1 else 0.0
    return row


class RightJointPlaybackTest(unittest.TestCase):
    def write_rows(self, rows):
        temporary = tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", newline="", delete=False
        )
        with temporary:
            writer = csv.DictWriter(temporary, fieldnames=FIELDS)
            writer.writeheader()
            writer.writerows(rows)
        self.addCleanup(lambda: os.unlink(temporary.name))
        return temporary.name

    def load(self, rows, **kwargs):
        return load_redundancy_trajectory(
            self.write_rows(rows),
            expected_frame_count=len(rows),
            expected_source_sha256="source",
            expected_file_sha256=None,
            **kwargs
        )

    def test_valid_trajectory_reports_step_and_velocity(self):
        frames, summary = self.load(
            [valid_row(0, 0.0), valid_row(1, 0.1, q1=0.02)]
        )
        self.assertEqual(len(frames), 2)
        self.assertAlmostEqual(summary.maximum_step, 0.02)
        self.assertAlmostEqual(summary.maximum_velocity, 0.2)

    def test_rejects_unselected_or_held_solution(self):
        rows = [valid_row(0, 0.0), valid_row(1, 0.1)]
        rows[1]["redundancy_held_previous"] = "True"
        with self.assertRaisesRegex(ValueError, "reused"):
            self.load(rows)

    def test_rejects_excessive_velocity(self):
        rows = [valid_row(0, 0.0), valid_row(1, 0.01, q1=0.02)]
        with self.assertRaisesRegex(ValueError, "velocity"):
            self.load(rows)

    def test_rejects_hardware_joint_limit_violation(self):
        rows = [valid_row(0, 0.0), valid_row(1, 0.1)]
        rows[1]["redundancy_q4"] = 2.5
        with self.assertRaisesRegex(ValueError, "outside"):
            self.load(rows)


if __name__ == "__main__":
    unittest.main()
