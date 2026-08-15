"""Test collection policy for the standalone Python environment."""

import importlib.util


collect_ignore = []
if importlib.util.find_spec("rospy") is None:
    # Legacy integration probes still exercise ROS services.  Keep them
    # available in a sourced ROS workspace without making the standalone
    # arm_teleop environment depend on ROS.
    collect_ignore.extend(
        [
            "test_hole_center.py",
            "test_ik.py",
            "test_visionpro_video.py",
        ]
    )
