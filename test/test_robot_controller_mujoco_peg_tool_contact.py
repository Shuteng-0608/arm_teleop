from pathlib import Path
import sys
import types
import unittest


ROOT = Path(__file__).resolve().parents[1]
VPTELE_ROOT = ROOT / "vptele"
if str(VPTELE_ROOT) not in sys.path:
    sys.path.insert(0, str(VPTELE_ROOT))


def _install_ros_stubs():
    rospy = types.ModuleType("rospy")
    rospy.Service = lambda *args, **kwargs: None
    rospy.wait_for_service = lambda *args, **kwargs: None
    rospy.ServiceProxy = lambda *args, **kwargs: None

    class _Response:
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)

    arm_teleop = types.ModuleType("arm_teleop")
    arm_teleop_srv = types.ModuleType("arm_teleop.srv")
    arm_teleop_srv.SetRecording = object
    arm_teleop_srv.SetRecordingResponse = _Response

    std_srvs = types.ModuleType("std_srvs")
    std_srvs_srv = types.ModuleType("std_srvs.srv")
    std_srvs_srv.Trigger = object
    std_srvs_srv.TriggerResponse = _Response

    sys.modules.setdefault("rospy", rospy)
    sys.modules.setdefault("arm_teleop", arm_teleop)
    sys.modules.setdefault("arm_teleop.srv", arm_teleop_srv)
    sys.modules.setdefault("std_srvs", std_srvs)
    sys.modules.setdefault("std_srvs.srv", std_srvs_srv)


try:
    import mujoco

    _install_ros_stubs()
    from arm_control.robot_controller_mujoco_peg_tool_contact import (
        RobotControllerMuJoCoPegTool,
    )
except ImportError:
    mujoco = None
    RobotControllerMuJoCoPegTool = None


@unittest.skipIf(mujoco is None, "MuJoCo not installed")
class RobotControllerMuJoCoPegToolTest(unittest.TestCase):
    def test_headless_physics_step_and_cached_model_addresses(self):
        controller = RobotControllerMuJoCoPegTool(
            model_path=str(ROOT / "model" / "pangu_all_right.xml"),
            config={
                "auto_start": False,
                "launch_viewer": False,
                "show_camera_streams": False,
                "record_data": False,
                "record_hdf5": False,
                "enable_visual_guides": False,
                "enable_task_success_auto_stop": True,
                "task_success_hole_site_name": "hole_goal_site",
                "task_success_only_when_recording": True,
            },
        )

        self.assertEqual(controller.arm_qpos_addrs.shape, (7,))
        self.assertEqual(controller.arm_dof_addrs.shape, (7,))
        self.assertIn("peg_ft_force", controller.sensor_slice_by_name)

        start_time = float(controller.data.time)
        for _ in range(10):
            controller._physics_step()
        self.assertAlmostEqual(
            float(controller.data.time) - start_time,
            10.0 * controller.sim_timestep,
            places=9,
        )
        self.assertEqual(len(controller.get_current_joints()), 7)
        controller.disconnect()


if __name__ == "__main__":
    unittest.main()
