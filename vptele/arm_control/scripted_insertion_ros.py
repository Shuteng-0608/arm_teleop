#!/usr/bin/env python3
"""ROS Trigger compatibility adapter for scripted collection."""

try:
    from vptele.arm_control.scripted_collection import ScriptedInsertionRunner
    from vptele.utils.logger import get_logger
except ModuleNotFoundError:  # Catkin's legacy package_dir exposes packages directly.
    from arm_control.scripted_collection import ScriptedInsertionRunner
    from utils.logger import get_logger


logger = get_logger()


class ScriptedInsertionROSNode(ScriptedInsertionRunner):
    """Expose manual collection through the legacy ROS Trigger service."""

    def __init__(self, robot_controller, ik_service_proxy, config):
        del ik_service_proxy  # The active controller uses MuJoCo Jacobian IK.
        super().__init__(robot_controller=robot_controller, config=config)

        import rospy
        from std_srvs.srv import Trigger

        service_name = self.cfg.get("service_name", "/scripted_insertion/run")
        self._service = rospy.Service(service_name, Trigger, self._handle_run)
        logger.info("ScriptedInsertion ROS service ready at %s", service_name)

    def _handle_run(self, _req):
        from std_srvs.srv import TriggerResponse

        outcome = self.run_manual_collection()
        return TriggerResponse(success=outcome.success, message=outcome.message)

    def _run_automatic_batch(self) -> None:
        """Keep the old ROS entry's shutdown behavior outside the core runner."""
        super()._run_automatic_batch()
        if bool(self.cfg.get("shutdown_on_batch_complete", True)):
            import rospy

            reason = (
                "automatic collection target reached"
                if self.batch_completed
                else "automatic collection stopped before reaching target"
            )
            rospy.signal_shutdown(reason)
