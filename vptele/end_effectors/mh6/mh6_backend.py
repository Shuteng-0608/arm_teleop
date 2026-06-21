"""MH6 backend interfaces for dry-run and future hardware output."""

from typing import Optional

from end_effectors.mh6.mh6_safety import HardwareSafetyDecision, evaluate_hardware_safety

try:
    import rospy
except ImportError:  # Allows pure import checks outside a ROS environment.
    rospy = None


def _loginfo(message, *args):
    if rospy is not None:
        rospy.loginfo(message, *args)


def _logwarn(message, *args):
    if rospy is not None:
        rospy.logwarn(message, *args)


def _logwarn_throttle(period, message, *args):
    if rospy is not None:
        rospy.logwarn_throttle(period, message, *args)


class MH6Backend:
    """Interface for MH6 actuator command sinks."""

    def start(self) -> None:
        pass

    def send(self, command, context: Optional[dict] = None) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        pass

    @property
    def hardware_enabled(self) -> bool:
        return False


class DryRunBackend(MH6Backend):
    """Stores the latest command without sending anything to hardware."""

    def __init__(self) -> None:
        self.latest_command = None
        self.latest_context = None
        self.sent_count = 0
        self.started = False

    def start(self) -> None:
        self.started = True
        _loginfo("MH6 backend=dry_run started; hardware output disabled")

    def send(self, command, context: Optional[dict] = None) -> None:
        self.latest_command = command
        self.latest_context = dict(context or {})
        self.sent_count += 1

    def stop(self) -> None:
        self.started = False


class DisabledHardwareBackend(MH6Backend):
    """Suppresses commands when hardware was requested but is unavailable."""

    def __init__(self, reason: str = None) -> None:
        self.reason = reason or (
            "Hardware requested, but hardware backend is not implemented/enabled "
            "in this milestone. Commands will be suppressed."
        )
        self.suppressed_count = 0
        self.latest_command = None
        self.latest_context = None
        self.started = False

    def start(self) -> None:
        self.started = True
        _logwarn(self.reason)

    def send(self, command, context: Optional[dict] = None) -> None:
        self.latest_command = command
        self.latest_context = dict(context or {})
        self.suppressed_count += 1
        _logwarn_throttle(
            5.0,
            "MH6 hardware command suppressed; hardware backend is disabled "
            "(suppressed_count=%d)",
            self.suppressed_count,
        )

    def stop(self) -> None:
        self.started = False


def create_backend(
    config: Optional[dict] = None,
    safety_decision: Optional[HardwareSafetyDecision] = None,
) -> MH6Backend:
    config = config or {}
    backend_name = str(config.get("backend", "dry_run")).strip().lower()
    safety_decision = safety_decision or evaluate_hardware_safety(config)

    if backend_name in ("dry_run", "dry-run", "disabled"):
        if safety_decision.requested:
            return DisabledHardwareBackend(safety_decision.reason)
        return DryRunBackend()

    if backend_name in ("hardware", "modbus"):
        if safety_decision.eligible:
            return DisabledHardwareBackend(
                "Hardware configuration is eligible, but ModbusHardwareBackend is "
                "not implemented in this milestone. Commands will be suppressed."
            )
        return DisabledHardwareBackend(safety_decision.reason)

    raise ValueError(
        f"Unknown MH6 backend: {backend_name!r}. "
        "Supported values in this milestone: 'dry_run', 'hardware', 'modbus'."
    )


def validate_backend_selection():
    dry_run = create_backend({"backend": "dry_run", "enable_hardware": False})
    hardware_requested = create_backend({"backend": "hardware", "enable_hardware": True})
    enabled_with_dry_run = create_backend({"backend": "dry_run", "enable_hardware": True})
    eligible_modbus = create_backend({
        "backend": "modbus",
        "enable_hardware": True,
        "dry_run": False,
        "hardware_confirm": "I_UNDERSTAND_MH6_HARDWARE_RISK",
        "port": "/dev/ttyUSB0",
        "baudrate": 115200,
        "hardware_update_hz": 20,
        "command_timeout_sec": 0.5,
        "max_consecutive_failures": 3,
    })
    unknown_raises = False
    try:
        create_backend({"backend": "surprise"})
    except ValueError:
        unknown_raises = True

    return {
        "dry_run_backend": dry_run.__class__.__name__,
        "hardware_requested_backend": hardware_requested.__class__.__name__,
        "enabled_with_dry_run_backend": enabled_with_dry_run.__class__.__name__,
        "eligible_modbus_backend": eligible_modbus.__class__.__name__,
        "hardware_enabled_flags": [
            dry_run.hardware_enabled,
            hardware_requested.hardware_enabled,
            enabled_with_dry_run.hardware_enabled,
            eligible_modbus.hardware_enabled,
        ],
        "unknown_raises": unknown_raises,
    }
