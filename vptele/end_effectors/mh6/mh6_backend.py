"""MH6 backend interfaces for dry-run and future hardware output."""

import threading
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


def _logerr(message, *args):
    if rospy is not None:
        rospy.logerr(message, *args)


def _logerr_throttle(period, message, *args):
    if rospy is not None:
        rospy.logerr_throttle(period, message, *args)


def build_mh6_register_block(command):
    """Build the exact register block written to MH6 registers 20..46."""
    finger_ids = list(command.finger_ids)
    finger_positions = list(command.finger_positions)
    palm_ids = list(command.palm_ids)
    palm_positions = list(command.palm_positions)
    palm_times = list(command.palm_times)

    if len(finger_ids) != len(finger_positions):
        raise ValueError("finger IDs and positions length mismatch")
    if len(palm_ids) != len(palm_positions) or len(palm_ids) != len(palm_times):
        raise ValueError("palm IDs, positions, and times length mismatch")
    if len(finger_ids) > 5 or len(palm_ids) > 5:
        raise ValueError("MH6 command supports at most 5 fingers and 5 palm servos")

    register_block = [0] * 27
    register_block[0] = len(finger_ids)
    for idx, (device_id, position) in enumerate(zip(finger_ids, finger_positions)):
        block_index = 1 + idx * 2
        register_block[block_index] = int(device_id)
        register_block[block_index + 1] = int(position)

    register_block[11] = len(palm_ids)
    for idx, (device_id, position, move_time) in enumerate(
        zip(palm_ids, palm_positions, palm_times)
    ):
        block_index = 12 + idx * 3
        register_block[block_index] = int(device_id)
        register_block[block_index + 1] = int(position)
        register_block[block_index + 2] = int(move_time)
    return register_block


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


class ModbusHardwareBackend(MH6Backend):
    """Gated hardware backend that sends already-validated actuator commands."""

    def __init__(self, config: Optional[dict] = None) -> None:
        config = config or {}
        self.port = str(config.get("port", "/dev/ttyUSB0"))
        self.baudrate = int(config.get("baudrate", 115200))
        self.timeout = float(config.get("command_timeout_sec", 0.5))
        self.max_consecutive_failures = int(config.get("max_consecutive_failures", 3))
        self.device_id = int(config.get("modbus_device_id", 1))
        self.client = None
        self.transaction_lock = threading.Lock()
        self.started = False
        self.latest_command = None
        self.latest_context = None
        self.consecutive_failures = 0
        self.suppressed = False
        self.last_error = None
        self.sent_count = 0

    @property
    def hardware_enabled(self) -> bool:
        return not self.suppressed

    def start(self) -> None:
        try:
            ModbusClient, framer_type = self._load_modbus_client()
        except RuntimeError as exc:
            self.suppressed = True
            self.last_error = str(exc)
            _logerr("%s; suppressing hardware sends", self.last_error)
            return

        self.client = ModbusClient(
            port=self.port,
            framer=framer_type,
            baudrate=self.baudrate,
            bytesize=8,
            parity="E",
            stopbits=1,
            timeout=self.timeout,
        )
        if not self.client.connect():
            self.suppressed = True
            self.last_error = f"failed to connect to MH6 Modbus device on {self.port}"
            _logerr("%s; suppressing hardware sends", self.last_error)
            return

        self.started = True
        self.suppressed = False
        self.consecutive_failures = 0
        _loginfo(
            "MH6 ModbusHardwareBackend started on %s at %d baud",
            self.port,
            self.baudrate,
        )

    def send(self, command, context: Optional[dict] = None) -> None:
        self.latest_command = command
        self.latest_context = dict(context or {})
        if self.suppressed:
            _logerr_throttle(
                5.0,
                "MH6 hardware send suppressed: %s",
                self.last_error or "backend suppressed",
            )
            return
        if not self.started or self.client is None:
            self._record_failure("ModbusHardwareBackend.send called before successful start")
            return

        try:
            self._send_actuator_command(command)
        except Exception as exc:
            self._record_failure(str(exc))
            return

        self.sent_count += 1
        self.consecutive_failures = 0
        self.last_error = None

    def stop(self) -> None:
        with self.transaction_lock:
            if self.client is not None:
                try:
                    self.client.close()
                except Exception as exc:
                    self.last_error = str(exc)
                    _logwarn("MH6 ModbusHardwareBackend close failed: %s", exc)
            self.client = None
            self.started = False

    def _send_actuator_command(self, command) -> None:
        register_block = build_mh6_register_block(command)
        with self.transaction_lock:
            if not self._client_connected():
                raise RuntimeError("Modbus client is not connected")
            self._write_registers_checked(20, register_block, "write MH6 command block")
            self._write_register_checked(0, 4, "trigger MH6 combined hand command")

    @staticmethod
    def _load_modbus_client():
        try:
            from pymodbus import FramerType
            from pymodbus.client import ModbusSerialClient as ModbusClient
        except ImportError as exc:
            raise RuntimeError(
                "pymodbus is required for ModbusHardwareBackend; install it before "
                "enabling MH6 hardware output"
            ) from exc

        return ModbusClient, FramerType.RTU

    def _client_connected(self):
        connected = getattr(self.client, "connected", False)
        return connected() if callable(connected) else bool(connected)

    def _write_register_checked(self, address, value, operation_name):
        try:
            result = self.client.write_register(
                address=address,
                value=value,
                device_id=self.device_id,
            )
        except TypeError:
            result = self.client.write_register(
                address=address,
                value=value,
                slave=self.device_id,
            )
        self._ensure_ok(result, operation_name)

    def _write_registers_checked(self, address, values, operation_name):
        try:
            result = self.client.write_registers(
                address=address,
                values=values,
                device_id=self.device_id,
            )
        except TypeError:
            result = self.client.write_registers(
                address=address,
                values=values,
                slave=self.device_id,
            )
        self._ensure_ok(result, operation_name)

    @staticmethod
    def _ensure_ok(response, operation_name):
        if response is None:
            raise RuntimeError(f"{operation_name}: no response")
        if response.isError():
            raise RuntimeError(f"{operation_name}: Modbus error {response}")

    def _record_failure(self, message):
        self.consecutive_failures += 1
        self.last_error = message
        _logerr_throttle(
            2.0,
            "MH6 hardware send failed (%d/%d): %s",
            self.consecutive_failures,
            self.max_consecutive_failures,
            message,
        )
        if self.consecutive_failures >= self.max_consecutive_failures:
            self.suppressed = True
            _logerr(
                "MH6 hardware backend suppressed after %d consecutive failures",
                self.consecutive_failures,
            )


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
            return ModbusHardwareBackend(config)
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


def validate_register_block_packing():
    from end_effectors.mh6.mh6_controller import ActuatorCommand

    command = ActuatorCommand(
        finger_ids=[1, 2, 3, 4, 5],
        finger_positions=[101, 102, 103, 104, 105],
        palm_ids=[1, 2, 3],
        palm_positions=[201, 202, 203],
        palm_times=[51, 52, 53],
    )
    register_block = build_mh6_register_block(command)
    expected = [
        5,
        1,
        101,
        2,
        102,
        3,
        103,
        4,
        104,
        5,
        105,
        3,
        1,
        201,
        51,
        2,
        202,
        52,
        3,
        203,
        53,
        0,
        0,
        0,
        0,
        0,
        0,
    ]
    return {
        "register_block": register_block,
        "matches_expected_layout": register_block == expected,
        "length": len(register_block),
    }
