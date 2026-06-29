"""Copied from DHandControl/scripts/modbus_dev.py for the MH6 ROS wrapper."""

import time
import threading

try:
    try:
        from pymodbus import FramerType
        RTU_FRAMER = FramerType.RTU
    except ImportError:
        from pymodbus.framer import Framer
        RTU_FRAMER = "rtu"
    from pymodbus.client import ModbusSerialClient as ModbusClient
except ImportError as exc:
    FramerType = None
    ModbusClient = None
    PYMODBUS_IMPORT_ERROR = exc
else:
    PYMODBUS_IMPORT_ERROR = None


TELEOP_FINGER_NAMES = ("thumb", "index", "middle", "ring", "little")
TELEOP_FINGER_IDS = [1, 2, 3, 4, 5]
TELEOP_PALM_IDS = [1, 2, 3]
TELEOP_PALM_TIMES = [80, 80, 80]

CMD_READ_FINGER_STATUS = 0x07
REG_FINGER_STATUS_BASE = 60
REG_FINGER_STATUS_COUNT = 12


def _clip(value, low, high):
    return min(max(float(value), low), high)


def _map_normalized_to_position(value, open_position, closed_position):
    try:
        value = float(value)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"归一化值必须是数值: {value!r}") from exc
    value = _clip(value, 0.0, 1.0)
    position = open_position + value * (closed_position - open_position)
    return int(round(position))


def _to_signed_int16(value):
    value = int(value) & 0xFFFF
    return value - 0x10000 if value & 0x8000 else value


def _decode_finger_error_flags(error_flags):
    error_flags = int(error_flags)
    return {
        "stall": bool(error_flags & 0x01),
        "over_temperature": bool(error_flags & 0x02),
        "over_current": bool(error_flags & 0x04),
        "motor_abnormal": bool(error_flags & 0x08),
    }


class DexHandControl:
    """
    机器人手控制类 - Modbus协议
    Modbus/RS485配置：115200波特率，8位数据位，偶校验，1位停止位
    """

    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, parity='E', stopbits=1, bytesize=8, timeout=3):
        """
        初始化Modbus连接参数
        :param port: 串口号 (Windows: 'COM3', Linux: '/dev/ttyUSB0')
        :param baudrate: Modbus/RS485波特率 115200
        :param parity: 校验位 'E' - 偶校验（Modbus标准）
        :param stopbits: 停止位 1
        :param bytesize: 数据位 8
        :param timeout: 超时时间 3秒
        """
        if ModbusClient is None:
            raise RuntimeError(
                "pymodbus is required for DexHandControl. Install with: pip install pymodbus"
            ) from PYMODBUS_IMPORT_ERROR

        self.client = ModbusClient(
            port=port,
            framer=RTU_FRAMER,
            baudrate=baudrate,
            bytesize=bytesize,
            parity=parity,
            stopbits=stopbits,
            timeout=timeout
        )
        self.last_status = 0
        self.persistent_connection = False
        self.transaction_lock = threading.Lock()
        self.palm_limit = {
            1: (753, 150),
            2: (500, 870),
            3: (500, 574),
        }
        self.finger_limit = {
            1: (20, 1950),
            2: (20, 1950),
            3: (20, 1950),
            4: (20, 1950),
            5: (20, 1950),
        }

    @staticmethod
    def _normalized_values_by_id(normalized_values, expected_ids):
        if isinstance(normalized_values, dict):
            unknown_ids = set(normalized_values) - set(expected_ids)
            if unknown_ids:
                raise ValueError(f"未知设备ID: {sorted(unknown_ids, key=str)}")
            missing_ids = set(expected_ids) - set(normalized_values)
            if missing_ids:
                raise ValueError(f"缺少设备ID: {sorted(missing_ids)}")
            return {device_id: normalized_values[device_id] for device_id in expected_ids}

        try:
            values = list(normalized_values)
        except TypeError as exc:
            raise TypeError("归一化值必须是列表、元组或ID字典") from exc
        if len(values) != len(expected_ids):
            raise ValueError(f"归一化值数量必须为{len(expected_ids)}")
        return dict(zip(expected_ids, values))

    def map_palm_positions(self, normalized_values, scale=0.5):
        """Map palm normalized values by ID using (open, closed) hardware limits."""
        values_by_id = self._normalized_values_by_id(normalized_values, TELEOP_PALM_IDS)
        return {
            device_id: _map_normalized_to_position(value * scale, *self.palm_limit[device_id])
            for device_id, value in values_by_id.items()
        }

    def map_finger_positions(self, normalized_values, scale=0):
        """Map finger normalized values by ID using (open, closed) hardware limits."""
        values_by_id = self._normalized_values_by_id(normalized_values, TELEOP_FINGER_IDS)
        finger_scales = {
            1: 1,
            2: 1,
            3: 1,
            4: 1,
            5: 1,
        }
        return {
            device_id: _map_normalized_to_position(value*finger_scales[device_id], *self.finger_limit[device_id])
            for device_id, value in values_by_id.items()
        }

    def _client_connected(self):
        connected = getattr(self.client, "connected", False)
        return connected() if callable(connected) else bool(connected)

    def connect(self):
        """连接Modbus设备"""
        try:
            if self._client_connected():
                return True
            return self.client.connect()
        except Exception as e:
            print(f"连接失败: {e}")
            return False

    def disconnect(self):
        """断开Modbus连接"""
        self.client.close()

    def start_persistent_connection(self):
        """开启持久Modbus连接，用于高频遥操作"""
        with self.transaction_lock:
            self.persistent_connection = True
            if not self.connect():
                self.persistent_connection = False
                print("持久Modbus连接失败")
                return False
            return True

    def stop_persistent_connection(self):
        """关闭持久Modbus连接"""
        with self.transaction_lock:
            self.persistent_connection = False
            self.disconnect()

    def _ensure_ok(self, response, operation_name):
        """验证Modbus响应，失败时抛出带上下文的异常"""
        if response is None:
            raise RuntimeError(f"{operation_name} 无响应")
        if response.isError():
            raise RuntimeError(f"{operation_name} 返回Modbus错误: {response}")
        return response

    def _write_register_checked(self, address, value, operation_name=None):
        """写单个寄存器并验证响应"""
        operation_name = operation_name or f"写寄存器 {address}"
        result = self.client.write_register(address=address, value=value, device_id=1)
        self._ensure_ok(result, operation_name)

    def _write_registers_checked(self, address, values, operation_name=None):
        """写多个连续寄存器并验证响应"""
        operation_name = operation_name or f"写多个寄存器 {address}"
        try:
            result = self.client.write_registers(address=address, values=values, device_id=1)
        except TypeError:
            print("write_registers(device_id=...)不可用，改用slave=...")
            result = self.client.write_registers(address=address, values=values, slave=1)
        self._ensure_ok(result, operation_name)

    def _read_register_checked(self, address, operation_name=None):
        """读单个保持寄存器并验证响应"""
        operation_name = operation_name or f"读寄存器 {address}"
        result = self.client.read_holding_registers(address=address, count=1, device_id=1)
        result = self._ensure_ok(result, operation_name)
        if not hasattr(result, "registers") or len(result.registers) < 1:
            raise RuntimeError(f"{operation_name} 响应缺少寄存器数据")
        return result.registers[0]

    def _send_command(self, cmd, params=None, wait_status=True):
        """
        发送Modbus命令（修正顺序）
        :param cmd: 命令ID (1=单个设备控制, 2=组控, 3=清除错误)
        :param params: 参数字典 {寄存器地址: 值}
        :param wait_status: 是否等待并读取状态寄存器
        :return: 是否成功执行
        """
        with self.transaction_lock:
            owns_connection = not self.persistent_connection

            if self.persistent_connection and not self._client_connected():
                print("持久Modbus连接已断开")
                return False

            if owns_connection and not self.connect():
                print("Modbus连接失败")
                return False

            try:
                # 先设置参数，最后设置命令寄存器触发执行
                if params:
                    for addr, value in params.items():
                        result = self.client.write_register(address=addr, value=value, device_id=1)
                        self._ensure_ok(result, f"写参数寄存器 {addr}")

                # 最后设置命令寄存器触发执行
                result = self.client.write_register(address=0, value=cmd, device_id=1)
                self._ensure_ok(result, "写命令寄存器")

                if not wait_status:
                    return True

                # 等待命令执行完成
                time.sleep(0.1)

                # 读取状态反馈
                result = self.client.read_holding_registers(address=5, count=1, device_id=1)
                result = self._ensure_ok(result, "读取状态寄存器")
                if not hasattr(result, "registers") or len(result.registers) < 1:
                    raise RuntimeError("读取状态寄存器响应缺少寄存器数据")
                self.last_status = result.registers[0]
                return True
            except Exception as e:
                print(f"Modbus通信错误: {e}")
                return False
            finally:
                if owns_connection:
                    self.disconnect()

    def move_fingers(self, id_list, pos_list):
        """
        同步控制多个电缸运动（手指）
        :param id_list: 电缸ID列表 [1,2,...]
        :param pos_list: 目标位置列表 [p1,p2,...] (0-2000)
        :return: 是否成功执行
        """
        if len(id_list) != len(pos_list):
            print("错误: ID列表和位置列表长度不一致")
            return False

        # 验证位置范围
        for pos in pos_list:
            if pos < 0 or pos > 2000:
                print(f"错误: 位置值 {pos} 超出范围 (0-2000)")
                return False

        group_size = len(id_list)
        if group_size > 5:
            print("错误: 组控数量不能超过5")
            return False

        params = {
            1: 0,  # 设备类型: 电缸
            6: group_size  # 组控数量
        }

        # 写入组控数据
        for i, (id_val, pos_val) in enumerate(zip(id_list, pos_list)):
            params[10 + i * 2] = id_val
            params[11 + i * 2] = pos_val

        return self._send_command(2, params)

    def move_palms(self, id_list, pos_list, time_list):
        """
        同步控制多个舵机运动（手掌）
        :param id_list: 舵机ID列表 [1,2,...]
        :param pos_list: 目标位置列表 [p1,p2,...] (0-2000)
        :param time_list: 运动时间列表 [t1,t2,...](毫秒)
        :return: 是否成功执行
        """
        if len(id_list) != len(pos_list) or len(id_list) != len(time_list):
            print("错误: ID列表、位置列表和时间列表长度不一致")
            return False

        # 验证位置范围
        for pos in pos_list:
            if pos < 0 or pos > 1000:
                print(f"错误: 位置值 {pos} 超出范围 (0-1000)")
                return False

        group_size = len(id_list)
        if group_size > 5:
            print("错误: 组控数量不能超过5")
            return False

        params = {
            1: 1,  # 设备类型: 舵机
            6: group_size  # 组控数量
        }

        # 写入组控数据
        for i, (id_val, pos_val, time_val) in enumerate(zip(id_list, pos_list, time_list)):
            params[10 + i * 3] = id_val
            params[11 + i * 3] = pos_val
            params[12 + i * 3] = time_val

        return self._send_command(2, params)

    def move_hand(self, finger_ids=None, finger_positions=None,
                  palm_ids=None, palm_positions=None, palm_times=None,
                  wait_status=True):
        """
        组合控制手指电缸和手掌舵机
        :param finger_ids: 电缸ID列表
        :param finger_positions: 电缸目标位置列表 (0-2000)
        :param palm_ids: 舵机ID列表
        :param palm_positions: 舵机目标位置列表 (0-1000)
        :param palm_times: 舵机运动时间列表(ms)
        :param wait_status: 是否等待并读取状态寄存器
        :return: 是否成功执行
        """
        finger_ids = [] if finger_ids is None else list(finger_ids)
        finger_positions = [] if finger_positions is None else list(finger_positions)
        palm_ids = [] if palm_ids is None else list(palm_ids)
        palm_positions = [] if palm_positions is None else list(palm_positions)
        palm_times = [] if palm_times is None else list(palm_times)

        if len(finger_ids) != len(finger_positions):
            print("错误: 手指ID列表和位置列表长度不一致")
            return False

        if len(palm_ids) != len(palm_positions) or len(palm_ids) != len(palm_times):
            print("错误: 手掌ID列表、位置列表和时间列表长度不一致")
            return False

        finger_count = len(finger_ids)
        palm_count = len(palm_ids)

        if finger_count == 0 and palm_count == 0:
            print("错误: 组合控制至少需要一个手指或手掌设备")
            return False

        if finger_count > 5:
            print("错误: 手指组控数量不能超过5")
            return False

        if palm_count > 5:
            print("错误: 手掌组控数量不能超过5")
            return False

        for id_val in finger_ids + palm_ids:
            if not isinstance(id_val, int) or id_val < 0 or id_val > 255:
                print(f"错误: 设备ID {id_val} 超出范围 (0-255)")
                return False

        for pos in finger_positions:
            if not isinstance(pos, int) or pos < 0 or pos > 2000:
                print(f"错误: 手指位置值 {pos} 超出范围 (0-2000)")
                return False

        for pos in palm_positions:
            if not isinstance(pos, int) or pos < 0 or pos > 1000:
                print(f"错误: 手掌位置值 {pos} 超出范围 (0-1000)")
                return False

        for time_val in palm_times:
            if not isinstance(time_val, int) or time_val < 0 or time_val > 65535:
                print(f"错误: 手掌运动时间 {time_val} 超出范围 (0-65535)")
                return False

        register_block = [0] * 27  # registers 20..46
        register_block[0] = finger_count  # REG_HAND_FINGER_COUNT

        for i, (id_val, pos_val) in enumerate(zip(finger_ids, finger_positions)):
            block_index = 1 + i * 2
            register_block[block_index] = id_val
            register_block[block_index + 1] = pos_val

        register_block[11] = palm_count  # REG_HAND_PALM_COUNT
        for i, (id_val, pos_val, time_val) in enumerate(zip(palm_ids, palm_positions, palm_times)):
            block_index = 12 + i * 3
            register_block[block_index] = id_val
            register_block[block_index + 1] = pos_val
            register_block[block_index + 2] = time_val

        with self.transaction_lock:
            owns_connection = not self.persistent_connection

            if self.persistent_connection and not self._client_connected():
                print("持久Modbus连接已断开")
                return False

            if owns_connection and not self.connect():
                print("Modbus连接失败")
                return False

            # try:
            #     self._write_registers_checked(20, register_block, "写组合控制寄存器块")
            #     # print("组合控制寄存器块写入:", register_block)
            #     self._write_register_checked(0, 4, "写组合控制命令")

            #     if not wait_status:
            #         return True

            #     time.sleep(0.1)
            #     self.last_status = self._read_register_checked(5, "读取状态寄存器")
            #     return True
            # except Exception as e:
            #     print(f"Modbus通信错误: {e}")
            #     return False
            try:
                current_step = "写组合控制寄存器块 20..46"

                try:
                    t0 = time.monotonic()
                    self._write_registers_checked(20, register_block, "写组合控制寄存器块")
                    dt_ms = (time.monotonic() - t0) * 1000.0
                    # 正常时不想刷屏的话，可以注释掉这一行
                    # print(f"[MH6 Modbus] OK: {current_step}, {dt_ms:.1f} ms")
                except Exception as e:
                    print("\n[MH6 Modbus DEBUG] FAILED at:", current_step)
                    print("finger_ids:", finger_ids)
                    print("finger_positions:", finger_positions)
                    print("palm_ids:", palm_ids)
                    print("palm_positions:", palm_positions)
                    print("palm_times:", palm_times)
                    print("register_block_dec:", register_block)
                    print(
                        "register_block_hex:",
                        " ".join(f"0x{int(v) & 0xFFFF:04X}" for v in register_block),
                    )
                    print("exception:", type(e).__name__, e)
                    raise

                current_step = "写组合控制命令 register 0 = 4"

                try:
                    t0 = time.monotonic()
                    self._write_register_checked(0, 4, "写组合控制命令")
                    dt_ms = (time.monotonic() - t0) * 1000.0
                    # 正常时不想刷屏的话，可以注释掉这一行
                    # print(f"[MH6 Modbus] OK: {current_step}, {dt_ms:.1f} ms")
                except Exception as e:
                    print("\n[MH6 Modbus DEBUG] FAILED at:", current_step)
                    print("finger_ids:", finger_ids)
                    print("finger_positions:", finger_positions)
                    print("palm_ids:", palm_ids)
                    print("palm_positions:", palm_positions)
                    print("palm_times:", palm_times)
                    print("register_block_dec:", register_block)
                    print(
                        "register_block_hex:",
                        " ".join(f"0x{int(v) & 0xFFFF:04X}" for v in register_block),
                    )
                    print("exception:", type(e).__name__, e)
                    raise

                if not wait_status:
                    return True

                current_step = "读取状态寄存器 register 5"

                try:
                    time.sleep(0.1)
                    self.last_status = self._read_register_checked(5, "读取状态寄存器")
                    return True
                except Exception as e:
                    print("\n[MH6 Modbus DEBUG] FAILED at:", current_step)
                    print("last_status:", self.last_status)
                    print("exception:", type(e).__name__, e)
                    raise

            except Exception as e:
                print(f"Modbus通信错误: {e}")
                return False
            finally:
                if owns_connection:
                    self.disconnect()

    def move_hand_normalized(
        self,
        finger_values,
        palm_values,
        finger_ids=None,
        palm_ids=None,
        palm_times=None,
        wait_status=False,
    ):
        """
        遥操作便捷接口：接收归一化命令并通过move_hand()下发。

        手指归一化值: 0=完全张开, 1=完全弯曲/闭合。
        手掌归一化值: 0=张开, 1=对应手掌块最大弯曲/包络。
        """
        if isinstance(finger_values, dict):
            if all(f"u_{name}" in finger_values for name in TELEOP_FINGER_NAMES):
                normalized_fingers = {
                    device_id: finger_values[f"u_{name}"]
                    for device_id, name in zip(TELEOP_FINGER_IDS, TELEOP_FINGER_NAMES)
                }
            elif all(name in finger_values for name in TELEOP_FINGER_NAMES):
                normalized_fingers = {
                    device_id: finger_values[name]
                    for device_id, name in zip(TELEOP_FINGER_IDS, TELEOP_FINGER_NAMES)
                }
            elif all(device_id in finger_values for device_id in TELEOP_FINGER_IDS):
                normalized_fingers = finger_values
            else:
                raise TypeError(
                    "手指归一化字典必须包含u_thumb..u_little、thumb..little或ID 1..5"
                )
        else:
            normalized_fingers = finger_values

        if isinstance(palm_values, dict):
            if all(device_id in palm_values for device_id in TELEOP_PALM_IDS):
                normalized_palms = palm_values
            else:
                try:
                    thumb_side = palm_values["thumbSide"]
                    little_side = palm_values["littleSide"]
                except KeyError as exc:
                    raise TypeError(
                        "手掌归一化字典必须包含thumbSide/littleSide或ID 1..3"
                    ) from exc

                if "UR" in palm_values and "LR" in palm_values:
                    try:
                        servo_2 = (
                            float(palm_values["UR"]) + float(palm_values["LR"])
                        ) * 0.5
                    except (TypeError, ValueError) as exc:
                        raise TypeError("UR和LR归一化值必须是数值") from exc
                else:
                    servo_2 = little_side
                normalized_palms = {
                    1: thumb_side,
                    2: servo_2,
                    3: little_side,
                }
        else:
            normalized_palms = palm_values

        requested_finger_ids = list(TELEOP_FINGER_IDS if finger_ids is None else finger_ids)
        requested_palm_ids = list(TELEOP_PALM_IDS if palm_ids is None else palm_ids)
        if requested_finger_ids != TELEOP_FINGER_IDS:
            raise ValueError("归一化遥操作仅支持手指ID [1, 2, 3, 4, 5]")
        if requested_palm_ids != TELEOP_PALM_IDS:
            raise ValueError("归一化遥操作仅支持手掌ID [1, 2, 3]")

        palm_times = list(TELEOP_PALM_TIMES if palm_times is None else palm_times)
        if len(palm_times) != len(TELEOP_PALM_IDS):
            raise ValueError("palm_times必须包含3个值")

        finger_position_dict = self.map_finger_positions(normalized_fingers)
        palm_position_dict = self.map_palm_positions(normalized_palms)

        return self.move_hand(
            finger_ids=TELEOP_FINGER_IDS,
            finger_positions=[
                finger_position_dict[device_id] for device_id in TELEOP_FINGER_IDS
            ],
            palm_ids=TELEOP_PALM_IDS,
            palm_positions=[
                palm_position_dict[device_id] for device_id in TELEOP_PALM_IDS
            ],
            palm_times=palm_times,
            wait_status=wait_status,
        )

    def single_control(self, dev_type, dev_id, position, time_val=1000):
        """
        单个设备控制
        :param dev_type: 设备类型 (0=电缸, 1=舵机)
        :param dev_id: 设备ID
        :param position: 目标位置 (0-1000)
        :param time_val: 执行时间(ms)，仅对舵机有效
        :return: 是否成功执行
        """
        if position < 0 or position > 2000:
            print(f"错误: 位置值 {position} 超出范围 (0-2000)")
            return False

        params = {
            1: dev_type,  # 设备类型
            2: dev_id,  # 设备ID
            3: position,  # 位置
            4: time_val  # 执行时间
        }

        return self._send_command(1, params)
    
    def free_all(self):
        """释放所有设备（手指电缸和手掌舵机）"""
        return self.move_hand_normalized(
            finger_values={device_id: 0.0 for device_id in TELEOP_FINGER_IDS},
            palm_values={device_id: 0.0 for device_id in TELEOP_PALM_IDS},
            palm_times=[2000,2000,2000],
            wait_status=False,
        )
    
    def palm_free(self, palm_ids=None):
        """释放指定手掌舵机"""
        if palm_ids is None:
            palm_ids = TELEOP_PALM_IDS
        for palm_id in palm_ids:
            if palm_id not in TELEOP_PALM_IDS:
                print(f"错误: 无效的手掌ID {palm_id}")
                return False
        return self.move_palms(
            id_list=palm_ids,
            pos_list=[0] * len(palm_ids),
            time_list=[2000] * len(palm_ids),
        )
    
    def finger_free(self, finger_ids=None):
        """释放指定手指电缸"""
        if finger_ids is None:
            finger_ids = TELEOP_FINGER_IDS
        for finger_id in finger_ids:
            if finger_id not in TELEOP_FINGER_IDS:
                print(f"错误: 无效的手指ID {finger_id}")
                return False
        return self.move_fingers(
            id_list=finger_ids,
            pos_list=[0] * len(finger_ids),
        )

    def clear_error(self, dev_id, dev_type=0):
        """
        清除设备错误状态。
        对电缸，成功表示固件已发送清错命令且清错后的状态验证通过；
        手掌舵机清错暂不受固件支持。
        :param dev_type: 设备类型 (0=电缸, 1=舵机；舵机清错暂不受固件支持)
        :param dev_id: 设备ID
        :return: 是否成功执行
        """
        if dev_type not in (0, 1):
            print("错误: 设备类型必须为0(电缸)或1(舵机)")
            return False

        if not isinstance(dev_id, int) or not 1 <= dev_id <= 253:
            print("错误: 设备ID必须为1到253之间的整数")
            return False

        params = {
            1: dev_type,  # 设备类型
            2: dev_id  # 设备ID
        }

        success = self._send_command(3, params)
        if not success:
            return False

        if self.last_status == 0xF0:
            return True

        print("清除错误失败:", self.decode_status())
        return False
    
    def clear_all_errors(self, finger_ids=None, delay=0.02):
        """依次清除指定手指电缸错误；不尝试清除手掌舵机错误。"""
        finger_ids = TELEOP_FINGER_IDS if finger_ids is None else list(finger_ids)
        results = {}

        for index, finger_id in enumerate(finger_ids):
            results[finger_id] = self.clear_error(finger_id, dev_type=0)
            if index + 1 < len(finger_ids):
                time.sleep(delay)

        return results
    
    def read_finger_status(self, finger_id):
        """通过ESP32 Modbus桥读取单个手指电缸状态。"""
        if not isinstance(finger_id, int) or finger_id < 1 or finger_id > 253:
            print(f"错误: 手指电缸ID {finger_id} 超出范围 (1-253)")
            return None

        with self.transaction_lock:
            owns_connection = not self.persistent_connection

            if self.persistent_connection and not self._client_connected():
                print("持久Modbus连接已断开")
                return None

            if owns_connection and not self.connect():
                print("Modbus连接失败")
                return None

            try:
                self._write_register_checked(1, 0, "写手指设备类型")
                self._write_register_checked(2, finger_id, "写手指电缸ID")
                self._write_register_checked(0, CMD_READ_FINGER_STATUS, "写电缸状态查询命令")

                time.sleep(0.03)

                status = self._read_register_checked(5, "读取电缸状态查询固件状态")
                self.last_status = status

                result = self.client.read_holding_registers(
                    address=REG_FINGER_STATUS_BASE,
                    count=REG_FINGER_STATUS_COUNT,
                    device_id=1,
                )
                result = self._ensure_ok(result, "读取电缸状态结果寄存器")
                if not hasattr(result, "registers") or len(result.registers) < REG_FINGER_STATUS_COUNT:
                    raise RuntimeError("读取电缸状态结果响应缺少寄存器数据")

                regs = result.registers[:REG_FINGER_STATUS_COUNT]
                error_flags = regs[7]
                return {
                    "id": regs[0],
                    "query_ok": bool(regs[1]),
                    "target_position": regs[2],
                    "current_position": _to_signed_int16(regs[3]),
                    "temperature_c": _to_signed_int16(regs[4]),
                    "current_ma": regs[5],
                    "force_g": _to_signed_int16(regs[6]),
                    "error_flags": error_flags,
                    "error": _decode_finger_error_flags(error_flags),
                    "ok": bool(regs[1]) and error_flags == 0,
                    "internal_1": regs[8],
                    "internal_2": regs[9],
                    "response_error_code": regs[10],
                    "checksum_ok": bool(regs[11]),
                    "firmware_status": status,
                    "firmware_status_text": self.decode_status(status),
                }
            except Exception as e:
                print(f"Modbus通信错误: {e}")
                return None
            finally:
                if owns_connection:
                    self.disconnect()

    def read_all_finger_status(self, finger_ids=None, delay=0.02):
        """依次读取多个手指电缸状态，默认读取ID 1..5。"""
        finger_ids = TELEOP_FINGER_IDS if finger_ids is None else list(finger_ids)
        results = {}

        for index, finger_id in enumerate(finger_ids):
            results[finger_id] = self.read_finger_status(finger_id)
            if index + 1 < len(finger_ids):
                time.sleep(delay)

        return results


    def read_device_id(self, device_type, query_id):
        """
        读取设备ID
        :param device_type: 设备类型 (0=电缸, 1=舵机)
        :param query_id: 查询ID
        :return: 读取到的ID，失败返回None
        """
        if device_type not in (0, 1):
            print("错误: 设备类型必须为0(电缸)或1(舵机)")
            return None

        if not isinstance(query_id, int) or query_id < 1 or query_id > 253:
            print(f"错误: 查询ID {query_id} 超出范围 (1-253)")
            return None

        with self.transaction_lock:
            owns_connection = not self.persistent_connection

            if self.persistent_connection and not self._client_connected():
                print("持久Modbus连接已断开")
                return None

            if owns_connection and not self.connect():
                print("Modbus连接失败")
                return None

            try:
                self._write_register_checked(1, device_type, "写设备类型")
                self._write_register_checked(2, query_id, "写查询ID")
                self._write_register_checked(0, 0x05, "写读取ID命令")

                time.sleep(0.1)

                self.last_status = self._read_register_checked(5, "读取状态寄存器")
                if self.last_status != 0x91:
                    print("读取设备ID失败:", self.decode_status())
                    return None

                return self._read_register_checked(8, "读取ID结果寄存器")
            except Exception as e:
                print(f"Modbus通信错误: {e}")
                return None
            finally:
                if owns_connection:
                    self.disconnect()

    def set_device_id(self, device_type, old_id, new_id, save=True):
        """
        修改设备ID
        :param device_type: 设备类型 (0=电缸, 1=舵机)
        :param old_id: 当前ID
        :param new_id: 新ID
        :param save: 是否请求保存配置
        :return: 是否成功
        """
        if device_type not in (0, 1):
            print("错误: 设备类型必须为0(电缸)或1(舵机)")
            return False

        if not isinstance(old_id, int) or old_id < 1 or old_id > 253:
            print(f"错误: 当前ID {old_id} 超出范围 (1-253)")
            return False

        if not isinstance(new_id, int) or new_id < 1 or new_id > 253:
            print(f"错误: 新ID {new_id} 超出范围 (1-253)")
            return False

        if old_id == new_id:
            print("错误: 当前ID和新ID相同")
            return False

        with self.transaction_lock:
            owns_connection = not self.persistent_connection

            if self.persistent_connection and not self._client_connected():
                print("持久Modbus连接已断开")
                return False

            if owns_connection and not self.connect():
                print("Modbus连接失败")
                return False

            try:
                self._write_register_checked(1, device_type, "写设备类型")
                self._write_register_checked(2, old_id, "写当前ID")
                self._write_register_checked(7, new_id, "写新ID")
                self._write_register_checked(9, 1 if save else 0, "写ID保存标志")
                self._write_register_checked(0, 0x06, "写设置ID命令")

                time.sleep(0.2)

                self.last_status = self._read_register_checked(5, "读取状态寄存器")
                if self.last_status != 0x92:
                    print("设置设备ID失败:", self.decode_status())
                    return False

                result_id = self._read_register_checked(8, "读取ID结果寄存器")
                if result_id != new_id:
                    print(f"设置设备ID失败: 回读ID {result_id} != 新ID {new_id}")
                    return False

                return True
            except Exception as e:
                print(f"Modbus通信错误: {e}")
                return False
            finally:
                if owns_connection:
                    self.disconnect()

    def read_palm_id(self, query_id):
        """读取手掌舵机ID"""
        return self.read_device_id(1, query_id)

    def set_palm_id(self, old_id, new_id, save=True):
        """修改手掌舵机ID"""
        return self.set_device_id(1, old_id, new_id, save)

    def read_finger_id(self, query_id):
        """读取手指电缸ID"""
        return self.read_device_id(0, query_id)

    def set_finger_id(self, old_id, new_id, save=True):
        """修改手指电缸ID"""
        return self.set_device_id(0, old_id, new_id, save)

    def scan_device_ids(self, device_type, start_id=1, end_id=30):
        """
        扫描设备ID
        :param device_type: 设备类型 (0=电缸, 1=舵机)
        :param start_id: 起始ID
        :param end_id: 结束ID
        :return: 响应的ID列表
        """
        if device_type not in (0, 1):
            print("错误: 设备类型必须为0(电缸)或1(舵机)")
            return []

        if (not isinstance(start_id, int) or not isinstance(end_id, int) or
                start_id < 1 or end_id > 253 or start_id > end_id):
            print("错误: 扫描范围必须在1..253内，且起始ID不能大于结束ID")
            return []

        found = []
        for query_id in range(start_id, end_id + 1):
            device_id = self.read_device_id(device_type, query_id)
            if device_id is not None:
                found.append(device_id)
        return found

    def get_status(self):
        """获取最后的状态码"""
        return self.last_status

    def decode_status(self, status=None):
        """
        解码状态寄存器值
        :param status: 状态码
        :return: 状态描述
        """
        if status is None:
            status = self.last_status

        status_map = {
            0x90: "组合手部控制命令已下发",
            0x91: "设备ID读取成功",
            0x92: "设备ID设置成功",
            0x93: "电缸状态查询成功",
            0xA0: "电缸控制成功",
            0xB0: "舵机控制成功",
            0xC0: "电缸组控成功",
            0xD0: "舵机组控成功",
            0xE0: "无效命令错误",
            0xE1: "固件校验错误: 无效设备类型",
            0xE2: "控制失败错误",
            0xE3: "固件校验错误: 无效组控数量",
            0xE4: "组控失败错误",
            0xE5: "清除错误暂不支持",
            0xE6: "固件校验错误: 无效寄存器地址",
            0xE7: "固件校验错误: 不支持的Modbus功能码",
            0xE8: "固件校验错误: 无效组合手指数量",
            0xE9: "固件校验错误: 无效组合手掌数量",
            0xEA: "固件校验错误: 组合控制寄存器范围无效",
            0xEB: "固件校验错误: 组合控制为空",
            0xEC: "设备ID读取失败",
            0xED: "设备ID设置失败",
            0xEE: "固件校验错误: 无效设备ID",
            0xEF: "设备ID操作暂不支持",
            0xF0: "清除错误成功",
            0xF1: "电缸状态查询超时",
            0xF2: "电缸状态返回帧格式错误",
            0xF3: "电缸状态返回帧校验错误",
            0xF4: "清除错误后状态验证失败",
            0xF5: "清除错误后仍有故障"
        }

        if status in status_map:
            return status_map[status]

        base_status = status & 0xF0
        if base_status in status_map:
            detail = status & 0x0F
            return f"{status_map[base_status]} (详情: 0x{detail:X})"

        return f"未知状态: 0x{status:X}"
    
if __name__ == "__main__":
    print("modbus_dev.py is a library. Use mh6_demo_gestures.py for manual gesture demos.")

   
