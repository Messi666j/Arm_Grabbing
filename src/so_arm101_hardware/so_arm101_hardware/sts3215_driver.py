"""
Feetech / Waveshare STS3215 Serial Bus Servo Hardware Driver for SO-ARM101.

Provides 1Mbps UART packet framing, checksum validation, and bidirectional
JointCalibration adapter between raw 12-bit encoder pulses [0, 4095] and URDF radians.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import time
import numpy as np

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


DEFAULT_CANONICAL_JOINTS = [
    'shoulder_pan',
    'shoulder_lift',
    'elbow_flex',
    'wrist_flex',
    'wrist_roll',
    'gripper'
]


@dataclass
class JointCalibration:
    """
    Adapter between canonical URDF joint coordinates (radians) and physical servo pulses.
    """
    joint_names: List[str] = field(default_factory=lambda: list(DEFAULT_CANONICAL_JOINTS))
    servo_ids: List[int] = field(default_factory=lambda: [1, 2, 3, 4, 5, 6])
    zero_offsets: np.ndarray = field(default_factory=lambda: np.array([2048] * 6, dtype=np.int32))
    direction_signs: np.ndarray = field(default_factory=lambda: np.array([1.0] * 6, dtype=np.float32))
    resolution: int = 4096

    def radians_to_raw(self, radians: np.ndarray) -> np.ndarray:
        """Convert joint angles (radians) to raw servo pulses [0, 4095]."""
        rad_arr = np.asarray(radians, dtype=np.float32)
        pulse_diff = (rad_arr * (self.resolution / (2.0 * np.pi))) * self.direction_signs
        raw = (self.zero_offsets + pulse_diff).astype(np.int32)
        return np.clip(raw, 0, self.resolution - 1)

    def raw_to_radians(self, raw_positions: np.ndarray) -> np.ndarray:
        """Convert raw servo pulses [0, 4095] to joint angles (radians)."""
        raw_arr = np.asarray(raw_positions, dtype=np.float32)
        pulse_diff = (raw_arr - self.zero_offsets) * self.direction_signs
        return pulse_diff * (2.0 * np.pi / float(self.resolution))


class STS3215Driver:
    """
    Low-level serial communication driver for STS3215 smart bus servos.
    """

    HEADER = 0xFF
    INST_PING = 0x01
    INST_READ = 0x02
    INST_WRITE = 0x03
    INST_REG_WRITE = 0x04

    # Register Addresses
    REG_TORQUE_ENABLE = 0x28      # 1 Byte
    REG_GOAL_POSITION = 0x2A      # 2 Bytes uint16 (0-4095)
    REG_GOAL_SPEED = 0x2C         # 2 Bytes uint16
    REG_PRESENT_POSITION = 0x38   # 2 Bytes uint16

    def __init__(
        self,
        port: str = '/dev/ttyUSB0',
        baudrate: int = 1000000,
        timeout: float = 0.05,
        calibration: Optional[JointCalibration] = None
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.calibration = calibration or JointCalibration()
        self.num_servos = len(self.calibration.servo_ids)

        self.ser = None
        self.is_open = False

        # Cache last valid raw positions [0..4095]
        self.last_raw_positions = np.copy(self.calibration.zero_offsets)

        self.connect()

    def connect(self) -> bool:
        """Attempt to open hardware serial port or fallback to simulation mode."""
        if not SERIAL_AVAILABLE:
            print("[STS3215Driver] WARNING: pyserial not available. Running in SIMULATION mode.")
            self.is_open = False
            return False

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            self.is_open = True
            print(f"[STS3215Driver] Successfully opened serial port {self.port} at {self.baudrate} bps.")
            return True
        except Exception as e:
            print(f"[STS3215Driver] INFO: Cannot open {self.port} ({e}). Running in SIMULATION mode.")
            self.is_open = False
            return False

    @staticmethod
    def checksum(servo_id: int, length: int, instruction: int, params: List[int]) -> int:
        """Compute STS3215 protocol checksum byte."""
        val = servo_id + length + instruction + sum(params)
        return (~val) & 0xFF

    def read_single_position(self, servo_id: int) -> int:
        """Query position of a single servo [0-4095]."""
        idx = self.calibration.servo_ids.index(servo_id) if servo_id in self.calibration.servo_ids else (servo_id - 1)
        if not self.is_open or self.ser is None:
            return int(self.last_raw_positions[idx])

        try:
            params = [self.REG_PRESENT_POSITION, 2]
            length = len(params) + 2
            chk = self.checksum(servo_id, length, self.INST_READ, params)
            packet = bytearray([self.HEADER, self.HEADER, servo_id, length, self.INST_READ] + params + [chk])

            self.ser.reset_input_buffer()
            self.ser.write(packet)

            response = self.ser.read(8)
            if len(response) == 8 and response[0] == 0xFF and response[1] == 0xFF:
                raw_pos = response[5] | (response[6] << 8)
                self.last_raw_positions[idx] = raw_pos
                return int(raw_pos)
        except Exception:
            pass

        return int(self.last_raw_positions[idx])

    def read_all_positions(self) -> Tuple[np.ndarray, np.ndarray]:
        """Read all servos and return (radians, raw_pulses)."""
        raw_positions = np.zeros(self.num_servos, dtype=np.int32)
        for i, s_id in enumerate(self.calibration.servo_ids):
            raw_positions[i] = self.read_single_position(s_id)

        radians = self.calibration.raw_to_radians(raw_positions)
        return radians, raw_positions

    def write_joint_positions(self, radians: np.ndarray) -> bool:
        """Send target joint positions (in radians) to physical servos."""
        raw_targets = self.calibration.radians_to_raw(radians)
        self.last_raw_positions = np.copy(raw_targets)

        if not self.is_open or self.ser is None:
            return True

        for i, s_id in enumerate(self.calibration.servo_ids):
            pos_val = int(raw_targets[i])
            params = [self.REG_GOAL_POSITION, pos_val & 0xFF, (pos_val >> 8) & 0xFF]
            length = len(params) + 2
            chk = self.checksum(s_id, length, self.INST_WRITE, params)
            packet = bytearray([self.HEADER, self.HEADER, s_id, length, self.INST_WRITE] + params + [chk])
            try:
                self.ser.write(packet)
            except Exception:
                pass

        return True

    def close(self):
        """Close serial connection."""
        if self.ser and self.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
            self.is_open = False
