import time
import struct
import numpy as np

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


class STS3215Driver:
    """
    Feetech / Waveshare STS3215 串行总线舵机 Python 底层通信驱动类
    提供 1Mbps 串口数据读写、多轴角度查询、目标位置下发与模拟防掉帧机制。
    """

    HEADER = 0xFF
    INST_PING = 0x01
    INST_READ = 0x02
    INST_WRITE = 0x03
    INST_REG_WRITE = 0x04

    # 常用寄存器地址
    REG_TORQUE_ENABLE = 0x28 # 1 Byte: 0/1
    REG_GOAL_POSITION = 0x2A # 2 Bytes uint16 (0-4095)
    REG_GOAL_SPEED = 0x2C    # 2 Bytes uint16
    REG_PRESENT_POSITION = 0x38 # 2 Bytes uint16

    def __init__(self, port='/dev/ttyUSB0', baudrate=1000000, timeout=0.05, num_servos=6):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.num_servos = num_servos
        self.ser = None
        self.is_open = False

        # 缓存最后一次读取到的有效关节角度 [0..4095]
        self.last_raw_positions = np.array([2048] * num_servos, dtype=np.int32)
        
        # 零位偏移与正反向映射 (可根据 SO-ARM101 实际装配调整)
        self.zero_offsets = np.array([2048, 2048, 2048, 2048, 2048, 2048], dtype=np.int32)
        self.direction_signs = np.array([1, 1, 1, 1, 1, 1], dtype=np.float32)

        self.connect()

    def connect(self):
        """建立串口连接"""
        if not SERIAL_AVAILABLE:
            print(f"[STS3215Driver] WARNING: pyserial not installed. Running in SIMULATION mode.")
            self.is_open = False
            return False

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            self.is_open = True
            print(f"[STS3215Driver] Successfully opened serial port {self.port} at {self.baudrate} bps.")
            return True
        except Exception as e:
            print(f"[STS3215Driver] WARNING: Cannot open {self.port}: {e}. Running in SIMULATION mode.")
            self.is_open = False
            return False

    def checksum(self, servo_id, length, instruction, params):
        """计算 STS3215 数据帧 Checksum"""
        val = servo_id + length + instruction + sum(params)
        return (~val) & 0xFF

    def read_single_position(self, servo_id):
        """读取单个舵机的当前位置 [0-4095]"""
        if not self.is_open or self.ser is None:
            # 仿真模式：返回模拟平滑位置
            return self.last_raw_positions[servo_id - 1]

        try:
            # 构建 READ 指令帧：Header(2) + ID + Length(4) + INST_READ + Address(0x38) + DataLen(2) + Checksum
            params = [self.REG_PRESENT_POSITION, 2]
            length = len(params) + 2
            chk = self.checksum(servo_id, length, self.INST_READ, params)
            packet = bytearray([self.HEADER, self.HEADER, servo_id, length, self.INST_READ] + params + [chk])

            self.ser.reset_input_buffer()
            self.ser.write(packet)
            
            # 返回应答帧解析: Header(2) + ID + Length + Error + Data(2) + Checksum
            response = self.ser.read(8)
            if len(response) == 8 and response[0] == 0xFF and response[1] == 0xFF:
                raw_pos = response[5] | (response[6] << 8)
                self.last_raw_positions[servo_id - 1] = raw_pos
                return raw_pos
        except Exception as e:
            pass

        return self.last_raw_positions[servo_id - 1]

    def read_all_positions(self):
        """读取所有 6 轴舵机位置并转为弧度 [rad]"""
        raw_positions = np.zeros(self.num_servos, dtype=np.int32)
        for i in range(1, self.num_servos + 1):
            raw_positions[i - 1] = self.read_single_position(i)

        # 转换为弧度 [rad]
        # 4096 对应 2*pi
        diff = (raw_positions - self.zero_offsets) * self.direction_signs
        radians = diff * (2.0 * np.pi / 4096.0)
        return radians, raw_positions

    def write_joint_positions(self, radians):
        """下发弧度指令至实物舵机"""
        rad_arr = np.asarray(radians, dtype=np.float32)
        diff = rad_arr / (2.0 * np.pi / 4096.0)
        raw_targets = (self.zero_offsets + diff * self.direction_signs).astype(np.int32)
        raw_targets = np.clip(raw_targets, 0, 4095)

        self.last_raw_positions = np.copy(raw_targets)

        if not self.is_open or self.ser is None:
            return True

        # 下发至各个舵机
        for i in range(1, self.num_servos + 1):
            pos_val = int(raw_targets[i - 1])
            params = [self.REG_GOAL_POSITION, pos_val & 0xFF, (pos_val >> 8) & 0xFF]
            length = len(params) + 2
            chk = self.checksum(i, length, self.INST_WRITE, params)
            packet = bytearray([self.HEADER, self.HEADER, i, length, self.INST_WRITE] + params + [chk])
            try:
                self.ser.write(packet)
            except Exception:
                pass
        return True

    def close(self):
        if self.ser and self.is_open:
            self.ser.close()
            self.is_open = False
