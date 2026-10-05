"""Unit tests for STS3215 hardware driver and JointCalibration adapter."""

import numpy as np
import pytest

from so_arm101_hardware.sts3215_driver import (
    STS3215Driver,
    JointCalibration,
    DEFAULT_CANONICAL_JOINTS,
)


def test_checksum_calculation():
    # STS3215 checksum formula: (~(servo_id + length + instruction + sum(params))) & 0xFF
    # Example: servo_id=1, length=4, inst=2, params=[0x38, 2]
    # sum = 1 + 4 + 2 + 56 + 2 = 65 = 0x41
    # ~0x41 = 0xBE
    chk = STS3215Driver.checksum(1, 4, 0x02, [0x38, 2])
    assert chk == 0xBE


def test_joint_calibration_roundtrip():
    calib = JointCalibration()

    assert len(calib.joint_names) == 6
    assert calib.joint_names == DEFAULT_CANONICAL_JOINTS

    # Zero radians should map to zero_offsets (2048)
    zeros = np.zeros(6, dtype=np.float32)
    raw = calib.radians_to_raw(zeros)
    assert np.all(raw == 2048)

    # Convert back to radians
    recovered_rad = calib.raw_to_radians(raw)
    assert np.allclose(zeros, recovered_rad)

    # 45 degrees
    test_rad = np.array([np.pi / 4.0, -np.pi / 4.0, 0.0, 0.5, -0.5, 0.0], dtype=np.float32)
    raw_pulses = calib.radians_to_raw(test_rad)
    recovered = calib.raw_to_radians(raw_pulses)
    # 4096 resolution -> angle discretization ~ 2*pi/4096 rad ~ 0.0015 rad
    assert np.allclose(test_rad, recovered, atol=0.002)


def test_driver_simulation_mode():
    # Attempting to open a mock port should fallback gracefully to simulation mode
    driver = STS3215Driver(port='/dev/non_existent_mock_port')
    assert not driver.is_open

    # Reading should return cached/zero position
    rad, raw = driver.read_all_positions()
    assert len(rad) == 6
    assert len(raw) == 6
    assert np.all(raw == 2048)

    # Writing should update cached positions
    new_targets = np.array([0.1, 0.2, -0.1, 0.0, 0.0, 0.5], dtype=np.float32)
    success = driver.write_joint_positions(new_targets)
    assert success

    rad_read, raw_read = driver.read_all_positions()
    assert np.allclose(new_targets, rad_read, atol=0.002)

    driver.close()
