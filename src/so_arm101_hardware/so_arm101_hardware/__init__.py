"""
SO-ARM101 Hardware Driver Package.
Provides STS3215 serial bus driver and ROS 2 hardware interface node.
"""

from so_arm101_hardware.sts3215_driver import (
    STS3215Driver,
    JointCalibration,
    DEFAULT_CANONICAL_JOINTS,
)

__version__ = '0.1.0'

__all__ = [
    'STS3215Driver',
    'JointCalibration',
    'DEFAULT_CANONICAL_JOINTS',
]
