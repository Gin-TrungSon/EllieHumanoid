# Copyright 2021 by Dong Trung Son, Nueremberg University.
# Email: trungsondo68839@th-nuernberg.de
# All rights reserved.
# This file is part of the Ellie-Project,
# and is released under the "MIT License Agreement". Please see the LICENSE
# file that should have been included as part of this package.

from enum import Enum


# EEPROM Area
MODEL_NUMBER = 0
FIRMWARE_VERSION = 2
ID = 3
BAUDRATE = 4
DELAY_TIME = 5
CW_ANGLE_LIMIT = 6
CCW_ANGLE_LIMIT = 8
TEMPERATURE_LIMIT = 11
MIN_VOLTAGE_LIMIT = 12
MAX_VOLTAGE_LIMIT = 13
MAX_TORQUE = 14
STATUS_RETURN_LEVEL = 16
ALARM_LED = 17
SHUTDOWN = 18
# RAM  Area
TORQUE_ENABLE = 24
STATUS_LED = 25

# AX
CW_COMPLIANCE_MARGIN = 26
CCW_COMPLIANCE_MARGIN = 27
CW_COMPLIANCE_SLOPE = 28
CCW_COMPLIANCE_SLOPE = 29

# MX
D_GAIN = 26
I_GAIN = 27
P_GAIN = 28

GOAL_POSITION = 30
MOVING_SPEED = 32
TORQUE_LIMIT = 34
PRESENT_POSITION = 36
PRESENT_SPEED = 38
PRESENT_LOAD = 40
PRESENT_VOLTAGE = 42
PRESENT_TEMPERATURE = 43
REGISTED = 44
MOVING = 46
LOCK = 47
PUNCH = 48
REALTIME_TICK = 50
GOAL_ACCELERATION = 73

AVAILABLE_BAUDRATE = [9600, 57600, 115200,
                      1000000, 2000000, 3000000, 4000000, 450000]


class MotorType(Enum):
    MX_106 = 320
    MX_64 = 310
    MX_28 = 29
    MX_12 = 28
    AX_12 = 12
    AX_18 = 18
    RX_24 = 24
    RX_28 = 28
    RX_64 = 64


position_range = {
    MotorType.MX_106: (4096, 360.0),
    MotorType.MX_64: (4096, 360.0),
    MotorType.MX_28: (4096, 360.0),
    MotorType.MX_12: (4096, 360.0),
    MotorType.AX_12: (1024, 300.0),
    MotorType.AX_18: (1024, 300.0),
    MotorType.RX_24: (1024, 300.0),
    MotorType.RX_28: (1024, 300.0),
    MotorType.RX_64: (1024, 300.0),
}
torque_max = {  # in N.m
    MotorType.MX_106: 8.4,
    MotorType.MX_64: 6.0,
    MotorType.MX_28: 2.5,
    MotorType.MX_12: 1.2,
    MotorType.AX_12: 1.2,
    MotorType.AX_18: 1.8,
    MotorType.RX_24: 2.6,
    MotorType.RX_28: 2.5,
    MotorType.RX_64: 4.0
}

velocity = {  # in degree/s
    MotorType.MX_106: 270.0,
    MotorType.MX_64: 378.0,
    MotorType.MX_28: 330.0,
    MotorType.MX_12: 2820.0,
    MotorType.AX_12: 354.0,
    MotorType.AX_18: 582.0,
    MotorType.RX_24: 756.0,
    MotorType.RX_28: 402.0,
    MotorType.RX_64: 294.0,

}
dynamixelModels = {
    12: MotorType.AX_12,    # 12 + (0<<8)
    18: MotorType.AX_18,    # 18 + (0<<8)
    24: MotorType.RX_24,    # 24 + (0<<8)
    28: MotorType.RX_28,    # 28 + (0<<8)
    29: MotorType.MX_28,    # 29 + (0<<8)
    64: MotorType.RX_64,    # 64 + (0<<8)
    360: MotorType.MX_12,   # 104 + (1<<8)
    310: MotorType.MX_64,   # 54 + (1<<8)
    320: MotorType.MX_106,  # 64 + (1<<8)


}


def dxl_to_degree(value, model_type):
    max_pos, max_deg = position_range[model_type]

    return round(((max_deg * float(value)) / (max_pos - 1)) - (max_deg / 2), 2)


def degree_to_dxl(value, model_type):
    max_pos, max_deg = position_range[model_type]

    pos = int(round((max_pos - 1) * ((max_deg / 2 + float(value)) / max_deg), 0))
    pos = min(max(pos, 0), max_pos - 1)

    return pos


def _speed_factor(model):
    if model == MotorType.MX_12:
        return 0.916

    if str(model).startswith('MX') :
        return 0.114

    return 0.111


def dxl_to_speed(value, model):
    cw, speed = divmod(value, 1024)
    direction = (-2 * cw + 1)

    return direction * (speed * _speed_factor(model)) * 6


def speed_to_dxl(value, model):
    direction = 1024 if value < 0 else 0
    speed_factor = _speed_factor(model)

    max_value = 1023 * speed_factor * 6
    value = min(max(value, -max_value), max_value)

    return int(round(direction + abs(value) / (6 * speed_factor), 0))

# MARK: - Torque


def dxl_to_torque(value, model):
    return round(value / 10.23, 1)


def torque_to_dxl(value, model):
    return int(round(value * 10.23, 0))


def dxl_to_load(value, model):
    cw, load = divmod(value, 1024)
    direction = -2 * cw + 1

    return dxl_to_torque(load, model) * direction

# MARK - Acceleration


def dxl_to_acceleration(value, model):
    """Converts from ticks to degress/second^2"""

    return value * 8.583  # degrees / sec**2


def acceleration_to_dxl(value, model):
    """Converts from degrees/second^2 to ticks"""

    return int(round(value / 8.583, 0))  # degrees / sec**2

# PID Gains


def dxl_to_pid(value, model):
    return (value[0] * 0.004,
            value[1] * 0.48828125,
            value[2] * 0.125)


def pid_to_dxl(value, model):
    def truncate(x):
        return int(max(0, min(x, 254)))
    return [truncate(x * y) for x, y in zip(value, (250, 2.048, 8.0))]



def clamp(value, min_value,max_value, id =None):
        if value > max_value or value < min_value:
            if id != None:
                print(f"[Id: {id}] Value {value} is out of range [{min_value},{max_value}] ")
            else:
                print(f"Value {value} is out of range [{min_value},{max_value}] ")
        return max(min(value, max_value), min_value)

if __name__ == '__main__':
    print(degree_to_dxl(50, MotorType.AX_12))