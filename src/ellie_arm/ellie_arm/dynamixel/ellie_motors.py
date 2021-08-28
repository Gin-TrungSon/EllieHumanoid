from collections import OrderedDict
from ellie_arm.dynamixel.hardwares import Motor
import os
from pathlib import Path
import json
from ellie_arm.dynamixel.ultis import *


class EllieMotors:
    def __init__(self, config=None):
        self._config = config
        self._motor_names = ['head_z', 'head_y', 'r_shoulder_y', 'r_shoulder_x', 'r_arm_z', 'r_elbow_y',
                             'abs_z', 'bust_y', 'bust_x', 'l_shoulder_y', 'l_shoulder_x', 'l_elbow_y', 'l_arm_z']
        if self._config == None:
            self._config = default_config
            self.__init_default__()
        else:
            self.__init_from_config__(self._config)

        self.head = [self.head_z, self.head_y]
        self.r_arm = [self.r_shoulder_y, self.r_shoulder_x,
                      self.r_arm_z, self.r_elbow_y]
        self.body = [self.abs_z, self.bust_y, self.bust_x]
        self.l_arm = [self.l_shoulder_y, self.l_shoulder_x,
                      self.l_arm_z, self.l_elbow_y]

        self.arms = [self.l_arm, self.r_arm]
        self._motors = {}
        for i in self.head + self.l_arm+self.r_arm+self.body:
            self._motors[i.id] = i

    def __init_from_config__(cls, config):

        for name in cls.motor_names:
            params = config['motors'][name]
            type_ = str(params['type'])
            if type_.startswith("MX"):
                MotorClass = Motor
            elif type_.startswith("AX"):
                MotorClass = Motor

            setattr(cls, name, MotorClass(
                id=params['id'],
                name=name,
                model=MotorType[type_.replace('-', '_')],
                direct=True if params['orientation'] == 'direct' else False,
                angle_limit=params['angle_limit'],
                offset=params['offset'],
            ))

    def __init_default__(self):
        self.l_elbow_y = Motor(
            name="l_elbow_y",
            offset=90,
            model=MotorType.MX_28,
            id=44,
            angle_limit=[0, 140],
            direct=True
        )

        self.head_y = Motor(
            name="head_y",
            offset=-20,
            model=MotorType.AX_12,
            id=37,
            angle_limit=[-80, 30],
            direct=False
        )
        self.r_arm_z = Motor(
            name="r_arm_z",
            offset=0.0,
            model=MotorType.MX_28,
            id=53,
            angle_limit=[-90, 90],
            direct=False
        )
        self.head_z = Motor(
            name="head_z",
            offset=0.0,
            model=MotorType.AX_12,
            id=36,
            angle_limit=[-100, 100],
            direct=False
        )
        self.r_shoulder_x = Motor(
            name="r_shoulder_x",
            offset=-90,
            model=MotorType.MX_28,
            id=52,
            angle_limit=[-90, 90],
            direct=False
        )
        self.r_shoulder_y = Motor(
            name="r_shoulder_y",
            offset=-90,
            model=MotorType.MX_28,
            id=51,
            angle_limit=[-140, 150],
            direct=False
        )
        self.r_elbow_y = Motor(
            name="r_elbow_y",
            offset=-90,
            model=MotorType.MX_28,
            id=54,
            angle_limit=[-140, 0],
            direct=False
        )
        self.l_arm_z = Motor(
            name="l_arm_z",
            offset=0.0,
            model=MotorType.MX_28,
            id=43,
            angle_limit=[-90, 90],
            direct=False
        )
        self.abs_z = Motor(
            name="abs_z",
            offset=0.0,
            model=MotorType.MX_28,
            id=33,
            angle_limit=[-80, 80],
            direct=True
        )
        self.bust_y = Motor(
            name="bust_y",
            offset=0.0,
            model=MotorType.MX_28,
            id=34,
            angle_limit=[-40, 36],
            direct=False
        )
        self.bust_x = Motor(
            name="bust_x",
            offset=0.0,
            model=MotorType.MX_28,
            id=35,
            angle_limit=[-20, 20],
            direct=False
        )
        self.l_shoulder_x = Motor(
            name="l_shoulder_x",
            offset=90,
            model=MotorType.MX_28,
            id=42,
            angle_limit=[-105, 105],
            direct=False
        )
        self.l_shoulder_y = Motor(
            name="l_shoulder_y",
            offset=90,
            model=MotorType.MX_28,
            id=41,
            angle_limit=[-150, 140],
            direct=True
        )

    def get_offset(self, name):
        return self.__dict__[name].offset

    def get_attribute(self, name):
        return self.__dict__[name]

    def get_motor_by_name(self, name):
        if not isinstance(name,Motor):
            return None
        return self.__dict__[name]

    def get_modelType(self, id):
        return self._motors[id].type

    def get_motor(self, id):
        if id not in self._motors.keys():
            print(f"Motor with Id {id} not found")
            return None
        return self._motors[id]

    def get_motorIds(self):
        return self._motors.keys()

    @property
    def motor_names(self):
        return self._motor_names

    @property
    def motors(self):
        return self._motors.values()


def default_config():
    config_path = os.path.join(
        Path(__file__).parent.parent, "config/ellie.json")
    with open(config_path) as f:
        config = json.load(f, object_hook=OrderedDict)
    return config


if __name__ == "__main__":
    config = default_config()
    motors = EllieMotors(config)
    print(motors.head)
