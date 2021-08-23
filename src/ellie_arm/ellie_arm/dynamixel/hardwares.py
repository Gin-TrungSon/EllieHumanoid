from ellie_arm.dynamixel.registers import *
from ellie_arm.dynamixel.sub_threading import *
from ellie_arm.dynamixel.trajectory import MinJerkTrajectory
from ellie_arm.dynamixel.ultis import MotorType
import numpy as np
from collections import defaultdict
from ellie_arm.dynamixel.ultis import *


class Motor:
    def __init__(self, name, id, model, offset, direct=True, angle_limit=[-90, 90], compliant=False, safe_compliant=False):
        self._name = name
        self._id = id
        self._type = model
        self._offset = offset
        self._angle_limit = angle_limit
        self._direct = direct
        self._compliant = compliant
        self._safe_compliant = safe_compliant

    @property
    def name(self):
        return self._name

    @property
    def id(self):
        return self._id

    @property
    def angle_limit(self):
        return self._angle_limit

    @property
    def type(self):
        return self._type

    @property
    def offset(self):
        return self._offset

    @property
    def direct(self):
        return self._direct


class DxlMotor(Motor, metaclass=MetaRegister):
    # registers = Motor.registers+['registers',
    #                                'goal_speed',
    #                                'compliant', 'safe_compliant',
    #                                'angle_limit']
    # id = DxlRegister()
    # name = DxlRegister()
    # model = DxlRegister()

    present_position = -1
    # goal_position = DxlPositionRegister(rw=True)
    # present_speed = DxlOrientedRegister()
    # moving_speed = DxlOrientedRegister(rw=True)
    # present_load = DxlOrientedRegister()
    # torque_limit = DxlRegister(rw=True)

    # lower_limit = DxlPositionRegister()
    # upper_limit = DxlPositionRegister()
    # present_voltage = DxlRegister()
    # present_temperature = DxlRegister()

    def __init__(self, id, name=None, model=None,
                 direct=True, offset=0.0,
                 broken=False,
                 angle_limit=None):

        super().__init__(name,id,model,offset,direct,angle_limit)
        self.__dict__['compliant'] = True

        # self._safe_compliance = SafeCompliance(self)
        # self.goto_behavior = 'dummy'
        # self.compliant_behavior = 'dummy'

        # self._broken = broken

        # self._read_synchronous = defaultdict(lambda: False)
        # self._read_synced = defaultdict(SyncEvent)

        # self._write_synchronous = defaultdict(lambda: False)
        # self._write_synced = defaultdict(SyncEvent)

        # if angle_limit is not None:
        #     self.__dict__['lower_limit'], self.__dict__['upper_limit'] = angle_limit

    def __repr__(self):
        return (f'<DxlMotor name={self.name} '
                f'id={self.id} '
                f'pos={self.present_position}>')

    @property
    def goal_speed(self):
        """ Goal speed (in degrees per second) of the motor.
            This property can be used to control your motor in speed. Setting a goal speed will automatically change the moving speed and sets the goal position as the angle limit.
            .. note:: The motor will turn until reaching the angle limit. But this is not a wheel mode, so the motor will stop at its limits.
            """
        return np.sign(self.goal_position) * self.moving_speed

    @goal_speed.setter
    def goal_speed(self, value):
        if abs(value) < np.finfo(np.float).eps:
            self.goal_position = self.present_position

        else:
            # 0.7 corresponds approx. to the min speed that will be converted into 0
            # and as 0 corredsponds to setting the max speed, we have to check this case
            value = np.sign(value) * 0.7 if abs(value) < 0.7 else value

            self.goal_position = np.sign(value) * self.max_pos
            self.moving_speed = abs(value)

    @property
    def compliant_behavior(self):
        return self._compliant_behavior

    @compliant_behavior.setter
    def compliant_behavior(self, value):
        if value not in ('dummy', 'safe'):
            raise ValueError(
                'Wrong compliant type! It should be either "dummy" or "safe".')

        if hasattr(self, '_compliant_behavior') and self._compliant_behavior == value:
            return

        self._compliant_behavior = value

        # Start the safe compliance behavior when the motor should be compliant
        if value == 'safe' and self.compliant:
            self._safe_compliance.start()

        if value == 'dummy':
            use_safe = self._safe_compliance.started
            if use_safe:
                self._safe_compliance.stop()
            self.compliant = self.compliant or use_safe

    @property
    def compliant(self):
        return None
        return bool(self.__dict__['compliant'])

    @compliant.setter
    def compliant(self, is_compliant):
        if self._safe_compliance.started and is_compliant:
            return

        if self.compliant_behavior == 'dummy':
            self._set_compliancy(is_compliant)

        elif self.compliant_behavior == 'safe':
            if is_compliant:
                self._safe_compliance.start()
            elif self._safe_compliance.started:
                self._safe_compliance.stop()

    def _set_compliancy(self, is_compliant):
        # Change the goal_position only if you switch from compliant to not compliant mode
        if not is_compliant and self.compliant:
            self.goal_position = self.present_position
        self.__dict__['compliant'] = is_compliant

    @property
    def angle_limit(self):
        return self.lower_limit, self.upper_limit

    @angle_limit.setter
    def angle_limit(self, limits):
        self.lower_limit, self.upper_limit = limits

    @property
    def goto_behavior(self):
        return self._default_goto_behavior

    @goto_behavior.setter
    def goto_behavior(self, value):
        if value not in ('dummy', 'minjerk', 'linear'):
            raise ValueError(
                'Wrong compliant type! It should be either "dummy", "minjerk" or "linear".')
        self._default_goto_behavior = value

    def goto_position(self, position, duration, control=None, wait=False):
        """ Automatically sets the goal position and the moving speed to reach the desired position within the duration. """

        if control is None:
            control = self.goto_behavior

        if control == 'minjerk':
            goto_min_jerk = MinJerkTrajectory(self, position, duration)
            goto_min_jerk.start()

            if wait:
                goto_min_jerk.wait_to_stop()


class DxlMXMotor(DxlMotor):
    """ This class represents the RX and MX robotis motor.
        This class adds access to:
            * PID gains (see the robotis website for details)
        """

    pid = []

    def __init__(self, id, name=None, model=None,
                 direct=True, offset=0.0, broken=False,
                 angle_limit=None):
        """ This class represents the RX and MX robotis motor.
            This class adds access to:
                * PID gains (see the robotis website for details)
            """
        super().__init__(id, name, model, direct,
                         offset, broken, angle_limit)

        self.max_pos = 180


class DxlAXRXMotor(DxlMotor):
    """ This class represents the AX robotis motor.
        This class adds access to:
            * compliance margin/slope (see the robotis website for details)
        """
    compliance_margin = -1.0
    compliance_slope = -1.0

    def __init__(self, id, name=None, model=None,
                 direct=True, offset=0.0, broken=False,
                 angle_limit=None):
        super().__init__(id, name, model, direct,
                         offset, broken, angle_limit)
        self.max_pos = 150


class SafeCompliance(ModifiedLoopThread):
    """ This class creates a controller to active compliance only if the current motor position is included in the angle limit, else the compliance is turned off. """

    def __init__(self, motor, frequency=50):
        ModifiedLoopThread.__init__(self, frequency)

        self.motor = motor

    def update(self):
        self.motor._set_compliancy(
            (min(self.motor.angle_limit) < self.motor.present_position < max(self.motor.angle_limit)))

    def teardown(self):
        self.motor._set_compliancy(False)
