# Copyright 2021 by Dong Trung Son, Nueremberg University.
# Email: trungsondo68839@th-nuernberg.de
# All rights reserved.
# This file is part of the Ellie-Project,
# and is released under the "MIT License Agreement". Please see the LICENSE
# file that should have been included as part of this package.

from ellie_arm.dynamixel.trajectory import MinimumJerkTrajectory
from ellie_arm.dynamixel.ultis import *
from dynamixel_sdk import *
import logging
import numpy as np
logger = logging.getLogger(__name__)


class DxlInterface:
    _protocol = 1.0

    @classmethod
    def get_used_ports(cls):
        return list(cls.__used_ports)

    def __init__(self, motors, port='/dev/ttyUSB0', baudrate=1000000, time_out=0.05):
        self._motors = motors
        self.open(port)

    def open(self, port):
        """Open port and set up read write methods

        Args:
            port ([type]): [description]
        """
        self.port = PortHandler(port)
        if self.port.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            exit()

        self.packetHandler = PacketHandler(self._protocol)
        self.groupSyncWritePosition = GroupSyncWrite(
            self.port, self.packetHandler, GOAL_POSITION, 2)
        self.groupSyncWriteVerlocity = GroupSyncWrite(
            self.port, self.packetHandler, MOVING_SPEED, 2)
        self.groupBulkReadPosition = GroupBulkRead(
            self.port, self.packetHandler)
        self.groupBulkReadVelocity = GroupBulkRead(
            self.port, self.packetHandler)
        for motor in self._motors.motors:
            self.groupBulkReadPosition.addParam(motor.id, PRESENT_POSITION, 2)
            self.groupBulkReadVelocity.addParam(motor.id, PRESENT_SPEED, 2)

    def factory_reset(self, dxl_id):
        """
        resets all settings of Dynamixel to default values .incl ID, Baudrate
        """
        self.packetHandler.factoryReset(self.port, dxl_id)

    @property
    def registered_motors(self):
        """Get all motors that have registed

        Returns:
            dict<motorID,Motor>: motors
        """
        return self._motors.motors

    def scan(self, ids=range(254)):
        """Scan avaiable motors

        Args:
            ids (list[int], optional): fessible motor ids. Defaults to range(254).

        Returns:
            lit[int]: list of motor ids
        """
        return [id for id in ids if self.ping(id)]

    def ping(self, dxl_id):
        """ping to a motor

        Args:
            dxl_id (int): motor id

        Returns:
            bool: return True if pingable
        """
        _, result, _ = self.packetHandler.ping(self.port, dxl_id)
        return True if result == COMM_SUCCESS else False

    def get_model(self, dxl_id):
        """Get motor model

        Args:
            dxl_id (int): motor id

        Returns:
            MotorType: type of motor  e.g MX_12, MX_24
        """
        return dynamixelModels[self.packetHandler.read2ByteTxRx(self.port, dxl_id, MODEL_NUMBER)[0]]

    def get_firmware_version(self, dxl_id):
        """Get firmware version

        Args:
            dxl_id (int): motor id

        Returns:
            Any: Protocol 1.0 or 2.0
        """
        return self.packetHandler.read1ByteTxRx(self.port, dxl_id, FIRMWARE_VERSION)

    def change_id(self, dxl_id, new_dxl_id):
        """change motor id

        Args:
            dxl_id (int): current motor id
            new_dxl_id (int): new motor id

        Raises:
            ValueError: rase a error if id is already used
        """
        if not self.ping(new_dxl_id):
            self.packetHandler.write1ByteTxRx(
                self.port, dxl_id, ID, new_dxl_id)
        else:
            raise ValueError(f'id {new_dxl_id} is already used.')

    def change_baudrate(self, dxl_id, new_baurate):
        """change communication baudrate of a motor

        Args:
            dxl_id (int): motor id
            new_baurate (int): new baudrate 

        Raises:
            ValueError: rase a error if baudrate is not supported
        """
        if new_baurate in AVAILABLE_BAUDRATE:
            self.packetHandler.write1ByteTxRx(
                self.port, dxl_id, BAUDRATE, new_baurate)
        else:
            raise ValueError(f' baudrate {new_baurate} is not supported')

    def get_status_return_level(self, dxl_id):
        """The Stuatus Return Level decides how to return Status Packet when DYNAMIXEL receives an Instruction Packet.

        Args:
            dxl_id (int): motor id

        Returns:
            int: 0 Returns the Status Packet for PING Instruction only
                 1 Returns the Status Packet for PING and READ Instruction
                 2 Returns the Status Packet for all Instructions
        """
        return self.packetHandler.read1ByteTxRx(self.port, dxl_id, STATUS_RETURN_LEVEL)

    def get_pid_gain(self, dxl_id):
        """Get PID value

        Args:
            dxl_id (int): motor id

        Returns:
            list<int>: PID value
        """
        return [self.packetHandler.read1ByteTxRx(self.port, dxl_id, P_GAIN),
                self.packetHandler.read1ByteTxRx(self.port, dxl_id, I_GAIN),
                self.packetHandler.read1ByteTxRx(self.port, dxl_id, D_GAIN), ]

    def set_pid_gain(self, dxl_id, P, I, D):
        """[summary]

        Args:
            dxl_id (int): motor id
            P (int): P value
            I (int): I value
            D (int): D value
        """
        self.packetHandler.writeByteTxRx(self.port, dxl_id, P_GAIN, P)
        self.packetHandler.writeByteTxRx(self.port, dxl_id, I_GAIN, I)
        self.packetHandler.writeByteTxRx(self.port, dxl_id, D_GAIN, D)

    def switch_led_on(self, dxl_id):
        """Turn the Led on

        Args:
            dxl_id (int): motor id
        """
        self.packetHandler.write1ByteTxRx(self.port, dxl_id, STATUS_LED, 1)

    def switch_led_off(self, dxl_id):
        """Turn the Led off

        Args:
            dxl_id (int): motor id
        """
        self.packetHandler.write1ByteTxRx(self.port, dxl_id, STATUS_LED, 0)

    def enable_torque(self, dxl_id):
        """Enable torque

        Args:
            dxl_id (int): motor id
        """
        self.packetHandler.write1ByteTxRx(self.port, dxl_id, TORQUE_ENABLE, 1)

    def disable_torque(self, dxl_id):
        """Disable torque

        Args:
            dxl_id (int): motor id
        """
        self.packetHandler.write1ByteTxRx(self.port, dxl_id, TORQUE_ENABLE, 0)

    def present_position(self, dxl_id):
        """Get current resolution divider motor position

        Args:
            dxl_id (int): motor id

        Returns:
            int: current resolution divider position 
        """
        return self.packetHandler.read2ByteTxRx(self.port, dxl_id, PRESENT_POSITION)[0]

    def present_position_degree(self, dxl_id, model_type):
        """Get current motor position in grads

        Args:
            dxl_id (int): motor id
            model_type (MotorType): type of motor

        Returns:
            float: current motor position in grads
        """
        return dxl_to_degree(self.packetHandler.read2ByteTxRx(self.port, dxl_id, PRESENT_POSITION)[0], model_type)

    def present_speed(self, dxl_id, model_type):
        """Get current moving speed 
           0 ~ 2,047 (0x000 ~ 0x7FF) can be used.
           If a value is in the rage of 0 ~ 1,023 then the motor rotates to the CCW direction.
           If a value is in the rage of 1,024 ~ 2,047 then the motor rotates to the CW direction.

        Args:
            dxl_id (int): motor id
            model_type (MotorType): type of motor

        Returns:
            int: current moving speed  0 ~ 2047
        """
        return dxl_to_speed(self.packetHandler.read2ByteTxRx(self.port, dxl_id, PRESENT_SPEED)[0], model_type)

    def _set_goal(self, dxl_id, value):
        """Set goal position for a motor

        Args:
            dxl_id (int): motor id
            value (float): goal position in degrees
        """
        motor = self._motors.get_motor(dxl_id)
        dxl_value = clamp(value,
                          motor.angle_limit[0], motor.angle_limit[1], dxl_id)
        result, error = self.packetHandler.write2ByteTxRx(
            self.port, dxl_id, GOAL_POSITION, degree_to_dxl(dxl_value, motor.type))
        if result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(result))
        elif error != 0:
            print("%s" % self.packetHandler.getRxPacketError(error))

    def set_moving_speed(self, dxl_id, value):
        """Set moving speed to a motor

        Args:
            dxl_id (int): motor id
            value (float): moving speed degrees/s
        """
        motor = self._motors.get_motor(dxl_id)
        dxl_value = speed_to_dxl(value, motor.type)
        result, error = self.packetHandler.write2ByteTxRx(
            self.port, dxl_id, MOVING_SPEED, dxl_value)
        if result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(result))
        elif error != 0:
            print("%s" % self.packetHandler.getRxPacketError(error))

    def goto_position(self, dxl_id, position, duration):
        """Go to postion in a known interval

        Args:
            dxl_id (int): motor id
            position (float): goal position in degrees
            duration (float): time interval for the motion implementing

        Raises:
            ValueError: rase a error if motor not valid
        """
        model_type = self._motors.get_modelType(dxl_id)
        present_position = dxl_to_degree(
            self.present_position(dxl_id), model_type)
        trajectory = MinimumJerkTrajectory(
            present_position, position, duration).get_generator()
        time_ = 0
        while time_ < duration:
            start = time.time()
            try:
                self._set_goal(dxl_id, trajectory(time_))
            except AttributeError:
                raise ValueError("motors are not registed yet !")
            finally:
                dt = time.time()-start
                time_ += dt

    def goto_position_group(self, duration, ids,  positions, velocities=None):
        """Go to position synchrony

        Args:
            duration (float): time interval for the motion implementing
            ids (list[int]): mototr ids
            positions (list[float]): goal positions
            velocities (float, optional): velocities. Defaults to None.
        """
        data = {}
        if velocities == None:
            for i, p in zip(ids, positions):
                motor = self._motors.get_motor(i)
                data[motor.name] = [p-motor.offset, 0]
        else:
            for i, p, v in zip(ids, positions, velocities):
                motor = self._motors.get_motor(i)
                data[motor.name] = [p-motor.offset, v]
        self.execute_trajectories(data, duration)

    def goto_position_sync(self, positions, step_duration):
        """go to stamped posititon in a KDdicht

        Args:
            positions (KDdict): a dictionary represens positions with time stamps
            duration (float): time interval for the motion implementing
        """
        time_ = 0
        self.execute_trajectories(positions[time_], 2)
        _lastcmd = time.time() 
        _nextcmd = _lastcmd+step_duration
        _timeout = _lastcmd + len(positions)*step_duration
        while _nextcmd <= _timeout:
            if time.time() > _nextcmd:
                item = positions[time_]
                self._goto_position_sync(item)
                _lastcmd = _nextcmd
                _nextcmd += step_duration
                time_ += step_duration

    def _goto_position_sync(self, item, max_speed=-1):
        """Go to goal position synchrony

        Args:
            item (list): list of motor state 
                         eg.  "r_elbow_y": [
                                 -10.090000000000003,
                                 -0.0
                                ],
            max_speed (int, optional): maximal verlocity. Defaults to -1.

        Raises:
            ValueError: raise a error if motor invalid
        """
        try:
            for name, value in item.items():
                motor = self._motors.get_attribute(name)

                goal_position = value[0]

                if max_speed >= 0:
                    velocity = np.sign(-goal_position +
                                       self.present_position(motor.id))*max_speed
                    print(f" {speed_to_dxl(velocity,motor.type)} ")
                else:
                    velocity = value[1]

                print(
                    f'id {motor.id} verlocity {velocity} goal {goal_position} velocity_dxl : {speed_to_dxl(velocity,motor.type)}  goal_dxl : {degree_to_dxl(goal_position,motor.type)}')

                goal_position = clamp(
                    goal_position + motor.offset, motor.angle_limit[0], motor.angle_limit[1])

                self.groupSyncWriteVerlocity.addParam(
                    motor.id, speed_to_dxl(velocity, motor.type).to_bytes(2, 'little'))
                self.groupSyncWritePosition.addParam(motor.id, degree_to_dxl(
                    goal_position, motor.type).to_bytes(2, 'little'))
            velocity_comm_result = self.groupSyncWriteVerlocity.txPacket()
            dxl_comm_result = self.groupSyncWritePosition.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        except AttributeError:
            raise ValueError("motors are not registed yet !")
        finally:

            self.groupSyncWritePosition.clearParam()
            self.groupSyncWriteVerlocity.clearParam()

    def read_data_sync(self):
        """Read motors state sychrony

        Raises:
            ValueError: raise a error if motor invalid

        Returns:
            dict<motorId,<position,velocity>>: motor position states
        """
        try:
            data = {}

            for motor in self._motors.motors:
                position = self.present_position_degree(
                    motor.id, motor.type)-motor.offset
                velocity = self.present_speed(motor.id, motor.type)
                data[motor.name] = [position, velocity]
            return data
        except AttributeError:
            raise ValueError("motors are not registed yet !")

    def record_trajectories(self, period, start_time, old_positions, data):
        """Create Minjerk trajectory

        Args:
            period (float): periode for position updating
            start_time (float): start time
            old_positions (dict): last position of motors
            data (KDdict): calculated trajectory in KDdict
        """
        trajectories = {}
        motor_names = []
        velocities = []
        start = time.time()
        for name, value in old_positions.items():
            motor_names.append(name)
            motor = self._motors.get_attribute(name)
            init_position = value[0]
            init_position = clamp(
                init_position,  motor.angle_limit[0], motor.angle_limit[1], motor.id)
            trajectories[name] = MinimumJerkTrajectory(init_position, self.present_position_degree(
                motor.id, motor.type) - motor.offset,  period).get_generator()
            velocities.append(self.present_speed(motor.id, motor.type))
        time_ = period
        duration = time.time()-start
        while True:
            pos = {}
            for name, vel in zip(motor_names, velocities):
                pos[name] = [float(trajectories[name](time_)) -
                             self._motors.get_motor_by_name(name).offset, vel]
            data[str(start_time + time_)] = pos
            if time_ + period > duration:
                old_positions = pos
                start_time = time_
            else:
                time_ += period

    def execute_trajectories(self, positions, duration):
        """Execute a trajectory from KDdict

        Args:
            positions (KDdict): position states
            duration (float): time interval for the motion implementing

        Raises:
            ValueError: raise a error if motor invalid
        """
        trajectories = {}
        motor_names = []
        for name, value in positions.items():
            motor_names.append(name)
            motor = self._motors.get_attribute(name)
            goal_position = value[0]
            goal_position = clamp(
                goal_position + motor.offset,  motor.angle_limit[0], motor.angle_limit[1])
            trajectories[name] = MinimumJerkTrajectory(dxl_to_degree(self.present_position(
                motor.id), motor.type), goal_position, duration).get_generator()
        time_ = 0
        while time_ < duration:
            start = time.time()
            try:
                for name in motor_names:
                    motor = self._motors.get_attribute(name)
                    self.groupSyncWritePosition.addParam(
                        motor.id, degree_to_dxl(trajectories[name](time_), motor.type).to_bytes(2, 'little'))
                    dxl_comm_result = self.groupSyncWritePosition.txPacket()
                    if dxl_comm_result != COMM_SUCCESS:
                        print("%s" % self.packetHandler.getTxRxResult(
                            dxl_comm_result))
            except AttributeError:
                raise ValueError("motors are not registed yet !")
            finally:
                dt = time.time()-start
                time_ += dt
                self.groupSyncWritePosition.clearParam()


if __name__ == "__main__":
    x = DxlInterface()
    print(x.get_firmware_version(53))
    print(x.get_model(53))
    # print(x.scan())
    # input("swith led on")
    # x.switch_led_on(13)
    # input("swith led off")
    # x.switch_led_off(13)
    while True:
        try:
            present_pos, _, _ = x.present_position(53)
            print(present_pos)
            degree = float(dxl_to_degree(present_pos, MotorType.MX_28))
            print(f"degree {degree}")
            print(degree_to_dxl(degree, MotorType.MX_28))
            value = float(input("goal degree : "))
            x._set_goal(53, value)
        except:
            break
