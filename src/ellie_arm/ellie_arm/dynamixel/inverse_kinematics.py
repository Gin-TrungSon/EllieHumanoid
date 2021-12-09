# Copyright 2021 by Dong Trung Son, Nueremberg University.
# Email: trungsondo68839@th-nuernberg.de
# All rights reserved.
# This file is part of the Ellie-Project,
# and is released under the "MIT License Agreement". Please see the LICENSE
# file that should have been included as part of this package.

from ikpy.chain import Chain
from ikpy.urdf.URDF import get_chain_from_joints
from numpy import deg2rad, rad2deg, array, eye
import numpy as np
# import matplotlib.pyplot as plt
# import ikpy.utils.plot as plot_utils

class IKChain(Chain):
    @classmethod
    def from_ellie(cls, ellie, motors, passiv, tip, reversed_motors=[]):

        chain_elements = get_chain_from_joints(
            ellie.urdf, [motor.name for motor in motors])

        activ = [False] + [motor not in passiv for motor in motors] + [False]
        print(activ)

        chain = cls.from_urdf_file(ellie.urdf, base_elements=chain_elements,
                                   last_link_vector=tip,
                                   active_links_mask=activ)
        chain.dxl_interface = ellie.dxl_interface
        chain.motors = [getattr(ellie.motors, l.name)
                        for l in chain.links[1:-1]]

        for m, l in zip(chain.motors, chain.links[1:-1]):
            if m in reversed_motors:
                lower_limit = -m.angle_limit[1]
                upper_limit = -m.angle_limit[0]
            else:
                lower_limit = m.angle_limit[0]
                upper_limit = m.angle_limit[1]
            l.bounds = tuple(map(deg2rad, (lower_limit, upper_limit)))

        chain._reversed = array(
            [(-1 if m in reversed_motors else 1) for m in motors])
        print(chain._reversed)
        return chain

    @property
    def joints_position(self):
        """ Returns the joints position of all motors [°]. """
        return [m.present_position for m in self.motors]

    @property
    def end_effector(self):
        """ Returns the position of the end effector [m]. """
        angles = self.convert_to_ik_angles(self.joints_position)
        return self.forward_kinematics(angles)[:3, 3]

    @property
    def present_joints_position(self):
        return [self.dxl_interface.present_position_degree(motor.id, motor.type) for motor in self.motors]

    def goto(self, position, duration, accurate=None):
        if len(position) != 3:
            raise ValueError('Position should be a list [x, y, z]!')
        self._goto(np.array(position), duration, accurate)

    def _goto(self, pose, duration, plot=False):

        joints_position = self.present_joints_position
        for i in range(3):
            joints_position[i] = 0  # always set motor 33,34,35 to 0 position
        q0 = self.convert_to_ik_angles(joints_position)
        q = self.inverse_kinematics(pose, initial_position=q0)

        joints = self.convert_from_ik_angles(q)
        for i in range(3):
            joints[i] = 0
        # if plot:
            
        #     fig, ax = plot_utils.init_3d_figure()
        #     self.plot(q, ax, pose)
        #     self.plot(joints=q0, ax=ax)

        #     plt.xlim(-0.5, 0.5)
        #     plt.ylim(-0.5, 0.5)
        #     plt.show()

        positions = {}
        for m, pos in list(zip(self.motors, joints)):
            positions[m.name] = [pos-m.offset]

        self.dxl_interface.execute_trajectories(positions, duration)

    def convert_to_ik_angles(self, joints):
        if len(joints) != len(self.motors):
            raise ValueError(
                'Incompatible data, len(joints) should be {}!'.format(len(self.motors)))

        raw_joints = joints
        raw_joints *= self._reversed

        return [0] + [deg2rad(j) for j in raw_joints] + [0]

    def convert_from_ik_angles(self, joints):
        if len(joints) != len(self.motors) + 2:
            raise ValueError(
                'Incompatible data, len(joints) should be {}!'.format(len(self.motors) + 2))

        joints = [rad2deg(j) for j in joints[1:-1]]
        joints *= self._reversed
        return joints


if __name__ == "__main__":
    print(IKChain.from_urdf_file("src/ellie_arm/ellie_arm/urdf/ellie.urdf"))
