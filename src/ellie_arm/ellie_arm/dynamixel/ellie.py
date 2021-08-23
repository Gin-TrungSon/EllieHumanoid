import time
from typing import OrderedDict
from ellie_arm.dynamixel.ellie_motors import EllieMotors
from ellie_arm.dynamixel.hardwares import DxlAXRXMotor, DxlMXMotor
from ellie_arm.dynamixel.inverse_kinematics import IKChain
from ellie_arm.dynamixel.behavior import Behavior
from ellie_arm.dynamixel.communication import DxlInterface
from ellie_arm.robot.robot import Robot
import pathlib
import json
import logging
from ellie_arm.dynamixel.ultis import *
import os
logger = logging.getLogger(__name__)
MAX_SETUP_TRIALS = 10
BEHAVIORS_DIR = "src/ellie/ellie_body/actions"

class EllieBase(Robot):
    def __new__(cls, config=None):
        for _ in range(MAX_SETUP_TRIALS):
            pass

    def from_json(json_file, sync=True, strict=True):
        with open(json_file) as f:
            config = json.load(object_hook=OrderedDict)

class Ellie():

    def __init__(self, config = None) :
        self._urdf = ""
        self.motors = EllieMotors(config)
        self.dxl_interface = DxlInterface(self.motors)
        self.l_arm_chain = IKChain.from_ellie(ellie= self, motors=self.motors.body + self.motors.l_arm,
        reversed_motors=[self.motors.l_shoulder_x],
        passiv=self.motors.body,
        tip =[0,0.18,0])

        self.r_arm_chain = IKChain.from_ellie(ellie= self, motors=self.motors.body + self.motors.r_arm,
        passiv=self.motors.body,
        tip =[0,0.18,0],
        reversed_motors=[self.motors.r_elbow_y,self.motors.r_shoulder_x])

        self.recording_behavior = None
        self.behaviors = {}



    @property
    def urdf(self):
        if self._urdf == "" :
            self._urdf = os.path.join(pathlib.Path(__file__).parent.parent,"urdf/ellie.urdf")
        return self._urdf

    def attach_behavior(self, behavior_path):
        key =pathlib.Path(behavior_path).stem
        with open(behavior_path) as f:
            self.behaviors[key]=Behavior.load(key,self.dxl_interface,f)

    def remove_behavior(self, behavior_id):
        if behavior_id in self.behaviors.keys():
            del self.behaviors[behavior_id]
            print(f"Remove behavior {behavior_id}")

    def execute_behavior(self,behavior_id):
        self.behaviors[behavior_id].execute()

    def start_recorder(self, id, frequency = 50):
        self.recording_behavior = Behavior(id,self.dxl_interface, frequency)
        self.recording_behavior.start_recorder()
    def stop_recorder(self):
        if self.recording_behavior == None:
            print("No recorder started yet ")
        else:
            self.recording_behavior.stop_recorder()

    def resume_recorder(self):
        if self.recording_behavior == None:
            print("No recorder started yet ")
        else:
            self.recording_behavior.resume_recorder()
    
    def replay(self):
        if self.recording_behavior == None:
            print("No recorder started yet ")
        else:
            self.recording_behavior.replay()

    def save_recorder_file(self, dir = BEHAVIORS_DIR ):
        if self.recording_behavior == None:
            print("No recorder started yet ")
        else:
            file = os.path.join(dir,self.recording_behavior.id+".move")
            self.recording_behavior.save(file)
            self.behaviors[self.recording_behavior.id] = self.recording_behavior
            self.delede_recorder()

    def delede_recorder(self):
        self.recording_behavior = None

if __name__ == "__main__":
    # import os
    # config_path = "src/ellie_arm/ellie_arm/config/ellie.json"
    # with open(config_path) as f:
    #     config = json.load(f, object_hook=OrderedDict)
   
    ellie = Ellie()
    # path = pathlib.Path("src/ellie/ellie_body/actions")
    # files = [e for e in path.iterdir() if e.is_file()]
    # for i in files:
    #     ellie.attach_behavior(i)
    #     ellie.execute_behavior(i.name.replace('.move',''))
    #     ellie.remove_behavior(i.name.replace('.move',''))
    #     break

    # ellie.attach_behavior("src/ellie/ellie_body/actions/test.move")
    # ellie.execute_behavior("test")
    # values =ellie.r_arm_chain.forward_kinematics(ellie.r_arm_chain.convert_to_ik_angles([0]*7))[:3,3]
    # ellie.goto(values,5)
    # print(values)
    # print(ellie.r_arm_chain.inverse_kinematics(values))
    #ellie.goto(values,10)

    # values = [0.08427061 ,0.0132507 , 0.36750004]
    # values =ellie.r_arm_chain.forward_kinematics(ellie.r_arm_chain.convert_to_ik_angles([0,0,0,-90,-90,0,-90]))[:3,3]
    # print(values)
    # ellie.r_arm_chain.goto(values,5)
    # values = [-0.10220375, -0.1802107 ,  0.36749935]
    # values =ellie.r_arm_chain.forward_kinematics(ellie.r_arm_chain.convert_to_ik_angles([0,0,0,-90,-90,90,-90]))[:3,3]
    # ellie.r_arm_chain.goto(values,5)

    
    # values =ellie.l_arm_chain.forward_kinematics(ellie.l_arm_chain.convert_to_ik_angles([0,0,0,90,90,0,90]))[:3,3]
    # print(values)
    # ellie.l_arm_chain.goto(values,5)
    # values =ellie.l_arm_chain.forward_kinematics(ellie.l_arm_chain.convert_to_ik_angles([0,0,0,90,90,-90,90]))[:3,3]
    # ellie.l_arm_chain.goto(values,5)

    # values = [-0.03330168 ,-0.1477004 , 0.41099998]
    # ellie.goto(values,10)
    # for i in ellie.motors.r_arm:
    #     ellie.dxl_interface.disable_torque(i.id) 
    #ellie.execute_behavior('behave_open')
    #ellie.execute_behavior('behave_grisou')

    # positions = self.behaviors[behavior_id].stamped_positions
    # duration = 1/float( self.behaviors[behavior_id].frame_rate)
    # self.controllers.goto_position_sync(positions,duration)
    #ellie.dxl_interface.goto_position(34,0,2)
    #ellie.dxl_interface.goto_position(35,0,2)
    input("start recording : "  )
    ellie.start_recorder("test")
    input("Stop : ")
    ellie.stop_recorder()
    input("Replay :")
    ellie.replay()
    input("Save : " )
    ellie.save_recorder_file()



    
