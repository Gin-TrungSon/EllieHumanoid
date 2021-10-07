# Copyright 2021 by Dong Trung Son, Nueremberg University.
# Email: trungsondo68839@th-nuernberg.de
# All rights reserved.
# This file is part of the Ellie-Project,
# and is released under the "MIT License Agreement". Please see the LICENSE
# file that should have been included as part of this package.

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from ellie_msgs.action import ExecuteBehavior
from ellie_msgs.action import ChainPosition
from ellie_msgs.action import JointPosition
from ellie_msgs.srv import String
from ellie_arm.dynamixel.ellie_arm import EllieArm
from rcl_interfaces.msg import ParameterDescriptor
from pathlib import Path
import os

IDs = [36,37,33,34,35,41,42,43,44,51,52,53,54]
IDLE = [0,0,0,0,0,80,5,0,0,-80,-5,0,0]
class Ros2Interface(Node):

    def __init__(self):
        super().__init__("ellie_action_server")
        self.declare_parameter("config_path","")

        self.ellie_arm = EllieArm()
        values = self.ellie_arm.r_arm_chain.forward_kinematics(
        self.ellie_arm.r_arm_chain.convert_to_ik_angles([0, 0, 0, -90, -90, 0, -90]))[:3, 3]
        print(values)
        path = Path(os.path.join(Path( __file__).parent.parent, "actions"))
        files = [e for e in path.iterdir() if e.is_file()]
        for i in files:
            self.ellie_arm.attach_behavior(i)
        self.get_logger().info(f"attached behaviors {self.ellie_arm.get_attached_behaviors()}")
        self._execute_behavior = ActionServer(self,ExecuteBehavior,"execute_behavior",self.execute_behavior_callback)
        self._goto_joint_position = ActionServer(self,JointPosition,"goto_joint_position",self.goto_joint_position_callback)
        self._goto_position = ActionServer(self,ChainPosition,"goto_position",self.goto_position_callback)
        self._start_recording = self.create_service(String,"start", self.start_recording)
        self._stop_recording = self.create_service(String, "stop",self.stop_recording)
        self._save_recorded_file= self.create_service(String,"save", self.save)
        self._replay = self.create_service(String,"replay",self.replay)
        self._resume = self.create_service(String, "resume", self.resume)
        self._execute_behavior_srv = self.create_service(String,"execute_behavior",self.execute_behavior_srv)
        self.gotoIdle()

    def gotoIdle(self, duration =5):
        self.ellie_arm.dxl_interface.goto_position_group(duration,IDs,IDLE)

    def start_recording(self, request, response):
        self.ellie_arm.start_recorder(str(request.request))
        self.get_logger().info("Recording ...")
        response.response = "Recording ..."
        return response
    
    def stop_recording(self,request,response):
        self.ellie_arm.stop_recorder()
        self.get_logger().info("Stopped Recording ...")
        response.response = "Stopped Recording ..."
        return response

    def replay(self, request, response):
        self.ellie_arm.replay()
        self.get_logger().info("Replaying ...")
        response.response = "Replaying ..."
        return response

    def save( self, request, response):
        if request.request != "" and Path(request.request).is_dir():
            self.ellie_arm.save_recorder_file(request)
        else:
            self.ellie_arm.save_recorder_file()
        self.get_logger().info("Saved ...")
        response.response = "Saved ..."
        return response

    def resume(self, request, response):
        self.ellie_arm.resume_recorder()
        self.get_logger().info("Continute to recording ...")
        response.response = "Continute to recording ..."
        return response

    def execute_behavior_srv(self,request, response):
        id = request.request
        if id in self.ellie_arm.get_attached_behaviors():
            try :
                self.get_logger().info(f"Executing behavior {id} ...")
                self.ellie_arm.execute_behavior(id)
                response.response= f"Executed behavior {id}"
            except SystemError as e:
                response.response = e.__str__()
        return response

    def goto_position_callback(self, goal_handle):
        chain_id = goal_handle.request.chain_id
        position = goal_handle.request.position
        duration = goal_handle.request.duration
        if chain_id in [0,1] and len(position)==3:
            self.ellie_arm.goto_position(chain_id,position,duration)
            goal_handle.succeed()
            result = ChainPosition.Result()
            chain_name ="Left_Arm" if chain_id == 0 else "Right_Arm"
            result.result = f"Chain [{chain_name}]  goal_position {position}  in {duration} seconds "
            return result
        else :
            goal_handle.canceled()
            result = ChainPosition.Result()
            result.result = f"chain_id should be 0 (left) or 1 (right) "
            return result
            
    def goto_joint_position_callback(self, goal_handle):
        id = goal_handle.request.id
        position = goal_handle.request.position
        duration = goal_handle.request.duration
        if self.ellie_arm.isIdExisted(id):
            self.ellie_arm.goto_joint_position(id, position,duration)
            goal_handle.succeed()
            result = JointPosition.Result()
            result.result = f"MotorID [{id}]  goal_position {position}  in {duration} seconds "
            return result
        else :
            goal_handle.canceled()
            result = JointPosition.Result()
            result.result = f"ID {id} not found"
            return result
    

    def execute_behavior_callback(self, goal_handle):
        id = goal_handle.request.name
        if id not in self.ellie_arm.get_attached_behaviors():
            self.get_logger().info(f"Behavior {id} not found")
            goal_handle.canceled()
            result = ExecuteBehavior.Result()
            result.result = f"Behavior {id} not found"
            return result
        else:
            self.get_logger().info(f"Executing behavior {id}")
            try :
                self.ellie_arm.execute_behavior(id)
            except:
                goal_handle.abort()
            goal_handle.succeed()
            result = ExecuteBehavior.Result()
            result.result = "executed"
            return result


def main(args=None):
    rclpy.init(args=args)

    interface = Ros2Interface()
    rclpy.spin(interface)

if __name__=="__main__":
    main()