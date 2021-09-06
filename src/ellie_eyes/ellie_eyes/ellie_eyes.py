# Copyright 2021 by Dong Trung Son, Nueremberg University.
# Email: trungsondo68839@th-nuernberg.de
# All rights reserved.
# This file is part of the Ellie-Project,
# and is released under the "MIT License Agreement". Please see the LICENSE
# file that should have been included as part of this package.

import sys
sys.path.append("")
from sensor_msgs.msg import CompressedImage
import json
from std_msgs.msg import String
import cv2.cv2 as cv2
from imutils.video import VideoStream
from ellie_eyes.object_detection_lite.lite_detection import *
from ellie_eyes.face_recognition.factical_face_rec import FaceRecognition
from time import sleep
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
import rclpy
from ellie_msgs.msg import DetectedInfo, Position

class EllieEyes(Node):
    def __init__(self):
        super().__init__("ellie_eyes")
        print("Initiating . . .")
        self.declare_parameter("flag_publishing", 1, ParameterDescriptor(
            description=" 0 : dont publish \n 1 : publish original image only \n 2 : publish handled image only  \n 3 publish all",
        ))
        self.declare_parameter("use_PiCamera", False)
        self.declare_parameter("object_detection_threshold", 0.6)
        self.declare_parameter("display", False)
        self.declare_parameter("period", 2.0)

        self.resolution = (640, 480)
        self.usePicamera = self.get_parameter(
            "use_PiCamera").get_parameter_value().bool_value
        self.object_detection_threshold = self.get_parameter(
            "object_detection_threshold").get_parameter_value()._bool_value
        self.flag_publishing = int(self.get_parameter(
            "flag_publishing").get_parameter_value().integer_value)
        self.display = self.get_parameter(
            "display").get_parameter_value().bool_value
        self.period = self.get_parameter(
            "period").get_parameter_value().double_value

        self.bridge = CvBridge()
        self.original_img_publisher = self.create_publisher(
            CompressedImage, "original_image", 1)
        self.handled_img_publisher = self.create_publisher(
            CompressedImage, "handled_image", 1)
        self.recognized_person_publisher = self.create_publisher(
            DetectedInfo, "recognized_person", 1)
        self.detected_objects_publisher = self.create_publisher(
            DetectedInfo, "detected_objects", 1)

        self.face_recogition = FaceRecognition()
        self.object_detection = Lite_Detector()
        if self.usePicamera:
            self.stream = VideoStream(
                usePiCamera=True, resolution=self.resolution, framerate=24)
        else:
            self.stream = VideoStream(resolution=self.resolution, framerate=24)
        self._obj_boxes = []
        self._obj_classes = []
        self._recognized_person = []
        self._centers = []
        self._stopped = False
        self.stream.start()
        # wait for camera starting
        sleep(2)
        print(self.flag_publishing)
        self.timer = self.create_timer(self.period, self.update)

    def update(self):
        frame = self.stream.read()
        self._obj_frame, self._obj_boxes, self._obj_classes, self.scores = self.object_detection.inference(
            frame)
        self.face_frame, self._recognized_person, self.face_centers = self.face_recogition.inference(
            frame)
        handled_frame, object_centers, object_classes = self.object_detection.draw_bbox(
            self.face_frame, self._obj_boxes, self._obj_classes, self.scores)
        if self.flag_publishing == 1:
            if frame.all() != None:
                print("Publish new msg")
                self.original_img_publisher.publish(
                    self.convertToRosImage(frame))
        elif self.flag_publishing == 2:
            if handled_frame.all() != None:
                self.handled_img_publisher.publish(
                    self.convertToRosImage(handled_frame))
        elif self.flag_publishing == 3:
            if handled_frame.all() != None and frame.all() != None:
                self.original_img_publisher.publish(
                    self.convertToRosImage(frame))
                self.handled_img_publisher.publish(
                    self.convertToRosImage(handled_frame))

        if len(object_centers) > 0:
            self.detected_objects_publisher.publish(
                self.generate_msg(object_centers, object_classes))
        if len(self.face_centers) > 0:
            self.recognized_person_publisher.publish(
                self.generate_msg(self.face_centers, self._recognized_person))
        if self.display:
            if handled_frame.all() != None:
                cv2.imshow("frame", f)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    cv2.destroyAllWindows()

    def generate_msg(self, centers, classes):
        msg = DetectedInfo()
        for i in range(len(centers)):
            msg.names.append(classes[i])
            position = Position() 
            position.x = centers[i][0]/self.resolution[0] -0.5
            position.y = 0.5-centers[i][1]/self.resolution[1] 
            msg.positions.append(position)
        print(msg)
        return msg

    def convertToRosImage(self, frame):
        return self.bridge.cv2_to_compressed_imgmsg(frame)

    def on_exit(self):
        self.stream.stop()
        cv2.destroyAllWindows()
        self._close()

    def _close(self):
        self._stopped = True

    def get_registeredPerson(self):
        return self._recognized_person

    def get_objects_in_frame(self):
        return self._obj_classes

    def has_someone_in_frame(self):
        return "person" in self._obj_classes

    def nof_people_in_frame(self):
        return len([i for i in self._obj_classes if i == "person"])


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    ellie_eyes = EllieEyes()

    rclpy.spin(ellie_eyes)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ellie_eyes.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
