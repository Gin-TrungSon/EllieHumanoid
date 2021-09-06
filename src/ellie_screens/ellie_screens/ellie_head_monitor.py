import os
from ellie_msgs.srv import String
from PyQt5.QtWidgets import QApplication, QDesktopWidget, QLabel
from PyQt5.QtGui import QMovie
from PyQt5.QtCore import QSize, QUrl, QTimer
from PyQt5.QtWebEngineWidgets import *
from threading import Thread
from rclpy.node import Node
import rclpy
import random
import datetime
import sys
class EyesAnimation (QDesktopWidget):
    def __init__(self):
        super().__init__()
        self.external_change = ""
        self.label_animation = QLabel()
        self.label_animation.setFixedSize(480, 240)
        self.motions = {}
        self.eyes_motion_path = os.path.join(
            os.path.dirname(__file__), "eye_motions")

        for file in os.listdir(self.eyes_motion_path):
            motion = os.path.join(self.eyes_motion_path, file)
            self.motions[file.replace(".gif", "")] = motion

        self.currentMotion = "idle"
        self.motion = QMovie(self.motions[self.currentMotion])
        self.motion.setScaledSize(QSize(480, 240))
        self.label_animation.setMovie(self.motion)
        self.motion.start()
        self.label_animation.setGeometry(self.screenGeometry(1))
        self.label_animation.show()
        self.timer = QTimer(self)
        self.timer_one = QTimer(self)
        self.timer_one.start(5000)
        self.timer_one.timeout.connect(self._timeout_run)
        self.motion.finished.connect(self._onFinished)

    def _onFinished(self):
        self.timer_one.stop()
        self.timer_one.start(5000)
        self._changeGIF()

    def _changeGIF(self):
        motionid = self.currentMotion
        if self.external_change != "":
            motionid = self.external_change
            self.external_change = ""
        else:
            while motionid == self.currentMotion:
                motionid = random.choices(
                    ["leftLooking", "smile", "blink", "rightLooking", "idle"], weights=[5, 3, 40, 5, 30], k=1)[0]
        self.currentMotion = motionid
        self.motion.stop()
        self.motion.setFileName(self.motions[self.currentMotion])
        self.motion.start()
        self.motion.loopCount()

    def changeEyeMotion(self, motionId):
        self.external_change = motionId

    def _timeout_run(self):

        sender = self.sender()
        current_time = datetime.datetime.now()

        if id(sender) == id(self.timer_one):
            self._changeGIF()

    def loopcount(self, move):
        if(move.currentFrameNumber() == move.frameCount()-1):
            return True
        else:
            return False

    def _startAnimation(self):
        self.motion.start()

    def _stopAnimation(self):
        self.motion.stop()
        self.close()

class EllieHeadMonitor(Node):
    def __init__(self):
        super().__init__("ellie_screen")
        self.changeEyesMotion_srv = self.create_service(
            String, "change_eyes_motion", self.changeEyesMotion_callback)

    def ui_init__(self):
        app = QApplication(sys.argv)
        self.monitor = QDesktopWidget().screenGeometry(1)
        self.eyes_animation = EyesAnimation()
        sys.exit(app.exec_())

    def changeEyesMotion_callback(self, request, response):
        print(request.request)
        try:
            self.changeEyesMotion(request.request)
            response.response = "eyes motion changed !"
        except Exception as e:
            response.response = e.__str__()
        return response

    def openUrl_callback(self, request, response):
        print(f"open : {request.request}")
        try:
            self.open(request.request)
            response.response = f"open : {request.request}"
        except Exception as e:
            response.response = e.__str__()
        return response

    def changeEyesMotion(self, motionId):
        self.eyes_animation.changeEyeMotion(motionId)

def main(args=None):
    rclpy.init(args=args)
    ellieHeadScreen = EllieHeadMonitor()
    thread_ = Thread(
        target=ros_shutdown, args=(ellieHeadScreen,), daemon=True)
    thread_.start()
    ellieHeadScreen.ui_init__()


def ros_shutdown(ellieHeadScreen):
    print("ros_spins")
    rclpy.spin(ellieHeadScreen)
    ellieHeadScreen.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
