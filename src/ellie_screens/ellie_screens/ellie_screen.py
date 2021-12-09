# Copyright 2021 by Dong Trung Son, Nueremberg University.
# Email: trungsondo68839@th-nuernberg.de
# All rights reserved.
# This file is part of the Ellie-Project,
# and is released under the "MIT License Agreement". Please see the LICENSE
# file that should have been included as part of this package.
import sys
sys.path.append("")
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



class EllieScreen(Node):
    def __init__(self):
        super().__init__("ellie_screen")
        self.changeEyesMotion_srv = self.create_service(
            String, "change_eyes_motion", self.changeEyesMotion_callback)
        self.openUrl_srv = self.create_service(
            String, "open_url", self.openUrl_callback)

    def ui_init__(self):
        app = QApplication(sys.argv)
        self.monitor = QDesktopWidget().screenGeometry(1)
        self.eyesBlinking = EyesBlinking()
        self.webView = OpenWebView()
        app.exec()

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
        self.eyesBlinking.changeEyeMotion(motionId)

    def open(self, url):
        self.webView.open(url)


class EyesBlinking (QDesktopWidget):
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


class OpenWebView(QDesktopWidget):
    def __init__(self):
        super().__init__()
        self.external_url = ""
        self.webview = QWebEngineView()
        self.webview.load(QUrl("https://www.th-nuernberg.de/"))
        self.screen(0)
        self.webview.showFullScreen()
        self.webview.mousePressEvent = lambda: self._resetTimer()
        self.timer = QTimer(self)
        self.timer.start(1000)
        self.timer.timeout.connect(self._onSignal_changed)
        self.timer_one = QTimer(self)
        self.timer_one.start(30000)
        self.timer_one.timeout.connect(self.resetHomePage)

    def _onSignal_changed(self):
        if self.external_url != "":
            self._open(self.external_url)
            self.external_url = ""
        else:
            self.timer_one.stop()
            self.timer.start(1000)

    def _resetTimer(self):
        self.timer_one.stop()
        self.timer_one.start(30000)

    def resetHomePage(self):

        self.webview.load(QUrl("https://www.th-nuernberg.de/"))

    def open(self, url):
        self.external_url = url

    def _open(self, url):
        self._resetTimer()
        self.webview.load(QUrl(url))


class ElleScreenNode(Node):
    def __init__(self, ellie_screen):
        super().__init__("ellie_screen")
        self.ellie_screen = ellie_screen
        self.changeEyesMotion_srv = self.create_service(
            String, "change_eyes_motion", self.changeEyesMotion_callback)

    def changeEyesMotion_callback(self, request, response):
        print(request.request)
        try:
            self.ellie_screen.changeEyesMotion(request.request)
            response.response = "eyes motion changed !"
        except Exception as e:
            response.response = e.__str__()
        return response


def main(args=None):
    rclpy.init(args=args)
    ellieScreen = EllieScreen()
    thread_ = Thread(
        target=ros_shutdown, args=(ellieScreen,), daemon=True)
    thread_.start()
    ellieScreen.ui_init__()


def ros_shutdown(ellieScreen):
    print("ros_spins")
    rclpy.spin(ellieScreen)
    ellieScreen.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
