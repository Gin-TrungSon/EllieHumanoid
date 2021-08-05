import sys
sys.path.append("")
from src.ellie.ellie_behavior import EllieBehavior
import datetime
import random
import time
from threading import Thread, Event
from PyQt5.QtWebEngineWidgets import *
from PyQt5.QtCore import QEvent, QObject, QPoint, QRect, QSize, QThread, QUrl, Qt, QTimer
from PyQt5.QtGui import QMovie
from PyQt5.QtWidgets import QApplication, QDesktopWidget, QLabel, QMainWindow, QWidget
from PyQt5 import QtCore, QtCore
from PyQt5 import QtWidgets
import os



class EllieScreen(EllieBehavior):
    def __init__(self) -> None:
        self.screen = Thread(target=self.__ui_init__())

    def update(self, context):
        return super().update(context)

    def __ui_init__(self):
        app = QApplication(sys.argv)
        self.monitor = QDesktopWidget().screenGeometry(1)
        self.eyesBlinking = EyesBlinking()
        self.webView = OpenWebView()
        app.exec_()
    
    def changeEyesMotion(self,motionId):
        self.eyesBlinking.changeEyeMotion(motionId=motionId)
    
    def open(self,url):
        self.webView.open(url)

class EyesBlinking (QDesktopWidget):
    def __init__(self):
        super().__init__()
        self.label_animation = QLabel()
        self.label_animation.setFixedSize(480, 240)
        self.motions = {}
        for file in os.listdir("src/ellie/ellie_eyes/eye_motions"):
            motion = f"src/ellie/ellie_eyes/eye_motions/{file}"
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
        while motionid == self.currentMotion:
            motionid = random.choices(
                ["leftLooking", "smile", "blink", "rightLooking", "idle"], weights=[5, 3, 30, 5, 6], k=1)[0]
        self.currentMotion = motionid
        self.motion.stop()
        self.motion.setFileName(self.motions[self.currentMotion])
        self.motion.start()
        self.motion.loopCount()

    def changeEyeMotion(self,motionId):
        self.currentMOtion = motionId
        self.motion.stop()
        self.motion.setFileName(self.motions[self.currentMotion])
        self.motion.start()
        self.motion.loopCount()

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
        self.webview = QWebEngineView()
        self.webview.load(QUrl("https://www.th-nuernberg.de/"))
        self.screen(0)
        self.webview.showFullScreen()
        self.webview.mousePressEvent = lambda: self._resetTimer()
        self.timer = QTimer(self)
        self.timer_one = QTimer(self)
        self.timer_one.start(30000)
        self.timer_one.timeout.connect(self.resetHomePage)

    def _resetTimer(self):
        self.timer_one.stop()
        self.timer_one.start(30000)

    def resetHomePage(self):
        self.webview.load(QUrl("https://www.th-nuernberg.de/"))

    def open(self,url):
        self._resetTimer(self)
        self.webview.load(QUrl(url))
        

if __name__ == "__main__":
    ellieScreen = EllieScreen()
