import sys
sys.path.append("")
import os
from PyQt5 import QtWidgets
from PyQt5 import QtCore, QtCore
from PyQt5.QtWidgets import QApplication, QDesktopWidget, QLabel, QMainWindow, QWidget
from PyQt5.QtGui import QMovie
from PyQt5.QtCore import QEvent, QObject, QPoint, QRect, QSize, QThread, QUrl, Qt, QTimer
from PyQt5.QtWebEngineWidgets import *
from threading import Thread, Event
import time
import random
import datetime
class EyesBlinking (QDesktopWidget):
    def __init__(self):
        super().__init__()   
        self.label_animation = QLabel() 
        self.label_animation.setFixedSize(480,240) 
        self.motions = {}
        for file in os.listdir("src/ellie/ellie_eyes/eye_motions"):
            motion = f"src/ellie/ellie_eyes/eye_motions/{file}"
            self.motions[file.replace(".gif","")]= motion

        self.currentMotion = "idle"
        self.motion = QMovie(self.motions[self.currentMotion])
        self.motion.setScaledSize(QSize(480,240))
        self.label_animation.setMovie(self.motion)
        self.motion.start()
        self.label_animation.setGeometry(self.screenGeometry(1))
        self.label_animation.show()
        self.timer=QTimer(self)
        self.timer_one = QTimer(self)
        self.timer_one.start(5000)
        self.timer_one.timeout.connect(self.timeout_run)
        self.motion.finished.connect(self.onFinished)


    def onFinished(self):
        self.timer_one.stop()
        self.timer_one.start(5000)
        self.changeGIF()

    def changeGIF(self):
        motionid = self.currentMotion
        while motionid== self.currentMotion:
            motionid = random.choices(["leftLooking","smile","blink", "rightLooking","idle"], weights=[5,3,30,5,6],k=1)[0]
        self.currentMotion = motionid
        self.motion.stop()
        self.motion.setFileName(self.motions[self.currentMotion])
        self.motion.start()
        self.motion.loopCount()

    def timeout_run(self):

        sender = self.sender()
        current_time = datetime.datetime.now()

        if id(sender) == id(self.timer_one):
            self.changeGIF()
            
   
            

       
    def loopcount(self, move ):
        if(move.currentFrameNumber() == move.frameCount()-1):
            return True
        else :
            return False

    def startAnimation(self):
        self.motion.start()

    def stopAnimation(self):
        self.motion.stop()
        self.close()
class OpenWebView(QDesktopWidget):
        def __init__(self):
            super().__init__()
            self.webview = QWebEngineView()
            self.webview.load(QUrl("https://www.th-nuernberg.de/"))
            self.screen(0)           
            self.webview.showFullScreen()
            self.webview.mousePressEvent= lambda: self.resetTimer()
            self.timer=QTimer(self)
            self.timer_one = QTimer(self)
            self.timer_one.start(30000)
            self.timer_one.timeout.connect(self.resetHomePage)
        
        def resetTimer(self):
            self.timer_one.stop()
            self.timer_one.start(30000)
        
        def resetHomePage(self):
            self.webview.load(QUrl("https://www.th-nuernberg.de/"))
        

if __name__=="__main__":
    app = QApplication(sys.argv)
    monitor = QDesktopWidget().screenGeometry(1)
    #demo = EyesBlinking()
    demo2= OpenWebView()
    app.exec_()

    