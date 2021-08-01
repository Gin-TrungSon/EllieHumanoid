from PyQt5 import QtWidgets
from PyQt5 import QtCore, QtCore
from PyQt5.QtWidgets import QApplication, QDesktopWidget, QLabel, QMainWindow, QWidget
from PyQt5.QtGui import QMovie
from PyQt5.QtCore import QUrl, Qt, QTimer
from PyQt5.QtWebEngineWidgets import *
import sys

class EyesBlinking (QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedSize(480,240) 
        self.label_animation = QLabel(self) 
        self.motion = QMovie("idle.gif")
        self.label_animation.setMovie(self.motion)    
        self.motion.start()
        self.show()
        self.webview = QWebEngineView()
        self.webview.load(QUrl("https://www.th-nuernberg.de/"))
        self.webview.show()

    def startAnimation(self):
        self.motion.start()

    def stopAnimation(self):
        self.motion.stop()
        self.close()
if __name__=="__main__":
    app = QApplication(sys.argv)
    monitor = QDesktopWidget().screenGeometry(1)
    demo = EyesBlinking()
    app.exec_()