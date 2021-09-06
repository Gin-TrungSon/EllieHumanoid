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
sys.path.append("")
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

class EllieBrustMotnior(Node):
    def __init__(self):
        super().__init__("ellie_brust_screen")
        self.openUrl_srv = self.create_service(
            String, "open_url", self.openUrl_callback)

    def ui_init__(self):
        app = QApplication(sys.argv)
        self.monitor = QDesktopWidget().screenGeometry(1)
        self.webView = OpenWebView()
        sys.exit(app.exec_())


    def openUrl_callback(self, request, response):
        print(f"open : {request.request}")
        try:
            self.open(request.request)
            response.response = f"open : {request.request}"
        except Exception as e:
            response.response = e.__str__()
        return response

    def open(self, url):
        self.webView.open(url)

def main(args=None):
    rclpy.init(args=args)
    ellieBrustScreen = EllieBrustMotnior()
    thread_ = Thread(
        target=ros_shutdown, args=(ellieBrustScreen,), daemon=True)
    thread_.start()
    ellieBrustScreen.ui_init__()


def ros_shutdown(ellieScreen):
    print("ros_spins")
    rclpy.spin(ellieScreen)
    ellieScreen.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()