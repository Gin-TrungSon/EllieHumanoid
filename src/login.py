import sys
from PyQt5.QtCore import QFile, QSettings, Qt
from PyQt5.QtGui import QCursor, QIcon
from PyQt5.QtWidgets import QApplication, QFrame, QHBoxLayout, QLabel, QPushButton, QStyle, QToolButton, QWidget
from PyQt5 import QtCore, QtGui, QtWidgets



def createButton(text,l_margin =50, r_margin=30):
    button = QPushButton()
    button.setText(text)
    button.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
    button.setFixedSize(150,50)
    # button.setStyle("margin: 1px; padding: 10px; \
    #   background-color: \
    #                        rgba(255,255,0,255); \
    #                        color: rgba(0,0,0,255); \
    #                        border-style: solid; \
    #                        border-radius: 4px; border-width: 3px; \
    #                        border-color: rgba(0,0,0,255);")
    return button
class Login(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowFlags(Qt.WindowType.FramelessWindowHint)

        self.backgroundContents = QFrame(self)
        self.backgroundContents.minimumSize=(0,30)
        self.backgroundContents.setLayoutDirection(QtCore.Qt.LayoutDirection.LeftToRight)

        self.closeButton = QPushButton(self.backgroundContents,icon= QIcon("assets/icons/16x16/cil-x.png"),clicked = lambda: self.close())
        self.closeButton.setGeometry(QtCore.QRect(210,11,28,28))
        self.closeButton.move(410, 118)

        self.pushButton = QtWidgets.QPushButton(self)
        self.pushButton.setGeometry(QtCore.QRect(110, 160, 151, 31))
        self.pushButton.setObjectName("pushButton")
        self.pushButton_2 = QtWidgets.QPushButton(self)
        self.pushButton_2.setGeometry(QtCore.QRect(110, 220, 151, 31))
        self.pushButton_2.setObjectName("pushButton_2")
        self.lineEdit = QtWidgets.QLineEdit(self)
        self.lineEdit.setEnabled(True)
        self.lineEdit.setGeometry(QtCore.QRect(110, 310, 151, 21))
        self.lineEdit.setAutoFillBackground(False)
        self.lineEdit.setEchoMode(QtWidgets.QLineEdit.Password)
        self.lineEdit.setAlignment(QtCore.Qt.AlignCenter)
        self.lineEdit.setObjectName("lineEdit")
        self.label = QtWidgets.QLabel(self)
        self.label.setGeometry(QtCore.QRect(140, 280, 111, 20))
        self.label.setObjectName("label")

        self.enterButton = QtWidgets.QPushButton(self)
        self.enterButton.setGeometry(QtCore.QRect(110, 360, 151, 31))

        self.retranslateUi()
        QtCore.QMetaObject.connectSlotsByName(self)
        self.show()

    def retranslateUi(self):
        _translate = QtCore.QCoreApplication.translate
        self.setWindowTitle(_translate("Form", "Form"))
        self.pushButton.setText(_translate("Form", "Continue"))
        self.pushButton_2.setText(_translate("Form", "Develop mode"))
        self.label.setText(_translate("Form", "Enter Password"))



if __name__=="__main__":
    app = QApplication(sys.argv)
    login = Login()
    file =  QFile("SpyBot.qss")
    if( file.open( QtCore.QFile.ReadOnly | QtCore.QFile.Text)):
        style = QtCore.QTextStream(file)
        app.setStyleSheet(style.readAll())
    app.exec_()
    
  