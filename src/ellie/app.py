# -*- coding: utf-8 -*-
import sys

from PyQt5.sip import delete
sys.path.append("")
from PyQt5 import QtCore, QtGui, QtWidgets
import os
import json
from pathlib import Path
from src.ellie.ellie_body.ellie_body import EllieBody
from src.ellie.ellie_behavior import EllieBehavior
class Ui_Form(QtWidgets.QWidget):
    def setupUi(self, Form):
        self.form = Form
        self.form .setObjectName("Form")
        self.form .setEnabled(True)
        self.form .resize(642, 365)
        self.form .setMaximumSize(QtCore.QSize(820, 472))
        self.form .setWindowFlags(QtCore.Qt.WindowType.FramelessWindowHint)

        file = QtCore.QFile("assets/styles/DDstyle.qss")
        if(file.open(QtCore.QFile.ReadOnly | QtCore.QFile.Text)):
            style = QtCore.QTextStream(file)
            Form.setStyleSheet(style.readAll())

        self.__authenticated = False

        self.frame = QtWidgets.QFrame(Form)
        self.frame.setGeometry(QtCore.QRect(0, 0, 651, 61))
        self.frame.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.frame.setStyleSheet("")
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        

        self.pushButton = QtWidgets.QPushButton(self.frame, clicked = lambda : self.form.close())
        self.pushButton.setGeometry(QtCore.QRect(600, 0, 31, 31))
        self.pushButton.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.pushButton.setStyleSheet("QPushButton {\n"
                                      "    color: rgb(255, 255, 255);\n"
                                      "    border: 0px solid;\n"
                                      "}\n"
                                      "QPushButton:hover {\n"
                                      "    background-color: rgb(59, 82, 73);\n"
                                      "}")
        self.pushButton.setText("")
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(
            "assets/icons/16x16/cil-x.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton.setIcon(icon)
        self.pushButton.setObjectName("pushButton")


        self.pushButton_2 = QtWidgets.QPushButton(self.frame )
        self.pushButton_2.setGeometry(QtCore.QRect(560, 0, 31, 31))
        self.pushButton_2.setStyleSheet("QPushButton {\n"
                                        "    color: rgb(255, 255, 255);\n"
                                        "    border: 0px solid;\n"
                                        "}\n"
                                        "QPushButton:hover {\n"
                                        "    background-color: rgb(59, 82, 73);\n"
                                        "}")
        self.pushButton_2.setText("")

        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap(
            "assets/icons/16x16/cil-window-maximize.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_2.setIcon(icon1)
        self.pushButton_2.setObjectName("pushButton_2")
        self.pushButton_3 = QtWidgets.QPushButton(self.frame)
        self.pushButton_3.setGeometry(QtCore.QRect(520, 0, 31, 31))
        self.pushButton_3.setStyleSheet("QPushButton {\n"
                                        "    color: rgb(255, 255, 255);\n"
                                        "    border: 0px solid;\n"
                                        "}\n"
                                        "QPushButton:hover {\n"
                                        "    background-color: rgb(59, 82, 73);\n"
                                        "}")
        self.pushButton_3.setText("")
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap(
            "assets/icons/16x16/cil-window-minimize.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_3.setIcon(icon2)
        self.pushButton_3.setObjectName("pushButton_3")
        self.label_3 = QtWidgets.QLabel(self.frame)
        self.label_3.setGeometry(QtCore.QRect(20, 10, 211, 31))
        self.label_3.setText("")
        self.label_3.setPixmap(QtGui.QPixmap("assets/logos/th.png"))
        self.label_3.setScaledContents(True)
        self.label_3.setObjectName("label_3")
        self.frame_2 = QtWidgets.QFrame(Form)
        self.frame_2.setGeometry(QtCore.QRect(50, 50, 591, 301))
        self.frame_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_2.setObjectName("frame_2")
        self.stackedWidget = QtWidgets.QStackedWidget(self.frame_2)
        self.stackedWidget.setGeometry(QtCore.QRect(10, 20, 571, 281))
        self.stackedWidget.setStyleSheet("background_color: transparent;\n"
                                         "QPushButton{\n"
                                         "   \n"
                                         "    color: rgb(255, 255, 255);\n"
                                         "    background-color: rgb(56, 41, 51);\n"
                                         "    border: 0px solid;\n"
                                         "}\n"
                                         "QPushButton:hover{\n"
                                         "    background-color: rgb(59, 82, 73);\n"
                                         "}")
        self.stackedWidget.setInputMethodHints(QtCore.Qt.ImhPreferNumbers)
        self.stackedWidget.setObjectName("stackedWidget")
        self.login_page = QtWidgets.QWidget()
        self.login_page.setObjectName("page")
        self.label_2 = QtWidgets.QLabel(self.login_page)
        self.label_2.setVisible(False)
        self.label_2.setGeometry(QtCore.QRect(180, 40, 91, 16))
        self.label_2.setStyleSheet("color: rgb(255, 0, 0);")
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.lineEdit = QtWidgets.QLineEdit(self.login_page)
        self.lineEdit.setGeometry(QtCore.QRect(180, 70, 201, 31))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.lineEdit.setFont(font)
        self.lineEdit.setStyleSheet("background-color: rgb(255,255,255);\n"
                                    "color: rgb(56, 41, 51);\n"
                                    "border-width: 1px;\n"
                                    "border-radius: 5px;\n"
                                    "")
        self.lineEdit.setInputMask("")
        self.lineEdit.setText("")
        self.lineEdit.setEchoMode(QtWidgets.QLineEdit.Password)
        self.lineEdit.setAlignment(QtCore.Qt.AlignCenter)
        self.lineEdit.setCursorMoveStyle(QtCore.Qt.VisualMoveStyle)
        self.lineEdit.setObjectName("lineEdit")
        self.developButton = QtWidgets.QPushButton(self.login_page, clicked = lambda: self.OnDevelopButtonClicked() )
        self.developButton.setGeometry(QtCore.QRect(180, 120, 201, 31))
        font = QtGui.QFont()
        font.setFamily("Microsoft JhengHei")
        font.setPointSize(10)
        self.developButton.setFont(font)
        self.developButton.setStyleSheet("QPushButton {\n"
                                        "    color: rgb(255, 255, 255);\n"
                                        "    background-color: rgb(59, 82, 73);\n"
                                        "    border: 0px solid;\n"
                                        "    border-radius: 5px;\n"
                                        "}\n"
                                        "QPushButton:hover {\n"
                                        "    \n"
                                        "    background-color: rgb(164, 180, 148);\n"
                                        "}")
        self.developButton.setObjectName("pushButton_4")
        self.label = QtWidgets.QLabel(self.login_page)
        self.label.setGeometry(QtCore.QRect(260, 160, 47, 13))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label.setFont(font)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.startButton = QtWidgets.QPushButton(self.login_page, clicked = lambda: self.OnStartButtonClicked())
        self.startButton.setGeometry(QtCore.QRect(180, 190, 201, 31))
        font = QtGui.QFont()
        font.setFamily("Microsoft JhengHei")
        font.setPointSize(10)
        self.startButton.setFont(font)
        self.startButton.setStyleSheet("QPushButton {\n"
                                        "    color: rgb(255, 255, 255);\n"
                                        "    background-color: rgb(59, 82, 73);\n"
                                        "    border: 0px solid;\n"
                                        "    border-radius: 5px;\n"
                                        "}\n"
                                        "QPushButton:hover {\n"
                                        "    \n"
                                        "    background-color: rgb(164, 180, 148);\n"
                                        "}")
        self.startButton.setObjectName("pushButton_5")
        self.label_12 = QtWidgets.QLabel(self.login_page)
        self.label_12.setGeometry(QtCore.QRect(30, 10, 81, 21))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_12.setFont(font)
        self.label_12.setAlignment(QtCore.Qt.AlignCenter)
        self.label_12.setObjectName("label_12")
        self.stackedWidget.addWidget(self.login_page)
        self.camera_page = QtWidgets.QWidget()
        self.camera_page.setObjectName("page_2")
        self.label_4 = QtWidgets.QLabel(self.camera_page)
        self.label_4.setGeometry(QtCore.QRect(40, 10, 81, 21))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_4.setFont(font)
        self.label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_4.setObjectName("label_4")
        self.frame_3 = QtWidgets.QFrame(self.camera_page)
        self.frame_3.setGeometry(QtCore.QRect(170, 70, 201, 31))
        self.frame_3.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_3.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_3.setObjectName("frame_3")
        self.label_5 = QtWidgets.QLabel(self.frame_3)
        self.label_5.setGeometry(QtCore.QRect(10, 0, 71, 16))
        self.label_5.setObjectName("label_5")
        self.comboBox = QtWidgets.QComboBox(self.frame_3)
        self.comboBox.setGeometry(QtCore.QRect(110, 0, 71, 31))
        self.comboBox.setStyleSheet("")
        self.comboBox.setObjectName("comboBox")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.frame_4 = QtWidgets.QFrame(self.camera_page)
        self.frame_4.setGeometry(QtCore.QRect(170, 110, 201, 31))
        self.frame_4.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_4.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_4.setObjectName("frame_4")
        self.label_6 = QtWidgets.QLabel(self.frame_4)
        self.label_6.setGeometry(QtCore.QRect(10, 0, 71, 16))
        self.label_6.setObjectName("label_6")
        self.comboBox_2 = QtWidgets.QComboBox(self.frame_4)
        self.comboBox_2.setGeometry(QtCore.QRect(110, 0, 71, 31))
        self.comboBox_2.setStyleSheet("\n"
                                      "")
        self.comboBox_2.setObjectName("comboBox_2")
        self.comboBox_2.addItem("")
        self.comboBox_2.addItem("")
        self.frame_5 = QtWidgets.QFrame(self.camera_page)
        self.frame_5.setGeometry(QtCore.QRect(170, 150, 201, 31))
        self.frame_5.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_5.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_5.setObjectName("frame_5")
        self.label_7 = QtWidgets.QLabel(self.frame_5)
        self.label_7.setGeometry(QtCore.QRect(10, 0, 101, 16))
        self.label_7.setObjectName("label_7")
        self.comboBox_3 = QtWidgets.QComboBox(self.frame_5)
        self.comboBox_3.setGeometry(QtCore.QRect(110, 0, 71, 31))
        self.comboBox_3.setStyleSheet("\n"
                                      "\n"
                                      "")
        self.comboBox_3.setObjectName("comboBox_3")
        self.comboBox_3.addItem("")
        self.comboBox_3.addItem("")
        self.frame_6 = QtWidgets.QFrame(self.camera_page)
        self.frame_6.setGeometry(QtCore.QRect(170, 190, 201, 31))
        self.frame_6.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_6.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_6.setObjectName("frame_6")
        self.label_8 = QtWidgets.QLabel(self.frame_6)
        self.label_8.setGeometry(QtCore.QRect(10, 0, 101, 16))
        self.label_8.setObjectName("label_8")
        self.lineEdit_3 = QtWidgets.QLineEdit(self.frame_6)
        self.lineEdit_3.setGeometry(QtCore.QRect(110, 0, 71, 31))
        self.lineEdit_3.setInputMethodHints(QtCore.Qt.ImhPreferNumbers)
        self.lineEdit_3.setObjectName("lineEdit_3")
        self.stackedWidget.addWidget(self.camera_page)
        self.motion_page = QtWidgets.QWidget()
        self.motion_page.setObjectName("page_3")
        self.label_9 = QtWidgets.QLabel(self.motion_page)
        self.label_9.setGeometry(QtCore.QRect(40, 10, 81, 21))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_9.setFont(font)
        self.label_9.setAlignment(QtCore.Qt.AlignCenter)
        self.label_9.setObjectName("label_9")
        self.frame_8 = QtWidgets.QFrame(self.motion_page)
        self.frame_8.setGeometry(QtCore.QRect(70, 50, 411, 111))
        self.frame_8.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_8.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_8.setObjectName("frame_8")
        self.lineEdit_2 = QtWidgets.QLineEdit(self.frame_8)
        self.lineEdit_2.setGeometry(QtCore.QRect(10, 30, 141, 31))
        self.lineEdit_2.setStyleSheet("border-radius: 5px;\n"
                                      "background-color: rgb(255,255,255);\n"
                                      "color: rgb(56, 41, 51);")
        self.lineEdit_2.setAlignment(QtCore.Qt.AlignCenter)
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.pushButton_8 = QtWidgets.QPushButton(self.frame_8)
        self.pushButton_8.setGeometry(QtCore.QRect(10, 70, 75, 23))
        self.pushButton_8.setStyleSheet("QPushButton {\n"
                                        "    color: rgb(255, 255, 255);\n"
                                        "    background-color: rgb(59, 82, 73);\n"
                                        "    border: 0px solid;\n"
                                        "    border-radius: 5px;\n"
                                        "}\n"
                                        "QPushButton:hover {\n"
                                        "background-color: rgb(164, 180, 148);\n"
                                        "}")
        self.pushButton_8.setObjectName("pushButton_8")

        self.pushButton_12 = QtWidgets.QPushButton(self.frame_8)
        self.pushButton_12.setGeometry(QtCore.QRect(90, 70, 75, 23))
        self.pushButton_12.setStyleSheet("QPushButton {\n"
                                         "    color: rgb(255, 255, 255);\n"
                                         "    background-color: rgb(59, 82, 73);\n"
                                         "    border: 0px solid;\n"
                                         "    border-radius: 5px;\n"
                                         "}\n"
                                         "QPushButton:hover {\n"
                                         "background-color: rgb(164, 180, 148);\n"
                                         "}")
        self.pushButton_12.setObjectName("pushButton_12")
        self.pushButton_12.setEnabled(False)

        self.pushButton_13 = QtWidgets.QPushButton(self.frame_8)
        self.pushButton_13.setGeometry(QtCore.QRect(170, 70, 75, 23))
        self.pushButton_13.setStyleSheet("QPushButton {\n"
                                         "    color: rgb(255, 255, 255);\n"
                                         "    background-color: rgb(59, 82, 73);\n"
                                         "    border: 0px solid;\n"
                                         "    border-radius: 5px;\n"
                                         "}\n"
                                         "QPushButton:hover {\n"
                                         "background-color: rgb(164, 180, 148);\n"
                                         "}")
        self.pushButton_13.setObjectName("pushButton_13")
        self.pushButton_13.setEnabled(False)

        self.pushButton_14 = QtWidgets.QPushButton(self.frame_8)
        self.pushButton_14.setGeometry(QtCore.QRect(250, 70, 75, 23))
        self.pushButton_14.setStyleSheet("QPushButton {\n"
                                         "    color: rgb(255, 255, 255);\n"
                                         "    background-color: rgb(59, 82, 73);\n"
                                         "    border: 0px solid;\n"
                                         "    border-radius: 5px;\n"
                                         "}\n"
                                         "QPushButton:hover {\n"
                                         "background-color: rgb(164, 180, 148);\n"
                                         "}")
        self.pushButton_14.setObjectName("pushButton_14")
        self.pushButton_14.setEnabled(False)

        self.pushButton_15 = QtWidgets.QPushButton(self.frame_8)
        self.pushButton_15.setGeometry(QtCore.QRect(330, 70, 75, 23))
        self.pushButton_15.setStyleSheet("QPushButton {\n"
                                         "    color: rgb(255, 255, 255);\n"
                                         "    background-color: rgb(59, 82, 73);\n"
                                         "    border: 0px solid;\n"
                                         "    border-radius: 5px;\n"
                                         "}\n"
                                         "QPushButton:hover {\n"
                                         "background-color: rgb(164, 180, 148);\n"
                                         "}")
        self.pushButton_15.setObjectName("pushButton_15")
        self.pushButton_15.setEnabled(False)

        self.label_10 = QtWidgets.QLabel(self.frame_8)
        self.label_10.setGeometry(QtCore.QRect(10, 10, 131, 16))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.label_10.setFont(font)
        self.label_10.setObjectName("label_10")
        self.frame_9 = QtWidgets.QFrame(self.motion_page)
        self.frame_9.setGeometry(QtCore.QRect(70, 170, 411, 61))
        self.frame_9.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_9.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_9.setObjectName("frame_9")
        self.pushButton_20 = QtWidgets.QPushButton(self.frame_9, clicked = lambda : self.resetBodyMotions())
        self.pushButton_20.setGeometry(QtCore.QRect(200, 30, 75, 23))
        self.pushButton_20.setStyleSheet("QPushButton {\n"
                                         "    color: rgb(255, 255, 255);\n"
                                         "    background-color: rgb(59, 82, 73);\n"
                                         "    border: 0px solid;\n"
                                         "    border-radius: 5px;\n"
                                         "}\n"
                                         "QPushButton:hover {\n"
                                         "background-color: rgb(164, 180, 148);\n"
                                         "}")
        self.pushButton_20.setObjectName("pushButton_20")
        self.comboBox_5 = QtWidgets.QComboBox(self.frame_9)
        self.comboBox_5.setGeometry(QtCore.QRect(10, 20, 151, 31))
        self.comboBox_5.setObjectName("comboBox_5")
        motions = self.getMotions()
        for i in motions:
            self.comboBox_5.addItem(i)
        self.label_11 = QtWidgets.QLabel(self.frame_9)
        self.label_11.setGeometry(QtCore.QRect(10, 0, 171, 16))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.label_11.setFont(font)
        self.label_11.setObjectName("label_11")
        self.stackedWidget.addWidget(self.motion_page)
        self.behavior_page = QtWidgets.QWidget()
        self.behavior_page.setObjectName("page_4")
        self.label_13 = QtWidgets.QLabel(self.behavior_page)
        self.label_13.setGeometry(QtCore.QRect(30, 10, 81, 21))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_13.setFont(font)
        self.label_13.setAlignment(QtCore.Qt.AlignCenter)
        self.label_13.setObjectName("label_13")
        self.behavior_id = QtWidgets.QLineEdit(self.behavior_page)
        self.behavior_id.setGeometry(QtCore.QRect(30, 50, 141, 31))
        self.behavior_id.setStyleSheet("border-radius: 5px;\n"
                                      "background-color: rgb(255,255,255);\n"
                                      "color: rgb(56, 41, 51);")
        self.behavior_id.setAlignment(QtCore.Qt.AlignCenter)
        self.behavior_id.setObjectName("lineEdit_4")
        self.frame_10 = QtWidgets.QFrame(self.behavior_page)
        self.frame_10.setGeometry(QtCore.QRect(30, 90, 271, 31))
        self.frame_10.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_10.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_10.setObjectName("frame_10")
        self.label_14 = QtWidgets.QLabel(self.frame_10)
        self.label_14.setGeometry(QtCore.QRect(10, 0, 71, 31))
        self.label_14.setObjectName("label_14")
        self.eye_motions = QtWidgets.QComboBox(self.frame_10)
        self.eye_motions.setGeometry(QtCore.QRect(110, 0, 141, 31))
        self.eye_motions.setStyleSheet("")
        self.eye_motions.setObjectName("comboBox_4")
        eye_motions = self.getEyeMotions()
        for i in eye_motions:
            self.eye_motions.addItem(i)
        self.frame_11 = QtWidgets.QFrame(self.behavior_page)
        self.frame_11.setGeometry(QtCore.QRect(290, 90, 271, 31))
        self.frame_11.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_11.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_11.setObjectName("frame_11")
        self.label_15 = QtWidgets.QLabel(self.frame_11)
        self.label_15.setGeometry(QtCore.QRect(10, 0, 71, 31))
        self.label_15.setObjectName("label_15")
        self.body_motions = QtWidgets.QComboBox(self.frame_11)
        self.body_motions.setGeometry(QtCore.QRect(110, 0, 141, 31))
        self.body_motions.setStyleSheet("")
        self.body_motions.setObjectName("comboBox_6")
        for i in motions:
            self.body_motions.addItem(i)
        self.frame_12 = QtWidgets.QFrame(self.behavior_page)
        self.frame_12.setGeometry(QtCore.QRect(30, 130, 331, 31))
        self.frame_12.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_12.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_12.setObjectName("frame_12")
        self.label_16 = QtWidgets.QLabel(self.frame_12)
        self.label_16.setGeometry(QtCore.QRect(10, 0, 101, 21))
        self.label_16.setObjectName("label_16")
        self.lineEdit_5 = QtWidgets.QLineEdit(self.frame_12)
        self.lineEdit_5.setGeometry(QtCore.QRect(110, 0, 151, 31))
        self.lineEdit_5.setInputMethodHints(QtCore.Qt.ImhNone)
        self.lineEdit_5.setText("")
        self.lineEdit_5.setObjectName("lineEdit_5")
        self.pushButton_16 = QtWidgets.QPushButton(self.behavior_page, clicked = lambda : self.createNewBehavior())
        self.pushButton_16.setGeometry(QtCore.QRect(380, 130, 81, 31))
        self.pushButton_16.setStyleSheet("QPushButton {\n"
                                         "    color: rgb(255, 255, 255);\n"
                                         "    background-color: rgb(59, 82, 73);\n"
                                         "    border: 0px solid;\n"
                                         "    border-radius: 5px;\n"
                                         "}\n"
                                         "QPushButton:hover {\n"
                                         "background-color: rgb(164, 180, 148);\n"
                                         "}")
        self.pushButton_16.setObjectName("pushButton_16")
        self.frame_13 = QtWidgets.QFrame(self.behavior_page)
        self.frame_13.setGeometry(QtCore.QRect(20, 190, 431, 61))
        self.frame_13.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_13.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_13.setObjectName("frame_13")
        self.removeBehaviorButton = QtWidgets.QPushButton(self.frame_13, clicked = lambda: self.removeBehavior())
        self.removeBehaviorButton.setGeometry(QtCore.QRect(200, 22, 71, 31))
        self.removeBehaviorButton.setStyleSheet("QPushButton {\n"
                                         "    color: rgb(255, 255, 255);\n"
                                         "    background-color: rgb(59, 82, 73);\n"
                                         "    border: 0px solid;\n"
                                         "    border-radius: 5px;\n"
                                         "}\n"
                                         "QPushButton:hover {\n"
                                         "background-color: rgb(164, 180, 148);\n"
                                         "}")
        self.removeBehaviorButton.setObjectName("pushButton_21")

        self.comboBox_7 = QtWidgets.QComboBox(self.frame_13)
        self.comboBox_7.setGeometry(QtCore.QRect(10, 20, 151, 31))
        self.comboBox_7.setObjectName("comboBox_7")
        behaviors = self.getBehaviors()
        for i in behaviors:
            self.comboBox_7.addItem(i)
        self.label_17 = QtWidgets.QLabel(self.frame_13)
        self.label_17.setGeometry(QtCore.QRect(10, 0, 161, 16))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.label_17.setFont(font)
        self.label_17.setObjectName("label_17")
        self.stackedWidget.addWidget(self.behavior_page)
        self.qa_page = QtWidgets.QWidget()
        self.qa_page.setObjectName("page_5")
        self.label_18 = QtWidgets.QLabel(self.qa_page)
        self.label_18.setGeometry(QtCore.QRect(30, 10, 81, 21))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_18.setFont(font)
        self.label_18.setAlignment(QtCore.Qt.AlignCenter)
        self.label_18.setObjectName("label_18")
        self.frame_14 = QtWidgets.QFrame(self.qa_page)
        self.frame_14.setGeometry(QtCore.QRect(30, 60, 501, 41))
        self.frame_14.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_14.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_14.setObjectName("frame_14")

        self.label_19 = QtWidgets.QLabel(self.frame_14)
        self.label_19.setGeometry(QtCore.QRect(10, 0, 101, 31))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.label_19.setFont(font)
        self.label_19.setObjectName("label_19")
        self.lineEdit_6 = QtWidgets.QLineEdit(self.frame_14)
        self.lineEdit_6.setGeometry(QtCore.QRect(110, 0, 221, 31))
        self.lineEdit_6.setInputMethodHints(QtCore.Qt.ImhNone)
        self.lineEdit_6.setObjectName("lineEdit_6")
        self.frame_15 = QtWidgets.QFrame(self.qa_page)
        self.frame_15.setGeometry(QtCore.QRect(30, 110, 501, 111))
        self.frame_15.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_15.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_15.setObjectName("frame_15")
        self.label_20 = QtWidgets.QLabel(self.frame_15)
        self.label_20.setGeometry(QtCore.QRect(10, 0, 101, 31))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.label_20.setFont(font)
        self.label_20.setObjectName("label_20")
        self.plainTextEdit = QtWidgets.QPlainTextEdit(self.frame_15)
        self.plainTextEdit.setGeometry(QtCore.QRect(110, 10, 221, 101))
        self.plainTextEdit.setObjectName("plainTextEdit")


        self.pushButton_22 = QtWidgets.QPushButton(self.frame_15)
        self.pushButton_22.setGeometry(QtCore.QRect(370,80,71, 31))
        self.pushButton_22.setStyleSheet("QPushButton {\n"
                                         "    color: rgb(255, 255, 255);\n"
                                         "    background-color: rgb(59, 82, 73);\n"
                                         "    border: 0px solid;\n"
                                         "    border-radius: 5px;\n"
                                         "}\n"
                                         "QPushButton:hover {\n"
                                         "background-color: rgb(164, 180, 148);\n"
                                         "}")
        self.pushButton_22.setObjectName("pushButton_22")

        self.stackedWidget.addWidget(self.qa_page)
        self.frame_7 = QtWidgets.QFrame(Form)
        self.frame_7.setGeometry(QtCore.QRect(10, 60, 41, 291))
        self.frame_7.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_7.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_7.setObjectName("frame_7")
        self.pushButton_6 = QtWidgets.QPushButton(self.frame_7,clicked = lambda : self.OnButtonClicked("pushButton_6"))
        self.pushButton_6.setGeometry(QtCore.QRect(-10, 0, 61, 41))
        self.pushButton_6.setStyleSheet("QPushButton {\n"
                                        "    color: rgb(255, 255, 255);\n"
                                        "    background-color: transparent;\n"
                                        "    border: 0px solid;\n"
                                        "}\n"
                                        "QPushButton:hover {\n"
                                        "    background-color: rgb(59, 82, 73);\n"
                                        "}")
        self.pushButton_6.setText("")
        icon3 = QtGui.QIcon()
        icon3.addPixmap(QtGui.QPixmap(
            "assets/icons/16x16/cil-menu.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_6.setIcon(icon3)
        self.pushButton_6.setObjectName("pushButton_6")
        self.pushButton_9 = QtWidgets.QPushButton(self.frame_7, clicked = lambda : self.OnButtonClicked("pushButton_9"))
        self.pushButton_9.setGeometry(QtCore.QRect(-10, 40, 61, 41))
        self.pushButton_9.setStyleSheet("QPushButton {\n"
                                        "    color: rgb(255, 255, 255);\n"
                                        "    background-color: transparent;\n"
                                        "    border: 0px solid;\n"
                                        "}\n"
                                        "QPushButton:hover {\n"
                                        "    background-color: rgb(59, 82, 73);\n"
                                        "}")
        self.pushButton_9.setText("")
        icon4 = QtGui.QIcon()
        icon4.addPixmap(QtGui.QPixmap(
            "assets/icons/16x16/cil-camera.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_9.setIcon(icon4)
        self.pushButton_9.setObjectName("pushButton_9")
        self.pushButton_10 = QtWidgets.QPushButton(self.frame_7, clicked = lambda : self.OnButtonClicked("pushButton_10"))
        self.pushButton_10.setGeometry(QtCore.QRect(-10, 80, 61, 41))
        self.pushButton_10.setStyleSheet("QPushButton {\n"
                                         "    color: rgb(255, 255, 255);\n"
                                         "    background-color: transparent;\n"
                                         "    border: 0px solid;\n"
                                         "}\n"
                                         "QPushButton:hover {\n"
                                         "    background-color: rgb(59, 82, 73);\n"
                                         "}")
        self.pushButton_10.setText("")
        icon5 = QtGui.QIcon()
        icon5.addPixmap(QtGui.QPixmap(
            "assets/icons/16x16/cil-library.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_10.setIcon(icon5)
        self.pushButton_10.setObjectName("pushButton_10")
        self.pushButton_11 = QtWidgets.QPushButton(self.frame_7, clicked = lambda : self.OnButtonClicked("pushButton_11"))
        self.pushButton_11.setGeometry(QtCore.QRect(-10, 120, 61, 41))
        self.pushButton_11.setStyleSheet("QPushButton {\n"
                                         "    color: rgb(255, 255, 255);\n"
                                         "    background-color: transparent;\n"
                                         "    border: 0px solid;\n"
                                         "}\n"
                                         "QPushButton:hover {\n"
                                         "    background-color: rgb(59, 82, 73);\n"
                                         "}")
        self.pushButton_11.setText("")
        icon6 = QtGui.QIcon()
        icon6.addPixmap(QtGui.QPixmap(
            "assets/icons/16x16/cil-layers.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_11.setIcon(icon6)
        self.pushButton_11.setObjectName("pushButton_11")
        self.pushButton_17 = QtWidgets.QPushButton(self.frame_7, clicked = lambda : self.OnButtonClicked("pushButton_17"))
        self.pushButton_17.setGeometry(QtCore.QRect(-10, 160, 61, 41))
        self.pushButton_17.setStyleSheet("QPushButton {\n"
                                         "    color: rgb(255, 255, 255);\n"
                                         "    background-color: transparent;\n"
                                         "    border: 0px solid;\n"
                                         "}\n"
                                         "QPushButton:hover {\n"
                                         "    background-color: rgb(59, 82, 73);\n"
                                         "}")
        self.pushButton_17.setText("")
        icon7 = QtGui.QIcon()
        icon7.addPixmap(QtGui.QPixmap(
            "assets/icons/16x16/cil-paperclip.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_17.setIcon(icon7)
        self.pushButton_17.setObjectName("pushButton_17")
        
        self.HideDevelopMode()
         
        self.retranslateUi(Form)
        self.stackedWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(Form)
        


     
    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.stackedWidget.setStatusTip(_translate("Form", "ogtolzto"))
        self.label_2.setText(_translate("Form", "Password incorrect"))
        self.lineEdit.setPlaceholderText(_translate("Form", "Password"))
        self.developButton.setText(_translate("Form", "Enter Develop Mode"))
        self.label.setText(_translate("Form", "Or"))
        self.startButton.setText(_translate("Form", "Continue"))
        self.label_12.setText(_translate("Form", "Login"))
        self.label_4.setText(_translate("Form", "Camera"))
        self.label_5.setText(_translate("Form", "UsePiCamera :"))
        self.comboBox.setItemText(0, _translate("Form", "True"))
        self.comboBox.setItemText(1, _translate("Form", "False"))
        self.label_6.setText(_translate("Form", "Method :"))
        self.comboBox_2.setItemText(0, _translate("Form", "Dnn"))
        self.comboBox_2.setItemText(1, _translate("Form", "Haarcascade"))
        self.label_7.setText(_translate("Form", "DetectionMethod :"))
        self.comboBox_3.setItemText(0, _translate("Form", "Dnn"))
        self.comboBox_3.setItemText(1, _translate("Form", "Cnn"))
        self.label_8.setText(_translate("Form", "Threshold :"))
        self.lineEdit_3.setText(_translate("Form", "0.6"))
        self.label_9.setText(_translate("Form", "Trainning"))
        self.lineEdit_2.setPlaceholderText(_translate("Form", "Moving Id"))
        self.pushButton_8.setText(_translate("Form", "Start"))
        self.pushButton_12.setText(_translate("Form", "Stop"))
        self.pushButton_13.setText(_translate("Form", "Replay"))
        self.pushButton_14.setText(_translate("Form", "Save"))
        self.pushButton_15.setText(_translate("Form", "Delete"))
        self.label_10.setText(_translate("Form", "Create new motion"))
        self.pushButton_20.setText(_translate("Form", "Remove"))
        self.label_11.setText(_translate("Form", "Remove existed motion"))
        self.label_13.setText(_translate("Form", "Behavior"))
        self.behavior_id.setPlaceholderText(_translate("Form", "Behavior Id"))
        self.label_14.setText(_translate("Form", "Eyes motion :"))
        self.label_15.setText(_translate("Form", "Body motion :"))
        self.label_16.setText(_translate("Form", "Speak :"))
        self.lineEdit_5.setPlaceholderText(_translate("Form", "say anything"))
        self.pushButton_16.setText(_translate("Form", "Create"))
        self.removeBehaviorButton.setText(_translate("Form", "Remove"))
        self.pushButton_22.setText(_translate("Form", "Add"))
        self.label_17.setText(_translate("Form", "Remove existed behavior"))
        self.label_18.setText(_translate("Form", "Listen"))
        self.label_19.setText(_translate("Form", "Question :"))
        self.lineEdit_6.setText(_translate("Form", "0.6"))
        self.label_20.setText(_translate("Form", "Answers:"))


    def OnButtonClicked(self,button_name):       
        if button_name == "pushButton_6":
            self.stackedWidget.setCurrentWidget(self.login_page)
        elif button_name == "pushButton_9" :
            self.stackedWidget.setCurrentWidget(self.camera_page)
        elif button_name == "pushButton_10" :
            self.stackedWidget.setCurrentWidget(self.motion_page)
        elif button_name == "pushButton_11" :
            self.stackedWidget.setCurrentWidget(self.behavior_page)
        elif button_name == "pushButton_17":
            self.stackedWidget.setCurrentWidget(self.qa_page)
    
    def OnDevelopButtonClicked(self):
        if self.lineEdit.text() == "1111":
            self.__authenticated = True
            self.label_2.setVisible(False)
            self.ShowDevelopMode()
        else :
            self.label_2.setVisible(True)
            self.__authenticated= False
        try:
           self.ellieBody = EllieBody("vrep")
        except: 
            self.ellieBody = None
    
    def startLearning(self):
        if self.ellieBody != None :
            self.ellieBody.startLearning()
            self.pushButton_8.setEnabled(False)
            self.pushButton_12.setEnabled(True)
    
    def stopLearning(self):
        if self.ellieBody != None :
            self.ellieBody.stopLearning()
            self.pushButton_13.setEnabled(True)
            self.pushButton_14.setEnabled(True)
            self.pushButton_15.setEnabled(True)
            self.pushButton_12.setEnabled(False)

    def replay(self):
        if self.ellieBody != None :
            self.ellieBody.replayLearnedMotion()
    
    def saveBodyMotion(self):
        if self.ellieBody != None :
            path = f"src/ellie/ellie_body/actions/{self.lineEdit_2.text()}.move"
            if Path(path).is_file():
                self.popUpFileExist(self.lineEdit_2.text())
            self.ellieBody.saveLearnedMotion(self.lineEdit_2.text())
            self.pushButton_8.setEnabled(True)
            self.pushButton_13.setEnabled(False)
            self.pushButton_14.setEnabled(False)
            self.pushButton_15.setEnabled(False)
            self.pushButton_12.setEnabled(False)
    
    def deleteLearnedMotion(self):
        if self.ellieBody != None :
            self.ellieBody.deleteLearnedMotion()
            self.pushButton_8.setEnabled(True)
            self.pushButton_13.setEnabled(False)
            self.pushButton_14.setEnabled(False)
            self.pushButton_15.setEnabled(False)
            self.pushButton_12.setEnabled(False)
    
    def OnStartButtonClicked(self):
        self.form.close()
 
    def ShowDevelopMode(self):
        self.pushButton_9.setVisible(True)
        self.pushButton_10.setVisible(True)
        self.pushButton_11.setVisible(True)
        self.pushButton_17.setVisible(True)
    def HideDevelopMode(self):
        self.pushButton_9.setVisible(False)
        self.pushButton_10.setVisible(False)
        self.pushButton_11.setVisible(False)
        self.pushButton_17.setVisible(False)

    def getMotions(self):
        files = os.listdir("src/ellie/ellie_body/actions/")
        self.motions= []
        for i in range(len(files)):
            self.motions.append(files[i].replace(".move",""))
        return self.motions
    
    def getBehaviors(self):
        files = os.listdir("src/ellie/behaviors")
        behaviours= []
        for i in range(len(files)):
            behaviours.append(files[i].replace(".json",""))
        return behaviours

    def getEyeMotions(self):
        files = os.listdir("src/ellie/ellie_eyes/eye_motions/")
        self.motions= []
        for i in range(len(files)):
            self.motions.append(files[i].replace(".gif",""))
        return self.motions

    def createNewBehavior(self):
        newBehavior = Behavior(self.behavior_id.text(), self.eye_motions.currentText(),self.body_motions.currentText(),self.lineEdit_5.text())
        if newBehavior.isFileExisted:
            if self.popUpFileExist(self.behavior_id.text()) == False:
                return
        newBehavior.save()
        self.resetBehaviors()
    
    def removeFile(self, file):
        if os.path.exists(file):
            os.remove(file)

    def removeBehavior(self):
        file = f"src/ellie/behaviors/{self.comboBox_7.currentText()}.json"
        self.removeFile(file)
        self.resetBehaviors()

    def removeBodyMotion(self):
        file = f"src/ellie/ellie_body/actions/{self.comboBox_5.currentText()}.move"
        self.removeFile(file)
        self.resetBodyMotions()

    def resetBehaviors(self):
        self.comboBox_7.clear()
        behaviors = self.getBehaviors()
        for i in behaviors:
            self.comboBox_7.addItem(i)
        self.comboBox_7.repaint()

    def resetBodyMotions(self):
        self.body_motions.clear()
        self.comboBox_5.clear()
        motions = self.getMotions()
        for i in motions:
            self.body_motions.addItem(i)
            self.comboBox_5.addItem(i)




    def popUpFileExist(self, id):
        msg = QtWidgets.QMessageBox()
        msg.setWindowTitle("File existed")
        msg.setText(f"The file {id} already exists. Do you want to replace it ?   ")
        msg.setStandardButtons(QtWidgets.QMessageBox.StandardButton.Cancel | QtWidgets.QMessageBox.StandardButton.Ok)
        msg.setDefaultButton(QtWidgets.QMessageBox.StandardButton.Ok)
        popup = msg.exec()
        if popup == QtWidgets.QMessageBox.StandardButton.Ok:
            return True
        elif popup == QtWidgets.QMessageBox.StandardButton.Cancel:
            return False
        
if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())
