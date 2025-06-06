#!/usr/bin/env python3
from PyQt5 import QtCore, QtGui, QtWidgets
import rclpy
from example_interfaces.msg import String

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(540, 280)
        MainWindow.setMinimumSize(QtCore.QSize(540, 280))
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.formLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.formLayoutWidget.setGeometry(QtCore.QRect(270, 30, 262, 71))
        self.formLayoutWidget.setObjectName("formLayoutWidget")
        self.formLayout = QtWidgets.QFormLayout(self.formLayoutWidget)
        self.formLayout.setContentsMargins(0, 0, 0, 0)
        self.formLayout.setObjectName("formLayout")
        self.label_lineer = QtWidgets.QLabel(self.formLayoutWidget)
        self.label_lineer.setObjectName("label_lineer")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.label_lineer)
        self.lineEdit = QtWidgets.QLineEdit(self.formLayoutWidget)
        self.lineEdit.setReadOnly(True)
        self.lineEdit.setObjectName("lineEdit")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.lineEdit)
        self.lineEdit_2 = QtWidgets.QLineEdit(self.formLayoutWidget)
        self.lineEdit_2.setReadOnly(True)
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.lineEdit_2)
        self.label_angular = QtWidgets.QLabel(self.formLayoutWidget)
        self.label_angular.setObjectName("label_angular")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.label_angular)
        self.label_control_3 = QtWidgets.QLabel(self.centralwidget)
        self.label_control_3.setGeometry(QtCore.QRect(270, 130, 131, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_control_3.setFont(font)
        self.label_control_3.setObjectName("label_control_3")
        self.gridLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(9, 32, 254, 89))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        self.button_right = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.button_right.setObjectName("button_right")
        self.gridLayout.addWidget(self.button_right, 1, 2, 1, 1)
        self.button_forward = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.button_forward.setObjectName("button_forward")
        self.gridLayout.addWidget(self.button_forward, 0, 1, 1, 1)
        self.button_left = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.button_left.setObjectName("button_left")
        self.gridLayout.addWidget(self.button_left, 1, 0, 1, 1)
        self.button_stop = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.button_stop.setObjectName("button_stop")
        self.gridLayout.addWidget(self.button_stop, 1, 1, 1, 1)
        self.button_backward = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.button_backward.setObjectName("button_backward")
        self.gridLayout.addWidget(self.button_backward, 2, 1, 1, 1)
        self.label_control = QtWidgets.QLabel(self.centralwidget)
        self.label_control.setGeometry(QtCore.QRect(9, 9, 102, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_control.setFont(font)
        self.label_control.setObjectName("label_control")
        self.label_control_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_control_2.setGeometry(QtCore.QRect(270, 7, 104, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_control_2.setFont(font)
        self.label_control_2.setObjectName("label_control_2")
        self.formLayoutWidget_2 = QtWidgets.QWidget(self.centralwidget)
        self.formLayoutWidget_2.setGeometry(QtCore.QRect(270, 150, 251, 61))
        self.formLayoutWidget_2.setObjectName("formLayoutWidget_2")
        self.formLayout_2 = QtWidgets.QFormLayout(self.formLayoutWidget_2)
        self.formLayout_2.setContentsMargins(0, 0, 0, 0)
        self.formLayout_2.setObjectName("formLayout_2")
        self.label = QtWidgets.QLabel(self.formLayoutWidget_2)
        self.label.setObjectName("label")
        self.formLayout_2.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.label)
        self.lineEdit_3 = QtWidgets.QLineEdit(self.formLayoutWidget_2)
        self.lineEdit_3.setReadOnly(True)
        self.lineEdit_3.setObjectName("lineEdit_3")
        self.formLayout_2.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.lineEdit_3)
        self.lineEdit_4 = QtWidgets.QLineEdit(self.formLayoutWidget_2)
        self.lineEdit_4.setReadOnly(True)
        self.lineEdit_4.setObjectName("lineEdit_4")
        self.formLayout_2.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.lineEdit_4)
        self.label_2 = QtWidgets.QLabel(self.formLayoutWidget_2)
        self.label_2.setObjectName("label_2")
        self.formLayout_2.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.label_2)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 540, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
        MainWindow.setTabOrder(self.button_forward, self.button_stop)
        MainWindow.setTabOrder(self.button_stop, self.button_right)
        MainWindow.setTabOrder(self.button_right, self.button_left)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label_lineer.setText(_translate("MainWindow", "Lineer Hız(m/s):"))
        self.label_angular.setText(_translate("MainWindow", "Açısal Hız(rad/s):"))
        self.label_control_3.setText(_translate("MainWindow", "Konum Göstergesi"))
        self.button_right.setText(_translate("MainWindow", "Sağ"))
        self.button_forward.setText(_translate("MainWindow", "İleri"))
        self.button_left.setText(_translate("MainWindow", "Sol"))
        self.button_stop.setText(_translate("MainWindow", "Dur"))
        self.button_backward.setText(_translate("MainWindow", "Geri"))
        self.label_control.setText(_translate("MainWindow", "Robot Kontrol"))
        self.label_control_2.setText(_translate("MainWindow", "Hız Göstergesi"))
        self.label.setText(_translate("MainWindow", "Konum X(m):"))
        self.label_2.setText(_translate("MainWindow", "Konum Y(m):"))

class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        rclpy.init()
        self.node = rclpy.create_node("deneme2")
        self.subscriber_ = self.node.create_subscription(
            String, "robot_news", self.callback_robot_news, 10)
        self.lineEdit.setText(str(0.0))
        self.lineEdit_2.setText(str(0.0))
        self.lineEdit_3.setText(str(0.0))
        self.lineEdit_4.setText(str(0.0))

    def callback_robot_news(self, msg):
        self.node.get_logger().info(msg.data)

import sys
app = QtWidgets.QApplication(sys.argv)
MainWindow = QtWidgets.QMainWindow()
ui = Ui_MainWindow()
ui.setupUi(MainWindow)
MainWindow.show()
sys.exit(app.exec_())
