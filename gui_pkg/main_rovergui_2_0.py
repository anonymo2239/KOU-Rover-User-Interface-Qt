import sys
import threading
import os
current_dir = os.path.dirname(os.path.abspath(__file__))


# ROS 2
from rclpy.node import Node
import rclpy
from std_msgs.msg import String
from PyQt6.QtCore import pyqtSignal
from std_msgs.msg import String
from example_interfaces.msg import Int32

# Qt
from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel, QDialog, QPushButton, QMessageBox
from PyQt6.QtCore import QTimer
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtCore import QObject, pyqtSignal

from gui_pkg.rovergui_2_0 import Ui_rover_gui

class MainWindow(QMainWindow, Ui_rover_gui, Node):

    qr_received = pyqtSignal(str)  # QR kod string'ini göndermek için sinyal
    start_const = 0
    approval = False
    i = 0

    # QRCODE AND ROS
    def connect_ros(self):
        self.qr_subscriber = self.create_subscription(
            String, 'qr_code', self.print_QR, 10)
        
        self.temperature_subscriber = self.create_subscription(
            Int32, 'temperature_info', self.print_temperature, 10)
        
        self.current_subscriber = self.create_subscription(
            Int32, 'current_info', self.print_current, 10)
        
        self.charge_subscriber = self.create_subscription(
            Int32, 'charge_info', self.print_charge, 10)

    def print_QR(self, msg):
        self.qr_received.emit(msg.data)  # QR verisi ile sinyali tetikle

    def update_text_edit(self, qr_data):
        _translate = QtCore.QCoreApplication.translate
        self.ui.plainQRCODE.setPlainText(_translate("rover_gui", qr_data))

    def print_temperature(self, msg):
        if self.approval == True:
            self.ui.lineEdit_temperature.setText(str(msg.data))
        else:
            pass

    def print_current(self, msg):
        if self.approval == True:
            self.ui.lineEdit_current.setText(str(msg.data))
        else:
            pass

    def print_charge(self, msg):
        if self.approval == True:
            self.ui.lineEdit_charge.setText(str(msg.data))
        else:
            pass


    def __init__(self, parent=None):
        QMainWindow.__init__(self, parent)
        Node.__init__(self, 'main_window_node')
        self.ui = Ui_rover_gui()
        self.ui.setupUi(self)

        self.get_logger().info("ROS initialized.")
        self.connect_ros()
        self.qr_received.connect(self.update_text_edit)

        self.ui.pushButton_2.clicked.connect(self.showExitDialog)
        self.ui.pushButton_exit_fullscreen.clicked.connect(self.setFullScreen)
        self.ui.routeInfoButton.clicked.connect(self.createRouteInfoDialog)
        self.ui.startButton.clicked.connect(self.start_vehicle)
        self.ui.finishButton.clicked.connect(self.get_through_vehicle)
        self.ui.emergencyButton.clicked.connect(self.emergency_sit)
        self.ui.pushButton.clicked.connect(self.approval_sit)

        self.centralwidget = QtWidgets.QWidget(self)
        self.centralwidget.setObjectName("centralwidget")
        self.timer = QTimer(self.centralwidget)
        self.elapsed_time = 0
        self.timer.timeout.connect(self.update_time)

    def showExitDialog(self):
        self.msgBox = QMessageBox()
        self.msgBox.setIcon(QMessageBox.Icon.Warning)
        self.msgBox.setText("Tekrar açıldığında baştan başlamanız gerekecek.")
        self.msgBox.setWindowTitle("Emin misiniz?")
        self.msgBox.setStandardButtons(QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        self.yes_button = self.msgBox.button(QMessageBox.StandardButton.Yes)
        self.no_button = self.msgBox.button(QMessageBox.StandardButton.No)
        self.yes_button.setText("Evet")
        self.no_button.setText("Hayır")

        returnValue = self.msgBox.exec()
        if returnValue == QMessageBox.StandardButton.Yes:
            sys.exit()

    def setFullScreen(self):
        if self.i % 2 == 0:
            self.showNormal()
            self.ui.pushButton_exit_fullscreen.setText("Tam ekran")
        else:
            self.showFullScreen()
            self.ui.pushButton_exit_fullscreen.setText("Tam ekrandan çık")
        self.i += 1

    def createRouteInfoDialog(self):
        self.dialog = QDialog()
        self.dialog.setWindowTitle('Güzergah Bilgisi')
        self.dialog.setGeometry(800, 500, 1000, 500)
        self.dialog.setStyleSheet("background-color: lightgray;")
        self.layout = QVBoxLayout()
        self.label = QLabel("Burada güzergah ile ilgili bilgiler ve resimler yer alacak.")
        font = self.label.font()
        font.setPointSize(20)
        font.setBold(True)
        self.label.setFont(font)
        self.layout.addWidget(self.label)
        self.dialog.setLayout(self.layout)
        self.dialog.exec()

    def approval_sit(self):
        self.approval = True
        self.comboBox_scen.setEnabled(False)

    def start_vehicle(self):
        if self.approval:
            if not self.timer.isActive():
                self.ui.startButton.setText("Durdur")
                icon1 = QtGui.QIcon()
                icon1.addPixmap(QtGui.QPixmap(current_dir + "/images/power-off.png"), QtGui.QIcon.Mode.Normal, QtGui.QIcon.State.Off)
                self.ui.startButton.setIcon(icon1)
                self.timer.start(1000)
                self.ui.finishButton.setVisible(True)
            else:
                self.ui.startButton.setText("Devam Et")
                icon1 = QtGui.QIcon()
                icon1.addPixmap(QtGui.QPixmap(current_dir + "/images/power-on.png"), QtGui.QIcon.Mode.Normal, QtGui.QIcon.State.Off)
                self.ui.startButton.setIcon(icon1)
                self.timer.stop()
                self.ui.comboBox_scen.setEnabled(True)
                self.ui.finishButton.setVisible(False)
        else:
            self.show_scenario_warning()

    def get_through_vehicle(self):
        if self.approval:
            self.ui.finishButton.setVisible(False)
            self.ui.startButton.setText("Başlat")
            icon1 = QtGui.QIcon()
            icon1.addPixmap(QtGui.QPixmap(current_dir + "/images/power-on.png"), QtGui.QIcon.Mode.Normal, QtGui.QIcon.State.Off)
            self.ui.startButton.setIcon(icon1)
            self.approval = False
            self.timer.stop()
            self.elapsed_time = 0
            self.ui.lineEdit_time.setText("00:00:00")
        else:
            self.show_scenario_warning()

    def show_scenario_warning(self):
        msgBox_select_scenario = QMessageBox()
        msgBox_select_scenario.setIcon(QMessageBox.Icon.Warning)
        msgBox_select_scenario.setText("Aracı başlatmadan önce lütfen senaryo seçiniz.")
        msgBox_select_scenario.setWindowTitle("Senaryo seçiniz.")
        msgBox_select_scenario.setStandardButtons(QMessageBox.StandardButton.Ok)
        msgBox_select_scenario.setStyleSheet("QMessageBox {background-color: #FF6666; color: white;} QPushButton {color: black;}")
        msgBox_select_scenario.exec()

    def emergency_sit(self):
        if self.approval:
            self.ui.startButton.setText("Devam Et")
            self.ui.emergencyButton.setText("Acil Durum İptal")
            icon1 = QtGui.QIcon()
            icon1.addPixmap(QtGui.QPixmap(current_dir + "/images/power-on.png"), QtGui.QIcon.Mode.Normal, QtGui.QIcon.State.Off)
            self.ui.startButton.setIcon(icon1)
            self.timer.stop()
            self.ui.centralwidget.setStyleSheet("background-color: #FF6666;")
            self.approval = False
            
            msgBox_emergency = QMessageBox(self.ui.centralwidget)
            msgBox_emergency.setIcon(QMessageBox.Icon.Warning)
            msgBox_emergency.setText("Araç acil durumdan dolayı durduruldu. Lütfen aracı kontrol ediniz.")
            msgBox_emergency.setWindowTitle("ACİL DURDURMA")
            msgBox_emergency.setStandardButtons(QMessageBox.StandardButton.Ok)
            msgBox_emergency.setStyleSheet("QMessageBox {background-color: #FF6666; color: white;} QPushButton {color: black;}")
            msgBox_emergency.exec()
        else:
            self.ui.centralwidget.setStyleSheet("")
            self.ui.emergencyButton.setText("Acil Durdurma")
            msgBox_emergency = QMessageBox(self.ui.centralwidget)
            msgBox_emergency.setIcon(QMessageBox.Icon.Warning)
            msgBox_emergency.setText("Araç şu anda çalışmıyor.")
            msgBox_emergency.setWindowTitle("Acil durdurma kapatıldı")
            msgBox_emergency.setStandardButtons(QMessageBox.StandardButton.Ok)
            msgBox_emergency.setStyleSheet("QMessageBox {background-color: #FF6666; color: white;} QPushButton {color: black;}")
            msgBox_emergency.exec()

    def update_time(self):
        self.elapsed_time += 1
        hours, remainder = divmod(self.elapsed_time, 3600)
        minutes, seconds = divmod(remainder, 60)
        self.ui.lineEdit_time.setText(f"{hours:02}:{minutes:02}:{seconds:02}")


def main():
    rclpy.init(args=None)
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    def run_ros():
        rclpy.spin(win)

    thread = threading.Thread(target=run_ros)
    thread.start()
    app.exec()
    rclpy.shutdown()
    thread.join()

if __name__ == "__main__":
    main()