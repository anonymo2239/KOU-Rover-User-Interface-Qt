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
from example_interfaces.msg import Int64
from geometry_msgs.msg import Twist
import select, termios, tty

# Qt
from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel, QDialog, QPushButton, QMessageBox
from PyQt6.QtCore import QTimer
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtCore import QObject, pyqtSignal

from gui_pkg.rovergui_2_0 import Ui_rover_gui

REVISION_MAX_LIN_VEL = 0.22
REVISION_MAX_ANG_VEL = 2.84

LIN_VEL_STEP_SIZE = 0.02
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Re-vision 2024!
-------------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (~ 0.22)
a/d : increase/decrease angular velocity (~ 2.84)

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

class MainWindow(QMainWindow, Ui_rover_gui, Node):

    qr_received = pyqtSignal(str)  # QR kod string'ini göndermek için sinyal
    start_const = 0
    approval = False
    forward_approval = False
    i = 0

    def __init__(self, parent=None):
        QMainWindow.__init__(self, parent)
        Node.__init__(self, 'main_window_node')
        self.ui = Ui_rover_gui()
        self.ui.setupUi(self)

        self.get_logger().info("ROS initialized.")
        self.connect_ros()
        self.qr_received.connect(self.update_text_edit)

        # Connecting Functions 
        self.ui.pushButton_2.clicked.connect(self.showExitDialog)
        self.ui.pushButton_exit_fullscreen.clicked.connect(self.setFullScreen)
        self.ui.routeInfoButton.clicked.connect(self.createRouteInfoDialog)
        self.ui.startButton.clicked.connect(self.start_vehicle)
        self.ui.finishButton.clicked.connect(self.get_through_vehicle)
        self.ui.emergencyButton.clicked.connect(self.emergency_sit)
        self.ui.pushButton.clicked.connect(self.approval_sit)
        
        self.timer = QTimer(self)  # Bu timer diğerlerinin yerine kullanılacak
        self.timer.timeout.connect(self.read_key)
        self.timer.timeout.connect(self.update_time)
        # self.timer.start(100)
        self.centralwidget = QtWidgets.QWidget(self)
        self.centralwidget.setObjectName("centralwidget")
        self.elapsed_time = 0

        # Turtle
        self.settings = termios.tcgetattr(sys.stdin)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status = 0
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.control_linear_vel = 0.0
        self.control_angular_vel = 0.0

        # Movement buttons
        self.ui.btn_forward.clicked.connect(lambda: self.update_twist('w'))
        self.ui.btn_backward.clicked.connect(lambda: self.update_twist('x'))
        self.ui.btn_left.clicked.connect(lambda: self.update_twist('a'))
        self.ui.btn_right.clicked.connect(lambda: self.update_twist('d'))
        self.ui.btn_brake.clicked.connect(lambda: self.update_twist(' '))



    # //////////////////// THE FUNCTIONS I WROTE ////////////////////

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
        self.ui.comboBox_scen.setEnabled(False)

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
            self.timer.stop()  # Timer'ı durdur
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

    # ////////////////////////////////////////////////



    # //////////////////// QRCODE AND ROS ////////////////////
    def connect_ros(self):
        self.qr_subscriber = self.create_subscription(
            String, 'qr_code', self.print_QR, 10)
        
        self.temperature_subscriber = self.create_subscription(
            Int32, 'temperature_info', self.print_temperature, 10)
        
        self.current_subscriber = self.create_subscription(
            Int32, 'current_info', self.print_current, 10)
        
        self.charge_subscriber = self.create_subscription(
            Int32, 'charge_info', self.print_charge, 10)
        
        self.load_subscriber = self.create_subscription(
            Int32, 'load_info', self.print_load, 10)
        
        self.velocity_subscriber = self.create_subscription(
            Int32, 'velocity_info', self.print_velocity, 10)

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

    def print_load(self, msg):
        if self.approval == True:
            self.ui.lineEdit_load.setText(str(msg.data))
        else:
            pass

    def print_velocity(self, msg):
        if self.approval == True:
            self.ui.lineEdit_velocity.setText(str(msg.data))
        else:
            pass
    
    # ////////////////////////////////////////////////


    # //////////////////// TURTLE ////////////////////

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self, target_linear_vel, target_angular_vel):
        return f"currently:\tlinear vel {target_linear_vel}\t angular vel {target_angular_vel}"

    def makeSimpleProfile(self, output, input, slop):
        if input > output:
            output = min(input, output + slop)
        elif input < output:
            output = max(input, output - slop)
        else:
            output = input
        return output

    def constrain(self, input, low, high):
        return max(min(input, high), low)

    def checkLinearLimitVelocity(self, vel):
        return self.constrain(vel, -REVISION_MAX_LIN_VEL, REVISION_MAX_LIN_VEL)

    def checkAngularLimitVelocity(self, vel):
        return self.constrain(vel, -REVISION_MAX_ANG_VEL, REVISION_MAX_ANG_VEL)

    def read_key(self):
        key = self.getKey()
        if key:
            self.update_twist(key)

    def keyPressEvent(self, event):
        key = event.key()
        if key == QtCore.Qt.Key.Key_W:
            self.update_twist('w')
            self.change_button_color(self.ui.btn_forward)
        elif key == QtCore.Qt.Key.Key_X:
            self.update_twist('x')
            self.change_button_color(self.ui.btn_backward)
        elif key == QtCore.Qt.Key.Key_A:
            self.update_twist('a')
            self.change_button_color(self.ui.btn_left)
        elif key == QtCore.Qt.Key.Key_D:
            self.update_twist('d')
            self.change_button_color(self.ui.btn_right)
        elif key == QtCore.Qt.Key.Key_Space or key == QtCore.Qt.Key.Key_S:
            self.update_twist(' ')
            self.change_button_color(self.ui.btn_brake)

    def change_button_color(self, button):
        button.setStyleSheet("background-color: #A9A9A9")
        QTimer.singleShot(500, self.reset_button_colors)

    def reset_button_colors(self):
        self.ui.btn_forward.setStyleSheet("")
        self.ui.btn_backward.setStyleSheet("")
        self.ui.btn_left.setStyleSheet("")
        self.ui.btn_right.setStyleSheet("")
        self.ui.btn_brake.setStyleSheet("")

    def update_twist(self, key):
        try:
            if key == 'w':
                self.target_linear_vel = self.checkLinearLimitVelocity(self.target_linear_vel + LIN_VEL_STEP_SIZE)
                self.status += 1
            elif key == 'x':
                self.target_linear_vel = self.checkLinearLimitVelocity(self.target_linear_vel - LIN_VEL_STEP_SIZE)
                self.status += 1
            elif key == 'a':
                self.target_angular_vel = self.checkAngularLimitVelocity(self.target_angular_vel + ANG_VEL_STEP_SIZE)
                self.status += 1
            elif key == 'd':
                self.target_angular_vel = self.checkAngularLimitVelocity(self.target_angular_vel - ANG_VEL_STEP_SIZE)
                self.status += 1
            elif key == ' ' or key == 's':
                self.target_linear_vel = 0.0
                self.control_linear_vel = 0.0
                self.target_angular_vel = 0.0
                self.control_angular_vel = 0.0

            if self.status == 20:
                print(msg)
                self.status = 0

            twist = Twist()
            self.control_linear_vel = self.makeSimpleProfile(self.control_linear_vel, self.target_linear_vel, (LIN_VEL_STEP_SIZE / 2.0))
            twist.linear.x = self.control_linear_vel
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            self.control_angular_vel = self.makeSimpleProfile(self.control_angular_vel, self.target_angular_vel, (ANG_VEL_STEP_SIZE / 2.0))
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = self.control_angular_vel
            self.pub.publish(twist)

        except Exception as ex:
            print("Hata: ", str(ex))

    # ////////////////////////////////////////////////


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