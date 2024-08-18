import sys
import threading
import os
current_dir = os.path.dirname(os.path.abspath(__file__))


# ROS 2
from rclpy.node import Node
import rclpy
from std_msgs.msg import String
from PyQt6.QtCore import pyqtSignal
from std_msgs.msg import String, Float32, Bool
from example_interfaces.msg import Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import select, termios, tty
from rclpy.qos_event import SubscriptionEventCallbacks

# Qt
from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QLabel, QDialog, QMessageBox
from PyQt6.QtCore import QTimer
from PyQt6.QtCore import pyqtSignal
from PyQt6.QtGui import QPixmap

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

    qr_received = pyqtSignal(str)
    start_const = 0
    approval = False
    emergency = False
    engine_running = False
    i = 0

    def __init__(self, parent=None):
        QMainWindow.__init__(self, parent)
        Node.__init__(self, 'main_window_node')
        self.ui = Ui_rover_gui()
        self.ui.setupUi(self)

        self.qr_widgets = {
            'qr_image_1': self.ui.qr_image_1,
            'qr_image_2': self.ui.qr_image_2,
            'qr_image_3': self.ui.qr_image_3,
            'qr_image_4': self.ui.qr_image_4,
            'qr_image_5': self.ui.qr_image_5,
            'qr_image_6': self.ui.qr_image_6,
            'qr_image_7': self.ui.qr_image_7,
            'qr_image_8': self.ui.qr_image_8,
            'qr_image_9': self.ui.qr_image_9,
            'qr_image_10': self.ui.qr_image_10,
            'qr_image_11': self.ui.qr_image_11,
            'qr_image_12': self.ui.qr_image_12,
            'qr_image_13': self.ui.qr_image_13,
            'qr_image_14': self.ui.qr_image_14,
            'qr_image_15': self.ui.qr_image_15,
            'qr_image_16': self.ui.qr_image_16,
            'qr_image_17': self.ui.qr_image_17,
            'qr_image_18': self.ui.qr_image_18,
            'qr_image_19': self.ui.qr_image_19,
            'qr_image_20': self.ui.qr_image_20,
            'qr_image_21': self.ui.qr_image_21,
            'qr_image_22': self.ui.qr_image_22,
            'qr_image_23': self.ui.qr_image_23,
            'qr_image_24': self.ui.qr_image_24,
            'qr_image_25': self.ui.qr_image_25,
            'qr_image_26': self.ui.qr_image_26,
            'qr_image_27': self.ui.qr_image_27,
            'qr_image_28': self.ui.qr_image_28,
            'qr_image_29': self.ui.qr_image_29,
            'qr_image_30': self.ui.qr_image_30,
            'qr_image_31': self.ui.qr_image_31,
            'qr_image_32': self.ui.qr_image_32,
            'qr_image_33': self.ui.qr_image_33,
            'qr_image_34': self.ui.qr_image_34,
            'qr_image_35': self.ui.qr_image_35,
            'qr_image_36': self.ui.qr_image_36,
            'qr_image_37': self.ui.qr_image_37,
            'qr_image_38': self.ui.qr_image_38,
            'qr_image_39': self.ui.qr_image_39,
            'qr_image_40': self.ui.qr_image_40,
            'qr_image_41': self.ui.qr_image_41,
            'qr_image_42': self.ui.qr_image_42,
            'qr_image_43': self.ui.qr_image_43,
            'qr_image_44': self.ui.qr_image_44,
            'qr_image_45': self.ui.qr_image_45,
            'qr_image_46': self.ui.qr_image_46,
            'qr_image_47': self.ui.qr_image_47,
            'qr_image_48': self.ui.qr_image_48,
            'qr_image_49': self.ui.qr_image_49,
            'qr_image_50': self.ui.qr_image_50,
            'qr_image_51': self.ui.qr_image_51,
            'qr_image_52': self.ui.qr_image_52
        }

        self.last_msg_time = None
        self.check_connection_timer = QTimer(self)
        self.check_connection_timer.timeout.connect(self.check_connection_status)
        self.check_connection_timer.start(1000)

        self.odom_subscription = self.create_subscription(
            Odometry, '/diff_cont/odom', self.check_vehicle, 10
        )

        self.get_logger().info("ROS initialized.")
        self.connect_ros()
        self.qr_received.connect(self.update_text_edit)

        # Connecting Functions 
        self.ui.pushButton_2.clicked.connect(self.showExitDialog)
        self.ui.pushButton_exit_fullscreen.clicked.connect(self.setFullScreen)
        self.ui.routeInfoButton.clicked.connect(self.createRouteInfoDialog)
        self.ui.pushButton.clicked.connect(self.publish_scenario)
        self.ui.startButton.clicked.connect(self.publish_engine_status_startbutton)
        self.ui.startButton.clicked.connect(self.start_vehicle)
        self.ui.finishButton.clicked.connect(self.publish_engine_status_finishbutton)
        self.ui.finishButton.clicked.connect(self.get_through_vehicle)
        self.ui.emergencyButton.clicked.connect(self.emergency_sit)
        self.ui.emergencyButton_2.clicked.connect(self.emergency_sit)
        self.ui.pushButton.clicked.connect(self.comboBox_status)
        
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.read_key)
        self.timer.timeout.connect(self.update_time)
        # self.timer.start(100)
        self.centralwidget = QtWidgets.QWidget(self)
        self.centralwidget.setObjectName("centralwidget")
        self.elapsed_time = 0

        # Turtle
        self.settings = termios.tcgetattr(sys.stdin)
        self.pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
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

    def reset_info(self):
        self.timer.stop()
        self.elapsed_time = 0
        self.ui.lineEdit_time.setText("00:00:00")
        self.ui.lineEdit_charge.setText("0")
        self.ui.lineEdit_current.setText("0")
        self.ui.lineEdit_load.setText("0")
        self.ui.lineEdit_temperature.setText("0")
        self.ui.lineEdit_velocity.setText("0")

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
            QApplication.quit()

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

        scenario_index = self.ui.comboBox_scen.currentIndex()
        if scenario_index == 0:
            image_path = current_dir + "/images/bostur1.jpg"
            text = "TEKNOFEST 2024"
        elif scenario_index == 1:
            image_path = current_dir + "/images/bostur2.jpg"
            text = "TEKNOFEST 2024"
        elif scenario_index == 2:
            image_path = current_dir + "/images/bostur3.jpg"
            text = "TEKNOFEST 2024"
        elif scenario_index == 3:
            image_path = current_dir + "/images/bostur4.jpg"
            text = "TEKNOFEST 2024"
        elif scenario_index == 4:
            image_path = current_dir + "/images/yuklutur1.jpg"
            text = "TEKNOFEST 2024"
        elif scenario_index == 5:
            image_path = current_dir + "/images/yuklutur2.jpg"
            text = "TEKNOFEST 2024"
        elif scenario_index == 6:
            image_path = current_dir + "/images/yuklutur3.jpg"
            text = "TEKNOFEST 2024"
        elif scenario_index == 7:
            image_path = current_dir + "/images/yuklutur4.jpg"
            text = "TEKNOFEST 2024"

        image_label = QLabel(self.dialog)
        pixmap = QPixmap(image_path)
        image_label.setPixmap(pixmap)
        image_label.setScaledContents(True)
        self.label = QLabel(text, self.dialog)
        font = self.label.font()
        font.setPointSize(20)
        font.setBold(True)
        self.label.setFont(font)
        self.layout.addWidget(image_label)
        self.layout.addWidget(self.label)
        self.dialog.setLayout(self.layout)
        self.dialog.exec()

    def comboBox_status(self):
        if self.emergency == False:
            self.approval = True
            self.ui.comboBox_scen.setEnabled(False)
        else:
            msgBox_select_scenario = QMessageBox()
            msgBox_select_scenario.setIcon(QMessageBox.Icon.Warning)
            msgBox_select_scenario.setText("Senaryo seçmeden önce acil durumu iptal ediniz.")
            msgBox_select_scenario.setWindowTitle("Acil durumu kontrol ediniz.")
            msgBox_select_scenario.setStandardButtons(QMessageBox.StandardButton.Ok)
            msgBox_select_scenario.setStyleSheet("QMessageBox {background-color: #FF6666; color: white;} QPushButton {color: black;}")
            msgBox_select_scenario.exec()

    def start_vehicle(self):
        if self.approval:
            if not self.timer.isActive():
                self.ui.startButton.setText("Durdur")
                self.ui.label_situation.setText("Çalışıyor")
                self.ui.label_situation_2.setText("Çalışıyor")
                self.engine_running = True
                icon1 = QtGui.QIcon()
                icon1.addPixmap(QtGui.QPixmap(current_dir + "/images/power-off.png"), QtGui.QIcon.Mode.Normal, QtGui.QIcon.State.Off)
                self.ui.startButton.setIcon(icon1)
                self.timer.start(1000)
                self.ui.finishButton.setVisible(True)
            else:
                self.ui.startButton.setText("Devam Et")
                self.ui.label_situation.setText("Duraklatıldı")
                self.ui.label_situation_2.setText("Duraklatıldı")
                self.engine_running = False
                icon1 = QtGui.QIcon()
                icon1.addPixmap(QtGui.QPixmap(current_dir + "/images/power-on.png"), QtGui.QIcon.Mode.Normal, QtGui.QIcon.State.Off)
                self.ui.startButton.setIcon(icon1)
                self.timer.stop()
        else:
            self.show_scenario_warning()

    def get_through_vehicle(self):
        if self.approval:
            self.ui.finishButton.setVisible(False)
            self.ui.startButton.setText("Başlat")
            self.ui.label_situation.setText("Beklemede")
            self.ui.label_situation_2.setText("Beklemede")
            icon1 = QtGui.QIcon()
            icon1.addPixmap(QtGui.QPixmap(current_dir + "/images/power-on.png"), QtGui.QIcon.Mode.Normal, QtGui.QIcon.State.Off)
            self.ui.startButton.setIcon(icon1)
            self.approval = False
            self.engine_running = False
            self.ui.comboBox_scen.setEnabled(True)
            self.ui.comboBox_scen.setEditable(True)
            self.reset_info()
        else:
            msgBox_select_scenario = QMessageBox()
            msgBox_select_scenario.setIcon(QMessageBox.Icon.Warning)
            msgBox_select_scenario.setText("Acil Durum İptal butonuna basınız.")
            msgBox_select_scenario.setWindowTitle("Acil")
            msgBox_select_scenario.setStandardButtons(QMessageBox.StandardButton.Ok)
            msgBox_select_scenario.setStyleSheet("QMessageBox {background-color: #FF6666; color: white;} QPushButton {color: black;}")
            msgBox_select_scenario.exec()

    def show_scenario_warning(self):
        msgBox_select_scenario = QMessageBox()
        msgBox_select_scenario.setIcon(QMessageBox.Icon.Warning)
        msgBox_select_scenario.setText("Aracı başlatmadan önce lütfen senaryo seçiniz ve onayla butonuna basınız.")
        msgBox_select_scenario.setWindowTitle("Senaryo seçiniz.")
        msgBox_select_scenario.setStandardButtons(QMessageBox.StandardButton.Ok)
        msgBox_select_scenario.setStyleSheet("QMessageBox {background-color: #FF6666; color: white;} QPushButton {color: black;}")
        msgBox_select_scenario.exec()

    def emergency_sit(self):
        if self.approval and self.engine_running:
            self.ui.startButton.setText("Acil Durum")
            self.ui.label_situation.setText("Acil Durumda")
            self.ui.label_situation_2.setText("Acil Durumda")
            self.ui.emergencyButton.setText("Acil Durum İptal")
            self.ui.emergencyButton_2.setText("Acil Durum İptal")
            icon1 = QtGui.QIcon()
            icon1.addPixmap(QtGui.QPixmap(current_dir + "/images/power-on.png"), QtGui.QIcon.Mode.Normal, QtGui.QIcon.State.Off)
            self.ui.startButton.setIcon(icon1)
            self.timer.stop()
            self.ui.centralwidget.setStyleSheet("background-color: #FF6666;")
            self.approval = False
            self.emergency = True
            self.engine_running = False
            self.publish_engine_status_emergencybutton()
            msgBox_emergency = QMessageBox(self.ui.centralwidget)
            msgBox_emergency.setIcon(QMessageBox.Icon.Warning)
            msgBox_emergency.setText("Araç acil durumdan dolayı durduruldu. Lütfen aracı kontrol ediniz.")
            msgBox_emergency.setWindowTitle("ACİL DURDURMA")
            msgBox_emergency.setStandardButtons(QMessageBox.StandardButton.Ok)
            msgBox_emergency.setStyleSheet("QMessageBox {background-color: #FF6666; color: white;} QPushButton {color: black;}")
            msgBox_emergency.exec()

        elif self.approval == False and self.emergency == True:
            self.ui.centralwidget.setStyleSheet("")
            self.ui.emergencyButton.setText("Acil Durdurma")
            self.ui.emergencyButton_2.setText("Acil Durdurma")
            self.ui.startButton.setText("Başlat")
            self.ui.label_situation.setText("Beklemede")
            self.ui.label_situation_2.setText("Beklemede")
            self.emergency = False
            self.ui.comboBox_scen.setEnabled(True)
            self.ui.finishButton.setVisible(False)
            self.reset_info()

        elif self.approval == False and self.emergency == False:
            self.emergency = False
            self.engine_running = False
            msgBox_emergency = QMessageBox(self.ui.centralwidget)
            msgBox_emergency.setIcon(QMessageBox.Icon.Warning)
            msgBox_emergency.setText("Araç şu anda çalışmıyor.")
            msgBox_emergency.setWindowTitle("Acil durdurma kapatıldı")
            msgBox_emergency.setStandardButtons(QMessageBox.StandardButton.Ok)
            msgBox_emergency.setStyleSheet("QMessageBox {background-color: #FF6666; color: white;} QPushButton {color: black;}")
            msgBox_emergency.exec()

        else:
            self.emergency = False
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
        self.start_scen = self.create_publisher(Int32, 'scene_gui', 10)
        
        self.gui_start_publisher = self.create_publisher(Bool, 'gui_start', 10)

        self.qr_subscriber = self.create_subscription(
            String, 'qr_code', self.print_QR, 10)
        
        self.temperature_subscriber = self.create_subscription(
            String, 'sicaklik_data', self.print_temperature, 10)
        
        self.current_subscriber = self.create_subscription(
            String, 'acisal_hiz_data', self.print_current, 10)
        
        self.charge_subscriber = self.create_subscription(
            String, 'aku1_data', self.print_charge, 10)
        
        self.load_subscriber = self.create_subscription(
            String, 'agirlik_data', self.print_load, 10)
        
        self.velocity_subscriber = self.create_subscription(
            String, 'lineer_hiz_data', self.print_velocity, 10)

    def print_QR(self, msg):
        self.qr_received.emit(msg.data)

    def update_text_edit(self, qr_data):
        _translate = QtCore.QCoreApplication.translate
        self.ui.plainQRCODE.setPlainText(_translate("rover_gui", qr_data))

        numeric_part = ''.join(filter(str.isdigit, qr_data))
        widget_name = f"qr_image_{numeric_part}"
        # Sözlükten widget'ı bul ve görünürlüğünü ayarla
        widget = self.qr_widgets.get(widget_name)
        widget.setVisible(True)

    def print_temperature(self, msg):
        if self.engine_running == True:
            self.ui.lineEdit_temperature.setText(str(msg.data))
        else:
            pass

    def print_current(self, msg):
        if self.engine_running == True:
            self.ui.lineEdit_current.setText(str(msg.data))
        else:
            pass

    def print_charge(self, msg):
        if self.engine_running == True:
            self.ui.lineEdit_charge.setText(str(msg.data))
        else:
            pass

    def print_load(self, msg):
        if self.engine_running == True:
            self.ui.lineEdit_load.setText(str(msg.data))
        else:
            pass

    def print_velocity(self, msg):
        if self.engine_running == True:
            self.ui.lineEdit_velocity.setText(str(msg.data))
        else:
            pass

    def publish_scenario(self):
        if self.engine_running == False and self.approval == False:
            msg = Int32()
            msg.data = int(self.ui.comboBox_scen.currentIndex()) + 1
            self.start_scen.publish(msg)
    
    def publish_engine_status_startbutton(self):
        msg = Bool()
        if self.approval == True and self.engine_running == False and self.emergency == False:
            msg.data = True
            self.gui_start_publisher.publish(msg)
        elif self.approval == True and self.engine_running == True and self.emergency == False:
            msg.data = False
            self.gui_start_publisher.publish(msg)
        else: 
            pass

    def publish_engine_status_finishbutton(self):
        msg = Bool()
        if self.approval == True and self.engine_running == True and self.emergency == False:
            msg.data = False
            self.gui_start_publisher.publish(msg)
        else:
            pass

    def publish_engine_status_emergencybutton(self):
        msg = Bool()
        if self.approval == False and self.engine_running == False and self.emergency == True:
            msg.data = False
            self.gui_start_publisher.publish(msg)
        else:
            pass
    
    def check_vehicle(self, msg):
        self.last_msg_time = self.get_clock().now()

    def check_connection_status(self):
        current_time = self.get_clock().now()
        if self.last_msg_time:
            elapsed = current_time - self.last_msg_time
            if elapsed.nanoseconds > 2000000000:
                self.update_connection_status(False)
            else:
                self.update_connection_status(True)
        else:
            self.update_connection_status(False)

    def update_connection_status(self, is_connected):
        if is_connected:
            pixmap = QtGui.QPixmap(current_dir + "/images/rss2.png")
            text = "Bağlı"
        else:
            pixmap = QtGui.QPixmap(current_dir + "/images/rss.png")
            text = "Bağlı Değil"

        self.ui.label_connection.setPixmap(pixmap)
        self.ui.label_connection_3.setPixmap(pixmap)
        self.ui.label_connection_situation.setText(text)
        self.ui.label_connection_situation_2.setText(text)


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
        QTimer.singleShot(2000, self.reset_button_colors)

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

            # Hızı sürekli güncel tut
            self.control_linear_vel = self.makeSimpleProfile(self.control_linear_vel, self.target_linear_vel, (LIN_VEL_STEP_SIZE / 2.0))
            self.control_angular_vel = self.makeSimpleProfile(self.control_angular_vel, self.target_angular_vel, (ANG_VEL_STEP_SIZE / 2.0))

            twist = Twist()
            twist.linear.x = self.control_linear_vel
            twist.linear.y = 0.0
            twist.linear.z = 0.0
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
    print(current_dir)

    def run_ros():
        rclpy.spin(win)

    thread = threading.Thread(target=run_ros)
    thread.start()
    app.exec()
    rclpy.shutdown()
    thread.join()

if __name__ == "__main__":
    main()