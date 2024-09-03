import sys
import threading
import os
current_dir = os.path.dirname(os.path.abspath(__file__))

# ROS 2
from rclpy.node import Node
import rclpy
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import select, termios, tty

# Qt
from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QLabel, QDialog, QMessageBox, QLineEdit
from PyQt6.QtCore import QTimer, pyqtSignal, QMutex, Qt
from PyQt6.QtGui import QPixmap

from gui_pkg.rovergui_2_0 import Ui_rover_gui

REVISION_MAX_LIN_VEL = 0.22
REVISION_MAX_ANG_VEL = 0.3

LIN_VEL_STEP_SIZE = 0.02
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Re-vision 2024!
-------------------------------
Moving around:
 w
 a s d
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
    update_gui_signal = pyqtSignal(str, str)
   
    def __init__(self, parent=None):
        QMainWindow.__init__(self, parent)
        Node.__init__(self, 'main_window_node')
        self.ui = Ui_rover_gui()
        self.ui.setupUi(self)

        self.data_mutex = QMutex()
       
        self.start_const = 0
        self.approval = False
        self.emergency = False
        self.engine_running = False
        self.remote_control = False
        self.i = 0

        self.qr_widgets = {f'qr_image_{i}': getattr(self.ui, f'qr_image_{i}') for i in range(1, 53)}

        self.last_msg_time = None
        self.check_connection_timer = QTimer(self)
        self.check_connection_timer.timeout.connect(self.check_connection_status)
        self.check_connection_timer.start(1000)

        self.odom_subscription = self.create_subscription(
            Odometry, '/diff_cont/odom', self.check_vehicle, 10
        )

        self.get_logger().info("ROS initialized.")
        self.connect_ros()
        self.qr_received.connect(self.update_text_edit, Qt.ConnectionType.QueuedConnection)
        self.update_gui_signal.connect(self.update_gui, Qt.ConnectionType.QueuedConnection)

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
        self.ui.pushButtonTurtle.clicked.connect(self.inActiveController)
       
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_time)

        self.centralwidget = QtWidgets.QWidget(self)
        self.centralwidget.setObjectName("centralwidget")
        self.elapsed_time = 0

    def update_gui(self, widget_name, value):
        if hasattr(self.ui, widget_name):
            widget = getattr(self.ui, widget_name)
            if isinstance(widget, QLineEdit):
                widget.setText(value)
            elif isinstance(widget, QLabel):
                widget.setText(value)

    def inActiveController(self):
        if not self.remote_control:
            self.ui.pushButtonTurtle.setText("Uzaktan Kontrol Açık")
            self.ui.btn_backward.setEnabled(True)
            self.ui.btn_brake.setEnabled(True)
            self.ui.btn_forward.setEnabled(True)
            self.ui.btn_left.setEnabled(True)
            self.ui.btn_right.setEnabled(True)
            icon3 = QtGui.QIcon()
            icon3.addPixmap(QtGui.QPixmap(current_dir + "/images/game_green.png"), QtGui.QIcon.Mode.Normal, QtGui.QIcon.State.Off)
            self.ui.pushButtonTurtle.setIcon(icon3)
            self.remote_control = True

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
           
            self.time_timer = QTimer(self)
            self.time_timer.timeout.connect(self.update_twist)
            self.time_timer.timeout.connect(self.read_key)
            self.time_timer.start(100)
        else:
            self.ui.pushButtonTurtle.setText("Uzaktan Kontrol Kapalı")
            self.ui.btn_backward.setEnabled(False)
            self.ui.btn_brake.setEnabled(False)
            self.ui.btn_forward.setEnabled(False)
            self.ui.btn_left.setEnabled(False)
            self.ui.btn_right.setEnabled(False)
            icon3 = QtGui.QIcon()
            icon3.addPixmap(QtGui.QPixmap(current_dir + "/images/game_red.png"), QtGui.QIcon.Mode.Normal, QtGui.QIcon.State.Off)
            self.ui.pushButtonTurtle.setIcon(icon3)
            self.remote_control = False

            self.target_linear_vel = 0.0
            self.target_angular_vel = 0.0
            self.control_linear_vel = 0.0
            self.control_angular_vel = 0.0
            self.update_twist()

            self.time_timer.stop()

    def reset_qr_map(self):
        for widget in self.qr_widgets.values():
            widget.setVisible(False)

    def reset_info(self):
        self.timer.stop()
        self.elapsed_time = 0
        self.ui.lineEdit_time.setText("00:00:00")
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
        image_path = current_dir + f"/images/{'yuklu' if scenario_index > 3 else 'bos'}tur{scenario_index % 4 + 1}.jpg"
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
        if not self.emergency:
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
                self.update_gui_signal.emit("label_situation", "Çalışıyor")
                self.update_gui_signal.emit("label_situation_2", "Çalışıyor")
                self.engine_running = True
                icon1 = QtGui.QIcon()
                icon1.addPixmap(QtGui.QPixmap(current_dir + "/images/power-off.png"), QtGui.QIcon.Mode.Normal, QtGui.QIcon.State.Off)
                self.ui.startButton.setIcon(icon1)
                self.timer.start(1000)
                self.ui.finishButton.setVisible(True)
                self.reset_qr_map()
            else:
                self.ui.startButton.setText("Devam Et")
                self.update_gui_signal.emit("label_situation", "Duraklatıldı")
                self.update_gui_signal.emit("label_situation_2", "Duraklatıldı")
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
            self.update_gui_signal.emit("label_situation", "Beklemede")
            self.update_gui_signal.emit("label_situation_2", "Beklemede")
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
            self.update_gui_signal.emit("label_situation", "Acil Durumda")
            self.update_gui_signal.emit("label_situation_2", "Acil Durumda")
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
        elif not self.approval and self.emergency:
            self.ui.centralwidget.setStyleSheet("")
            self.ui.emergencyButton.setText("Acil Durdurma")
            self.ui.emergencyButton_2.setText("Acil Durdurma")
            self.ui.startButton.setText("Başlat")
            self.update_gui_signal.emit("label_situation", "Beklemede")
            self.update_gui_signal.emit("label_situation_2", "Beklemede")
            self.emergency = False
            self.ui.comboBox_scen.setEnabled(True)
            self.ui.finishButton.setVisible(False)
            self.reset_info()
        else:
            self.emergency = False
            self.engine_running = False
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

    def connect_ros(self):
        self.start_scen = self.create_publisher(Int32, 'scene_gui', 10)
        self.gui_start_publisher = self.create_publisher(Bool, 'gui_start', 10)
        self.qr_subscriber = self.create_subscription(String, 'qr_code', self.print_QR, 10)
        self.lift_status_sub = self.create_subscription(Bool, 'lift_command', self.lift_status, 10)
        self.overload_sub = self.create_subscription(Bool, 'overload_error', self.overload, 10)
        self.obstacle_subscriber = self.create_subscription(Int32, 'obstacle', self.print_obstacle, 10)
        self.temperature_subscriber = self.create_subscription(String, 'sicaklik_data', self.print_temperature, 10)
        self.current_subscriber = self.create_subscription(String, 'acisal_hiz_data', self.print_current, 10)
        self.velocity_subscriber = self.create_subscription(String, 'lineer_hiz_data', self.print_velocity, 10)
        self.load_subscriber = self.create_subscription(String, 'agirlik_data', self.print_load, 10) # 
    
    def print_QR(self, msg):
        self.qr_received.emit(msg.data)

    def update_text_edit(self, qr_data):
        self.data_mutex.lock()
        try:
            _translate = QtCore.QCoreApplication.translate
            self.ui.plainQRCODE.setPlainText(_translate("rover_gui", qr_data))

            self.reset_qr_map()
            numeric_part = ''.join(filter(str.isdigit, qr_data))
            widget_name = f"qr_image_{numeric_part}"
            widget = self.qr_widgets.get(widget_name)
            if widget:
                widget.setVisible(True)
        finally:
            self.data_mutex.unlock()

    def print_temperature(self, msg):
        if self.engine_running:
            self.update_gui_signal.emit("lineEdit_temperature", str(msg.data))

    def print_load(self, msg):# 
        if self.engine_running:
            self.update_gui_signal.emit("lineEdit_load", str(msg.data))

    def print_obstacle(self, msg):
        if msg.data == 1:
            self.update_gui_signal.emit("label_situation", "Engel Algılandı")
            self.update_gui_signal.emit("label_situation_2", "Engel Algılandı")
        else:
            if self.engine_running:
                self.update_gui_signal.emit("label_situation", "Çalışıyor")
                self.update_gui_signal.emit("label_situation_2", "Çalışıyor")
            else:
                self.update_gui_signal.emit("label_situation", "Beklemede")
                self.update_gui_signal.emit("label_situation_2", "Beklemede")

    def print_velocity(self, msg):
        if self.engine_running == True:
            self.update_gui_signal.emit("lineEdit_velocity", str(msg.data))

    def print_current(self, msg):
        if self.engine_running:
            self.update_gui_signal.emit("lineEdit_current", str(msg.data))

    def overload(self, msg):
        if msg.data:
            self.update_gui_signal.emit("label_load_response", "Aşırı Yük")
            pixmap = QtGui.QPixmap(current_dir + "/images/boxes_red.png")
            self.ui.label_load.setPixmap(pixmap)
        else:
            pixmap = QtGui.QPixmap(current_dir + "/images/boxes.png")
            self.ui.label_load.setPixmap(pixmap)
            pass

    def lift_status(self, msg):
        if msg.data and self.ui.label_load_response.text() != "Aşırı Yük":
            self.update_gui_signal.emit("label_load_response", "Yüklü")
        elif not msg.data:
            self.update_gui_signal.emit("label_load_response", "Yüklü Değil")

    def publish_scenario(self):
        if not self.engine_running and not self.approval:
            msg = Int32()
            msg.data = int(self.ui.comboBox_scen.currentIndex()) + 1
            self.start_scen.publish(msg)

    def publish_engine_status_startbutton(self):
        msg = Bool()
        if self.approval and not self.engine_running and not self.emergency:
            msg.data = True
            self.gui_start_publisher.publish(msg)
        elif self.approval and self.engine_running and not self.emergency:
            msg.data = False
            self.gui_start_publisher.publish(msg)

    def publish_engine_status_finishbutton(self):
        if self.approval and self.engine_running and not self.emergency:
            msg = Bool()
            msg.data = False
            self.gui_start_publisher.publish(msg)

    def publish_engine_status_emergencybutton(self):
        if not self.approval and not self.engine_running and self.emergency:
            msg = Bool()
            msg.data = False
            self.gui_start_publisher.publish(msg)

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
        pixmap = QtGui.QPixmap(current_dir + ("/images/rss2.png" if is_connected else "/images/rss.png"))
        text = "Bağlı" if is_connected else "Bağlı Değil"

        self.ui.label_connection.setPixmap(pixmap)
        self.ui.label_connection_3.setPixmap(pixmap)
        self.update_gui_signal.emit("label_connection_situation", text)
        self.update_gui_signal.emit("label_connection_situation_2", text)

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
        else:
            self.update_twist()

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
        for btn in [self.ui.btn_forward, self.ui.btn_backward, self.ui.btn_left, self.ui.btn_right, self.ui.btn_brake]:
            btn.setStyleSheet("")

    def update_twist(self, key=None):
        try:
            if key:
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

def main():
    rclpy.init(args=None)
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()

    def run_ros():
        while rclpy.ok():
            rclpy.spin_once(win, timeout_sec=0.1)

    ros_thread = threading.Thread(target=run_ros, daemon=True)
    ros_thread.start()

    exit_code = app.exec()
    rclpy.shutdown()
    ros_thread.join(timeout=1.0)
    sys.exit(exit_code)

if __name__ == "__main__":
    main()