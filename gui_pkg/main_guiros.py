import sys
import threading
from PyQt6.QtWidgets import QMainWindow, QApplication
from PyQt6.QtCore import QTimer, pyqtSignal, QThread
from PyQt6 import QtCore, QtGui
from rclpy.node import Node
import rclpy
from std_msgs.msg import String
from example_interfaces.msg import Int64
from geometry_msgs.msg import Twist
import select, termios, tty

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

from gui_pkg.guiros import Ui_MainWindow 

class MainWindow(QMainWindow, Ui_MainWindow, Node):
    qr_received = pyqtSignal(str)

    def __init__(self, parent=None):
        QMainWindow.__init__(self, parent)
        Node.__init__(self, 'main_window_node')
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.get_logger().info("ROS initialized.")
        self.connect_ros()
        self.qr_received.connect(self.update_text_edit)
        
        self.start_stop_publisher = self.create_publisher(String, "start_stop", 10)
        self.ui.pushButton_2.clicked.connect(self.publish_start)

        self.settings = termios.tcgetattr(sys.stdin)

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.status = 0
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.control_linear_vel = 0.0
        self.control_angular_vel = 0.0

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.read_key)
        self.timer.start(100)

        # GUI buttons
        self.ui.btn_forward.clicked.connect(lambda: self.update_twist('w'))
        self.ui.btn_backward.clicked.connect(lambda: self.update_twist('x'))
        self.ui.btn_left.clicked.connect(lambda: self.update_twist('a'))
        self.ui.btn_right.clicked.connect(lambda: self.update_twist('d'))
        self.ui.btn_brake.clicked.connect(lambda: self.update_twist(' '))

        # Timer for resetting button color
        self.reset_color_timer = QTimer(self)
        self.reset_color_timer.setSingleShot(True)
        self.reset_color_timer.timeout.connect(self.reset_button_colors)

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

    def connect_ros(self):
        self.qr_subscriber = self.create_subscription(
            String, 'qr_code', self.print_QR, 10)
        self.get_logger().info("QR Scanner Node has been started.")

        self.continuous_subscriber = self.create_subscription(
            Int64, 'robot_news', self.print_continuous, 10)

    def publish_start(self):
        msg = String()
        msg.data = self.ui.lineEdit_2.text()
        self.start_stop_publisher.publish(msg)

    def print_QR(self, msg):
        self.qr_received.emit(msg.data)

    def update_text_edit(self, qr_data):
        self.ui.plainTextEdit.setPlainText(qr_data)

    def print_continuous(self, msg):
        self.ui.lineEdit.setText(str(msg.data))

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
        button.setStyleSheet("background-color: yellow")
        self.reset_color_timer.start(500)  # Reset color after 500ms

    def reset_button_colors(self):
        self.ui.btn_forward.setStyleSheet("background-color: #00008B;")
        self.ui.btn_backward.setStyleSheet("background-color: #00008B;")
        self.ui.btn_left.setStyleSheet("background-color: #00008B;")
        self.ui.btn_right.setStyleSheet("background-color: #00008B;")
        self.ui.btn_brake.setStyleSheet("background-color: #00008B;")

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
