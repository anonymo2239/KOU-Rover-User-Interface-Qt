import sys
import threading
from PyQt6.QtWidgets import QMainWindow, QApplication
from PyQt6.QtCore import QTimer, pyqtSignal, QThread, QObject
from PyQt6 import QtCore
from rclpy.node import Node
import rclpy
from std_msgs.msg import String
from example_interfaces.msg import Int64

from gui_pkg.guiros import Ui_MainWindow

class MainWindow(QMainWindow, Ui_MainWindow, Node):
    qr_received = pyqtSignal(str)  # QR kod string'ini göndermek için sinyal

    def __init__(self, parent=None):
        QMainWindow.__init__(self, parent)
        Node.__init__(self, 'main_window_node')
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.get_logger().info("ROS initialized.")
        self.connect_ros()
        self.qr_received.connect(self.update_text_edit)

        self.start_stop_publisher = self.create_publisher(String, "start_stop", 10)
        self.get_logger().info("Robot News Station has been started")
        self.ui.pushButton_2.clicked.connect(self.publish_start)


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
        self.qr_received.emit(msg.data)  # QR verisi ile sinyali tetikle

    def update_text_edit(self, qr_data):
        _translate = QtCore.QCoreApplication.translate
        self.ui.plainTextEdit.setPlainText(_translate("MainWindow", qr_data))

    def print_continuous(self, msg):
        self.ui.lineEdit.setText(str(msg.data))


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