from flask import Flask, send_file, jsonify
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import threading
from PIL import Image, ImageOps
import io

app = Flask(__name__)

global ros_node

class ROS2Subscriber(Node):
    def __init__(self):
        super().__init__('map_subscriber')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.topic_callback,
            10)
        self.last_message = None

    def topic_callback(self, msg):
        print("Map data received")
        self.last_message = msg

def run_ros_node():
    global ros_node
    rclpy.init()
    ros_node = ROS2Subscriber()
    rclpy.spin(ros_node)

ros_thread = threading.Thread(target=run_ros_node, daemon=True)
ros_thread.start()

@app.route('/map')
def map():
    global ros_node
    if ros_node.last_message is not None:
        width = ros_node.last_message.info.width
        height = ros_node.last_message.info.height
        data = ros_node.last_message.data

        # Görüntüyü oluştur
        image = Image.new('L', (width, height))
        image.putdata([255 if x == 0 else 0 if x == 100 else 128 for x in data])
        image = image.transpose(Image.FLIP_TOP_BOTTOM)  # ROS koordinat sistemini düzelten döndürme

        # Görüntüyü büyüt
        scale_factor = 4  # Görüntüyü büyütme faktörü
        new_width = width * scale_factor
        new_height = height * scale_factor
        image = image.resize((new_width, new_height), Image.NEAREST)

        # Görüntüyü bellekte tut
        img_byte_arr = io.BytesIO()
        image.save(img_byte_arr, format='PNG')
        img_byte_arr.seek(0)

        return send_file(img_byte_arr, mimetype='image/png')
    else:
        return jsonify({'error': 'No map data received yet'})


app.run(debug=True, use_reloader=False)