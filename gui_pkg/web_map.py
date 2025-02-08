from flask import Flask, jsonify, render_template_string, send_file
from flask_socketio import SocketIO, emit
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from PIL import Image, ImageDraw
import io
import base64
import threading
import numpy as np
import math

app = Flask(__name__)
socketio = SocketIO(app)

global ros_node

class ROS2Subscriber(Node):
    def __init__(self):
        super().__init__('map_and_odom_subscriber')
        
        self.subscription_map = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/diff_cont/odom',
            self.odom_callback,
            10)
        
        self.last_map_message = None
        self.last_odom_message = None
        self.get_logger().info('Node initialized')

    def map_callback(self, msg):
        self.get_logger().info("Map data received")
        self.last_map_message = msg
        self.send_map_data()

    def odom_callback(self, msg):
        self.get_logger().info("Odom data received")
        self.last_odom_message = msg
        self.send_map_data()

    def send_map_data(self):
        if self.last_map_message is not None and self.last_odom_message is not None:
            image = self.create_map_image()
            img_byte_arr = io.BytesIO()
            image.save(img_byte_arr, format='PNG')
            img_byte_arr.seek(0)
            img_base64 = base64.b64encode(img_byte_arr.getvalue()).decode('utf-8')

            socketio.emit('map_data', {
                'data': img_base64,
                'width': image.width,
                'height': image.height
            })

    def create_map_image(self):
        width = self.last_map_message.info.width
        height = self.last_map_message.info.height
        data = np.array(self.last_map_message.data).reshape((height, width))

        # Scale up the image
        scale_factor = 4
        data_upscaled = np.kron(data, np.ones((scale_factor, scale_factor)))

        # Create a grayscale image
        image = Image.fromarray(data_upscaled.astype(np.uint8))
        image = image.point(lambda p: p * 255 // 100)
        
        # Convert to RGB
        image = image.convert('RGB')
        
        # Create a color map
        color_map = np.array([[192, 192, 192],  # Light gray for unknown
                              [255, 255, 255],  # White for free space
                              [0, 0, 0]])       # Black for occupied space
        
        # Apply color map
        image_array = np.array(image)
        image_array[data_upscaled == -1] = color_map[0]
        image_array[data_upscaled == 0] = color_map[1]
        image_array[data_upscaled == 100] = color_map[2]
        
        image = Image.fromarray(image_array)
        image = image.transpose(Image.FLIP_TOP_BOTTOM)

        # Draw robot position
        if self.last_odom_message is not None:
            draw = ImageDraw.Draw(image)
            robot_x = self.last_odom_message.pose.pose.position.x
            robot_y = self.last_odom_message.pose.pose.position.y
            orientation = self.last_odom_message.pose.pose.orientation

            # Convert robot position to pixel coordinates
            pixel_x = int((robot_x - self.last_map_message.info.origin.position.x) / self.last_map_message.info.resolution * scale_factor)
            pixel_y = int((robot_y - self.last_map_message.info.origin.position.y) / self.last_map_message.info.resolution * scale_factor)
            pixel_y = image.height - pixel_y  # Flip y-coordinate

            # Calculate robot orientation
            yaw = math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                             1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))

            # Draw triangle representing the robot
            triangle_size = 20
            x1 = pixel_x + triangle_size * math.cos(yaw)
            y1 = pixel_y - triangle_size * math.sin(yaw)
            x2 = pixel_x + triangle_size * math.cos(yaw + 2.0944)  # 2.0944 radians = 120 degrees
            y2 = pixel_y - triangle_size * math.sin(yaw + 2.0944)
            x3 = pixel_x + triangle_size * math.cos(yaw - 2.0944)
            y3 = pixel_y - triangle_size * math.sin(yaw - 2.0944)

            draw.polygon([(x1, y1), (x2, y2), (x3, y3)], fill=(255, 0, 0))  # Red triangle

        return image


@app.route('/')
def index():
    return render_template_string('''
        <!DOCTYPE html>
        <html>
        <head>
            <title>Map Viewer</title>  <!-- Başlık metni değiştirildi -->
            <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
            <style>
                body, html {
                    margin: 0;
                    padding: 0;
                    height: 100%;
                    width: 100%;
                    overflow: hidden;
                    background-color: #C0C0C0;  /* Daha açık gri arka plan */
                    display: flex;
                    flex-direction: column;
                }
                h1 {
                    color: black;
                    text-align: center;
                    padding: 10px;
                    margin: 0;
                }
                #map-container {
                    flex-grow: 1;
                    display: flex;
                    justify-content: center;
                    align-items: center;
                    overflow: hidden;
                }
                #map {
                    max-width: 98%;
                    max-height: 98%;
                    object-fit: contain;
                }
            </style>
        </head>
        <body>
            <h1></h1>  <!-- Başlık metni kaldırıldı -->
            <div id="map-container">
                <img id="map" src="" alt="Map">
            </div>
            <script>
                var socket = io();
                var mapImage = document.getElementById('map');

                socket.on('map_data', function(data) {
                    mapImage.src = 'data:image/png;base64,' + data.data;
                });
            </script>
        </body>
        </html>
    ''')

@app.route('/')
def get_map():
    if ros_node and ros_node.last_map_message:
        image = ros_node.create_map_image()
        img_io = io.BytesIO()
        image.save(img_io, 'PNG')
        img_io.seek(0)
        return send_file(img_io, mimetype='image/png')
    else:
        return "Map not available", 404

def run_flask():
    socketio.run(app, debug=True, use_reloader=False, port=5000)

def main(args=None):
    global ros_node
    rclpy.init(args=args)
    ros_node = ROS2Subscriber()
    
    flask_thread = threading.Thread(target=run_flask)
    flask_thread.start()

    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
