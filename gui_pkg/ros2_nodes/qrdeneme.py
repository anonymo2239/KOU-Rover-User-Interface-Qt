#usb device qr code reader ubuntu ros2 publisher
#/dev/input/event21 Newland Auto-ID FM430 usb-0000:00:14.0-9/input0

import selectors
import evdev
import rclpy
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node('qr_publisher')
publisher = node.create_publisher(String, 'qr_code', 10)

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('qr_publisher')
    publisher = node.create_publisher(String, 'qr_code', 10)

    msg = String()
    msg.data = qr_code

    rclpy.spin_once(node)
    publisher.publish(msg)

    node.destroy_node()
    rclpy.shutdown()

selector = selectors.DefaultSelector()

# Change the device path to your specific device
device = evdev.InputDevice('/dev/input/event21')

# Grab the device to prevent it from acting as a keyboard input
device.grab()

selector.register(device, selectors.EVENT_READ)

#\000026Q9;9625.0;0.0

# Map of evdev key names to characters
key_map = {
    'KEY_1': '1', 'KEY_2': '2', 'KEY_3': '3', 'KEY_4': '4', 'KEY_5': '5',
    'KEY_6': '6', 'KEY_7': '7', 'KEY_8': '8', 'KEY_9': '9', 'KEY_0': '0',
    'KEY_A': 'A', 'KEY_B': 'B', 'KEY_C': 'C', 'KEY_D': 'D', 'KEY_E': 'E',
    'KEY_F': 'F', 'KEY_G': 'G', 'KEY_H': 'H', 'KEY_I': 'I', 'KEY_J': 'J',
    'KEY_K': 'K', 'KEY_L': 'L', 'KEY_M': 'M', 'KEY_N': 'N', 'KEY_O': 'O',
    'KEY_P': 'P', 'KEY_Q': 'Q', 'KEY_R': 'R', 'KEY_S': 'S', 'KEY_T': 'T',
    'KEY_U': 'U', 'KEY_V': 'V', 'KEY_W': 'W', 'KEY_X': 'X', 'KEY_Y': 'Y',
    'KEY_Z': 'Z', 'KEY_MINUS': '-', 'KEY_EQUAL': '=', 'KEY_LEFTBRACE': '[',
    'KEY_RIGHTBRACE': ']', 'KEY_BACKSLASH': '', 'KEY_SEMICOLON': ';',
    'KEY_APOSTROPHE': '-', 'KEY_GRAVE': '`', 'KEY_COMMA': ',', 'KEY_DOT': 'ü',
    'KEY_SLASH': ',', 'KEY_SPACE': ' ', 'KEY_LEFTSHIFT': ';', 
}

qr_code = ''

while True:
    for key, _ in selector.select():
        device = key.fileobj
        for event in device.read():
            if event.type == evdev.ecodes.EV_KEY:
                data = evdev.categorize(event)
                if data.keystate == 1:  # key down
                    print(data.keycode)
                    key_name = data.keycode[0] if isinstance(data.keycode, list) else data.keycode
                    if key_name in key_map:
                        qr_code += key_map[key_name]
                    elif key_name == 'KEY_ENTER':
                        start_index = qr_code.find('Q')
                        if start_index != -1:
                            qr_code = qr_code[start_index:]
                        
                        print('QR Code: ', qr_code)
                        msg = String()
                        msg.data = qr_code
                        publisher.publish(msg)
                        qr_code = ''  # Reset the QR code

#publish qtr code