import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np, cv2
rclpy.init()
node = Node('frame_grabber')
done = []
def cb(msg):
    if done: return
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    if msg.encoding == 'rgb8':
        arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    cv2.imwrite('/tmp/frame.jpg', arr)
    print(f'Saved /tmp/frame.jpg  ({msg.width}x{msg.height}, encoding={msg.encoding})')
    done.append(True)
from rclpy.qos import qos_profile_sensor_data
node.create_subscription(Image, '/camera/image_raw', cb, qos_profile_sensor_data)
while not done:
    rclpy.spin_once(node, timeout_sec=2.0)
node.destroy_node(); rclpy.shutdown()