import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import json

OUTPUT_PATH = '/tmp/current_position.json'

class PositionLogger(Node):
    def __init__(self):
        super().__init__('position_logger')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/fixposition/odometry_llh',
            self.listener_callback,
            10)
    def listener_callback(self, msg):
        pos = {'lat': msg.latitude, 'lon': msg.longitude}
        with open(OUTPUT_PATH, 'w') as f:
            json.dump(pos, f)

def main():
    rclpy.init()
    node = PositionLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
