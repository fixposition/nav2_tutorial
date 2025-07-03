#!/usr/bin/env python3
import sys
import rclpy
from geometry_msgs.msg import PointStamped

def main():
    if len(sys.argv) < 3:
        print("Usage: send_clicked_point.py LAT LON")
        sys.exit(1)
    lat = float(sys.argv[1])
    lon = float(sys.argv[2])

    rclpy.init()
    node = rclpy.create_node('clicked_point_sender')
    pub = node.create_publisher(PointStamped, '/clicked_point', 1)
    msg = PointStamped()
    msg.header.frame_id = 'wgs84'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.point.x = lon
    msg.point.y = lat
    msg.point.z = 0.0
    for _ in range(3):  # Publish a few times
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
    print(f"Published clicked point: lat={lat} lon={lon}")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
