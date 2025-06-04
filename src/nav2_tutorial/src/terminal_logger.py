#!/usr/bin/env python3
import os
import sys
import yaml
import select
import termios
import tty
from datetime import datetime

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

from ament_index_python.packages import get_package_share_directory

from src.utils.gps_utils import euler_from_quaternion


class GpsKeyLogger(Node):
    """
    ROS2 node to log GPS waypoints to a YAML file on keypress.
    Press 'f' to log, 'q' to quit.
    """

    def __init__(self, logging_file_path):
        super().__init__('gps_waypoint_keylogger')

        self.logging_file_path = logging_file_path
        self.last_gps = None
        self.last_yaw = 0.0

        self.create_subscription(
            NavSatFix,
            '/fixposition/odometry_llh',
            self.gps_callback,
            1
        )
        self.create_subscription(
            Odometry,
            '/fixposition/odometry_enu',
            self.yaw_callback,
            1
        )

        self.get_logger().info(
            f"Logging to '{self.logging_file_path}'.\n"
            "Press 'f' to log a waypoint; 'q' to quit."
        )

    def gps_callback(self, msg: NavSatFix):
        self.last_gps = msg

    def yaw_callback(self, msg: Odometry):
        # Extract yaw from quaternion
        _, _, self.last_yaw = euler_from_quaternion(msg.pose.pose.orientation)

    def log_waypoint(self):
        if self.last_gps is None:
            self.get_logger().warn("No GPS fix yet; skipping log.")
            return

        # Load existing file or generate a new one
        try:
            with open(self.logging_file_path, 'r') as f:
                data = yaml.safe_load(f) or {}
        except FileNotFoundError:
            data = {}
        except Exception as e:
            self.get_logger().error(f"Failed to read log file: {e}")
            return

        wps = data.get('waypoints', [])

        wp = {
            'latitude':  self.last_gps.latitude,
            'longitude': self.last_gps.longitude,
            'yaw':       self.last_yaw
        }
        wps.append(wp)
        data['waypoints'] = wps

        try:
            with open(self.logging_file_path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
        except Exception as e:
            self.get_logger().error(f"Failed to write log file: {e}")
            return

        self.get_logger().info(
            f"Logged waypoint #{len(wps)}: "
            f"lat={wp['latitude']:.6f}, lon={wp['longitude']:.6f}, yaw={wp['yaw']:.2f}"
        )


def stdin_ready():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


def main(argv=None):
    rclpy.init(args=argv)

    # If the user passed an explicit path, use it
    if len(sys.argv) > 1:
        yaml_path = sys.argv[1]

    else:
        # Find the *install* share directory for nav2_tutorial
        share_dir = get_package_share_directory('nav2_tutorial')
        
        # Climb up: share/nav2_tutorial -> share -> install -> workspace root
        ws_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(share_dir))))
        
        # Build the path to your src/config
        src_config = os.path.join(ws_root, 'src', 'nav2_tutorial', 'trajectories')

        if os.path.isdir(src_config):
            traj_dir = src_config
        else:
            # Fallback to the installed share/config
            traj_dir = os.path.join(share_dir, 'trajectories')

        os.makedirs(traj_dir, exist_ok=True)

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        yaml_path = os.path.join(traj_dir, f'gps_waypoints_{stamp}.yaml')

    node = GpsKeyLogger(yaml_path)

    # Set terminal into raw mode
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if stdin_ready():
                c = sys.stdin.read(1)
                if c == 'f':
                    node.log_waypoint()
                elif c == 'q':
                    node.get_logger().info("Quit key pressed, shutting down.")
                    break
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user, shutting down.")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
