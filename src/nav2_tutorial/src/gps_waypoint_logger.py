import os
import sys
import yaml

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

import tkinter as tk
from tkinter import messagebox

from src.utils.gps_utils import euler_from_quaternion


class GpsGuiLogger(tk.Tk, Node):
    """
    ROS2 node to log GPS waypoints to a file
    """

    def __init__(self, logging_file_path):
        tk.Tk.__init__(self)
        Node.__init__(self, 'gps_waypoint_logger')
        self.title("GPS Waypoint Logger")

        self.logging_file_path = logging_file_path

        self.gps_pose_label = tk.Label(self, text="Current Coordinates:")
        self.gps_pose_label.pack()
        self.gps_pose_textbox = tk.Label(self, text="", width=45)
        self.gps_pose_textbox.pack()

        self.log_gps_wp_button = tk.Button(self, text="Log GPS Waypoint",
                                           command=self.log_waypoint)
        self.log_gps_wp_button.pack()

        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/fixposition/odometry_llh',
            self.gps_callback,
            1
        )
        self.last_gps_position = NavSatFix()

        self.yaw_subscription = self.create_subscription(
            Odometry,
            '/fixposition/odometry_enu',
            self.yaw_callback,
            1
        )
        self.last_heading = 0.0

    def gps_callback(self, msg: NavSatFix):
        """
        Callback to store the last GPS pose
        """
        self.last_gps_position = msg
        self.updateTextBox()

    def yaw_callback(self, msg: Odometry):
        """
        Callback to store the last heading
        """
        self.last_heading = euler_from_quaternion(msg.pose.pose.orientation)[2]
        self.updateTextBox()

    def updateTextBox(self):
        """
        Function to update the GUI with the last coordinates
        """
        self.gps_pose_textbox.config(
            text=f"Lat: {self.last_gps_position.latitude:.6f}, Lon: {self.last_gps_position.longitude:.6f}, yaw: {self.last_heading:.2f} rad")

    def log_waypoint(self):
        """
        Function to save a new waypoint to a file
        """
        # Read existing waypoints
        try:
            with open(self.logging_file_path, 'r') as yaml_file:
                existing_data = yaml.safe_load(yaml_file)
        # If the file does not exist, create with the new wps
        except FileNotFoundError:
            existing_data = {"waypoints": []}
        # If other exception, raise the warning
        except Exception as ex:
            messagebox.showerror(
                "Error", f"Error logging position: {str(ex)}")
            return

        # Build new waypoint object
        data = {
            "latitude": self.last_gps_position.latitude,
            "longitude": self.last_gps_position.longitude,
            "yaw": self.last_heading
        }
        existing_data["waypoints"].append(data)

        # Write updated waypoints
        try:
            with open(self.logging_file_path, 'w') as yaml_file:
                yaml.dump(existing_data, yaml_file, default_flow_style=False)
        except Exception as ex:
            messagebox.showerror(
                "Error", f"Error logging position: {str(ex)}")
            return

        messagebox.showinfo("Info", "Waypoint logged succesfully")


def main(args=None):
    rclpy.init(args=args)

    # Allow to pass the logging path as an argument
    default_yaml_file_path = os.path.expanduser("~/gps_waypoints.yaml")
    if len(sys.argv) > 1:
        yaml_file_path = sys.argv[1]
    else:
        yaml_file_path = default_yaml_file_path

    gps_gui_logger = GpsGuiLogger(yaml_file_path)

    # Spin ROS system and Tk interface
    while rclpy.ok():
        # Run ROS2 callbacks
        rclpy.spin_once(gps_gui_logger, timeout_sec=0.1)
        # Update the Tkinter interface
        gps_gui_logger.update()
    
    # Clean up
    rclpy.shutdown()


if __name__ == '__main__':
    main()
