import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PointStamped
from src.utils.gps_utils import latLonYaw2Geopose


class InteractiveGpsWpCommander(Node):
    """
    ROS2 node to send gps waypoints to nav2 received from mapviz's point click publisher
    """

    def __init__(self):
        super().__init__(node_name="gps_wp_commander")
        self.navigator = BasicNavigator("basic_navigator")

        self.mapviz_wp_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.mapviz_wp_cb, 1)

    def mapviz_wp_cb(self, msg: PointStamped):
        """
        clicked point callback, sends received point to nav2 gps waypoint follower if its a geographic point
        """
        if msg.header.frame_id != "wgs84":
            self.get_logger().warning(
                "Received point from mapviz that is not in the WGS84 frame. This is not a gps point and won't be followed")
            return

        self.navigator.waitUntilNav2Active(localizer='robot_localization')
        wp = [latLonYaw2Geopose(msg.point.y, msg.point.x)]
        self.navigator.followGpsWaypoints(wp)
        if (self.navigator.isTaskComplete()):
            self.get_logger().info("wps completed successfully")


def main():
    rclpy.init()
    gps_wpf = InteractiveGpsWpCommander()
    rclpy.spin(gps_wpf)


if __name__ == "__main__":
    main()



# import rclpy
# from rclpy.node import Node
# from nav2_simple_commander.robot_navigator import BasicNavigator
# from geometry_msgs.msg import PointStamped
# from .utils.gps_utils import latLonYaw2Geopose
# from geometry_msgs.msg import PoseStamped
# from robot_localization.srv import FromLL


# class InteractiveGpsWpCommander(Node):
#     """
#     ROS2 node to send gps waypoints to nav2 received from mapviz's point click publisher
#     """

#     def __init__(self):
#         super().__init__(node_name="gps_wp_commander")
#         self.navigator = BasicNavigator("basic_navigator")

#         self.mapviz_wp_sub = self.create_subscription(
#             PointStamped, "/clicked_point", self.mapviz_wp_cb, 1)
        
#         self.localizer = self.create_client(FromLL, '/fromLL')
#         while not self.localizer.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('Service not available, waiting again...')
#         self.client_futures = []

#         self.get_logger().info('Ready for waypoints...')

#     def mapviz_wp_cb(self, msg: PointStamped):
#         """
#         clicked point callback, sends received point to nav2 gps waypoint follower if its a geographic point
#         """
#         if msg.header.frame_id != "wgs84":
#             self.get_logger().warning(
#                 "Received point from mapviz that is not in the wgs84 frame. This is not a gps point and won't be followed")
#             return
    
#         wps = [latLonYaw2Geopose(msg.point.y, msg.point.x)]

#         for wp in wps:
#             self.req = FromLL.Request()
#             self.req.ll_point.longitude = wp.position.longitude
#             self.req.ll_point.latitude = wp.position.latitude
#             self.req.ll_point.altitude = wp.position.altitude

#             self.get_logger().info("Waypoint added to conversion queue...")
#             self.client_futures.append(self.localizer.call_async(self.req))

#     def command_send_cb(self, future):        
#         self.resp = PoseStamped()
#         self.resp.header.frame_id = 'map'
#         self.resp.header.stamp = self.get_clock().now().to_msg()
#         self.resp.pose.position = future.result().map_point
#         #self.navigator.goToPose(self.resp)
        
#         goal_pose = PoseStamped()
#         goal_pose.header.frame_id = 'map'
#         goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
#         goal_pose.pose.position.x = 2.0
#         goal_pose.pose.position.y = -1.0
        
#         # At a minimum, set a valid orientation (e.g. facing forward)
#         goal_pose.pose.orientation.w = 1.0
        
#         # Send the goal
#         self.navigator.goToPose(goal_pose)

#         # (Optional) Spin while the robot navigates to the goal
#         while not self.navigator.isTaskComplete():
#             feedback = self.navigator.getFeedback()
#             if feedback:
#                 print(feedback)
#                 pass

#         # Check the final result
#         result = self.navigator.getResult()
#         print(result)
#         # if result == NavigationResult.SUCCEEDED:
#         #     print("Goal succeeded!")
#         # else:
#         #     print("Goal failed or was canceled.")

#     def spin(self):
#         while rclpy.ok():
#             rclpy.spin_once(self)
#             incomplete_futures = []
#             for f in self.client_futures:
#                 if f.done():
#                     self.get_logger().info("Following converted waypoint...")
#                     self.command_send_cb(f)
#                 else:
#                     incomplete_futures.append(f)
                    
#             self.client_futures = incomplete_futures

# def main():
#     rclpy.init()
#     gps_wpf = InteractiveGpsWpCommander()
#     gps_wpf.spin()


# if __name__ == "__main__":
#     main()
