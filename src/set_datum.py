import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from robot_localization.srv import SetDatum

class DatumPublisher(Node):
    def __init__(self):
        super().__init__('datum_publisher')
        # Subscribe to the NavSatFix message that contains the datum LLH coordinates.
        self.subscription = self.create_subscription(
            NavSatFix,
            '/fixposition/datum',  # Change this if your topic is named differently.
            self.navsatfix_callback,
            10
        )
        # Create a service client for the /datum service.
        self.client = self.create_client(SetDatum, '/datum')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /datum service...')
        self.has_called_service = False

    def navsatfix_callback(self, msg: NavSatFix):
        # Only call the service once.
        if self.has_called_service:
            return

        self.has_called_service = True
        self.get_logger().info(f"Received NavSatFix: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, alt={msg.altitude:.2f}")

        # Prepare the SetDatum service request with the LLH from the NavSatFix message.
        req = SetDatum.Request()
        req.geo_pose.position.latitude = msg.latitude
        req.geo_pose.position.longitude = msg.longitude
        req.geo_pose.position.altitude = msg.altitude
        req.geo_pose.orientation.x = 0.0
        req.geo_pose.orientation.y = 0.0
        req.geo_pose.orientation.z = 0.0
        req.geo_pose.orientation.w = 1.0

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is not None:
            self.get_logger().info("SetDatum service call succeeded")
        else:
            self.get_logger().error("SetDatum service call failed")
        # Optionally, you could shutdown the node here if no further work is needed.
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DatumPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
