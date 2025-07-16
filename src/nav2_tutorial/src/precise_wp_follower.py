import sys
import argparse
from uuid import UUID

# ROS packages
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# ROS messages
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker, MarkerArray

# Custom utils
from src.utils.nav_utils import _pose_dist, _path_length, _closest_wp_index
from src.utils.parser_utils import YamlWaypointParser, _select_yaml

# Visualization
MARKER_VIZ = True


class GpsWpCommander:
    """Follow a precise sequence of GPS waypoints."""

    def __init__(self, wps_file_path: str, reverse: bool = False, num_loop: int = 1) -> None:
        self.navigator = BasicNavigator("basic_navigator")
        self.reverse   = reverse
        self.num_loop  = max(1, num_loop)

        # Odometry topic with ENU pose in map frame
        self._last_odom_ns: int | None = None
        self._last_odom_pose: PoseStamped | None = None
        self.navigator.create_subscription(Odometry, "/fixposition/odometry_enu", self._odom_cb, 10)
        
        # Initialize waypoint parser
        base_lat, base_lon, base_alt = self._get_datum()
        self.wp_parser  = YamlWaypointParser(wps_file_path, base_lat, base_lon, base_alt)

        # Parameters for the GoToPose task
        self.odom_timeout_s = 2.0  # [s] wait for odom on start‑up
        self.max_retries    = 1    # per‑waypoint retries
        self.start_wp_tol   = 3.0  # [m] skip wp if already this close
        
        # Statistics
        self._total_wps   = 0
        self._visited_wps = 0
        
        # Marker visualization
        if MARKER_VIZ:
            latched_qos = QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

            # Path and MarkerArray for visualizing the full trajectory
            self.full_traj_pub = self.navigator.create_publisher(
                Path, "/gps_full_traj", latched_qos)
            self.full_traj_mrk_pub = self.navigator.create_publisher(
                MarkerArray, "/gps_full_traj_markers", latched_qos)
            
    def _get_datum(self, topic: str = "/fixposition/datum",
               timeout_s: float = 3.0) -> tuple[float, float, float]:
        """
        Block until a NavSatFix is received on topic (max timeout_s).
        Returns (lat [deg], lon [deg], alt [m]). Raises RuntimeError on timeout.
        """
        datum = None
        def _cb(msg: NavSatFix):
            nonlocal datum
            datum = (msg.latitude, msg.longitude, msg.altitude)

        sub = self.navigator.create_subscription(NavSatFix, topic, _cb, qos_profile_sensor_data)

        deadline = self.navigator.get_clock().now() + rclpy.duration.Duration(seconds=timeout_s)

        while rclpy.ok() and datum is None:
            rclpy.spin_once(self.navigator, timeout_sec=0.05)
            if self.navigator.get_clock().now() > deadline:
                raise RuntimeError(f"No NavSatFix on {topic} within {timeout_s}s")

        self.navigator.destroy_subscription(sub)
        return datum

    def start_ntp(self) -> None:
        """Start the GoToPose task with the waypoints."""
        # Wait for the robot_localization node to be active
        self.navigator.waitUntilNav2Active(localizer="robot_localization")

        # Convert the whole YAML path once
        all_poses = self.wp_parser.get_wps(reverse=self.reverse)
        
        # Publish the full trajectory for visualization
        self._publish_full_traj(all_poses)

        # Loop through the waypoints
        for loop_idx in range(self.num_loop):
            if self.num_loop > 1:
                self.navigator.get_logger().info(
                    f"Performing loop {loop_idx + 1} out of {self.num_loop}…")
            
            self._total_wps = len(all_poses)
            total_len       = _path_length(all_poses)
            self.navigator.get_logger().info(
                f"Preparing segments for trajectory of {total_len:0.2f} m "
                f"consisting of {self._total_wps} waypoints…")
            remaining = all_poses

            # Skip early waypoints already within tolerance
            robot = self._get_robot_pose()

            if self.num_loop == 1:
                # Fast-forward to the first pose within start_wp_tol
                first_inside = None
                for i, pose in enumerate(remaining):
                    if _pose_dist(robot, pose) <= self.start_wp_tol:
                        first_inside = i
                        break

                # Fall back to the very beginning if none are close enough
                closest_idx = first_inside if first_inside is not None else 0

                if closest_idx:
                    self._visited_wps += closest_idx   # statistics
                    skipped = closest_idx
                    remaining = remaining[closest_idx:]
                    self.navigator.get_logger().info(
                        f"Skipped {skipped} waypoint(s); "
                        f"starting with wp #{skipped+1}/{self._total_wps}")

            # Navigate to each remaining waypoint
            for local_idx, pose in enumerate(remaining, start=1):
                wp_global_idx = self._visited_wps + 1
                
                retries_left = self.max_retries
                while True:
                    self.navigator.goToPose(pose)

                    # Wait until action finishes (spinning keeps callbacks alive)
                    while rclpy.ok() and not self.navigator.isTaskComplete():
                        rclpy.spin_once(self.navigator, timeout_sec=0.05)

                    result = self.navigator.getResult()

                    if result == TaskResult.SUCCEEDED:
                        self.navigator.get_logger().info(
                            f"Waypoint {wp_global_idx}/{self._total_wps} reached")
                        break

                    if retries_left > 0:
                        self.navigator.get_logger().warn(
                            f"Waypoint {wp_global_idx} "
                            f"{result.name.lower()} - retrying ({retries_left} left)")
                        retries_left -= 1
                        continue

                    # Exceeded retries → abort entire mission
                    self.navigator.get_logger().error(
                        f"Waypoint {wp_global_idx} failed after retry - aborting mission")
                    raise RuntimeError("Navigation failed")
                
                self._visited_wps += 1

        self.navigator.get_logger().info("All waypoints completed - mission finished")
        
    def _odom_cb(self, msg: Odometry) -> None:
        """Cache the latest odometry pose, skipping out-of-order messages."""
        stamp = msg.header.stamp
        curr_ns = stamp.sec * 1_000_000_000 + stamp.nanosec

        # Reject if header time went backwards
        if self._last_odom_ns is not None and curr_ns < self._last_odom_ns:
            self.navigator.get_logger().warn(
                "Skipping out-of-order odometry message (time jumped backwards)" )
            return

        # Accept the pose
        self._last_odom_ns = curr_ns
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose   = msg.pose.pose
        self._last_odom_pose = ps

    def _get_robot_pose(self) -> PoseStamped:
        """
        Return the most recent pose from /fixposition/odometry_enu.
        Blocks (spins) until a message arrives or the timeout elapses.
        """
        deadline = (self.navigator.get_clock().now() + 
                    rclpy.duration.Duration(seconds=self.odom_timeout_s))

        while self._last_odom_pose is None:
            if self.navigator.get_clock().now() > deadline:
                raise RuntimeError(
                    f"No Odometry on /fixposition/odometry_enu "
                    f"after {self.odom_timeout_s:.1f}s")
            rclpy.spin_once(self.navigator, timeout_sec=0.05)

        return self._last_odom_pose

    def _publish_full_traj(self, poses: list[PoseStamped]) -> None:
        """Publish the whole trajectory as a latched Path + MarkerArray."""
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp    = rclpy.time.Time().to_msg()
        path.poses           = poses
        self.full_traj_pub.publish(path)

        # one violet line-strip + tiny white spheres at every waypoint
        line = Marker()
        line.header = path.header
        line.ns     = "full_traj"
        line.id     = 0
        line.type   = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.04
        line.color.r, line.color.g, line.color.b, line.color.a = 0.7, 0.3, 1.0, 1.0
        line.points = [p.pose.position for p in poses]

        dots = MarkerArray()
        for i, p in enumerate(poses):
            m = Marker()
            m.header = path.header
            m.ns, m.id = "full_traj_pts", i
            m.type, m.action = Marker.SPHERE, Marker.ADD
            m.pose = p.pose
            m.scale.x = m.scale.y = m.scale.z = 0.05
            m.color.r = m.color.g = m.color.b = 1.0
            m.color.a = 1.0
            dots.markers.append(m)

        self.full_traj_mrk_pub.publish(MarkerArray(
            markers=[line] + dots.markers))


def main(argv: list[str] | None = None):
    """Main entry point for the GPS waypoint follower."""
    rclpy.init(args=argv)

    parser = argparse.ArgumentParser(description="Follow GPS waypoints from a YAML file")
    parser.add_argument("yaml_file", nargs="?", default=None, help="Path to the YAML waypoints file")
    parser.add_argument("--last", action="store_true", help="Load the most recent waypoints file in the config directory")
    parser.add_argument("--reverse", action="store_true", help="Follow the trajectory in reverse order (adds 180° to yaw)")
    parser.add_argument("--loop", type=int, default=1, help="Number of times to loop through the waypoints (default: 1)")
    args = parser.parse_args()

    try:
        yaml_file_path = _select_yaml(args)
    except FileNotFoundError as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        sys.exit(1)

    # Feedback to the user
    print(f"Loading waypoints file: {yaml_file_path}")
    try:
        gps_wpf = GpsWpCommander(yaml_file_path, reverse=args.reverse, num_loop=args.loop)
        gps_wpf.start_ntp()
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        sys.exit(1)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
