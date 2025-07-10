import sys
import argparse
from uuid import UUID

# ROS packages
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# ROS messages
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Path, Odometry
from nav2_msgs.action import FollowPath
from visualization_msgs.msg import Marker, MarkerArray

# Custom utils
from src.utils.nav_utils import (_pose_dist, _path_length, _segment_length, _closest_wp_index, 
                                 _make_window, _hsv_to_rgb, _get_goal_handle, _get_current_uuid)
from src.utils.parser_utils import YamlWaypointParser, _latest_file, _resolve_ws_paths, _select_yaml

# Visualization
MARKER_VIZ = True


class GpsWpCommander:
    """Follow a smooth sequence of GPS waypoints."""

    def __init__(self, wps_file_path: str, reverse: bool = False, num_loop: int = 1, stopping: bool = False):
        self.navigator  = BasicNavigator("basic_navigator")
        self.reverse    = reverse
        self.num_loop   = num_loop
        self.stopping  = stopping

        # Odometry topic with ENU pose in map frame
        self._last_odom_pose: PoseStamped | None = None
        self.navigator.create_subscription(Odometry, "/fixposition/odometry_enu", self._odom_cb, 10)
        
        # Initialize waypoint parser
        base_lat, base_lon, base_alt = self._get_datum()
        self.wp_parser  = YamlWaypointParser(wps_file_path, base_lat, base_lon, base_alt)

        # Parameters for the GoThroughPoses task
        self.odom_timeout_s = 2.0  # wait this long for first /fixposition msg
        self.max_retries    = 1    # re-attempts for each failed segment
        self._retries_left  = self.max_retries
        self._fp_client     = None
        self._last_goal_uuid: UUID | None = None
        
        # Sliding-window parameters
        # For long mission: seg_len_max = 9.0, advance_tol = 3.60, overlap_tol = 1.30
        # For several loops: seg_len_max = 20.0, advance_tol = 4.60, overlap_tol = 3.00
        self.seg_len_max      = 6.0   # m - length of window Nav2 sees
        self.advance_tol      = 2.6   # m - when to roll the window forward
        self.overlap_tol      = 1.0   # m - keep poses this close as overlap
        self.start_seg_tol    = 3.0   # m - max distance we “snap” to the path
        
        # Statistics
        self._total_wps   = 0
        self._visited_wps = 0
        
        # Optional: Marker visualization
        if MARKER_VIZ:
            latched_qos = QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

            # Path and MarkerArray for visualizing the current segment
            self.waypoint_path_pub = self.navigator.create_publisher(
                    Path, "/gps_waypoints_path", latched_qos)
            self.waypoint_marker_pub = self.navigator.create_publisher(
                    MarkerArray, "/gps_waypoints_markers", latched_qos)
        
            # Path and MarkerArray for visualizing the full trajectory
            self.full_traj_pub   = self.navigator.create_publisher(
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

    def start_ntp(self):
        """Start the GoThroughPoses task with the waypoints."""
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
            seg_idx   = 0

            # Prime the first window
            robot = self._get_robot_pose()

            if self.num_loop == 1:
                # Fast-forward to the first pose within start_seg_tol
                closest_idx = _closest_wp_index(robot, remaining)
                if _pose_dist(robot, remaining[closest_idx]) > self.start_seg_tol:
                    closest_idx = 0

                if closest_idx:
                    self._visited_wps += closest_idx
                    skipped = closest_idx
                    remaining = remaining[closest_idx:]
                    self.navigator.get_logger().info(
                        f"Skipped {skipped} waypoint(s); "
                        f"starting with wp #{skipped+1}/{self._total_wps}")

            # If everything was within the tolerance, finish trajectory follower
            if not remaining:
                self.navigator.get_logger().info("Trajectory already completed.")
                return

            window  = _make_window(robot, remaining, self.seg_len_max)
            if len(window) == 1:          # no waypoint fit inside seg_len_max
                window.append(remaining[0])
                new_wps = 1
            else:
                new_wps = len(window) - 1
            
            self._visited_wps += new_wps
            self._log_segment(window, overlap_cnt=0, seg_idx=seg_idx)
            self._publish_waypoints(window, seg_idx)
            self.navigator.goThroughPoses(window)
            
            remaining = remaining[new_wps:]
            seg_idx += 1

            # Continuous streaming loop
            while rclpy.ok():
                rclpy.spin_once(self.navigator, timeout_sec=0.05)

                if self.navigator.isTaskComplete():
                    status = self.navigator.getResult()

                    goal_handle = _get_goal_handle(self.navigator)
                    current_id  = _get_current_uuid(self.navigator)

                    # If we can’t identify the result reliably, treat it as current
                    if goal_handle is None or current_id is None:
                        result_is_current = True
                    else:
                        result_id = UUID(bytes=goal_handle.goal_id.uuid)
                        result_is_current = (result_id == UUID(bytes=current_id))

                    # Stale result from the goal we just pre-empted
                    if not result_is_current:
                        continue

                    # Status handling
                    robot = self._get_robot_pose()

                    # 1) SUCCEEDED: accept only if really at tail and nothing remains in the global list
                    if status == TaskResult.SUCCEEDED:
                        if (_pose_dist(robot, window[-1]) <= self.advance_tol) and not remaining:
                            self.navigator.get_logger().info("Trajectory finished")
                            break
                        else:
                            # Success was premature: resend the unreached tail
                            unreached = [p for p in window
                                        if _pose_dist(robot, p) > self.advance_tol]
                            if not unreached:
                                # corner-case: we are at tail but still have poses in ‘remaining’. 
                                # Continue; the normal sliding-window logic will pick them up.
                                continue
                            retry_window = [robot] + unreached
                            self.navigator.get_logger().warn(
                                f"Premature SUCCEEDED - retrying {len(unreached)} "
                                f"poses of current segment")
                            self._publish_waypoints(retry_window, seg_idx)
                            self.navigator.goThroughPoses(retry_window)
                            window = retry_window
                            continue

                    # 2) FAILED or CANCELLED: Retry the segment
                    if status in (TaskResult.FAILED, TaskResult.CANCELED):
                        if self._retries_left > 0:
                            self._retries_left -= 1
                            unreached = [p for p in window
                                        if _pose_dist(robot, p) > self.advance_tol]
                            if not unreached:
                                self.navigator.get_logger().error(
                                    "Retry requested but no poses remain in window")
                                break
                            retry_window = [robot] + unreached
                            self.navigator.get_logger().warn(
                                f"Segment {seg_idx+1} {status.name.lower()} - "
                                f"retrying {len(unreached)} poses")
                            self._publish_waypoints(retry_window, seg_idx)
                            self.navigator.goThroughPoses(retry_window)
                            window = retry_window
                            continue
                        else:
                            self.navigator.get_logger().error(
                                "Segment failed twice - aborting")
                            break

                # Live pose
                robot = self._get_robot_pose()

                # Assess whether we are close to the window’s tail
                if _pose_dist(robot, window[-1]) > self.advance_tol:
                    continue
                
                # Slide the window forward
                tail = window[-1]

                # Overlap: walk backwards inside the old window
                dist_rt = _pose_dist(robot, tail)

                overlap = []
                for p in reversed(window):
                    if (_pose_dist(p, tail) <= self.overlap_tol and      # close to tail
                        _pose_dist(robot, p) <= dist_rt + 1e-6):         # not behind robot
                        overlap.append(p)
                    else:
                        break
                overlap.reverse()

                if not remaining:
                    continue   # path finished; let Nav2 coast to the end

                # Build the forward part of the new window
                forward  = _make_window(tail, remaining, self.seg_len_max)[1:]  # skip duplicate ‘tail’
                new_window = overlap + forward

                # Drop from ‘remaining’ exactly the poses we just attached
                remaining = remaining[len(forward):]
                
                # Log stats before cancel / send goal
                self._visited_wps += len(forward)
                self._log_segment(new_window, overlap_cnt=len(overlap), seg_idx=seg_idx)

                # Optional: explicit cancellation before sending new window
                if self.stopping:
                    self.navigator.cancelTask()
                    cancel_deadline = self.navigator.get_clock().now() + rclpy.duration.Duration(seconds=3.0)
                    while not self.navigator.isTaskComplete():
                        if self.navigator.get_clock().now() > cancel_deadline:
                            self.navigator.get_logger().error("Cancel timeout - aborting")
                            return
                        rclpy.spin_once(self.navigator, timeout_sec=0.05)
                    # extra guard: ensure internal follow_path is cancelled
                    if self._fp_client is None:
                        self._fp_client = ActionClient(self.navigator, FollowPath, "follow_path")
                        if not self._fp_client.wait_for_server(timeout_sec=2.0):
                            self.navigator.get_logger().warn("follow_path server not available")
                    fp_deadline = self.navigator.get_clock().now() + rclpy.duration.Duration(seconds=3.0)
                    while True:
                        goal_handles = self._fp_client._goal_handles
                        if not goal_handles:
                            break
                        status = goal_handles[0].status
                        if status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED):
                            break
                        if self.navigator.get_clock().now() > fp_deadline:
                            self.navigator.get_logger().error("follow_path cancel timeout - aborting")
                            return
                        rclpy.spin_once(self.navigator, timeout_sec=0.05)
                
                # Send next window
                self._publish_waypoints(new_window, seg_idx)
                self.navigator.goThroughPoses(new_window)
                self._retries_left = self.max_retries
                seg_idx += 1
                window = new_window
        
    def _odom_cb(self, msg: Odometry) -> None:
        """Cache the most recent odometry pose as a `PoseStamped`."""
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose   = msg.pose.pose
        self._last_odom_pose = ps

    def _get_robot_pose(self) -> PoseStamped:
        """
        Return the most recent pose from /fixposition/odometry_enu.
        Blocks (spins) until a message arrives or the timeout elapses.
        """
        deadline = self.navigator.get_clock().now() + rclpy.duration.Duration(
            seconds=self.odom_timeout_s)

        while self._last_odom_pose is None:
            if self.navigator.get_clock().now() > deadline:
                raise RuntimeError(
                    f"No Odometry on /fixposition/odometry_enu "
                    f"after {self.odom_timeout_s:.1f}s")
            rclpy.spin_once(self.navigator, timeout_sec=0.05)

        return self._last_odom_pose

    def _log_segment(self, window,
                     *, overlap_cnt: int,
                     seg_idx: int) -> None:
        """Log the segment statistics."""
        seg_len  = _segment_length(window)
        num_wps  = len(window)
        progress = self._visited_wps

        self.navigator.get_logger().info(
            f"Segment {seg_idx+1} length: {seg_len:0.2f} m, "
            f"Num. waypoints: {num_wps}, "
            f"Overlap: {overlap_cnt}, "
            f"Progress: {progress}/{self._total_wps}")
    
    def _publish_waypoints(self, poses: list[PoseStamped], seg_idx: int):
        """Publish the current segment as a Path and MarkerArray."""
        # Path message
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp    = rclpy.time.Time().to_msg()
        path.poses           = poses
        
        # RViz colours Paths per-topic, not per-message → we publish a Marker
        # LINE_STRIP instead (same header) so each segment gets its own hue.
        line = Marker()
        line.header = path.header
        line.ns     = "seg_line"
        line.id     = seg_idx          # unique id per segment
        line.type   = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0  # identity
        line.scale.x = 0.03            # line width
        hue = (seg_idx * 0.18) % 1.0   # golden-ratio hop around the circle
        r, g, b = _hsv_to_rgb(hue, 1.0, 1.0)
        line.color.r, line.color.g, line.color.b, line.color.a = r, g, b, 1.0
        
        for p in poses:
            line.points.append(p.pose.position)
        self.waypoint_path_pub.publish(path)
        self.waypoint_marker_pub.publish(MarkerArray(markers=[line]))

        # Optional coloured spheres
        markers = MarkerArray()
        for i, p in enumerate(poses):
            m = Marker()
            m.header   = path.header
            m.ns       = "wps"
            m.id       = seg_idx*1000 + i
            m.type     = Marker.SPHERE
            m.action   = Marker.ADD
            m.pose     = p.pose
            m.scale.x = m.scale.y = m.scale.z = 0.05
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
            markers.markers.append(m)
        self.waypoint_marker_pub.publish(markers)
    
    def _publish_full_traj(self, poses: list[PoseStamped]):
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
    parser.add_argument("--stopping", action="store_true", help="Stop at the end of every segment")
    args = parser.parse_args()

    try:
        yaml_file_path = _select_yaml(args)
    except FileNotFoundError as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        sys.exit(1)

    # Feedback to the user
    print(f"Loading waypoints file: {yaml_file_path}")
    try:
        gps_wpf = GpsWpCommander(yaml_file_path, reverse=args.reverse, num_loop=args.loop, stopping=args.stopping)
        gps_wpf.start_ntp()
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        sys.exit(1)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
