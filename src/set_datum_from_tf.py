import rclpy
from rclpy.node import Node
from robot_localization.srv import SetDatum
import tf2_ros
import math
from rclpy.duration import Duration

# WGS-84 constants
WGS84_A = 6378137.0                        # major axis [m]
WGS84_B = 6356752.314245                   # minor axis [m]
WGS84_E2 = 6.69437999014e-3                # first eccentricity squared
WGS84_A2 = WGS84_A ** 2
WGS84_B2 = WGS84_B ** 2
WGS84_EE2 = WGS84_A2 / WGS84_B2 - 1        # second eccentricity squared

def cbrt(x):
    """Cube root that handles negative values."""
    if x >= 0:
        return x**(1.0/3.0)
    else:
        return -(-x)**(1.0/3.0)

def tf_wgs84_llh_ecef(ecef):
    """
    Convert ECEF coordinates to latitude, longitude, and altitude.
    
    :param ecef: A tuple (x, y, z) in ECEF meters.
    :return: Tuple (lat, lon, alt) with lat and lon in radians.
    """
    x, y, z = ecef
    x_2 = x * x
    y_2 = y * y
    z_2 = z * z
    r_2 = x_2 + y_2
    r = math.sqrt(r_2)

    F = 54.0 * WGS84_B2 * z_2
    G = r_2 + (1 - WGS84_E2) * z_2 - WGS84_E2 * (WGS84_A2 - WGS84_B2)
    c_val = WGS84_E2 * WGS84_E2 * F * r_2 / (G * G * G)
    s = cbrt(1 + c_val + math.sqrt(c_val * c_val + 2 * c_val))
    denom = (s + 1.0 + 1.0 / s)
    P = F / (3.0 * (denom * denom) * G * G)
    Q = math.sqrt(1 + 2 * WGS84_E2 * WGS84_E2 * P)
    r0 = -P * WGS84_E2 * r / (1 + Q) + math.sqrt(0.5 * WGS84_A2 * (1.0 + 1.0 / Q) -
         (P * (1 - WGS84_E2) * z_2 / (Q + Q * Q)) - 0.5 * P * r_2)
    t1 = (r - WGS84_E2 * r0)
    t1_2 = t1 * t1
    U = math.sqrt(t1_2 + z_2)
    V = math.sqrt(t1_2 + (1 - WGS84_E2) * z_2)
    a_V = WGS84_A * V
    z0 = WGS84_B2 * z / a_V

    h = U * (1 - WGS84_B2 / a_V)
    lat = math.atan((z + WGS84_EE2 * z0) / r)
    lon = math.atan2(y, x)
    return (lat, lon, h)

class DatumPublisher(Node):
    def __init__(self):
        super().__init__('datum_publisher')
        # Create a service client for the SetDatum service.
        self.client = self.create_client(SetDatum, '/datum')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /datum service...')
        
        # Setup TF2 to listen for static transforms.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Wait until the static transform from FP_ECEF to FP_ENU0 is available.
        transform = None
        while transform is None and rclpy.ok():
            try:
                transform = self.tf_buffer.lookup_transform(
                    'FP_ECEF', 'FP_ENU0', self.get_clock().now(),
                    Duration(seconds=1, nanoseconds=0)
                )
            except Exception as e:
                self.get_logger().info('Waiting for transform from FP_ECEF to FP_ENU0...')
                rclpy.spin_once(self, timeout_sec=0.1)
        
        if transform is not None:
            self.get_logger().info('Static transform received.')
            # Extract the translation vector (assumed to be in ECEF coordinates).
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            qw = transform.transform.rotation.w
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            self.get_logger().info(f'ECEF Translation: x={x}, y={y}, z={z}')
            
            # Convert the ECEF translation to LLH.
            lat_rad, lon_rad, alt = tf_wgs84_llh_ecef((x, y, z))
            # Convert radians to degrees for latitude and longitude.
            lat_deg = math.degrees(lat_rad)
            lon_deg = math.degrees(lon_rad)
            self.get_logger().info(f'Computed LLH: lat={lat_deg:.6f}°, lon={lon_deg:.6f}°, alt={alt:.2f} m')
            
            # Prepare the SetDatum service request.
            self.req = SetDatum.Request()
            self.req.geo_pose.position.latitude = lat_deg
            self.req.geo_pose.position.longitude = lon_deg
            self.req.geo_pose.position.altitude = alt
            self.req.geo_pose.orientation.x = qx
            self.req.geo_pose.orientation.y = qy
            self.req.geo_pose.orientation.z = qz
            self.req.geo_pose.orientation.w = qw
        else:
            self.get_logger().error('Failed to receive the static transform.')

    def send_request(self):
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    datum_publisher = DatumPublisher()
    response = datum_publisher.send_request()
    if response:
        datum_publisher.get_logger().info(f'Response: {response}')
    else:
        datum_publisher.get_logger().error('Service call failed.')
    datum_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
