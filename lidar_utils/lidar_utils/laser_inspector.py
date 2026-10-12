
import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from lidar_utils.laser_utils import get_index_from_angle


# ============================================================
# Terminal Colors
# ============================================================

RED = "\033[91m"
GREEN = "\033[1;32m"
YELLOW = "\033[93m"
NC = "\033[0m"


# ============================================================
# Preferred LaserScan topic names: You can add any Laserscan topics in this list, it will show the details accordingly.
# ============================================================

PREFERRED_SCAN_TOPICS = [
    '/scan',
    '/laserscan'
]


class LaserInspectorNode(Node):

    """
    ROS2 utility node for inspecting LaserScan geometry.
    """

    def __init__(self):

        super().__init__('laser_inspector_node')

        self.discovery_attempts = 0
        self.max_attempts = 25     # Attempt to find laserscan topic upto 25 times.

        self.data_printed = False   # Switch to enable printing the data only one time

        self.get_logger().info("--------------------------------------------------")
        self.get_logger().info("🔍 Searching for LaserScan topic...")
        self.get_logger().info("--------------------------------------------------")

        self.discovery_timer = self.create_timer(0.2,self.try_discover_scan_topic)

    # Topic Discovery

    def try_discover_scan_topic(self):

        self.discovery_attempts += 1   # Keep incrementing until attempts reach 25.

        topics = self.get_topic_names_and_types()  # Get a list of topic names and types.

        laser_topics = [name for name, types in topics if 'sensor_msgs/msg/LaserScan' in types]

        # If no laser topic is detected
        if not laser_topics:

            if self.discovery_attempts >= self.max_attempts:
                self.discovery_timer.cancel()
                self.print_failure_message()

            return

        self.get_logger().info("📡 LaserScan topic discovered")
        self.get_logger().info("--------------------------------------------------")

        scan_topic = None

        # Check preferred list for laser topic
        for preferred in PREFERRED_SCAN_TOPICS:

            if preferred in laser_topics:
                scan_topic = preferred
                break

        # If no preferred topic is found
        if scan_topic is None:

            scan_topic = laser_topics[0]  # simply select the first laserscan topic from the list of topics

            self.get_logger().warn(f" Using non-standard LaserScan topic: {scan_topic}")

            self.get_logger().info("--------------------------------------------------")

        self.subscription = self.create_subscription(LaserScan,scan_topic,self.laserscan_callback,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE
            )
        )

        self.get_logger().info(f"✅ Subscribing to LaserScan topic: {scan_topic}")

        self.get_logger().info("--------------------------------------------------")

        self.discovery_timer.cancel()

    # Failure Handling

    def print_failure_message(self):

        self.get_logger().info("❌ No LaserScan topic found.\n")

        self.get_logger().info("Checked preferred topic names:")

        for topic in PREFERRED_SCAN_TOPICS:
            self.get_logger().info(f" - {topic}")

        self.get_logger().info("\nWhat you can do:")
        self.get_logger().info("  1. Run: ros2 topic list")
        self.get_logger().info("  2. Identify the LaserScan topic")
        self.get_logger().info("  3. Add the topic to PREFERRED_SCAN_TOPICS")

        self.get_logger().info("\nNode will remain active. Press Ctrl+C to exit.")

    # LaserScan Callback

    def laserscan_callback(self, msg):

        if self.data_printed:    # print the data only once.
            return

        self.data_printed = True

        self.scan_angle_min = msg.angle_min
        self.scan_angle_max = msg.angle_max
        self.scan_angle_increment = msg.angle_increment

        self.scan_range_min = msg.range_min
        self.scan_range_max = msg.range_max

        self.num_ranges = len(msg.ranges)

        self.print_laser_info()

        self.get_logger().info("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

        self.print_direction_indices()

        self.get_logger().info(f"\n🤓🎉 {GREEN}Laser inspection complete, Press Ctrl+C to exit{NC}")

    # LaserScan Statistics
    
    def print_laser_info(self):

        self.get_logger().info("Laser Scan Info")
        self.get_logger().info("-------------------")

        self.get_logger().info(f"angle_min : {self.scan_angle_min}")
        self.get_logger().info(f"angle_max : {self.scan_angle_max}")
        self.get_logger().info(f"Total scan ranges / indices : {self.num_ranges}")
        self.get_logger().info(f"angle_increment : {self.scan_angle_increment}")
        self.get_logger().info(f"range_min : {self.scan_range_min}")
        self.get_logger().info(f"range_max : {self.scan_range_max}")

        min_angle_deg = round(math.degrees(self.scan_angle_min))
        max_angle_deg = round(math.degrees(self.scan_angle_max))

        self.get_logger().info(f"LiDAR angular range : "f"{min_angle_deg}° to {max_angle_deg}°")

        fov_deg = max_angle_deg - min_angle_deg

        self.get_logger().info(f"FOV in degrees : {fov_deg}°")

    # ========================================================
    # Angle → Index Conversion
    # ========================================================

    def get_index(self, angle):

        return get_index_from_angle(
            angle,
            self.scan_angle_min,
            self.scan_angle_max,
            self.scan_angle_increment,
            self.num_ranges
        )

    # ========================================================
    # Direction Index Debugging
    # ========================================================

    def print_direction_indices(self):

        self.get_logger().info("Direction indices (based on LiDAR scan):")

        self.get_logger().info("--------------------------------")

        direction_angles = {
            "front": 0.0,
            "left": math.pi / 2,
            "back": math.pi,
            "right": -math.pi / 2
        }

        for direction, angle in direction_angles.items():

            idx = self.get_index(angle)

            if idx is None:

                self.get_logger().info(f"For {direction:<6}side --> not visible in LiDAR FOV")

            else:

                self.get_logger().info(f"For {direction:<6}side: index is {idx}")

        self.get_logger().info("--------------------------------")

def main():

    rclpy.init()

    node = LaserInspectorNode()

    if not rclpy.ok():
        return

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:

        print(f"\n{RED} Ctrl+C pressed. Shutting down Laser Inspector... {NC}")

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
