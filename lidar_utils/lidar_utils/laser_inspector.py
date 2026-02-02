import rclpy
import math
import time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy
from lidar_utils.laser_utils import get_index_from_angle

# ANSI color codes for terminal output
RED = "\033[91m"
GREEN = "\033[1;32m"
YELLOW = "\033[93m"
NC = "\033[0m"

# Preferred topic names (easy to extend later)
PREFERRED_SCAN_TOPICS = [
    '/scan',
    '/laserscan'
]

class RobotLaserDataNode(Node):

    def __init__(self):
        super().__init__('laser_inspector_node')

        self.subscription = None
        self.discovery_attempts = 0
        self.max_attempts = 25   # ~5 seconds (25 × 0.2s)

        self.get_logger().info("🔍 Searching for LaserScan topic...")
        print("--------------------------------------------------")

        # Timer-based discovery (CRITICAL FIX)
        self.discovery_timer = self.create_timer(0.2, self.try_discover_scan_topic)

        self.printed = False


    def try_discover_scan_topic(self):
        self.discovery_attempts += 1

        topics = self.get_topic_names_and_types()

        laser_topics = [
            name for name, types in topics
            if 'sensor_msgs/msg/LaserScan' in types
        ]

        if not laser_topics:
            if self.discovery_attempts >= self.max_attempts:
                self.discovery_timer.cancel()
                self.print_failure_message()
            return

        # Prefer known topic names
        scan_topic = None
        for preferred in PREFERRED_SCAN_TOPICS:
            if preferred in laser_topics:
                scan_topic = preferred
                break

        # Fallback to any LaserScan topic
        if scan_topic is None:
            scan_topic = laser_topics[0]
            self.get_logger().warn(f"⚠️ Using non-standard LaserScan topic: {scan_topic}")
            print("--------------------------------------------------")

        
        self.get_logger().info(f"✅ Subscribing to LaserScan topic: {scan_topic}")
        print("--------------------------------------------------")
        self.get_logger().info("📡 LaserScan data received")
        print("--------------------------------------------------")

        self.subscription = self.create_subscription(
            LaserScan,
            scan_topic,
            self.laserscan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.discovery_timer.cancel()

    def print_failure_message(self):
        print("❌ No LaserScan topic found.\n")
        print("Checked preferred topic names:")
        for t in PREFERRED_SCAN_TOPICS:
            print(f" - {t}")

        print("\nWhat you can do:")
        print("  1. Run: ros2 topic list")
        print("  2. Find the LaserScan topic (type: sensor_msgs/msg/LaserScan)")
        print("  3. Add it to PREFERRED_SCAN_TOPICS in laser_inspector.py\n")
        print("Node will remain running. Press Ctrl+C to exit.")
    
    # callback function to process incoming LaserScan messages ...
    def laserscan_callback(self, msg):
        
        if self.printed:        # Just print only one time.
            return

        self.printed = True

        # Cache scan metadata
        self.scan_angle_min = msg.angle_min
        self.scan_angle_max = msg.angle_max
        self.scan_angle_increment = msg.angle_increment
        self.scan_range_min = msg.range_min
        self.scan_range_max = msg.range_max
        self.scan_ranges = [round(value, 3) for value in msg.ranges]
        self.num_ranges = len(msg.ranges)

        self.print_laser_info()
        print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")
        self.print_direction_indices()
        print(f"\n🤓🎉 {GREEN}Laser inspection complete. Press Ctrl+C to exit.{NC}\n")
        
        
    # define print_laser_info to display laser scan statistics ...
    def print_laser_info(self):

        print("Laser Scan Info")
        print("-------------------")
        print(f"angle_min : {self.scan_angle_min}")
        print(f"angle_max : {self.scan_angle_max}")
        print(f"The number of scan ranges and therefore indices are: {self.num_ranges}")
        print(f"increment : {self.scan_angle_increment}")
        print(f"range_min : {self.scan_range_min}")
        print(f"range_max : {self.scan_range_max}")
        min_angle_deg = round(math.degrees(self.scan_angle_min))
        max_angle_deg = round(math.degrees(self.scan_angle_max))
        print(f" For given robot, LIDAR ranges from {min_angle_deg} to {max_angle_deg} degrees")

        fov_deg = math.degrees(self.scan_angle_max - self.scan_angle_min)
        print(f"FOV in degrees : {round(fov_deg, 2)}")

    # function to get index from angle, will make use of get_angle_from_index function from laser_utils.py script ..
    def get_index(self, angle):
        return get_index_from_angle(
            angle,
            self.scan_angle_min,
            self.scan_angle_max,
            self.scan_angle_increment,
            self.num_ranges
        )

     # function to get direction indices for sides, this will help us understand what's the index for each side i.e front, rear, left, right...
    def print_direction_indices(self):
        
        """
        Get the LaserScan indices for standard directions: front, left, right, back.

        Returns:
        dict -> A dictionary with direction names as keys and their corresponding indices as values.
        """

        print("Direction indices (based on LiDAR scan):")
        print("--------------------------------")

        direction_indices = {}  # create an empty dictionary to hold direction indices

        # Define standard direction angles in radians.. the angles are universal by convention and do not depend on the LiDAR configuration
        direction_angles = {
            "front": 0.0,
            "left": math.pi / 2,
            "back": math.pi,
            "right": -math.pi / 2
       
        }

        for name, angle in direction_angles.items():
            idx = self.get_index(angle)
            if idx is None:
                print(f" For {name:<6}side --> not visible in LiDAR FOV")
            else:
                print(f" For {name:<6}side: index is {idx}")

        print("--------------------------------")


def main():
    rclpy.init()
    node = RobotLaserDataNode()
    
    if not rclpy.ok():
        return
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\n{RED}Ctrl + C was pressed, Shutting down node, Thank you... {NC}")
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()