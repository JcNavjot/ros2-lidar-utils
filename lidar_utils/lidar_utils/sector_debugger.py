
"""
Sector Debugger Utility:
====================

A lightweight ROS2 utility for debugging LiDAR obstacle distances
using sector-based analysis.

Main Features
-------------
- Automatic LaserScan topic discovery
- Works on real robots and simulations
- Dynamic sector-based obstacle inspection
- Time-throttled terminal output using timer

Design Philosophy
-----------------
This utility is intentionally designed to stay:
- simple
- reusable
- universal
- easy to understand

Instead of probing only single rays, the node computes the
minimum obstacle distance inside angular sectors.

This provides more stable and practical obstacle debugging
for real robotics systems.
"""


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


class SectorDebuggerNode(Node):

    """
    Real-time LiDAR sector debugging utility.
    """

    def __init__(self):

        super().__init__('sector_debugger_node')

        self.discovery_attempts = 0
        self.max_attempts = 10     # Attempt to find laserscan topic upto 10 times.

        self.data_printed = False   # Switch to enable printing the data only one time
        self.scan_data_ready = False  # Check if laserscan data is ready

        self.get_logger().info("--------------------------------------------------")
        self.get_logger().info("🔍 Searching for LaserScan topic...")
        self.get_logger().info("--------------------------------------------------")

        # Timer to try discover laserscan topic every 0.2 seconds
        self.discovery_timer = self.create_timer(0.2,self.try_discover_scan_topic)

        # Set print Interval, you can change the print interval here.
        # It is the time duration between two consecutive sector readings
        self.declare_parameter("print_interval", 1.0)

        self.print_interval = (self.get_parameter("print_interval").value)
        
        # Timer to print the minimum distance in each sector every 1 second
        self.print_timer = self.create_timer(self.print_interval,self.print_sector_distances)

    # Topic Discovery
    def try_discover_scan_topic(self):

        self.discovery_attempts += 1   # Keep incrementing until attempts reach 10.

        topics = self.get_topic_names_and_types()  # Get a list of topic names and types.

        laser_topics = [name for name, types in topics if 'sensor_msgs/msg/LaserScan' in types]

        # If no laser topic is detected
        if not laser_topics:

            if self.discovery_attempts >= self.max_attempts:  # if attempts reach 10 and no laserscan topic is found
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
            QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE))

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

    def laserscan_callback(self,msg):

        # Store LaserScan data for processing ...
        self.scan_ranges = msg.ranges
        self.scan_angle_min = msg.angle_min
        self.scan_angle_max = msg.angle_max
        self.scan_angle_increment = msg.angle_increment
        self.scan_range_min = msg.range_min
        self.scan_range_max = msg.range_max
        self.number_of_ranges = len(msg.ranges)

        self.scan_data_ready = True

    def detect_obstacles_by_sector(self):

        """
        Detect minimum obstacle distance in each predefined angular sector.

        Returns:
            dict: sector_name -> min distance (float) or None
        """

        # Define sector angles in radians, this will be dynamic based on the robot's laser scanner configuration., you have to check the FOV using sector_utils.py
        # Using angles instead of indices for better adaptability. 
       
        SECTORS = {

            "Front":       (-0.35, 0.35),     # ~ -20° to +20°      # Feel free to change limits of sectors as per your requirement. 
            #"Front Left":  (0.35, 1.05),      # ~ 20° to 60°       # You can also add or remove sectors as per your requirement 
            "Left":        (1.22, 1.92),      # ~ 70° to 110°
            "Rear":        (2.97, 3.31),      # ~ 170° to 190°
            "Right":       (4.36, 5.06),      # ~ 250° to 290°
            #"Front Right": (5.23, 5.93),      # ~ 300° to 340°                                                            
            
        }      

        sector_distances = {}

        # for each sector-one at a time(whole for loop for each dictionary key-word pair), get the respective min and max indices ...
        for sector_name, (angle_start, angle_end) in SECTORS.items():
            start_idx = get_index_from_angle(angle_start, self.scan_angle_min, self.scan_angle_max, self.scan_angle_increment, self.number_of_ranges)
            end_idx = get_index_from_angle(angle_end, self.scan_angle_min, self.scan_angle_max, self.scan_angle_increment, self.number_of_ranges)

            if start_idx is None or end_idx is None:
                sector_distances[sector_name] = None
                continue
                   
            
            if start_idx < self.number_of_ranges and end_idx < self.number_of_ranges:
                # ✅ Correct wrap-aware slicing
                if start_idx <= end_idx:
                    # Normal case (no wrap)
                    sector_ranges = self.scan_ranges[start_idx:end_idx + 1]
                else:
                    sector_ranges = (
                        self.scan_ranges[start_idx:] +
                        self.scan_ranges[:end_idx + 1]
                    )

                valid_ranges = [
                    r for r in sector_ranges         # return r if r falls in between scan_range_min to scan_range max values.
                    if self.scan_range_min <= r <= self.scan_range_max
                ]

                if valid_ranges:
                    sector_distances[sector_name] = min(valid_ranges)
                else:
                    print(f"{RED}Can't find any Obstacles, within the LIDAR range{NC}")
                    sector_distances[sector_name] = float('inf')

        return sector_distances
    
    def print_sector_distances(self):

        if not self.scan_data_ready:  # Start printing only when laserscan data is ready
            return                     # do not proceed further until laserscan data is ready

        self.get_logger().info("📊 Sector-wise minimum obstacle distances:")
        self.get_logger().info("--------------------------------------------------")

        sector_distances = self.detect_obstacles_by_sector()

        for sector, distance in sector_distances.items():

            if distance is None:

                self.get_logger().warn(f"{RED}{sector}: Outside LiDAR FOV{NC}")

            elif math.isinf(distance):

                self.get_logger().info(f"{YELLOW}{sector}: CLEAR{NC}")

            else:

                self.get_logger().info(f"{GREEN}{sector}: {distance:.2f} m{NC}")        
                
        
def main(args=None):

    rclpy.init(args=args)
    node = SectorDebuggerNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print(f"\n{RED} Shutting down Sector Debugger Node...{NC}")

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
    