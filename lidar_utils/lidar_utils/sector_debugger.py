"""
Sector Debugger Node (YAML-free, real-robot safe)

Two debugging approaches using LiDAR (LaserScan):

Approach 1 (DEBUG_MODE = "sector"):
    - Compute minimum obstacle distance in predefined angular sectors.

Approach 2 (DEBUG_MODE = "cardinal"):
    - Probe distances at specific angles (front, left, right, rear, etc.).

Design goals:
- Zero configuration required
- Robust LaserScan discovery
- Time-based throttling inside callback
- Clear, colored logging
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math
import time
import threading

from lidar_utils.laser_utils import get_index_from_angle

 
    # print(f"⏳ {GREEN}Waiting for input, Please enter what mode and time/duration(seconds) you want to use...{NC}")
    
def get_startup_config(timeout=10):
    DEFAULT_MODE = "cardinal"
    DEFAULT_PERIOD = 1.0

    user_input = {"received": False}

    def timeout_watcher():
        time.sleep(timeout)
        if not user_input["received"]:
            print(f"\n⏱ Timeout reached ({timeout}s). Starting with defaults...\n")

    # Start timeout watcher
    threading.Thread(target=timeout_watcher, daemon=True).start()

    print("\n🧭 Sector Debugger Setup")
    print("----------------------------------")
    print("Mode options available:")
    print("  - cardinal (default)")
    print("  - sector")
    print("")
    print("----------------------------------")
    print(f"➡{GREEN} For loading with Default values, either press ENTER, y or wait {timeout}s\n{NC}")
    print(f"➡{YELLOW} Or press 'n' or 'N' to manually enter Mode and time interval between 2 consective readings(seconds) prompts below (timeout {timeout}s){NC}")
    print("----------------------------------")

    try:
        choice = input("Start with default values? [Y/n]: ").strip().lower()
        user_input["received"] = True
    except EOFError:
        return DEFAULT_MODE, DEFAULT_PERIOD

    if choice in ("", "y", "yes"):
        return DEFAULT_MODE, DEFAULT_PERIOD

    # ---- MODE ----
    while True:
        mode = input("Enter mode (cardinal / sector): ").strip().lower()
        if mode in ("c", "cardinal"):
            debug_mode = "cardinal"
            break
        elif mode in ("s", "sector"):
            debug_mode = "sector"
            break
        else:
            print("❌ Invalid mode. Please enter 'cardinal' or 'sector'.")

    # ---- PERIOD ----
    while True:
        period = input("Enter print period in seconds (> 0): ").strip()
        try:
            print_period = float(period)
            if print_period > 0:
                break
            else:
                raise ValueError
        except ValueError:
            print("❌ Invalid value. Enter a positive number.")

    print("\n✅ Starting with:")
    print(f"   Mode        : {debug_mode.upper()}")
    print(f"   Print period: {print_period:.2f} s")
    print("----------------------------------\n")

    return debug_mode, print_period

# --------------------------------------------------
# ANSI color codes
# --------------------------------------------------
RED = "\033[91m"
GREEN = "\033[1;32m"
YELLOW = "\033[93m"
NC = "\033[0m"


PREFERRED_SCAN_TOPICS = [
    '/scan',
    '/laserscan'
]

class SectorDebuggerNode(Node):

    def __init__(self, debug_mode="cardinal", print_period=1.0):
        super().__init__('sector_debugger_node')

        # -------------------------------
        # Sector definitions (radians)
        # -------------------------------
        self.sectors = {
            "Front": (-0.261, 0.261),   # ~ -15° to +15°
            "Left":  (1.309, 1.833),    # ~ 75° to 105°
            "Rear" : (3.01, 3.271),     # ~ 173° → 187°
            "Right": (-1.833, -1.309),  # ~ -105° to -75°
        }

        self.debug_mode = debug_mode
        self.print_period = print_period
        self.last_print_time = 0.0
        

        self.subscription = None

        self.get_logger().info(
            f"{GREEN}🧭 Sector Debugger started in "
            f"{self.debug_mode.upper()} mode{NC}"
        )
        self.get_logger().info(f"{GREEN}🕒 Print period: {self.print_period:.2f} s{NC}")
        self.get_logger().info(f"{GREEN}📐 Active sectors: {list(self.sectors.keys())}{NC}")
        print("----------------------------------------------------------------")
        self.get_logger().info("🔍 Searching for LaserScan topic...")
        print('----------------------------------------------------------------')

        # Discovery state
        self.discovery_attempts = 0
        self.max_attempts = 25  # ~5 seconds

        self.discovery_timer = self.create_timer(
            0.2, self.try_discover_laserscan
        )

    # --------------------------------------------------
    # LaserScan discovery (robust)
    # --------------------------------------------------
    def try_discover_laserscan(self):
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

        scan_topic = None
        for preferred in PREFERRED_SCAN_TOPICS:
            if preferred in laser_topics:
                scan_topic = preferred
                break

        if scan_topic is None:
            scan_topic = laser_topics[0]
            print("--------------------------------------------------")
            self.get_logger().info(f"{YELLOW}⚠️ Using non-standard LaserScan topic: {scan_topic}{NC}")
            print("--------------------------------------------------")

        self.get_logger().info(f"✅ Subscribing to LaserScan topic: {scan_topic}")
        print("------------------------------------------------------------------")
        self.get_logger().info("📡 LaserScan data received")

        self.subscription = self.create_subscription(
            LaserScan,
            scan_topic,
            self.laserscan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        print("------------------------------------------------------------------")
        self.discovery_timer.cancel()

    # --------------------------------------------------
    # LaserScan callback (time-throttled)
    # --------------------------------------------------
    def laserscan_callback(self, msg):
        now = time.time()
        if now - self.last_print_time < self.print_period:
            return
        self.last_print_time = now

        # Cache scan metadata
        self.scan_angle_min = msg.angle_min
        self.scan_angle_max = msg.angle_max
        self.scan_angle_increment = msg.angle_increment
        self.scan_range_min = msg.range_min
        self.scan_range_max = msg.range_max
        self.scan_ranges = msg.ranges
        self.number_of_ranges = len(msg.ranges)

        if self.debug_mode == "sector":
            self.debug_sectors()
        else:
            self.print_cardinal_distances()

        self.get_logger().info("--------------------------------------------------")

    # --------------------------------------------------
    # Approach 1: Sector-based debugging
    # --------------------------------------------------
    def detect_obstacles_by_sector(self):
        sector_distances = {}

        for name, (angle_start, angle_end) in self.sectors.items():
            start_idx = get_index_from_angle(
                angle_start,
                self.scan_angle_min,
                self.scan_angle_max,
                self.scan_angle_increment,
                self.number_of_ranges
            )
            end_idx = get_index_from_angle(
                angle_end,
                self.scan_angle_min,
                self.scan_angle_max,
                self.scan_angle_increment,
                self.number_of_ranges
            )

            if start_idx is None or end_idx is None:
                sector_distances[name] = None
                continue

            if start_idx <= end_idx:
                sector_ranges = self.scan_ranges[start_idx:end_idx + 1]
            else:
                sector_ranges = (
                    self.scan_ranges[start_idx:] +
                    self.scan_ranges[:end_idx + 1]
                )

            valid_ranges = [
                r for r in sector_ranges
                if self.scan_range_min <= r <= self.scan_range_max
            ]

            if valid_ranges:
                sector_distances[name] = min(valid_ranges)
            else:
                sector_distances[name] = float('inf')

        return sector_distances

    def debug_sectors(self):
        self.get_logger().info("📊 Sector-wise minimum distances:")

        sector_distances = self.detect_obstacles_by_sector()
        for sector, distance in sector_distances.items():
            if distance is None:
                self.get_logger().warn(
                    f"{RED}{sector}: Outside LiDAR FOV{NC}"
                )
            elif math.isinf(distance):
                self.get_logger().info(
                    f"{YELLOW}{sector}: CLEAR (no obstacle within range){NC}"
                )
            else:
                self.get_logger().info(
                    f"{GREEN}{sector}: {distance:.2f} m{NC}"
                )

    # --------------------------------------------------
    # Approach 2: Cardinal probing
    # --------------------------------------------------
    def get_distance_at_angle(self, angle_rad):
        idx = get_index_from_angle(
            angle_rad,
            self.scan_angle_min,
            self.scan_angle_max,
            self.scan_angle_increment,
            self.number_of_ranges
        )

        if idx is None:
            return None

        dist = self.scan_ranges[idx]
        if not (self.scan_range_min <= dist <= self.scan_range_max):
            return float('inf')

        return dist

    def print_cardinal_distances(self):
        angles = {
            "FRONT": 0.0,
            "LEFT": math.pi / 2,
            "BACK": math.pi,
            "RIGHT": -math.pi / 2,
            "OTHER": math.radians(-125),
        }

        for name, angle in angles.items():
            dist = self.get_distance_at_angle(angle)

            if dist is None:
                self.get_logger().warn(
                    f"{RED}{name}: Outside LiDAR FOV — check mounting{NC}")
            elif math.isinf(dist):
                self.get_logger().info(f"{YELLOW}{name}: No obstacle detected within range{NC}")
            else:
                self.get_logger().info(f"{GREEN}{name}: Obstacle at {dist:.2f} m{NC}")

    # --------------------------------------------------
    # Failure handling
    # --------------------------------------------------
    def print_failure_message(self):
        print('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
        print("❌ No LaserScan topic found.\n")
        print("Checked preferred topic names:")
        for t in PREFERRED_SCAN_TOPICS:
            print(f" - {t}")

        print("\nWhat you can do:")
        print("  1. Run: ros2 topic list")
        print("  2. Find the LaserScan topic (type: sensor_msgs/msg/LaserScan)")
        print("  3. Add it to PREFERRED_SCAN_TOPICS in laser_inspector.py\n")
        print("Node will remain running. Press Ctrl+C to exit.")


def main(args=None):
    debug_mode, print_period = get_startup_config(timeout=10)
    rclpy.init(args=args)
    node = SectorDebuggerNode(debug_mode=debug_mode, print_period=print_period)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\n{RED}Shutting down Sector Debugger Node...{NC}")
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
