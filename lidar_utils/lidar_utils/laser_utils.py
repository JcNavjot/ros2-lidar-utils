import math

def get_index_from_angle(angle_rad, scan_angle_min, scan_angle_max, scan_angle_increment, num_ranges):

    """
    Convert an angle (rad) to a LaserScan index.

    This implementation is intentionally SIMPLE and CORRECT.
    It works for:
      - [0, 2π] scans
      - [-π, +π] scans
      - partial FOV scans

    Assumption (guaranteed by ROS):
      index i corresponds to angle:
        angle = scan_angle_min + i * scan_angle_increment
    """

    # “If the LiDAR scan uses a 0 → 2π coordinate system, but I supplied a negative angle, reinterpret that angle in the same coordinate frame.”
    if scan_angle_min >= 0:
        # scan is [0, 2π]
        if angle_rad < 0:
            angle_rad += 2 * math.pi

    # Reject angles outside physical FOV
    if angle_rad < scan_angle_min or angle_rad > scan_angle_max:
        return None

    index = int(round((angle_rad - scan_angle_min) / scan_angle_increment))

    if 0 <= index < num_ranges:
        return index
    else:
        return None



# you can call this function to test the above utility function, just uncomment and run this script directly putting in some sample values

# get_index = get_index_from_angle(0.262, 0.0,  6.28000020980835, 0.008734352886676788, 720)  # Example call
# print(f"Index for the provided angle is: {get_index}")  # Expected output: Index for the provided angle