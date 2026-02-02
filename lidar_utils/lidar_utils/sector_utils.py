#!/usr/bin/env python3

"""
Sector Utility Script
---------------------
Human-facing helper for designing LiDAR angular sectors.

• Application-dependent (wall following, obstacle avoidance, etc.)
• Angle-normalized to robot frame [-180°, +180°]
• Anti-clockwise ordering
• Semantic naming (Front / Rear / Left / Right)
• Auto-suggests sector count (recommendation only)
• Copy-paste ready output
"""

import math


# ==========================================================
# UI / NOTICE
# ==========================================================

def print_sector_design_notice():
    GREEN = "\033[1;32m"
    RED = "\033[1;31m"
    NC = "\033[0m"

    print(f"""{GREEN}
ℹ️  Sector Design Notice
-------------------------------------------------------------
LiDAR sectors should be chosen based on the application:
• Wall following      → side-focused narrow rays / ROI
• Obstacle avoidance  → forward-biased sectors
• Exploration         → wider angular coverage
• Safety              → narrow front sector
-------------------------------------------------------------
{NC}""")

    print(f"""{RED}
⚠️  NOTE:
Validate these sectors for your application.
Feel free to modify them as needed — these are design recommendations,
NOT enforced rules.
{NC}""")


# ==========================================================
# ANGLE HELPERS
# ==========================================================

def rad_to_deg(angle_rad: float, precision: int = 1) -> float:
    return round(math.degrees(angle_rad), precision)


def deg_to_rad(angle_deg: float) -> float:
    return math.radians(angle_deg)


def normalize_deg(angle_deg: float) -> float:
    """
    Normalize angle to robot-centric range [-180, +180] degrees.
    """
    return (angle_deg + 180) % 360 - 180


# ==========================================================
# SECTOR COUNT RECOMMENDATION
# ==========================================================

def recommend_sector_count(fov_deg: float) -> int:
    """
    Recommend number of sectors based on total FOV (degrees).
    This is guidance only — user may override.
    """
    if fov_deg >= 330:
        return 8      # Full 360° LiDAR (TurtleBot)
    elif fov_deg >= 250:
        return 6      # Wide forward LiDAR (Leo)
    elif fov_deg >= 180:
        return 4
    else:
        return 3


# ==========================================================
# INTERACTIVE SECTOR GENERATOR
# ==========================================================

def interactive_sector_generator():
    print("\n🧭 Sector Helper (Application-Dependent)")
    print("----------------------------------------")

    try:
        fov_input = input(
            "Enter FOV range in degrees (e.g. -135 5 or 0 360): "
        ).strip().split()

        fov_start_deg = float(fov_input[0])
        fov_end_deg = float(fov_input[1])

    except (ValueError, IndexError):
        print("\n❌ Invalid input format.\n")
        return

    # -------------------------------
    # Validation
    # -------------------------------

    if abs(fov_start_deg) > 360 or abs(fov_end_deg) > 360:
        print("\n❌ Invalid FOV:")
        print("Angles must be within [-360°, +360°].\n")
        return

    if (fov_end_deg - fov_start_deg) <= 0:
        print("\n❌ Invalid FOV range:")
        print("End angle must be greater than start angle.\n")
        return

    if (fov_end_deg - fov_start_deg) > 360:
        print("\n❌ Invalid FOV span:")
        print("Total FOV cannot exceed 360°.\n")
        return

    fov_span = fov_end_deg - fov_start_deg
    recommended = recommend_sector_count(fov_span)

    print(f"\n📐 FOV span           : {round(fov_span)}°")
    print(f"💡 Recommended sectors: {recommended}")

    try:
        num_sectors = int(
            input("Enter number of sectors (press Enter to accept recommendation): ")
            or recommended
        )

        if num_sectors <= 0:
            raise ValueError

    except ValueError:
        print("\n❌ Invalid sector count.\n")
        return

    # --------------------------------------------------
    # Generate sectors
    # --------------------------------------------------

    fov_start_rad = deg_to_rad(fov_start_deg)
    fov_end_rad = deg_to_rad(fov_end_deg)
    sector_width = (fov_end_rad - fov_start_rad) / num_sectors

    raw_sectors = []
    for i in range(num_sectors):
        a_start = fov_start_rad + i * sector_width
        a_end = a_start + sector_width
        raw_sectors.append((a_start, a_end))

    # --------------------------------------------------
    # Canonical semantic names (anti-clockwise)
    # --------------------------------------------------

    CANONICAL_NAMES = {
        8: [
            "Right_Rear",
            "Right",
            "Front_Right",
            "Front",
            "Front_Left",
            "Left",
            "Left_Rear",
            "Rear",
        ],
        6: [
            "Right_Rear",
            "Right",
            "Front_Right",
            "Front_Left",
            "Left",
            "Left_Rear",
        ],
        4: [
            "Right",
            "Front",
            "Left",
            "Rear",
        ],
        3: [
            "Right",
            "Front",
            "Left",
        ],
    }

    names = CANONICAL_NAMES.get(num_sectors)

    if names is None:
        print("\n❌ Unsupported sector count for semantic naming.")
        print("Supported: 3, 4, 6, 8\n")
        return

    # --------------------------------------------------
    # Print result
    # --------------------------------------------------

    print("\n📌 Copy-paste ready sector definition:\n")
    print("SECTORS = {")

    for name, (a_start, a_end) in zip(names, raw_sectors):
        start_deg = normalize_deg(rad_to_deg(a_start))
        end_deg = normalize_deg(rad_to_deg(a_end))

        print(
            f'    "{name:<12}": ({a_start:>7.3f}, {a_end:>7.3f}),'
            f'   # {start_deg:>6}° → {end_deg:>6}°'
        )

    print("}")
    print("\n✔ Sector generation complete.\n")


# ==========================================================
# MAIN
# ==========================================================

def main():
    try:
        print_sector_design_notice()
        interactive_sector_generator()
    except KeyboardInterrupt:
        RED = "\033[1;31m"
        GREEN = "\033[1;32m"
        NC = "\033[0m"
        print(f"\n{GREEN} Ctrl + C detected. Exiting sector helper cleanly.{NC}\n")


if __name__ == "__main__":
    main()
