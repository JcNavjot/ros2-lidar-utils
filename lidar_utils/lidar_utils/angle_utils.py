# !/usr/bin/env python3

import math

# --------------------------------------------------
# Angle conversion utilities
# --------------------------------------------------

def deg_to_rad(angle_deg: float, precision: int = 3) -> float:
    """
    Convert degrees to radians with rounding.
    """
    return round(math.radians(angle_deg), precision)


def rad_to_deg(angle_rad: float, precision: int = 1) -> float:
    """
    Convert radians to degrees with rounding.
    """
    return round(math.degrees(angle_rad), precision)


# --------------------------------------------------
# Quick CLI usage (optional)
# --------------------------------------------------

if __name__ == "__main__":
    print("🧮 Angle Conversion Utility")
    print("---------------------------")

    try:
        value = float(input("Enter angle value you want to convert in either degree or radian: "))
        mode = input("Convert to (deg/rad): ").strip().lower()

        if mode == "rad":
            print(f"→ {deg_to_rad(value)} rad")
        elif mode == "deg":
            print(f"→ {rad_to_deg(value)} deg")
        else:
            print("❌ Invalid mode. Use 'deg' or 'rad'.")

    except ValueError:
        print("❌ Invalid numeric input.")
