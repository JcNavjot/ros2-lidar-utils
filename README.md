# ROS2 LiDAR Utilities — LaserScan Debugging & Inspection Toolkit

A lightweight yet powerful ROS2 toolkit for inspecting, understanding, and debugging `sensor_msgs/LaserScan` data on both **real robots** and **simulation environments**.

Designed for:

* Robotics engineers
* ROS2 developers
* Students learning perception
* Navigation & autonomy debugging
* LiDAR integration and validation

---

# Why this package exists

During robot integration, many navigation failures are not caused by algorithms — they are caused by incorrect assumptions about:

* LiDAR angle conventions
* scan indexing
* field-of-view interpretation
* coordinate alignment
* obstacle probing logic

Raw `LaserScan` messages are difficult to interpret directly.

This toolkit was created to provide a **human-readable debugging layer** between raw sensor data and higher-level robotic behaviors.

The goal is simple:

> Help developers understand what the robot actually sees before building autonomy on top of it.

---

# Universal Design Philosophy

These utilities are intentionally designed to be hardware-agnostic.

✔ Works on real robots and simulations
✔ Supports different LiDAR brands and configurations
✔ Supports different LiDAR FoVs
✔ ROS2-native architecture
✔ Automatically discovers LaserScan topics
✔ Lightweight and easy to extend

No robot-specific assumptions are hardcoded.

---

# Important Notes Before Using

This repository is designed as a complete LiDAR debugging toolkit.

Please clone/download the full repository.

Some scripts rely on shared utility modules inside the package.

For example:

* `laser_inspector.py`
* `sector_debugger.py`

both depend on:

* `laser_utils.py`
* `angle_utils.py`

Running scripts individually outside the package may result in import errors.

---

# Features

* LaserScan topic auto-discovery
* LiDAR geometry inspection
* Angle-to-index conversion helpers
* Sector-based obstacle debugging
* Configurable obstacle sectors
* ROS2 launch file support
* Parameterized runtime configuration
* Real-time obstacle monitoring
* Beginner-friendly output formatting
* Works in Gazebo and real hardware

---

# Toolkit Overview

## 1. Laser Inspector

A LiDAR geometry inspection utility.

This tool helps developers understand:

* FoV (Field of View)
* angle increment
* scan indexing
* directional indices
* scan ranges
* LiDAR geometry conventions

This is typically the FIRST tool to run when integrating a new LiDAR.

---

### Example Output

![Laser Inspector Output](assets/laser_inspector_output.png)

---

## 2. Sector Debugger

A real-time obstacle inspection utility.

This tool continuously computes the minimum obstacle distance inside predefined angular sectors.

Useful for:

* obstacle avoidance debugging
* navigation tuning
* behavior-tree debugging
* wall-following logic
* autonomous robot bring-up

---

### Example Output

![Sector Debugger Output](assets/sector_debugger_output.png)

---

# Sector Philosophy

Unlike single-ray probing approaches, this debugger works using angular sectors.

This is generally more robust because:

* single rays can miss obstacles
* sector minimums are safer
* better for real-world robotics

Example sectors:

* Front
* Front-Left
* Front-Right
* Left
* Right
* Rear

Users can modify sector definitions directly inside the script depending on their robot and application.

---

# Repository Structure

```text
ros2-lidar-utils/
├── assets/
├── lidar_utils/
│   ├── laser_inspector.py
│   ├── sector_debugger.py
│   ├── laser_utils.py
│   ├── angle_utils.py
│   └── sector_utils.py
├── launch/
├── README.md
└── EVOLUTION.md
```

---

# Quick Start

## Build the package

```bash
colcon build --packages-select lidar_utils
source install/setup.bash
```

---

# Running the Tools

## Laser Inspector

```bash
ros2 run lidar_utils laser_inspector
```

or

```bash
ros2 launch lidar_utils laser_inspector.launch.py
```

---

## Sector Debugger

```bash
ros2 run lidar_utils sector_debugger
```

or

```bash
ros2 launch lidar_utils sector_debugger.launch.py
```

---

# ROS2 Parameters

The sector debugger supports ROS2 parameters.

Example:

```bash
ros2 launch lidar_utils sector_debugger.launch.py print_interval:=0.2
```

or:

```bash
ros2 run lidar_utils sector_debugger --ros-args -p print_interval:=0.2
```

---

# Parameter Reference

| Parameter        | Description                   | Default |
| ---------------- | ----------------------------- | ------- |
| `print_interval` | Time between obstacle updates | `1.0`   |

---

# Typical Workflow

1. Launch robot or simulation
2. Verify LaserScan topic availability
3. Run `laser_inspector`
4. Understand scan geometry
5. Run `sector_debugger`
6. Tune navigation or obstacle logic
7. Integrate into higher-level autonomy

---

# Compatibility

* ROS2 Humble
* Python ROS2 nodes
* Gazebo simulation
* Real robots

---

# Future Improvements

Planned future additions include:

* RViz visualization tools
* Dynamic sector configuration
* Visual debugging overlays
* Occupancy visualization helpers
* Advanced LiDAR analysis utilities

---

# Contributions & Feedback

Suggestions, improvements, and discussions are always welcome.

If you build something using this toolkit or improve it further, feel free to reach out or contribute.

---

# Development Notes

This repository is part of my robotics systems engineering work focused on:

* perception tooling
* ROS2 debugging infrastructure
* navigation support systems
* developer tooling for robotics

For design reasoning and package evolution history, see:

👉 `EVOLUTION.md`
