# ROS2 LiDAR Utilities

A collection of ROS2 utilities and debugging tools for working with
`sensor_msgs/LaserScan` data on real robots and simulations.

## Overview
This repository contains a ROS2 package focused on inspecting, interpreting,
and debugging LiDAR `sensor_msgs/LaserScan` data in a human-readable and
system-safe way during real robot integration.

## Why this package exists

During early robot integration, LiDAR issues are often not caused by
algorithms but by incorrect assumptions about angle ranges, indexing,
field-of-view, or frame alignment. These problems are difficult to
understand when looking only at raw `LaserScan` messages.

This package was created to provide a safe and human-readable way to
inspect LiDAR data before using it in navigation or control logic.

The goal is not to replace perception algorithms, but to help engineers
understand *what the robot actually sees* and avoid silent configuration
mistakes.

## Features
- Robust LaserScan topic discovery
- Angle-to-index conversion utilities
- Sector-based obstacle inspection
- Cardinal direction probing
- Designed for real robots and Gazebo simulation

## Package Structure
- `lidar_utils/` — core ROS2 package
  - `laser_inspector.py` — LaserScan inspection and metadata debugging
  - `sector_debugger.py` — sector and cardinal-based obstacle probing
  - `laser_utils.py` — angle/index conversion helpers
  - `sector_utils.py` — sector design and generation helpers
  - `angle_utils.py` — angle conversion utilities
 
## Example Usage Workflow

This package is typically used during system bring-up or debugging:

1. Launch a robot or simulation that publishes `sensor_msgs/LaserScan`or similar topic. Examples can be /scan, /laser_scan etc...
2. Run `laser_inspector` to verify angle limits, ranges, and scan metadata. It's useful for understanding LIDAR data parameters.
3. Use `sector_debugger` to probe obstacles in specific directions
4. Integrate validated scan interpretation into higher-level behaviors

## Compatibility
- ROS2 Humble
- Python-based ROS2 nodes

## Notes
This package is part of my robotics systems work and is intended as a reusable
debugging and perception support module.

For design reasoning and development history, see [EVOLUTION.md](EVOLUTION.md).
