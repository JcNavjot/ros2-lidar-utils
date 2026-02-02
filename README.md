# ROS2 LiDAR Utilities

A collection of ROS2 utilities and debugging tools for working with
`sensor_msgs/LaserScan` data on real robots and simulations.

## Overview
This repository contains a ROS2 package focused on inspecting, interpreting,
and debugging LiDAR data in a human-readable and system-safe way.

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

## Compatibility
- ROS2 Humble
- Python-based ROS2 nodes

## Notes
This package is part of my robotics systems work and is intended as a reusable
debugging and perception support module.
