# Evolution Log — ROS2 LiDAR Utilities

This document records the design reasoning and evolution of this package.
It is not a strict changelog, but a reflection of how the system developed
over time and why certain decisions were made.

## v0.1 — Initial release

- Created a standalone ROS2 package for inspecting and debugging
  `sensor_msgs/LaserScan` data.
- Focused on human-readable inspection before algorithm development.
- Separated geometry utilities from ROS-facing tools to keep logic modular.

### Design decisions

- Avoided hard-coded angle assumptions to support different LiDAR models.
- Kept utilities lightweight and Python-based for rapid debugging.
- Prioritized clarity and safety over performance optimizations.
- Treated this package as a system inspection layer rather than a full perception stack.

## v0.2 — Diagnostics refinement

- Improved `laser_inspector` diagnostics and safety checks.
- Focused on clearer scan validation during early robot integration.


