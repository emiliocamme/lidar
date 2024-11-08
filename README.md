# Lidar project
## Project Overview
This project integrates a simulated LiDAR sensor in CoppeliaSim with ROS 2 for real-time SLAM (Simultaneous Localization and Mapping) and navigation in RViz. The sensor data is processed and published using a Python node, which broadcasts transforms and handles obstacle avoidance by referencing an occupancy grid.
## Prerequisites
Ensure you have the following installed:
- [CoppeliaSim](https://www.coppeliarobotics.com/) (with a compatible version for your system)
- [ROS 2 Humble](https://docs.ros.org/en/humble/index.html)
- Python 3.x (compatible with ROS 2 packages)
## CoppeliaSim Scene File
- **LiDAR Scene File (.ttt)**: Required CoppeliaSim scene with LiDAR object configured. The `LidarPublisher` node retrieves and processes data from this scene.
## Dependencies
* Cartographer
  ```
  sudo apt install ros-humble-cartographer
  sudo apt install ros-humble-cartographer-ros
  sudo apt install ros-humble-cartographer-rviz
  ```

## Ros2 packages
* TF2
* Sensor_msgs
* nav_msgs
* geometry_msgs
* rclpy.node

## Python dependencies
* time
* math
* coppeliasim_zmqremoteapi_client
  ```sh
  pip install coppeliasim-zmqremoteapi-client
  ```
