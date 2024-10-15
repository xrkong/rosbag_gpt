---
license: mit
task_categories:
- object-detection
tags:
- rosbag
- ros2
- lidar
- vehicle
- car
- canbus
- autonomous_vehicles
pretty_name: nuway_rosbags
size_categories:
- 100M<n<1B
---

# nUWAy2 ROS Data Repository

## Overview
This repository contains a collection of ROS bag files (in MCAP format) and video streams from an autonomous shuttle bus. It includes data from single run, featuring a variety of sensors such as VLP-16 LiDAR, safety lidar, CAN bus signals, GPS, a 9-axis IMU, and two camera streams. 

## Dataset Description
The data is organized into multiple runs, each containing synchronized streams from the following sensors:

- VLP-16 LiDAR: 3D point cloud data capturing the vehicle's surroundings.
- CAN Bus: Vehicle's internal communication data including speed, steering angle, and more.
- GPS: Geolocation data showing the vehicle's position.
- 9-Axis IMU: Inertial data providing acceleration, orientation, and gyroscope measurements.
- Camera Streams: Four video streams capturing front, rear views.

## Download
```bash
huggingface-cli download --repo-type dataset --local-dir ./  xrkong/nuway_rosbag
```

## License
This project is licensed under the MIT License - see the LICENSE file for details.