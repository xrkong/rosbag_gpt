<img src="resources/rosbaggpt.png" width="150" height="150"/>  

# ROSBAG GPT
![License](https://img.shields.io/github/license/ros2/rosbag2)
![ROS2 Version](https://img.shields.io/badge/ROS2-Humble%20Hawksbill-brightgreen)
![Issues](https://img.shields.io/github/issues/xrkong/rosbag_gpt)

## Goal

Use LLM to analyze and visualize sensor data in rosbag and generate reports.

[report.webm](https://github.com/user-attachments/assets/eecf9b97-3d9c-46b4-97d9-c10d4ef185a8)

## Features
- ros version: foxy, humble
- rosbag: ```mcap```, ```db3```
- ros topic type: [```sensor_msgs/Image```](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html), [```sbg_driver/SbgGpsPos```](http://docs.ros.org/en/api/sbg_driver/html/msg/SbgGpsPos.html), [```sensor_msgs/PointCloud2```](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html), [```sensor_msgs/LaserScan```](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html)
- gpt-4o

[report_generation.webm](https://github.com/user-attachments/assets/0bb99378-3e2d-4075-9145-157609968325)

## Installation

### Conda

<!-- conda install -c conda-forge gcc=12.1.0 -->

```bash
conda env create -n rosbag_gpt -f environment.yaml
conda activate rosbag_gpt
python3 demo.py
```

## Example Snippets
- **Exrtact specific topic messages from ros2bag**

- **Extract all messages frame given a timestamp from ros2bag**

- **Draw path map from ros2bag / csv**

- **Use gpt-4o to analyse images and generate report**


## Unittest 
The unittest fixture files can be found at [xrkong/nuway_rosbag](https://huggingface.co/datasets/xrkong/nuway_rosbag).  
```bash
huggingface-cli download --repo-type dataset --local-dir ./unittest/fixture  xrkong/nuway_rosbag
```
Put the files under ```/unittest/fixture```

## Tutorial
please check the ```/unittest/test_unittest.py```

# Roadmap

- [x] Deserialize from ros2 bag, (.db3 file)
- [x] Get data, like /lidar_safety/front_left/scan, /ins0/gps_pos, /ins0/orientation
- [x] Plot them
- [x] If there is a icon of the vechicle, add into a middile.
- [x] plot map



