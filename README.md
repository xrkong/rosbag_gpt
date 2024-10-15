<img src="resources/rosbaggpt.png" width="150" height="150"/>  

# ROSBAG GPT

Use LLM to analyze and visualize sensor data in rosbag and generate reports.

# Supported Versions
- ros version: foxy, humble
- rosbag: ```mcap```, ```db3```
- ros topic type: [```sensor_msgs/Image```](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html), [```sbg_driver/SbgGpsPos```](http://docs.ros.org/en/api/sbg_driver/html/msg/SbgGpsPos.html), [```sensor_msgs/PointCloud2```](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html), [```sensor_msgs/LaserScan```](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html)

# Quickstart

## Installation
```
$ git clone git@github.com:xrkong/rosbag_plot.git
```

## Tutorial
```
$ python plot_ros2bag -h
```

# Roadmap

- [x] Deserialize from ros2 bag, (.db3 file)
- [x] Get data, like /lidar_safety/front_left/scan, /ins0/gps_pos, /ins0/orientation
- [x] Plot them
- [x] If there is a icon of the vechicle, add into a middile.
- [x] plot map

# Unit test 
## Utility function
1. Input a file path, validate the rosbag file. If failed, throw an error. 
1. Input a message list, check the message in the rosbag or not. 
1. Input a topic name, extract the message list. 
1. Input a topic name and a time stamp, return the closest message. 

## Map & Path
1. Input a rosbag and gps topic name, retun a html path file.
1. Input a rosbag and gps topic name, retun a csv path file.
1. Input a rosbag, regular gps topic name, and incident topic name, return a html incident path map.
1. Input a regular rosbag, and a incident rosbag, return a html incident path map.
1. Input two csv file (regular and incident), return a html incident path map. 
<!-- 2. Save Point cloud and Images to local. -->
<!-- 3. TODO. image and pcd will be inferenced by LLMs -->

## Sensors
1. Input a rosbag file and a timestamp, visualize pointcloud and images into a single file.
1. Input a rosbag file and a timestamp, save pointcloud and images into a single file.
1. Input a rosbag file, a timestamp, and a saving path, save all pointcloud and images to the saving path.
1. Input a rosbag, a image topics, and a saving path, extract images to the given path.
1. Input a rosbag, a pcd topics, and a saving path, extract point cloud files to the given path.


## Inferences (TODO)
1. Input a image, and a topic name, inference it.

