# 🗺️ plot rosbag NOW!

A tool to visualise an autonomous vheicle data including path, lidar and camera from a ros2bag file (support '.db3' and '.mcap' format).   

## rosbag plot
Plot data including gps, LiDAR scan, point clouds, images etc from .mcap and .db3 rosbag.

(image
### install
```
$  pip install xxx
```
### tutorial

### roadmap
plot pop up on map with plot images and given csv.  
    --popup /path/to/folder --csv /path/to/file
support PointCloud.  
path type incident/regular. 
package to pip.  


## csv plot
If you already have some csv files with longitude and latitude, you can use this tool to plot with customized vehicle icon and pop up interesting subpages.

(image


In order to plot a set of laserscan at a time from a big rosbag file (or db3 file).

- [x] Deserialize from ros2 bag, (.db3 file)
- [x] Get data, like /lidar_safety/front_left/scan, /ins0/gps_pos, /ins0/orientation
- [x] Plot them
- [x] If there is a icon of the vechicle, add into a middile.
- [x] plot map

plot a 3*2 subfig should look like this.    
0,0 front left scan   
0,1 camera front   
0,2 front right scan   
1,0 rear left scan   
1,1 camera rear    
1,2 rear right scan   

plot the bus icon on another plot considering orientation

Not all the function only support .db3 file   
Since we change the storge format into mcap   
It is better to use .mcap as a ros2bag format.   

Support .mcap and .db3 file format
