In order to plot a set of laserscan at a time from a big rosbag file (or db3 file).

- [x] Deserialize from ros2 bag, (.db3 file)
- [x] Get data, like /lidar_safety/front_left/scan, /ins0/gps_pos, /ins0/orientation
- [x] Plot them
- [ ] If there is a icon of the vechicle, add into a middile.
- [ ] plot map

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

(TODO) Support .mcap file format
