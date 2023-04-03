In order to plot a set of laserscan at a time from a big rosbag file (or db3 file).

- Deserialize from ros2 bag, (.db3 file)
- Get the interesting data, like /lidar_safety/front_left/scan, /ins0/gps_pos, /ins0/orientation
- Plot them
- If there is a icon of the vechicle, add into a middile.
- plot map

plot a 3*2 subfig should look like this 
0,0 front left scan 
0,1 camera front 
0,2 front right scan 
1,0 rear left scan 
1,1 camera rear 
1,2 rear right scan 

plot the bus icon on another plot considering orientation

