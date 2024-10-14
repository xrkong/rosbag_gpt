
import os
import cv2
import datetime
import numpy as np
import open3d as o3d
import sensor_msgs.msg
from cv_bridge import CvBridge
import tf_transformations
from sensor_msgs_py import point_cloud2 as pc2

'''
data frame from a parser
'''
class Frame():
    def __init__(self, parser, timestamp, ) -> None:
        try: 
            self.timestamp = timestamp

            # left priority in Australia
            self.scan_rr = parser.get_msg_frame("/lidar_safety/front_left/cloud", timestamp)
            self.scan_rl = parser.get_msg_frame("/lidar_safety/front_right/cloud", timestamp)
            self.scan_fr = parser.get_msg_frame("/lidar_safety/rear_left/cloud", timestamp)
            self.scan_fl = parser.get_msg_frame("/lidar_safety/rear_right/cloud", timestamp)
            self.vld_rear = parser.get_msg_frame("/lidar/velodyne/front/cloud", timestamp)
            self.vld_front = parser.get_msg_frame("/lidar/velodyne/rear/cloud", timestamp)

            self.cam_front = parser.get_msg_frame("/CameraFront", timestamp)
            self.cam_rear  = parser.get_msg_frame("/CameraRear", timestamp)

            self.ori = parser.get_msg_frame("/imu/data", timestamp) #ori[times]
            self.gps_pos = parser.get_msg_frame("/sbg/gps_pos", timestamp)
            self.gps_vel = parser.get_msg_frame("/sbg/gps_vel", timestamp)

            # self.battery_percent = parser.get_messages("/battery_percent")
            # self.battery_voltage = parser.get_messages("/battery_voltage")
            # self.print_msg_timestamp()
            # self.print_bus_status()
            dt = datetime.datetime.utcfromtimestamp(self.timestamp / 1e9)
            self.time_str = dt.strftime("%y-%m-%d-%H:%M:%S.%f")[:-3]
        except TypeError as e:
            print(f"Error: {e}")
            return None
    
    def print_time(self, timestamp, name:str): 
        try:
            secs = timestamp.sec + timestamp.nanosec / 1e9
            dt = datetime.datetime.utcfromtimestamp(secs)
            time_str = name + ": " + dt.strftime("%y-%m-%d-%H:%M:%S.%f")[:-3]
            print(time_str)
        except TypeError as e:
            print(f"Error: {e}")
            return None
    
    def print_msg_timestamp(self):
        fl = self.scan_fl[1].header.stamp
        fr = self.scan_fr[1].header.stamp
        rl = self.scan_rl[1].header.stamp
        rr = self.scan_rr[1].header.stamp
        ori=self.ori[1].header.stamp
        gps_pos=self.gps_pos[1].header.stamp
        gps_vel=self.gps_vel[1].header.stamp
        self.print_time(fl, "fl")
        self.print_time(fr, "fr")
        self.print_time(rl, "rl")
        self.print_time(rr, "rr")
        self.print_time(ori, "ori")
        self.print_time(gps_pos, "gps_pos")
        self.print_time(gps_vel, "gps_vel")

    def print_location(self):
        quaternion = [self.ori[1].orientation.x, 
                      self.ori[1].orientation.y, 
                      self.ori[1].orientation.z, 
                      self.ori[1].orientation.w]

        # Convert quaternion to Euler angles
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(quaternion)
        lat = self.gps_pos[1].latitude
        lon = self.gps_pos[1].longitude

        # Now you can use roll, pitch, and yaw
        print("Roll:", roll)
        print("Pitch:", pitch)
        print("Yaw:", yaw)
        print("Lat:{:.7f}, Lon:{:.7f}".format(lat, lon))
        return [yaw, lat, lon]
    
    def print_battery(self):
        print("Battery percent: ", self.battery_percent[1].data)
        print("Battery voltage: ", self.battery_voltage[1].data)
        return [self.battery_percent[1].data, self.battery_voltage[1].data]

    def save_camera_image(self, out_dir:str):
        bridge = CvBridge()
        if self.cam_front is not None:
            cv_img_front = bridge.imgmsg_to_cv2(
                self.cam_front[1], 
                desired_encoding='passthrough'
                )
            front_output_path = os.path.join(out_dir, 'front.png')
            cv2.imwrite(front_output_path, cv_img_front)
            print("Image saved: front")
        else:
            front_output_path = None
            print("No front camera data")
        
        if self.cam_rear is not None:
            cv_img_rear = bridge.imgmsg_to_cv2(
                self.cam_rear[1], 
                desired_encoding='passthrough'
                )
            rear_output_path = os.path.join(out_dir, 'rear.png')
            cv2.imwrite(rear_output_path, cv_img_rear)
            print("Image saved: rear")
        else:
            rear_output_path = None
            print("No rear camera data")
        
        return [front_output_path, rear_output_path]
    
    def msg_to_o3d_pcd(self, msg : sensor_msgs.msg.PointCloud2):
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        # points_array = list(points)
        points_list = np.array([[p[0], p[1], p[2]] for p in points], dtype=np.float32)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_list[:, :3])
        return pcd
    
    def save_pcd(self, 
                 output_path : str, 
                 pcd : sensor_msgs.msg.PointCloud2, 
                 topic_name : str="point_cloud"):
        pcd = self.msg_to_o3d_pcd(pcd)
        # Save the point cloud as a PCD file
        filename = os.path.join(output_path, topic_name+".pcd")
        o3d.io.write_point_cloud(filename, pcd)
        print(f"{pcd} saved to {filename}")
        return [filename, pcd]
    
    def save_pcd2png(self, 
                     output_path : str, 
                     pcd_msg : sensor_msgs.msg.PointCloud2, 
                     output_name : str="point_cloud"):
        
        pcd_o3d = self.msg_to_o3d_pcd(pcd_msg) 
        # pcd = o3d.io.read_point_cloud(o3d_pcd)
        points = np.asarray(pcd_o3d.points)
        XMAX = np.max(points[:, 0])
        YMAX = np.max(points[:, 1])
        ZMAX = np.max(points[:, 2])  

        # Create a visualizer object
        vis = o3d.visualization.Visualizer()
        vis.create_window()

        # Add the point cloud to the visualizer
        vis.add_geometry(pcd_o3d)
        ctr = vis.get_view_control()

        bbox = pcd_o3d.get_axis_aligned_bounding_box()
        center = bbox.get_center()

        ctr.set_front([0,0,1])  
        ctr.set_lookat(center)        
        ctr.set_up([1,0,0])         
        ctr.set_zoom(0.5)  

        # Visualize the point cloud with the adjusted camera position
        # vis.run()
        output = os.path.join(output_path, output_name+".png")
        vis.capture_screen_image(output, do_render=True)
        vis.destroy_window()

        return output