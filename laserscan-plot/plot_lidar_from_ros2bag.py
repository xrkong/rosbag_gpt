import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan

import matplotlib.pyplot as plt
import numpy as np
import random
from PIL import Image
import datetime
import math
import os

'''
Load ros2bag file 
'''
class BagFileParser():
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        ## create a message type map
        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of:type_of for id_of,name_of,type_of in topics_data}
        self.topic_id = {name_of:id_of for id_of,name_of,type_of in topics_data}
        self.topic_msg_message = {name_of:get_message(type_of) for id_of,name_of,type_of in topics_data}

    def __del__(self):
        self.conn.close()

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def get_messages(self, topic_name):
        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
        # Deserialise all and timestamp them
        return [ (timestamp,deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp,data in rows]
    
    # Return [(timestamp0, message0)] at time_stamp
    def get_msg_frame(self, topic_name, timestamp):
        topic_id = self.topic_id[topic_name]
        try:
            rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {} AND ABS(timestamp - {}) < 5E8".format(topic_id, timestamp)).fetchall()
            return (rows[round(len(rows)/2)][0], deserialize_message(rows[round(len(rows)/2)][1], self.topic_msg_message[topic_name])) # get the middle data from rows.# round(len(rows)/2)
        except TypeError as e:
            print(f"Error: {e}")
            return None
        # Deserialise all and timestamp them

'''
data frame from a parser
'''
class Frame():
    def __init__(self, parser, timestamp) -> None:
        try: 
            self.timestamp = timestamp
            self.scan_fl = parser.get_msg_frame("/lidar_safety/front_left/scan", timestamp)
            self.scan_fr = parser.get_msg_frame("/lidar_safety/front_right/scan", timestamp)
            self.scan_rl = parser.get_msg_frame("/lidar_safety/rear_left/scan", timestamp)
            self.scan_rr = parser.get_msg_frame("/lidar_safety/rear_right/scan", timestamp)
            self.ori = parser.get_msg_frame("/ins0/orientation", timestamp) #ori[times]
            self.gps_pos = parser.get_msg_frame("/ins0/gps_pos", timestamp) # gps_pos[ts][1].position.SLOT_TYPES.x
            self.gps_vel = parser.get_msg_frame("/sbg/gps_vel", timestamp)
            self.print_msg_timestamp()
            self.print_bus_status()
        except TypeError as e:
            print(f"Error: {e}")
            return None
    
    def plot_all(self):
        '''
        (TODO) plot all information that frame obj has. 
        If any data is empty, ignore it.
        - print_msg_timestamp
        - plot bus status using icon
        - plot lidar_safety
        - plot camera
        '''
        pass
        
    def save_frame(self, abs_path:str):
        '''
        (TODO) save image and data as a file
        '''
        pass
    
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

    def print_bus_status(self):
        ori=self.ori[1].angle.x
        gps_pos=self.gps_pos[1].position#x,y,z
        gps_vel=self.gps_vel[1].vel #x,y,z
        print("ori:{:.1f}, lat:{:.7f}, lon:{:.7f}, vel:{:.3f}".format(math.degrees(ori), gps_pos.x, gps_pos.y, gps_vel.x))

    def plot_lidar(self):
        fig, axs = plt.subplots(2, 2, figsize=(10, 10), subplot_kw=dict(projection='polar'))
        fl = self.scan_fl[1]
        fr = self.scan_fr[1]
        rl = self.scan_rl[1]
        rr = self.scan_rr[1]

        r = [x/ 10 for x in fl._ranges ]
        theta = np.linspace(fl._angle_min, fl._angle_max, len(r), endpoint=False) 
        axs[0,0].plot(theta, r, color='r', linewidth=1)
        # Fill the area enclosed by the line
        axs[0,0].fill(theta, r, alpha=0.2)
        axs[0,0].set_theta_zero_location('NW')
        axs[0,0].set_title('front left', fontweight='bold')

        r = [x/ 10 for x in fr._ranges ]
        theta = np.linspace(fr._angle_min, fr._angle_max, len(r), endpoint=False) 
        axs[0,1].plot(theta, r, color='r', linewidth=1)
        # Fill the area enclosed by the line
        axs[0,1].fill(theta, r, alpha=0.2)
        axs[0,1].set_theta_zero_location('NE')
        axs[0,1].set_title('front right', fontweight='bold')

        r = [x/ 10 for x in rl._ranges ]
        theta = np.linspace(rl._angle_min, rl._angle_max, len(r), endpoint=False) 
        axs[1,0].plot(theta, r, color='r', linewidth=1)
        # Fill the area enclosed by the line
        axs[1,0].fill(theta, r, alpha=0.2)
        axs[1,0].set_theta_zero_location('SW')
        axs[1,0].set_title('rear left', fontweight='bold')

        r = [x/ 10 for x in rr._ranges ]
        theta = np.linspace(rr._angle_min, rr._angle_max, len(r), endpoint=False) 
        axs[1,1].plot(theta, r, color='r', linewidth=1)
        # Fill the area enclosed by the line
        axs[1,1].fill(theta, r, alpha=0.2)
        axs[1,1].set_theta_zero_location('SE')
        axs[1,1].set_title('rear right', fontweight='bold')

        for ax in axs.flat:
            ax.set_ylim(0, 5)
            ax.set_theta_direction(-1)
            ax.set_rlabel_position(0)
        dt = datetime.datetime.utcfromtimestamp(self.timestamp / 1e9)
        time_str = dt.strftime("%y-%m-%d-%H:%M:%S.%f")[:-3]
        fig.suptitle(time_str)
        plt.show()
        plt.pause(0.01)
    
    def plot_camera(self):
        '''
        (TODO) use try-exception to plot camera front and rear if they exist.
        
        
        '''
        pass

if __name__ == "__main__":
    # (TODO) input rosbag as an argument
    bag_file = '/home/kong/ws/rosbag2_2023_03_22-10_13_05/rosbag2_2023_03_22-10_13_05_0.db3'
    parser = BagFileParser(bag_file)

    # choose a topic as a time line and get a timestamp at topic_list[index][0]
    # (TODO) add function to support input a time like (MM-DD-hh:mm:ss)
    scan_list = parser.get_messages("/lidar_safety/front_left/scan")

    test = Frame(parser, scan_list[1000][0])
    test.plot_lidar()