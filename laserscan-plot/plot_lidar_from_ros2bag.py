import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan

import matplotlib.pyplot as plt
import numpy as np
import random

import datetime

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

def plot_lidar(timestamp, scan_parser):
    # plot scan_fl[0]
    frame = scan_parser[timestamp]
    # for i in range(len(df[1].ranges)):
    r = [x/ 10 for x in frame[1]._ranges ]
    theta = np.linspace(frame[1]._angle_min, frame[1]._angle_max, len(r), endpoint=False) 
    return r, theta     

if __name__ == "__main__":

    bag_file = 'rosbag2_2023_03_22-10_13_05/rosbag2_2023_03_22-10_13_05_0.db3'

    parser = BagFileParser(bag_file)

    scan_fl = parser.get_messages("/lidar_safety/front_left/scan")
    scan_fr = parser.get_messages("/lidar_safety/front_right/scan")
    scan_rl = parser.get_messages("/lidar_safety/rear_left/scan")
    scan_rr = parser.get_messages("/lidar_safety/rear_right/scan")

    fig, axs = plt.subplots(2, 2, figsize=(10, 10), subplot_kw=dict(projection='polar'))

    #for i in range(len(scan_fl)-500): # use for loop to plot all lidar in this rosbag    
    index = random.randint(1, len(scan_fl))
    #index = i+500

    timestamp = scan_fl[index][1].header._stamp
    secs = timestamp.sec + timestamp.nanosec / 1e9
    dt = datetime.datetime.utcfromtimestamp(secs)
    time_str = dt.strftime("%y-%m-%d-%H:%M:%S")

    r, theta  = plot_lidar(index, scan_fl)
    # Create a polar plot
    axs[0,0].plot(theta, r, color='r', linewidth=1)
    # Fill the area enclosed by the line
    axs[0,0].fill(theta, r, alpha=0.2)
    axs[0,0].set_theta_zero_location('NW')
    axs[0,0].set_title('front left', fontweight='bold')

    r, theta  = plot_lidar(index, scan_fr)
    axs[0,1].plot(theta, r, color='r', linewidth=1)
    # Fill the area enclosed by the line
    axs[0,1].fill(theta, r, alpha=0.2)
    # Customize the plot
    axs[0,1].set_theta_zero_location('NE')
    axs[0,1].set_title('front right', fontweight='bold')

    r, theta  = plot_lidar(index, scan_rl)
    axs[1,0].plot(theta, r, color='r', linewidth=1)
    # Fill the area enclosed by the line
    axs[1,0].fill(theta, r, alpha=0.2)
    # Customize the plot
    axs[1,0].set_theta_zero_location('SW')
    axs[1,0].set_title('rear left', fontweight='bold')

    r, theta  = plot_lidar(index, scan_rr)
    axs[1,1].plot(theta, r, color='r', linewidth=1)
    # Fill the area enclosed by the line
    axs[1,1].fill(theta, r, alpha=0.2)
    # Customize the plot
    axs[1,1].set_theta_zero_location('SE')
    axs[1,1].set_title('rear right', fontweight='bold')

    for ax in axs.flat:
        ax.set_ylim(0, 5)
        ax.set_theta_direction(-1)
        ax.set_rlabel_position(0)

    # fig.suptitle(time_str)
    # plt.draw()
    # plt.pause(0.01)
    # #plt.cla()
    # for ax in axs.flat:
    #     ax.cla()

    plt.show()
