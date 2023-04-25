#!/usr/bin/env python3
import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge
from mcap.reader import make_reader

import cv2
import matplotlib.pyplot as plt
import numpy as np
import random
from PIL import Image
import datetime
import math
import os

'''
Ros2bag file parser factory class
'''
class BagFileParserFactory():
    def __init__(self, bag_file:str): 
        self.bag_file = bag_file
        self.parser = None
        self.file_type = None
        self.file_type = self.get_file_type()
        print("start parsing...")
        if self.file_type == "db3":
            self.parser = db3Parser(self.bag_file)
        elif self.file_type == "mcap":
            self.parser = mcapParser(self.bag_file)
        print("parsing done")
    
    def get_file_type(self):
        if self.bag_file.split(".")[-1]=="db3":
            return "db3"
        elif self.bag_file.split(".")[-1]=="mcap":
            return "mcap"
        else:
            print("Error: unknown file type")
            exit(1)
    
    def get_parser(self):
        return self.parser

'''
Load .db3 ros2bag file 
'''
class db3Parser():
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
Load .mcap ros2bag file 
'''
class mcapParser():
    def __init__(self, file:str) -> None:
        self.reader =  make_reader(open(file, "rb"))

    def __del__(self):
        pass
    
    def get_messages(self, topic_name:str):
        print(f"get_messages: {topic_name}")
        reader = self.reader
        messages = []
        for msg in reader.iter_messages(topics=[topic_name]):
            topic = deserialize_message(msg[2].data, get_message(msg[0].name))
            timestamp = topic.header.stamp.sec * 1e9 + topic.header.stamp.nanosec
            messages.append([timestamp, topic])
        print(f"get_messages: {len(messages)}")
        return messages

    def get_msg_frame(self, topic_name:str, timestamp):
        print(f"get_msg_frame: {topic_name}")
        reader = self.reader
        messages = []
        for msg in reader.iter_messages(topics=[topic_name], start_time=timestamp - 5e8, end_time=timestamp + 5e8):
            topic = deserialize_message(msg[2].data, get_message(msg[0].name))
            cur_timestamp = topic.header.stamp.sec * 1e9 + topic.header.stamp.nanosec
            messages.append([cur_timestamp, topic])
        print(f"get_msg_frame: {len(messages)}")
        if len(messages) == 0:
            return None
        return messages[math.floor(len(messages)/2)]

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
            self.cam_front = parser.get_msg_frame("/CameraFront", timestamp)
            self.cam_rear  = parser.get_msg_frame("/CameraRear", timestamp)
            self.ori = parser.get_msg_frame("/ins0/orientation", timestamp) #ori[times]
            self.gps_pos = parser.get_msg_frame("/ins0/gps_pos", timestamp) # gps_pos[ts][1].position.SLOT_TYPES.x
            self.gps_vel = parser.get_msg_frame("/sbg/gps_vel", timestamp)
            self.print_msg_timestamp()
            self.print_bus_status()
            dt = datetime.datetime.utcfromtimestamp(self.timestamp / 1e9)
            self.time_str = dt.strftime("%y-%m-%d-%H:%M:%S.%f")[:-3]
        except TypeError as e:
            print(f"Error: {e}")
            return None
        
    def save_frame(self, abs_path:str):
        '''
        TODO: save image and data as a file
        '''
        fl = self.scan_fl[1]
        fr = self.scan_fr[1]
        rl = self.scan_rl[1]
        rr = self.scan_rr[1]

        r = [x/ 10 for x in fl._ranges ]
        theta = np.linspace(fl._angle_min, fl._angle_max, len(r), endpoint=False) 
        fig = plt.figure()
        axs = fig.add_subplot(1, 1, 1, projection='polar')

        axs.plot(theta, r, linewidth=0) # no line
        # Fill the area enclosed by the line
        axs.fill(theta, r, alpha=0.7) # fill area
        axs.set_theta_zero_location('NW') 
        plt.axis('off')
        if not os.path.exists(abs_path):
            os.makedirs(abs_path)         
        plt.savefig(abs_path+'lidar_safety_fl.png', dpi=300, bbox_inches='tight', transparent=True)
        fig.clf()

        r = [x/ 10 for x in fr._ranges ]
        theta = np.linspace(fr._angle_min, fr._angle_max, len(r), endpoint=False) 
        axs = fig.add_subplot(1, 1, 1, projection='polar')
        axs.plot(theta, r, linewidth=0) # no line
        axs.fill(theta, r, alpha=0.7) # fill area
        axs.set_theta_zero_location('NE')
        plt.axis('off')       
        plt.savefig(abs_path+'lidar_safety_fr.png', dpi=300, bbox_inches='tight', transparent=True)
        fig.clf()

        r = [x/ 10 for x in rl._ranges ]
        theta = np.linspace(rl._angle_min, rl._angle_max, len(r), endpoint=False)
        axs = fig.add_subplot(1, 1, 1, projection='polar')
        axs.plot(theta, r, linewidth=0) # no line
        axs.fill(theta, r, alpha=0.7) # fill area
        axs.set_theta_zero_location('SW')
        plt.axis('off')
        plt.savefig(abs_path+'lidar_safety_rl.png', dpi=300, bbox_inches='tight', transparent=True)
        fig.clf()

        r = [x/ 10 for x in rr._ranges ]
        theta = np.linspace(rr._angle_min, rr._angle_max, len(r), endpoint=False)
        axs = fig.add_subplot(1, 1, 1, projection='polar')
        axs.plot(theta, r, linewidth=0) # no line
        axs.fill(theta, r, alpha=0.7) # fill area
        axs.set_theta_zero_location('SE')
        plt.axis('off')
        plt.savefig(abs_path+'lidar_safety_rr.png', dpi=300, bbox_inches='tight', transparent=True)
    
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

    def plot_all(self):
        #fig, axs = plt.subplots(2, 3, figsize=(10, 10), subplot_kw=dict(polar=True))
        fig = plt.figure()
        fl = self.scan_fl[1]
        fr = self.scan_fr[1]
        rl = self.scan_rl[1]
        rr = self.scan_rr[1]

        r = [x for x in fl._ranges ]
        theta = np.linspace(fl._angle_min, fl._angle_max, len(r), endpoint=False) 
        axs = fig.add_subplot(2, 3, 1, projection='polar')
        axs.plot(theta, r, linewidth=0) # no line
        # Fill the area enclosed by the line
        axs.fill(theta, r, alpha=0.7) # fill area
        axs.set_theta_zero_location('NW') 
        axs.set_title('front left', fontweight='bold')
        axs.set_ylim(0, 50)
        axs.set_rlabel_position(0) 
        axs.set_theta_direction(-1) # clockwise axis Z is down

        r = [x for x in fr._ranges ]
        theta = np.linspace(fr._angle_min, fr._angle_max, len(r), endpoint=False) 
        axs = fig.add_subplot(2, 3, 3, projection='polar')
        axs.plot(theta, r, color='r', linewidth=0)
        axs.fill(theta, r, alpha=0.7)
        axs.set_theta_zero_location('NE')
        axs.set_title('front right', fontweight='bold')
        axs.set_ylim(0, 50)
        axs.set_rlabel_position(0)
        axs.set_theta_direction(-1)

        r = [x for x in rl._ranges ]
        theta = np.linspace(rl._angle_min, rl._angle_max, len(r), endpoint=False) 
        axs = fig.add_subplot(2, 3, 4, projection='polar')
        axs.plot(theta, r, color='r', linewidth=0)
        # Fill the area enclosed by the line
        axs.fill(theta, r, alpha=0.7)
        axs.set_theta_zero_location('SW')
        axs.set_title('rear left', fontweight='bold')
        axs.set_ylim(0, 50)
        axs.set_rlabel_position(0)        
        axs.set_theta_direction(-1)

        r = [x for x in rr._ranges ]
        theta = np.linspace(rr._angle_min, rr._angle_max, len(r), endpoint=False) 
        axs = fig.add_subplot(2, 3, 6, projection='polar')
        axs.plot(theta, r, color='r', linewidth=0.0)
        # Fill the area enclosed by the line
        axs.fill(theta, r, alpha=0.7)
        axs.set_theta_zero_location('SE')
        axs.set_ylim(0, 50)
        axs.set_rlabel_position(0)
        axs.set_title('rear right', fontweight='bold')
        axs.set_theta_direction(-1)

        bridge = CvBridge()
        if self.cam_front is not None:
            cv_img_front = bridge.imgmsg_to_cv2(self.cam_front[1], desired_encoding='passthrough')
            img_front = cv2.cvtColor(cv_img_front, cv2.COLOR_BGR2RGB)
            axs = fig.add_subplot(2, 3, 2)
            axs.cla()
            axs.imshow(img_front)
            
        if self.cam_rear is not None:
            cv_img_rear = bridge.imgmsg_to_cv2(self.cam_rear[1], desired_encoding='passthrough')
            img_rear = cv2.cvtColor(cv_img_rear, cv2.COLOR_BGR2RGB)
            axs = fig.add_subplot(2, 3, 5)
            axs.cla()
            axs.imshow(img_rear)
        fig.suptitle(self.time_str)
        plt.savefig('plot_'+self.time_str+'.png', dpi=300, bbox_inches='tight')
        plt.show()

'''
Map class
[ ] Get all path point based on rosbag
[ ] Get a frame data from Frame class
[ ] Attach lidar images with a bus icon ori = 0, save as a new image
[ ] Add a bus icon based on the frame data with a given oriention
'''
class Map:
    def  __init__(self):
        self.path = None
    
    def draw_bus_lidar(self, lidar_path:str):
        bus =Image.open(os.path.dirname(os.path.abspath(__file__))+"/bus.png").convert("RGBA")
        lidar_img = {"fl":Image.open(lidar_path+"lidar_safety_fl.png").convert("RGBA").rotate(90), 
                     "fr":Image.open(lidar_path+"lidar_safety_fr.png").convert("RGBA").rotate(90),
                     "rl":Image.open(lidar_path+"lidar_safety_rl.png").convert("RGBA").rotate(90),
                     "rr":Image.open(lidar_path+"lidar_safety_rr.png").convert("RGBA").rotate(90)}
        bus_w, bus_h = bus.size #w1072 h1500
        lidar_w, lidar_h = lidar_img["fl"].size #h1168 w1168
        bus.paste(lidar_img["fl"], (0, 0), lidar_img["fl"])
        # bus.paste(lidar_img["fr"], ((bus_w-lidar_w)//2, 0), lidar_img["fr"])
        # bus.paste(lidar_img["rl"], (0, (bus_h-lidar_w)), lidar_img["rl"])
        # bus.paste(lidar_img["rr"], ((bus_w), (bus_h-lidar_h)), lidar_img["rr"])
        bus.save("demo.png")
        

    def draw_path(self, path):
        pass

    def draw_frame(self, frame:Frame):
        pass
        

if __name__ == "__main__":
    parser = mcapParser("/home/kong/Documents/rosbag2.mcap")
    # gps_list = parser.get_messages("/ins0/gps_pos")

    test = Frame(parser, 1681451133151763439) #gps_list[1000][0])
    test.plot_all()
    test.save_frame("./test_frame/")

    map = Map()
    map.draw_bus_lidar("./test_frame/")

