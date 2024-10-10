#!/usr/bin/env python3
import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import sensor_msgs.msg
from cv_bridge import CvBridge
from mcap.reader import make_reader
import tf_transformations
from sensor_msgs_py import point_cloud2 as pc2
import pdfkit
from jinja2 import Environment, FileSystemLoader
import open3d as o3d
import time
import json
# from typing import Dict, List, Union
from openai import OpenAI
from dotenv import find_dotenv, load_dotenv
# import pclpy 
# from pclpy import pcl

_ = load_dotenv(find_dotenv())

import yaml
import sys
import cv2
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import datetime
import math
import os
import argparse
import pandas as pd
from io import BytesIO
import folium
import webbrowser
import base64


'''
Ros2bag file parser factory class
'''
class BagFileParserFactory:
    def __init__(self, bag_file: str, duration: int = None):
        self.bag_file = bag_file
        self.file_type = self.get_file_type()
        self.parser = None
        self.load_metadata()

        if self.file_type == "db3":
            self.parser = Db3Parser(self.bag_file, duration)
        elif self.file_type == "mcap":
            self.parser = McapParser(self.bag_file, duration)
        else:
            raise ValueError("Unsupported file type.")

    def get_file_type(self) -> str:
        if self.bag_file.endswith(".db3"):
            return "db3"
        elif self.bag_file.endswith(".mcap"):
            return "mcap"
        else:
            raise ValueError("Unsupported file type.")

    def load_metadata(self):
        metadata_file = os.path.join(os.path.dirname(self.bag_file), "metadata.yaml")
        with open(metadata_file, 'r') as stream:
            self.metadata = yaml.safe_load(stream)
            self.start_time = self.metadata['rosbag2_bagfile_information']["starting_time"]['nanoseconds_since_epoch']
            self.duration = self.metadata['rosbag2_bagfile_information']["duration"]['nanoseconds']

    def get_parser(self):
        return self.parser

'''
Load .db3 ros2bag file 
'''
class Db3Parser:
    def __init__(self, bag_file: str, duration: int = 5e6):
        self.duration = duration
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()
        self.load_topics()

    def __del__(self):
        self.conn.close()

    def load_topics(self):
        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name: type for id, name, type in topics_data}
        self.topic_id = {name: id for id, name, type in topics_data}
        self.topic_msg_message = {name: get_message(type) for id, name, type in topics_data}

    def get_messages(self, topic_name: str):
        topic_id = self.topic_id[topic_name]
        rows = self.cursor.execute(f"SELECT timestamp, data FROM messages WHERE topic_id = {topic_id}").fetchall()
        return [(timestamp, deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp, data in rows]

    def get_msg_frame(self, topic_name: str, timestamp: int):
        topic_id = self.topic_id[topic_name]
        rows = self.cursor.execute(f"SELECT timestamp, data FROM messages WHERE topic_id = {topic_id} AND ABS(timestamp - {timestamp}) < 5E8").fetchall()
        if rows:
            return (rows[round(len(rows)/2)][0], deserialize_message(rows[round(len(rows)/2)][1], self.topic_msg_message[topic_name]))
        return None

'''
Load .mcap ros2bag file 
'''
class McapParser:
    def __init__(self, bag_file: str, duration: int = 1e12):
        self.reader = make_reader(open(bag_file, "rb"))
        self.duration = duration

    def get_messages(self, topic_name: str, start_time=None, end_time=None):
        messages = []
        for msg in self.reader.iter_messages(topics=[topic_name], start_time=start_time, end_time=end_time):
            topic = deserialize_message(msg[2].data, get_message(msg[0].name))
            timestamp = topic.header.stamp.sec * 1e9 + topic.header.stamp.nanosec
            messages.append([timestamp, topic])
        return messages if messages else None

    def get_msg_frame(self, topic_name: str, timestamp):
        messages = []
        for msg in self.reader.iter_messages(topics=[topic_name], start_time=timestamp - self.duration / 2, end_time=timestamp + self.duration / 2):
            topic = deserialize_message(msg[2].data, get_message(msg[0].name))
            cur_timestamp = topic.header.stamp.sec * 1e9 + topic.header.stamp.nanosec
            messages.append([cur_timestamp, topic])
        return messages[math.floor(len(messages) / 2)] if messages else None

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

'''
Map class
'''
class Map:
    def __init__(self, 
                 zoom_start:int=17, 
                 output_path:str=None, 
                 path_waypoints:np.array=None,  
                 incident_waypoints:np.array=None, 
                 bus_frame:Frame=None): 
        self.zoom_start = zoom_start # for folium map
        self.path = np.array([[0,0]]) 

        if path_waypoints is not None:
            self.path = path_waypoints
        elif bus_frame is not None:
            self.path = [[bus_frame.gps_pos[1].longitude, 
                          bus_frame.gps_pos[1].latitude]]

        if incident_waypoints is not None:
            self.stop = incident_waypoints
        else:
            self.stop = None
        
        self.pic_center = np.mean(self.path, 0) 
        self.output_path = output_path
        self.map = folium.Map(location = [-31.595436, 115.662379], 
                              zoom_start = self.zoom_start) # -31.595436, 115.662379 for Eglinton
        self.frame = bus_frame
    
    def draw_bus_lidar(self, lidar_path:str):
        oritation = self.frame.ori[1].angle.z
        gps_pos = [self.frame.gps_pos[1].longitude, self.frame.gps_pos[1].latitude]
        bus =Image.open(os.path.dirname(os.path.abspath(__file__))+"/bus.png").convert("RGBA")
        bus_w, bus_h = bus.size 

        lidar_img = {"fl":Image.open(lidar_path+"lidar_safety_fl.png").convert("RGBA"), 
                     "fr":Image.open(lidar_path+"lidar_safety_fr.png").convert("RGBA"),
                     "rl":Image.open(lidar_path+"lidar_safety_rl.png").convert("RGBA"),
                     "rr":Image.open(lidar_path+"lidar_safety_rr.png").convert("RGBA")}
        lidar_w, lidar_h = lidar_img["fl"].size 
        
        bus_lidar = {"fl":(2720, 2490), "fr":(3220, 2490), "rl":(2720, 3450), "rr":(3220, 3450)}

        bus.paste(lidar_img["fl"], (bus_lidar["fl"][0]-lidar_w//2, bus_lidar["fl"][1]-lidar_h//2), lidar_img["fl"])
        bus.paste(lidar_img["fr"], (bus_lidar["fr"][0]-lidar_w//2, bus_lidar["fr"][1]-lidar_h//2), lidar_img["fr"])
        bus.paste(lidar_img["rl"], (bus_lidar["rl"][0]-lidar_w//2, bus_lidar["rl"][1]-lidar_h//2), lidar_img["rl"])
        bus.paste(lidar_img["rr"], (bus_lidar["rr"][0]-lidar_w//2, bus_lidar["rr"][1]-lidar_h//2), lidar_img["rr"])

        bus.save("output/bus_demo.png") # save the original bus image
        bus = bus.rotate((-oritation)/math.pi*180+180).save("output/demo-ori.png")
        if gps_pos is not None:
            self.pic_center = gps_pos
        # else: self.pic_center at UWA

        icon_path = os.path.join(os.getcwd(),"output/demo-ori.png")
        icon = folium.features.CustomIcon(
            icon_image=icon_path,
            icon_size=(bus_w//20, bus_h//20)
            )
        folium.Marker(gps_pos, icon=icon).add_to(self.map)

    def draw_path(self):
        if self.map is not None and self.path is not None:
            self.pic_center = np.mean(self.path, 0)
            for i in range(len(self.path)-2):
                # if math.dist(self.path[i], self.path[i+1]) < 0.0002: # 0.0002 ~20meters
                folium.PolyLine(locations=self.path[i:i+2],     
                                color="blue",
                                weight=2, ).add_to(self.map)           
        #self.stop
        if self.map is not None and self.stop is not None:
            for i in range(len(self.stop)):
                folium.CircleMarker(location=self.stop[i],     
                                    radius=10,
                                    weight=3,
                                    fill=False,
                                    fill_opacity=0.6,
                                    opacity=1,
                                    color="red",).add_to(self.map)
        # else:
        #     print("No path data")

    def show_html_map(self):
        self.map.save(self.output_path)
        webbrowser.open(self.output_path)

    def save_html_map(self):
        self.map.save(self.output_path)
        return self.output_path

    def save_csv_map(self):
        if self.stop:
            csv_path = os.path.splitext(self.output_path)[0]+'_inc.csv'
        else:
            csv_path = os.path.splitext(self.output_path)[0]+'_gps.csv'
        np.savetxt(
            csv_path, 
            self.path, 
            delimiter=',', 
            fmt='%s', 
            header="Latitude,Longitude", 
            comments=''
            )
        return csv_path


# class Incident():
#     def __init__(self, 
#                     timestamp : float = 0.0, 
#                     gps_pos = None, 
#                     image = None, 
#                     pcd = None,
#                     explanation : str = None):
#         self.timestamp = timestamp
#         self.gps_pos = gps_pos
#         self.image = image
#         self.pcd = pcd
#         self.explanation = explanation

'''
Gnereate report based on Map() and Frame()
'''
class Report():
    def __init__(
        self, 
        waypoint_rosbag_pathname : str,
        stop_rosbag_pathname : str, 
        output_dir : str,
        report_template_pathname : str = "./", 
        ):
        
        self.template_path = report_template_pathname
        self.waypoint_rosbag_pathname = waypoint_rosbag_pathname
        self.stop_rosbag_pathname = stop_rosbag_pathname
        self.output_dir = output_dir
        # [self.waypoint_map, 
        #  self.waypoint_pathname] = self.__add_map_from_rosbag(waypoint_rosbag_pathname,
        #                                                       stop_rosbag_pathname,
        #                                                       output_dir)
        self.history_path_content = "This image shows the overall path taken by the shuttle."
        self.stops_content = [] 
        self.stops_data = [] 
        # extract path 

    def __add_map_from_csv(
            self, path : str) -> list[Map, str]:
        # update self.history_path
        pass
        
    def __add_map_from_rosbag(
            self, 
            waypoint_rosbag_pathname : str,
            stop_rosbag_pathname : str, 
            output_dir : str
            )->list[Map, str]:
        # update self.history_path
        # output_dir = os.path.join(output_dir, "map.html")
        waypoints = extract_rosbag_path(waypoint_rosbag_pathname, "/sbg/gps_pos")
        stops = extract_rosbag_path(stop_rosbag_pathname, "/sbg/gps_pos")
        # output_path = "./unittest_file/output/test_two_bag2html.html"
        map = Map(17,
                  output_path=output_dir,
                  path_waypoints=waypoints,
                  incident_waypoints=stops)
        map.draw_path()
        map_pathname = map.save_html_map()
        return [map, map_pathname]

    def add_image_explanation(self, image_pathname:str, type:str):
        # image messages
        if type != "front" and type != "rear":
            return None
        
        system_prompt = """RAW TEXT ONLY. Using the provided front/rear camera images from an autonomous shuttle, generate a detailed disengagement description paragraph. Focus on the following points:
Use one sentence to analyze the visible environment, including any relevant road conditions, traffic (moving or stationary vehicles), pedestrians, and obstacles.
Use one sentence to describe the surrounding infrastructure (buildings, fences, signage) and road features (lanes, intersections, roundabouts).
Use one sentence Identify visible weather conditions (cloud cover, rain, etc.) and assess their potential impact on visibility or driving conditions.
Based on the scene, use two sentence to hypothesize possible reasons for system disengagement or manual intervention.
Specify if the image is from the front or rear camera to ensure proper context."""
        
        text = "this is the " + type + " camera image, generate a detailed disengagement description."

        def encode_image(image_pathname):
            with open(image_pathname, "rb") as image_file:
                return base64.b64encode(image_file.read()).decode('utf-8')

        messages = [
            {
            "role": "user",
            "content": [
                {
                "type": "text",
                "text": str(system_prompt + text)
                },
                {
                "type": "image_url",
                "image_url": {
                    "url": f"data:image/jpeg;base64,{encode_image(image_pathname)}"
                }
                }
            ]
            }
        ]


        # send prompt to llm

        client = OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))

        try:

            response = client.chat.completions.create(
                model="gpt-4o",
                messages=messages,
            )
            usage = response.usage
            content = response.choices[0].message.content
            return content
        except Exception as e:
            error_str = "Unable to generate ChatCompletion response, check your api."
            print(error_str)
            print(f"Exception: {e}")
            return error_str


    def __get_incident_data(self, stop_rosbag_pathname : str) -> list[Frame]:
        '''
        - input a rosbag pathname, return a Frame list. Update self.stops_data
        '''
        try: 
            bag_parser = BagFileParserFactory(stop_rosbag_pathname,duration=1e10)
        except IOError:
            print ('Cannot open file: ' 
                   + stop_rosbag_pathname 
                   + ', please check the rosbag file path')

        # get the timestamp list from gps_pos topic
        gps_list = bag_parser.get_parser().get_messages("/CameraFront")
        timestamps = [gps_list[i][0] for i in range(len(gps_list))]
        stop_timestamps = self.__estimate_incident_times(timestamps, 50) # 10 seconds

        for ts in stop_timestamps:
            stop = Frame(bag_parser.get_parser(), ts)
            self.stops_data.append(stop)

        return self.stops_data

    def __estimate_incident_times(self, timestamps:int, window_width:int) -> list[int]:
        '''
        - Estimate incident timestamps using snapshot rosbag and window width.
        - Methods
            - Sort the timestamp list
            - Group them with given window width
            - Return the last timestamp in each group.
        - Only called by self.__get_incident_data(...)
        '''
        # Convert input float timestamps to datetime objects (assuming they are in nanoseconds)
        timestamps = [datetime.datetime.fromtimestamp(ts / 1e9) for ts in timestamps]
        
        # Sort the timestamps in ascending order
        timestamps.sort()

        # Initialize variables
        groups = []
        current_group = []
        group_start = timestamps[0]

        # Group timestamps according to the window duration
        for ts in timestamps:
            if (ts - group_start).total_seconds() <= window_width:
                current_group.append(ts)
            else:
                # Store the last timestamp of the group
                last_timestamp = current_group[-1]
                groups.append(last_timestamp)

                # Start a new group
                current_group = [ts]
                group_start = ts
        
        # Handle the last group
        if current_group:
            last_timestamp = current_group[-1]
            groups.append(last_timestamp)
        
        # Convert the last timestamps back to float format (nanoseconds)
        groups = [int(dt.timestamp() * 1e9) for dt in groups]

        return groups

    def __extract_incident_frame(
            self, 
            data : Frame, 
            index : int) -> dict[str, str]:
        '''
        - Extract sensors data from given Frame  
        - Save them to output_dir/stop_<index>
        - Return a dictionary includes the pathnames
        - Called by self.generate_report(...) iteratively with __group_timestamps()

        required data: timestamp, gps_pos, image, pcd, explanation
        {
            "gps": "Latitude: -31.9505, Longitude: 115.8605",  
            "date_time": "2024-10-04 14:35:00",  
            "front_camera": front_image_path,  
            "front_lidar": front_pcd_path,  
            "rear_camera": rear_image_path,  
            "rear_lidar": rear_pcd_path,  
            "front_explanation": "This is the front camera view during the incident.",  
            "rear_explanation": "This is the rear camera view during the incident.",
        }
        '''
        out_pathname = os.path.join(self.output_dir, "stop_"+str(index))
        os.makedirs(out_pathname, exist_ok=True)

        [front_cam, rear_cam] = data.save_camera_image(out_pathname)

        rear_pcd = data.save_pcd2png(out_pathname, data.vld_rear[1], "velodyne_rear")
        front_pcd = data.save_pcd2png(out_pathname, data.vld_front[1], "velodyne_front")

        # [ ] Add inference add_explanation
        front_exp = self.add_image_explanation(front_cam, "front")
        rear_exp = self.add_image_explanation(rear_cam, "rear")

        out = {
                "gps": f"Latitude: {data.gps_pos[1].latitude:.5f}, Longitude: {data.gps_pos[1].longitude:.5f}",
                "date_time": data.time_str,
                "front_camera": front_cam,
                "rear_camera": rear_cam,
                "front_lidar": front_pcd,
                "rear_lidar": rear_pcd,
                "front_explanation": front_exp,
                "rear_explanation": rear_exp,
            }
        self.stops_content.append(out)
        return out

    def generate_report(self) -> None:
        # generate the report based on the template, and src files
        map_pathname = os.path.join(os.path.dirname(os.path.abspath(__file__)), "map.html")
        [_, map_pathname] = self.__add_map_from_rosbag(
                                self.waypoint_rosbag_pathname, 
                                self.stop_rosbag_pathname, 
                                map_pathname
                                )
        
        self.__get_incident_data(self.stop_rosbag_pathname)

        for stop in self.stops_data:
            self.__extract_incident_frame(stop, self.stops_data.index(stop))
            time.sleep(0.5)

        # Create a basic environment for the HTML report template
        env = Environment(loader=FileSystemLoader('./resources'))
        template = env.get_template('report_template.html')

        # [ ] Render the template with the data
        html_content = template.render(
            date_time='00-00-00', 
            path_html=map_pathname, 
            path_explanation=self.history_path_content,
            incidents=self.stops_content # based on stops_data
        )

        # Write the HTML content to a file (optional for debugging)
        with open('report.html', 'w') as file:
            file.write(html_content)

    def save_report(self, output_path : str):
        pass

def is_valid_timestamp(timestamp, duration):
    """
    Checks if the given timestamp (in seconds) is non-negative using a try-except approach.
    
    Args:
        timestamp (float): Timestamp in seconds to validate.
        
    Returns:
        tuple: (bool, str). True and an empty string if valid, 
              False and the error message otherwise.
    """
    try:
        if timestamp < 0:
            raise ValueError("Timestamp should be a non-negative value.")
        if timestamp > duration/1e9:
            raise ValueError(f"Timestamp should not exceed the maximum duration of {duration/1e9} seconds.")
        return (True, "")
    except ValueError as e:
        return (False, str(e))
    
def save_img(abs_path:str, img, name:str):
    bridge = CvBridge()
    New_Image_Save_Path=os.path.join(abs_path, "img")
    cv_img_front = bridge.imgmsg_to_cv2(img[1], desired_encoding='passthrough')
    output_path = os.path.join(New_Image_Save_Path, str(name)+'.png')
    cv2.imwrite(output_path, cv_img_front)
    print("Image saved: ", name)

def parse_args(args):
    arg_parser = argparse.ArgumentParser(description="ROS bag processing tool. By xrkong")
    # process rosbag file
    arg_parser.add_argument("rosbag_path", 
                        type=str, 
                        help="Path to the rosbag file, acceptable file types are .db3 and .mcap")
    arg_parser.add_argument("-g", "--gps-topic", 
                        type=str, 
                        default="/sbg/gps_pos",  
                        help="Name of the GPS topic in the rosbag. Defaults to '/gps'.")
    arg_parser.add_argument("-t", "--timestamp", 
                        type=float, 
                        help="Timestamp for the rosbag processing in seconds (e.g., 123.123456). If not provided, only draw path in rosbag.",
                        default=None)
    arg_parser.add_argument("-o", "--output-path", 
                        type=str, 
                        default="./output", 
                        help="Path to the output directory. If it doesn't exist, it will be created.")
    arg_parser.add_argument("-s", "--snapshot-path", 
                        type=str, 
                        default=None, 
                        help="Incident rosbag file path. If None draw nothing.")
    # process .csv file
    arg_parser.add_argument("-c", "--gps-csv-path", 
                        type=str, 
                        default=None, 
                        help="get the GPS csv file.")
    arg_parser.add_argument("-i", "--incident-csv-path", 
                        type=str, 
                        default=None, 
                        help="get the incident csv file.")
    return arg_parser.parse_args()

def extract_csv_path(file_path):
    # input a csv file, return the path waypoints
    gps_csv = pd.read_csv(file_path)
    gps_list = np.array(gps_csv[['Latitude', 'Longitude']])
    return gps_list

def extract_rosbag_path(file_path, waypoint_topic_name):
    # input a rosbag file and the waypoint topic name, return the path waypoints
    try: 
        bag_parser = BagFileParserFactory(file_path)
    except IOError:
        print ('Cannot open file: ' + file_path + ', please check the rosbag file path')
    gps_topic_list = bag_parser.get_parser().get_messages(waypoint_topic_name) #/sbg/ekf_nav /ins0/gps_pos gps_pos[1].longitude:lat, gps_pos[1].latitude:lon
    gps_list = np.array([[gps_topic_list[i][1].latitude , 
                          gps_topic_list[i][1].longitude] 
                            for i in range(len(gps_topic_list))])
    return gps_list

def extract_rosbag_images(file_path, image_topic_name):
    # input a rosbag file and the waypoint topic name, return the path waypoints
    try: 
        bag_parser = BagFileParserFactory(file_path)
    except IOError:
        print ('Cannot open file: ' + file_path + ', please check the rosbag file path')
    gps_topic_list = bag_parser.get_parser().get_messages(image_topic_name) #/sbg/ekf_nav /ins0/gps_pos gps_pos[1].longitude:lat, gps_pos[1].latitude:lon
    gps_list = np.array([[gps_topic_list[i][1].latitude , 
                          gps_topic_list[i][1].longitude] 
                            for i in range(len(gps_topic_list))])
    return gps_list

def main():
    print(sys.argv[1:])
    args = parse_args(sys.argv[1:])

    gps_pos_list = extract_csv_path(args.gps_csv_path)
    stop_gps_pos_list = extract_csv_path(args.incident_csv_path)

    # gps_csv = pd.read_csv(args.gps_csv_path)
    # gps_pos_list = [(gps_csv['Latitude'][i], gps_csv['Longitude'][i]) for i in range(len(gps_csv))]

    # incident_csv = pd.read_csv(args.incident_csv_path)
    # stop_gps_pos_list = [(incident_csv['Latitude'][i], incident_csv['Longitude'][i]) for i in range(len(incident_csv))]

    try:    
        bag_parser = BagFileParserFactory(args.rosbag_path)
    except IOError:
        print ('Cannot open file: ' + args.rosbag_path + ', please check the rosbag file path')

    # check output path
    output_path = args.output_path
    if not os.path.exists(output_path):
        try:
            os.makedirs(output_path)
            print(f"Output directory {output_path} created.")
        except OSError as e:
            print(f"Error creating directory {output_path}: {str(e)}")
            return
        
    # get the frame data
    if args.timestamp is not None:
        print("timestamp is input" )
        valid, error_message = is_valid_timestamp(args.timestamp, bag_parser.duration)
        if args.timestamp is not None and not valid:
            print(f"Error with the provided timestamp '{args.timestamp}': {error_message}")
            return
        frame_data = Frame(bag_parser.get_parser(), bag_parser.start_time + args.timestamp * 1e9)
        frame_data.plot_all(output_path)

    # get the snapshot
    stop_gps_pos_list = None
    if args.snapshot_path is not None:
        snapshot_path = args.snapshot_path
        print("Get incident rosbag, validate the file..." )
        if os.path.exists(snapshot_path) is False:
            print (' Cannot open the incident file: ', snapshot_path, ' Ignore it')

        try:    
            snapshot_parser = BagFileParserFactory(snapshot_path)
            stop_gps_pos_topic_list = snapshot_parser.get_parser().get_messages(args.gps_topic)
            stop_gps_pos_list = [(stop_gps_pos_topic_list[i][1].latitude , 
                             stop_gps_pos_topic_list[i][1].longitude) for i in range(len(stop_gps_pos_topic_list))]
        except IOError:
            print (' Cannot open the file, ignore the incident file: ', snapshot_path)
            pass

    # else no frame data, only draw path
    map_name = '/'+os.path.splitext(os.path.basename(args.rosbag_path))[0]
    gps_pos_topic_list = bag_parser.get_parser().get_messages(args.gps_topic) #/sbg/ekf_nav /ins0/gps_pos gps_pos[1].longitude:lat, gps_pos[1].latitude:lon
    gps_pos_list = [(gps_pos_topic_list[i][1].latitude , 
                     gps_pos_topic_list[i][1].longitude) for i in range(len(gps_pos_topic_list))]

    #timestamp = float(time_str) * 1e9
    #frame_data = Frame(parser.get_parser(), parser.start_time + timestamp) #1681451854074038952
    #frame_data = Frame(parser.get_parser(), timestamp) #1681451854074038952

    #frame_data.plot_all(output_path)
    #lidar_fig_path = frame_data.save_frame(output_path+"lidar_frame/")

    #map = Map(17, html_file_path, gps_pos_list)
    #map.draw_path(gps_pos_list)
    # all_joy = bag_parser.get_parser().get_messages("/joy") 
    # start_time = bag_parser.start_time
    # end_time = bag_parser.start_time + bag_parser.duration // 2
    # all_image = bag_parser.get_parser().get_messages("/CameraFront", 
    #                                                  start_time = start_time, 
    #                                                  end_time = end_time) 
    # img_index = 0
    # if all_image is None:
    #     print("No image data")
    #     return
    
    # for msg in all_image:
    #     save_img(output_path, msg, img_index)
    #     img_index = img_index + 1

    html_file_path = output_path+map_name+'.html'
    map = Map(17, html_file_path, gps_pos_list, stop_gps_pos_list)

    map.draw_path()
    map.show_html_map()
    map.save_csv_map()

if __name__ == "__main__":
    main()
