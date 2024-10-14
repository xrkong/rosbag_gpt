
import os
import cv2
import argparse
import numpy as np
import pandas as pd
from cv_bridge import CvBridge
from src.bag_parser import BagFileParserFactory

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