#!/usr/bin/env python3

from src.utils import *

import sys
import os
from frame import Frame
from path_map import PathMap

def main():
    print(sys.argv[1:])
    args = parse_args(sys.argv[1:])

    gps_pos_list = extract_csv_path(args.gps_csv_path)
    stop_gps_pos_list = extract_csv_path(args.incident_csv_path)

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
    map = PathMap(17, html_file_path, gps_pos_list, stop_gps_pos_list)

    map.draw_path()
    map.show_html_map()
    map.save_csv_map()

if __name__ == "__main__":
    main()
