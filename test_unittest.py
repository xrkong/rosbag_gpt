import unittest
from plot_ros2bag import * 
from sensor_msgs.msg import Image
import pdfkit
from PIL import Image
from jinja2 import Environment, FileSystemLoader
import os
import datetime
import numpy as np
from skimage.metrics import structural_similarity as ssim

# ground truth
GT_MCAP_STOPS = "./unittest_file/gt/rosbag2_2024_09_03-10_05_53_stops/rosbag2_2024_09_03-10_05_53_0.mcap"
GT_MCAP_WAYPOINTS = "./unittest_file/gt/rosbag2_2024_09_03-10_05_53_waypoints/rosbag2_2024_09_03-10_05_53_0.mcap"
GT_MCAP_RAW = "./unittest_file/gt/rosbag2_2024_09_03-10_06_24_raw/rosbag2_2024_09_03-10_06_24_0.mcap"
GT_MCAP_FAKE = "/home/kong/fake_rosbag.mcap" # doesn't exist

GT_CSV_WAYPOINTS = "./unittest_file/gt/waypoints.csv"
GT_CSV_STOPS = "./unittest_file/gt/stops.csv"

GT_HTML_WAYPOINTS_STOPS = "./unittest_file/gt/waypoints_and_stops.html"
GT_HTML_WAYPOINTS = "./unittest_file/gt/waypoints.html"
GT_HTML_STOPS = "./unittest_file/gt/stops.html"

GT_PNG_FRONT = "./unittest_file/gt/front.png"
GT_PNG_REAR = "./unittest_file/gt/rear.png"
GT_PNG_VLD_FRONT = "./unittest_file/gt/velodyne_front.png"
GT_PNG_VLD_REAR = "./unittest_file/gt/velodyne_rear.png"
GT_PCD_VLD_FRONT = "./unittest_file/gt/velodyne_front.pcd"
GT_PCD_VLD_REAR = "./unittest_file/gt/velodyne_rear.pcd"

OUTPUT = "./unittest_file/output"

class TestUtils(unittest.TestCase):
    '''
    - Input a file path, validate the rosbag file. If failed, throw an error.
    - Input a message list, check the message in the rosbag or not.
    - Input a topic name, extract the message list.
    - Input a topic name and a time stamp, return the closest message.
    '''

    # ==========================
    # rosbag parser factory test
    # ==========================

    def test_parser_factory(self):
        parser = BagFileParserFactory(GT_MCAP_STOPS)
        self.assertTrue(parser is not None)

    def test_load_rosbag(self):
        parser = BagFileParserFactory(GT_MCAP_STOPS)
        self.assertTrue(parser.get_file_type() is not None)

    def test_load_error_rosbag(self):
        try:
            parser = BagFileParserFactory(GT_MCAP_FAKE)
        except IOError:
            print("FileNotFoundError raised")
        else:
            self.fail("unexpected exception raised")

    # ===============
    # db3 rosbag test
    # ===============

    def test_db3_parser(self):
        import sqlite3
        with self.assertRaises(sqlite3.DatabaseError):
            Db3Parser(GT_MCAP_STOPS)

    # ===============
    # mcap rosbag test
    # ===============

    def test_mcap_parser(self):
        parser = McapParser(GT_MCAP_STOPS)
        self.assertTrue(parser is not None)

    def test_get_messages(self):
        TOPIC_NAME = "/CameraFront"
        TOPIC_LENGTH = 343 ##18 images in the rosbag
        parser = McapParser(GT_MCAP_STOPS)
        image_list = parser.get_messages(TOPIC_NAME)
        self.assertTrue(len(image_list) == TOPIC_LENGTH)

    def test_get_empty_messages(self):
        TOPIC_NAME = "/CameraXXXX" # fake topic name
        parser = McapParser(GT_MCAP_STOPS)
        image_list = parser.get_messages(TOPIC_NAME)
        self.assertIsNone(image_list)

    def test_get_msg_frame(self):
        TOPIC_NAME = "/CameraFront"
        TIME_STAMP = 1725329305498373948 
        parser = McapParser(GT_MCAP_STOPS)
        image = parser.get_msg_frame(TOPIC_NAME, TIME_STAMP)
        self.assertIs(type(image[0]), float)
        self.assertIs(type(image[1]), sensor_msgs.msg.Image)

class TestMap(unittest.TestCase):
    '''
    - Input a rosbag and gps topic name, retun a html path file.
    - Input a rosbag and gps topic name, retun a csv path file.
    - Input a rosbag, regular gps topic name, and incident topic name, return a html incident path map.
    - Input a regular rosbag, and a incident rosbag, return a html incident path map.
    - Input two csv file (regular and incident), return a html incident path map.
    '''
    def test_pathbag2csv(self):
        path = extract_rosbag_path(GT_MCAP_WAYPOINTS, "/sbg/gps_pos")
        output = "./unittest_file/output/test_pathbag2csv.csv"
        map = Map(17,
                  output_path=output, 
                  path_waypoints=path)
        save_path = map.save_csv_map()
        self.assertEqual(os.path.getsize(save_path),
                         os.path.getsize(GT_CSV_WAYPOINTS),
                         "file size not match")
        # self.fail("Not implemented yet")

        
    def test_pathbag2html(self):
        path = extract_rosbag_path(GT_MCAP_WAYPOINTS, "/sbg/gps_pos")
        output = "./unittest_file/output/test_pathbag2html.html"
        map = Map(17,
                  output_path=output, 
                  path_waypoints=path)
        map.draw_path()
        file_name = map.save_html_map()
        self.assertAlmostEqual(os.path.getsize(file_name), 
                         os.path.getsize(GT_HTML_WAYPOINTS), 
                         -6,
                         "file size not match")
    
    def test_stops_bag2html(self):
        path = extract_rosbag_path(GT_MCAP_STOPS, "/sbg/gps_pos")
        output = "./unittest_file/output/test_stops_bag2html.html"
        map = Map(17,
                  output_path=output, 
                  incident_waypoints=path) 
        map.draw_path()
        file_name = map.save_html_map()
        self.assertAlmostEqual(os.path.getsize(file_name), 
                         os.path.getsize(GT_HTML_STOPS), 
                         -4,
                         "file size not match")

    def test_two_bag2html(self):
        waypoints = extract_rosbag_path(GT_MCAP_WAYPOINTS, "/sbg/gps_pos")
        stops = extract_rosbag_path(GT_MCAP_STOPS, "/sbg/gps_pos")
        output_path = "./unittest_file/output/test_two_bag2html.html"
        map = Map(17,
                  output_path=output_path,
                  path_waypoints=waypoints,
                  incident_waypoints=stops)
        map.draw_path()
        file_name = map.save_html_map()
        self.assertEqual(os.path.getsize(file_name), 
                         os.path.getsize(GT_HTML_WAYPOINTS_STOPS), 
                         "file size not match")

    def test_two_csv2html(self):
        waypoint = extract_csv_path(GT_CSV_WAYPOINTS)
        stops = extract_csv_path(GT_CSV_STOPS)
        output_path = "./unittest_file/output/test_two_csv2html.html"
        map = Map(17,
                  output_path=output_path,
                  path_waypoints=waypoint,
                  incident_waypoints=stops)
        map.draw_path()
        file_name = map.save_html_map()
        self.assertAlmostEqual(os.path.getsize(file_name), 
                         os.path.getsize(GT_HTML_WAYPOINTS_STOPS), 
                         -6,
                         "file size not match")

    def test_csv2html(self):
        path_wp = extract_csv_path(GT_CSV_WAYPOINTS)
        output_path = "./unittest_file/output/test_csv2html.html"
        map = Map(17, 
                  output_path=output_path, 
                  path_waypoints=path_wp)
        map.draw_path()
        file_name = map.save_html_map()
        self.assertEqual(os.path.getsize(file_name), os.path.getsize(GT_HTML_WAYPOINTS), "file size not match")

class TestSensors(unittest.TestCase):
    '''
    - Input a rosbag file and a timestamp, visualize pointcloud and images into a single file.
    - Input a rosbag file and a timestamp, save pointcloud and images into a single file.
    - Input a rosbag file, a timestamp, and a saving path, save all pointcloud and images to the saving path.
    - Input a rosbag, a image topics, and a saving path, extract images to the given path.
    - Input a rosbag, a pcd topics, and a saving path, extract point cloud files to the given path.
    '''
    def setUp(self):
        self.parser = BagFileParserFactory(GT_MCAP_STOPS, duration=1e12) # must be 1e12 (~100sec)
        self.timstamp = 1725329453522752369 #1725329433.522752369 or 1725330646.261834535

    def test_bus_status(self):
        frame_data = Frame(self.parser.get_parser(), timestamp=self.timstamp)
        data = frame_data.print_location()
        '''
        Roll: 0.008607011288404465
        Pitch: -0.05061418190598488
        Yaw: -2.8885398546801966
        Lat:-31.5959749, Lon:115.6692532
        '''
        self.assertAlmostEqual(data[0], -2.8885398546801966, 5)
        self.assertAlmostEqual(data[1], -31.5959749, 5)
        self.assertAlmostEqual(data[2], 115.6692532, 5)
    
    # def test_battery_status(self):
    #     # TODO timestamp of battery status is missing.
    #     frame_data = Frame(self.parser.get_parser(), timestamp=self.timstamp)
    #     [percent, voltage] = frame_data.print_battery()

    def test_save_images(self):
        frame_data = Frame(self.parser.get_parser(), timestamp=self.timstamp)
        [front_path, rear_path] = frame_data.save_camera_image(OUTPUT)

        # compare the image content
        front = cv2.imread(front_path, cv2.IMREAD_GRAYSCALE)
        rear = cv2.imread(rear_path, cv2.IMREAD_GRAYSCALE)

        gt_front = cv2.imread(GT_PNG_FRONT, cv2.IMREAD_GRAYSCALE)
        gt_rear = cv2.imread(GT_PNG_REAR, cv2.IMREAD_GRAYSCALE)

        # compare the image size
        self.assertEqual(front.shape, gt_front.shape, "Front Images have different sizes")
        self.assertEqual(rear.shape, gt_rear.shape, "Rear Images have different sizes")

        # compare the image similarity
        score, _ = ssim(front, gt_front, full=True)
        self.assertGreater(score, 0.99, "Front Images have different content")

        score, _ = ssim(rear, gt_rear, full=True)
        self.assertGreater(score, 0.99, "Rear Images have different content")
    
    def test_save_pcd(self):
        frame_data = Frame(self.parser.get_parser(), timestamp=self.timstamp)
        [path, _] = frame_data.save_pcd(OUTPUT, frame_data.vld_rear[1], "velodyne_rear")
        self.assertEqual(os.path.getsize(path), 
                         os.path.getsize(GT_PCD_VLD_REAR), 
                         "file size not match")

    def test_save_pcd2png(self):
        '''
        Visualizate point cloud data into single image file (PNG). 
        ready to send to inference
        '''

        frame_data = Frame(self.parser.get_parser(), timestamp=self.timstamp)
        output = frame_data.save_pcd2png(OUTPUT, 
                                       frame_data.vld_rear[1], 
                                       "velodyne_rear")

        gt = cv2.imread(GT_PNG_VLD_REAR, cv2.IMREAD_GRAYSCALE)
        out = cv2.imread(output, cv2.IMREAD_GRAYSCALE)

        self.assertEqual(out.shape, gt.shape, "Images have different sizes")
        score, _ = ssim(gt, out, full=True)
        self.assertGreater(score, 0.99, "Front Images have different content")

class TestReport(unittest.TestCase):
    '''
    - [ ] input a stop rosbag, a path rosbag and a output path, generate all files.
    - [ ] input a src path and a template file, generate a html report.
    - [ ] input a html report, convert to pdf.
    '''
    def setUp(self):
        self.parser = BagFileParserFactory(GT_MCAP_STOPS)
        self.timstamp = 1725329453522752369

    def test_generate_html(self):
        '''
        Generate a report for a given timestamp.
        Report includes:
        - [x] Date and time
        - [x] GPS location
        - [ ] Weather condition based on the location and time
        - [x] Front and rear camera images
        - [ ] Front and rear velodyne point cloud data
        - [ ] Images analysis (e.g. object detection, lane detection)
        '''
        report = Report(GT_MCAP_WAYPOINTS, 
                        GT_MCAP_STOPS, 
                        "./resources/", 
                        "./resources/report_template.html")
        
        report.generate_report()

        # self.fail("Not implemented yet")

    def test_add_explaination(self):    
        '''
        Add explaination to the report.
        '''
        report = Report(GT_MCAP_WAYPOINTS, 
                        GT_MCAP_STOPS, 
                        "./resources/", 
                        "./resources/report_template.html")
        
        content = report.add_image_explanation(GT_PNG_FRONT, "front")
        self.assertTrue(content is not None)

