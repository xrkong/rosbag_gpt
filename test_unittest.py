import unittest
from plot_ros2bag import BagFileParserFactory, db3Parser, mcapParser
from sensor_msgs.msg import Image

TEST_MCAP_ROSBAG = "/home/kong/eglinton/vtm-data/raw/rosbag2_2024_06_14-10_40_49/rosbag2_2024_06_14-10_40_49_0.mcap"
TEST_MCAP_ERROR_ROSBAG = "/home/kong/fake_rosbag.mcap"

class TestUtils(unittest.TestCase):
    '''
    - [x] Input a file path, validate the rosbag file. If failed, throw an error.
    - [x] Input a message list, check the message in the rosbag or not.
    - [x] Input a topic name, extract the message list.
    - [x] Input a topic name and a time stamp, return the closest message.
    '''

    # ==========================
    # rosbag parser factory test
    # ==========================

    def test_parser_factory(self):
        parser = BagFileParserFactory(TEST_MCAP_ROSBAG)
        self.assertTrue(parser is not None)

    def test_load_rosbag(self):
        parser = BagFileParserFactory(TEST_MCAP_ROSBAG)
        self.assertTrue(parser.get_file_type() is not None)

    def test_load_error_rosbag(self):
        try:
            parser = BagFileParserFactory(TEST_MCAP_ERROR_ROSBAG)
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
            db3Parser(TEST_MCAP_ROSBAG)

    # ===============
    # mcap rosbag test
    # ===============

    def test_mcap_parser(self):
        parser = mcapParser(TEST_MCAP_ROSBAG)
        self.assertTrue(parser is not None)

    def test_get_messages(self):
        TOPIC_NAME = "/CameraFront"
        TOPIC_LENGTH = 18 ##18 images in the rosbag
        parser = mcapParser(TEST_MCAP_ROSBAG)
        image_list = parser.get_messages(TOPIC_NAME)
        self.assertTrue(len(image_list) == TOPIC_LENGTH)

    def test_get_empty_messages(self):
        TOPIC_NAME = "/CameraXXXX" # fake topic name
        parser = mcapParser(TEST_MCAP_ROSBAG)
        image_list = parser.get_messages(TOPIC_NAME)
        self.assertIsNone(image_list)

    def test_get_msg_frame(self):
        TOPIC_NAME = "/CameraFront"
        TIME_STAMP = 1718334194741677391 # timestamp of the first image
        parser = mcapParser(TEST_MCAP_ROSBAG)
        image = parser.get_msg_frame(TOPIC_NAME, TIME_STAMP)
        self.assertIs(type(image[0]), float)
        self.assertIs(type(image[1]), Image)

class TestMap(unittest.TestCase):
    '''
    - [ ] Input a rosbag and gps topic name, retun a html path file.
    - [ ] Input a rosbag and gps topic name, retun a csv path file.
    - [ ] Input a rosbag, regular gps topic name, and incident topic name, return a html incident path map.
    - [ ] Input a regular rosbag, and a incident rosbag, return a html incident path map.
    - [ ] Input two csv file (regular and incident), return a html incident path map.
    '''
    def test_pathbag2csv(self):
        self.fail("Not implemented yet")
        
    def test_pathbag2html(self):
        self.fail("Not implemented yet")
    
    def test_incident_bag2html(self):
        self.fail("Not implemented yet")

    def test_two_bag2html(self):
        self.fail("Not implemented yet")

    def test_two_csv2html(self):
        self.fail("Not implemented yet")

class TestSensors(unittest.TestCase):
    '''
    - [ ] Input a rosbag file and a timestamp, visualize pointcloud and images into a single file.
    - [ ] Input a rosbag file and a timestamp, save pointcloud and images into a single file.
    - [ ] Input a rosbag file, a timestamp, and a saving path, save all pointcloud and images to the saving path.
    - [ ] Input a rosbag, a image topics, and a saving path, extract images to the given path.
    - [ ] Input a rosbag, a pcd topics, and a saving path, extract point cloud files to the given path.
    '''
    def test_plot_frame_report(self):
        self.fail("Not implemented yet")

    def test_save_frame_report(self):
        self.fail("Not implemented yet")

    def test_export_images(self):
        self.fail("Not implemented yet")

    def test_export_pcd(self):
        self.fail("Not implemented yet")    
        
    def test_export_framedata(self):
        self.fail("Not implemented yet")