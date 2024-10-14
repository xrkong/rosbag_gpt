import os
import sqlite3
import yaml
import math
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
from mcap.reader import make_reader

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
    