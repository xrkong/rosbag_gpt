from src.report import *
from dotenv import find_dotenv, load_dotenv

GT_MCAP_STOPS = "./unittest/fixture/rosbag2_2024_09_03-10_05_53_stops/rosbag2_2024_09_03-10_05_53_0.mcap"
GT_MCAP_WAYPOINTS = "./unittest/fixture/rosbag2_2024_09_03-10_05_53_waypoints/rosbag2_2024_09_03-10_05_53_0.mcap"

OUTPUT = "./output"


_ = load_dotenv(find_dotenv())

report = Report(GT_MCAP_WAYPOINTS, 
                GT_MCAP_STOPS, 
                OUTPUT, 
                "./unittest/resources/report_template.html")

report.generate_report()
