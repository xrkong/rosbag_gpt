import os
import datetime
import base64
import pytz
from src.frame import Frame
from src.path_map import PathMap
from src.utils import extract_rosbag_path
from jinja2 import Environment, FileSystemLoader
from openai import OpenAI
from src.bag_parser import BagFileParserFactory

# from dotenv import find_dotenv, load_dotenv
# _ = load_dotenv(find_dotenv())

'''
Gnereate report based on Map() and Frame()
'''
class Report():
    def __init__(
        self, 
        waypoint_rosbag_pathname : str,
        stop_rosbag_pathname : str, 
        output_dir : str = "./",
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
            self, path : str) -> list[PathMap, str]:
        # update self.history_path
        pass
        
    def __add_map_from_rosbag(
            self, 
            waypoint_rosbag_pathname : str,
            stop_rosbag_pathname : str, 
            output_dir : str
            )->list[PathMap, str]:
        # update self.history_path
        # output_dir = os.path.join(output_dir, "map.html")
        waypoints = extract_rosbag_path(waypoint_rosbag_pathname, "/sbg/gps_pos")
        stops = extract_rosbag_path(stop_rosbag_pathname, "/sbg/gps_pos")
        map = PathMap(17,
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


    def __rosbag_to_inc_frames(self, stop_rosbag_pathname : str) -> list[Frame]:
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

    def __inc_frame_to_dict(
            self, 
            frame : Frame, 
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

        [front_cam, rear_cam] = frame.save_camera_image(out_pathname)

        rear_pcd = frame.save_pcd2png(out_pathname, frame.vld_rear[1], "velodyne_rear")
        front_pcd = frame.save_pcd2png(out_pathname, frame.vld_front[1], "velodyne_front")

        front_exp = self.add_image_explanation(front_cam, "front")
        print("Front: "+front_exp)
        rear_exp = self.add_image_explanation(rear_cam, "rear")
        print("Rear: "+rear_exp)

        timezone = pytz.timezone('Australia/Perth')
        dt_object = datetime.datetime.fromtimestamp(frame.timestamp/1e9, tz=timezone)

        out = {
                "gps": f"Latitude: {frame.gps_pos[1].latitude:.5f}, Longitude: {frame.gps_pos[1].longitude:.5f}",
                "date_time": dt_object, # add time zone est
                "front_camera": os.path.relpath(front_cam, self.output_dir), #front_cam,
                "rear_camera":  os.path.relpath(rear_cam, self.output_dir),
                "front_lidar":  os.path.relpath(front_pcd, self.output_dir),
                "rear_lidar":   os.path.relpath(rear_pcd, self.output_dir),
                "front_explanation": front_exp,
                "rear_explanation":  rear_exp,
            }
        self.stops_content.append(out)
        return out

    def generate_report(self) -> None:
        # generate the report based on the template, and src files
        map_pathname = os.path.join(self.output_dir, "map.html")
        [_, map_pathname] = self.__add_map_from_rosbag(
                                self.waypoint_rosbag_pathname, 
                                self.stop_rosbag_pathname, 
                                map_pathname
                                )
        self.__rosbag_to_inc_frames(self.stop_rosbag_pathname)
        for stop in self.stops_data:
            self.__inc_frame_to_dict(stop, self.stops_data.index(stop))

        # Create a basic environment for the HTML report template
        env = Environment(loader=FileSystemLoader('./resources'))
        template = env.get_template('report_template.html')

        # [ ] Render the template with the data
        html_content = template.render(
            date_time='00-00-00', 
            path_html=os.path.relpath(map_pathname, self.output_dir), 
            path_explanation=self.history_path_content,
            incidents=self.stops_content # based on stops_data
        )
        # Write the HTML content to a file (optional for debugging)
        output_pathname = os.path.join(self.output_dir, "report.html")
        with open(output_pathname, 'w') as file:
            file.write(html_content)

    def save_report(self, output_path : str):
        pass