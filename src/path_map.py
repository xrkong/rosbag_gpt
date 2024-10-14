import os
import webbrowser
import numpy as np
import folium
from PIL import Image
import math
import src.frame as Frame

'''
PathMap class
'''
class PathMap:
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