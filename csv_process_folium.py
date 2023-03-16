
# Convert all 'csv' files to 'kml' files, and plot to 'html' files
# To get a better view of folium, read https://python-visualization.github.io/folium/
import os
import sys
import pandas as pd
import numpy as np
from simplekml import Kml
import folium
import webbrowser

class Map:
    def __init__(self, zoom_start, file_path, trail_coordinates):  
        self.zoom_start = zoom_start
        self.path = trail_coordinates
        self.pic_center = np.mean(self.path, 0)
        self.file_path = file_path
        self.map = None
    
    def saveMap(self):
        #Create the map
        #center = np.mean(self.path, axis=0)
        self.map = folium.Map(location = self.pic_center, zoom_start = self.zoom_start)
        folium.PolyLine(self.path).add_to(self.map)
        self.map.save(self.file_path)
        webbrowser.open(self.file_path)

# Loop through all the CSV files in the folder
def convertCsv(folder_path):
    for file_name in os.listdir(folder_path):
        if file_name.endswith('.csv'):

            latitudes = []
            longitudes = []
            # Read the CSV file into a pandas DataFrame
            csv_file_path = os.path.join(folder_path, file_name)
            df = pd.read_csv(csv_file_path)

            # Create a new KML file with the same name as the CSV file
            kml_file_path = os.path.splitext(csv_file_path)[0] + ".kml"
            kml = Kml()
            for index, row in df.iterrows():
                kml.newpoint(name=row['time'], coords=[(row['lon'], row['lat'])]) # Change name according to your csv file
            kml.save(kml_file_path)

            # Extract latitude and longitude data from KML file
            with open(kml_file_path, 'r') as f:
                kml_data = f.read()

            for line in kml_data.splitlines():
                if "<coordinates>" in line:
                    coordinates = line.strip().split(">")[1].split("<")[0]
                    longitude, latitude, height = map(float, coordinates.split(","))
                    latitudes.append(latitude)
                    longitudes.append(longitude)

            if len(latitudes) == 0:
                print('No trival detected!')
            else:   
                html_file_path = os.path.splitext(csv_file_path)[0] + '.html'
                # Convert the lists into NumPy arrays and stack them horizontally to create a 10x2 matrix
                path_points = np.vstack((latitudes, longitudes)).T
                op_map = Map(17, html_file_path, path_points)
                print('Done!')
                op_map.saveMap()

    # # Draw one html for all kml
    # if len(latitudes) == 0:
    #     print('No trival detected!')
    # else:   
    #     html_file_path = folder_path + '/output.html'
    #     # Convert the lists into NumPy arrays and stack them horizontally to create a 10x2 matrix
    #     path_points = np.hstack((np.array(latitudes).reshape((len(latitudes), 1)), np.array(longitudes).reshape((len(longitudes), 1))))
    #     op_map = Map(18, html_file_path, path_points)
    #     print('Done!')
    #     #op_map.showMap()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Converting CSV in current path')
        folder_path = os.getcwd()
        convertCsv(folder_path)
    else:
        folder_path = sys.argv[1]
        if os.path.isdir(folder_path):
            print('Converting CSV in path ' + folder_path)
            convertCsv(folder_path)
        else :
            print('Please provide a directory path')

