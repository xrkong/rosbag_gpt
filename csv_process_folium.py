
# Convert all 'csv' files to 'kml' files, and plot to 'html' files
# $ sudo apt-get install wkhtmltopdf
# To get a better view of folium, read https://python-visualization.github.io/folium/
import os
import pandas as pd
import numpy as np
from PIL import Image
from simplekml import Kml
import folium
import webbrowser

# Define the folder path where the CSV files are located
FOLDER_PATH = "/home/kong/Documents/data" # Change path to your folder

class Map:
    def __init__(self, zoom_start, file_path, trail_coordinates):
        self.center = coords = list(np.mean(trail_coordinates, axis=0))
        self.zoom_start = zoom_start
        self.path = trail_coordinates
        self.file_path = file_path
    
    def showMap(self):
        #Create the map
        my_map = folium.Map(location = self.center, zoom_start = self.zoom_start)
        folium.PolyLine(self.path).add_to(my_map)

        #Display the map
        my_map.save(self.file_path)
        webbrowser.open(self.file_path)

# Loop through all the CSV files in the folder
for file_name in os.listdir(FOLDER_PATH):
    if file_name.endswith('.csv'):
        # Read the CSV file into a pandas DataFrame
        csv_file_path = os.path.join(FOLDER_PATH, file_name)
        df = pd.read_csv(csv_file_path)

        # Create a new KML file with the same name as the CSV file
        kml_file_path = os.path.splitext(csv_file_path)[0] + ".kml"
        kml = Kml()
        for index, row in df.iterrows():
            kml.newpoint(name=row['time'], coords=[(row['lon'], row['lat'])])
        kml.save(kml_file_path)

        # Extract latitude and longitude data from KML file
        with open(kml_file_path, 'r') as f:
            kml_data = f.read()
        latitudes = []
        longitudes = []
        for line in kml_data.splitlines():
            if "<coordinates>" in line:
                coordinates = line.strip().split(">")[1].split("<")[0]
                longitude, latitude, height = map(float, coordinates.split(","))
                latitudes.append(latitude)
                longitudes.append(longitude)

        # Create Google Maps plot using gmplot
        html_file_path = os.path.splitext(kml_file_path)[0] + '.html'

        # Convert the lists into NumPy arrays and stack them horizontally to create a 10x2 matrix
        path_points = np.hstack((np.array(latitudes).reshape((10, 1)), np.array(longitudes).reshape((10, 1))))
        op_map = Map(18, html_file_path, path_points)
        op_map.showMap()



