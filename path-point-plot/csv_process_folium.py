# Convert all 'csv' files to 'kml' files, and plot to 'html' files
# To get a better view of folium, read https://python-visualization.github.io/folium/
import os
import sys
import pandas as pd
import numpy as np
import math
import folium
import webbrowser
import base64
import io

from PIL import Image
from simplekml import Kml

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
        for i in range(len(self.path)-2):
            if math.dist(self.path[i], self.path[i+1]) < 0.0002: # 0.0002 ~20meters
                folium.PolyLine(self.path[i:i+2]).add_to(self.map)
        self.map.save(self.file_path)
        webbrowser.open(self.file_path)

    def plot_dot(self):
        self.map = folium.Map(location = self.pic_center, zoom_start = self.zoom_start)

        for i, path_point in enumerate(self.path):

            with Image.open('popup_image/' + str(i) + '.png') as img:
                # Define the new dimensions (width, height)
                new_width = 960
                new_height = 540

                # Resize the image
                resized_img = img.resize((new_width, new_height))

                with io.BytesIO() as buffer:
                    resized_img.save(buffer, format="PNG")  # You can specify the format you want (JPEG, PNG, etc.)
                    binary_data = buffer.getvalue()

                encoded_image = base64.b64encode(binary_data).decode()
                # Save the resized image to a new file
                #resized_img.save('resized_image.jpg')
                
            # with open('popup_image/' + str(i) + '.png') as image_file:

            #     resized_img = image_file.read().resize((new_width, new_height))
            #     encoded_image = base64.b64encode(resized_img.read()).decode()
            title_text = "No."+str(i+1)+' stop'
            html = title_text+'<img src="data:image/png;base64,{}" title="{}">'.format(encoded_image, title_text)
            # encoded = base64.b64encode(open('popup_image/'+str(i)+'.png', 'rb').read())
            # html = '<img src="data:image/png;base64,{}">'.format
            
            iframe = folium.IFrame(html, width=960, height=540)
            popup = folium.Popup(iframe, max_width=1080)

            folium.Marker(
                location=path_point,
                popup=popup #"No."+str(i+1)+' stop',
            ).add_to(self.map)
        # for path_point in self.path:
        #     folium.CircleMarker(location=[path_point[0], path_point[1]],
        #                 radius=2,
        #                 weight=5).add_to(self.map)
        self.map.save(self.file_path)
        webbrowser.open(self.file_path)

# Loop through all the CSV files in the folder
# rescale this function
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
                # TODO use args to contorl save map or plot dot
                html_file_path = os.path.splitext(csv_file_path)[0] + '.html'
                # Convert the lists into NumPy arrays and stack them horizontally to create a 10x2 matrix
                path_points = np.vstack((latitudes, longitudes)).T
                op_map = Map(17, html_file_path, path_points)
                print('Done!')
                op_map.saveMap()
                #op_map.plot_dot()

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

