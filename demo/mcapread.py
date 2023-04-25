# Importing the required modules
import matplotlib.pyplot as plt
import numpy  as np

# Creating a new figure and setting up the resolution
fig = plt.figure(dpi=200)

# Change the coordinate system from scaler to polar
ax = fig.add_subplot(projection='polar')

# Generating the X and Y axis data points
r=[0,4,8,8,8,8,8,8,8]
theta = np.deg2rad(np.arange(-45,406-90,45))

# plotting the polar coordinates on the system
plt.polar(theta,r,marker='o')

# Setting the axis limit
ax.set_ylim(0,10)

# Displaying the plot
plt.show()