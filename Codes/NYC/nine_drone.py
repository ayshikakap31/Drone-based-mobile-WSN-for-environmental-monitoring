import numpy as np
from pykrige.ok3d import OrdinaryKriging3D
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import csv
from scipy.spatial import cKDTree
from scipy.interpolate import RegularGridInterpolator
from scipy.interpolate import griddata

points = set()
counter = 0
indexDict = dict()
groundTruthDict = dict()
XY = []
avg = 0
with open('nyc_data.csv', mode='r') as file:
    csvFile = csv.DictReader(file)
    for lines in csvFile:
        if len(lines.get('AirTemp')) == 0:
            continue
        temp = float(lines.get('AirTemp'))
        avg += temp
        lat = float(lines.get('Latitude'))
        long = float(lines.get('Longitude'))
        day = str(lines.get('Day'))
        points.add(str([lat, long]))
        if str([lat, long]) in indexDict:
            indexDict[str([lat, long])] += 1
        else:
            indexDict[str([lat, long])] = 0
        groundTruthDict[str([lat, long]) + str(indexDict[str([lat, long])])] = temp
        if indexDict[str([lat, long])] == 2947:
            XY.append(str([lat, long]))
        if len(XY) == 100:
            break
        counter += 1

avg /= counter

XY = sorted(XY)
pointIndexDict = dict()
ix = 0
jx = 0
for i in XY:
    pointIndexDict[i] = [ix, jx]
    jx += 1
    if jx == 10:
        jx = 0
        ix += 1
groundTruth = np.zeros((2948, 10, 10))
for pts in XY:
    for z in range(2948):
        i = pointIndexDict[pts][0]
        j = pointIndexDict[pts][1]
        groundTruth[z][i][j] = groundTruthDict[pts + str(z)]
r = 3
c = 3
r_beg = 0
r_end = r
c_beg = 0
c_end = c
pts1 = []      
while r_beg < r_end and c_beg < c_end:
    for i in range(c_beg, c_end):
        pts1.append([r_beg, i])
    for i in range(r_beg + 1, r_end):
        pts1.append([i, c_end - 1])
    for i in range(c_end - 1, c_beg - 1, -1):
        if r_end - 1 - r_beg <= 0:
            break
        pts1.append([r_end - 1, i])
    for i in range(r_end - 2, r_beg, -1):
        if c_end - 1 - c_beg <= 0:
            break
        pts1.append([i, c_beg])
    r_beg += 1
    r_end -= 1
    c_beg += 1
    c_end -= 1
temp = [pts1[0]]
for i in range(1, len(pts1)):
    if temp[len(temp) - 1] == pts1[i]:
        continue
    temp.append(pts1[i])
pts1 = temp
pts2 = []
pts3 = []
pts4 = []
pts5 = []
pts6 = []
pts7 = []
pts8 = []
pts9 = []
for i in range(0, len(pts1)):
    pts2.append([pts1[i][0], pts1[i][1] + 3])
    pts3.append([pts1[i][0], pts1[i][1] + 3])
    pts4.append([pts1[i][0] + 3, pts1[i][1]])
    pts5.append([pts1[i][0] + 3, pts1[i][1] + 3])
    pts6.append([pts1[i][0] + 3, pts1[i][1] + 6])
    pts7.append([pts1[i][0] + 6, pts1[i][1] + 6])
    pts8.append([pts1[i][0] + 6, pts1[i][1] + 3])
    pts9.append([pts1[i][0] + 6, pts1[i][1] + 6])
    
XYTPts = []
t = 0
i = 0
while t < 2948:
    XYTPts.append([pts1[i][0], pts1[i][1], t])
    XYTPts.append([pts2[i][0], pts2[i][1], t])
    XYTPts.append([pts3[i][0], pts3[i][1], t])
    XYTPts.append([pts4[i][0], pts4[i][1], t])
    XYTPts.append([pts5[i][0], pts5[i][1], t])
    XYTPts.append([pts6[i][0], pts6[i][1], t])
    XYTPts.append([pts7[i][0], pts7[i][1], t])
    XYTPts.append([pts8[i][0], pts8[i][1], t])
    XYTPts.append([pts9[i][0], pts9[i][1], t])
    i += 2
    i %= len(pts1)
    t += 1
    
nineDrones = []
for i in range(0, len(XYTPts)):
    nineDrones.append(groundTruth[XYTPts[i][2]][XYTPts[i][0]][XYTPts[i][1]])

nineDrones = np.array(nineDrones)
coordinates_array = np.array(XYTPts)

# Split the coordinates array into individual arrays for x, y, z
x_coords = coordinates_array[:, 0]
y_coords = coordinates_array[:, 1]
z_coords = coordinates_array[:, 2]

#%%
print('Performing interpolation')
def idw_interpolation(known_coords, known_values, query_coords, power=3):
    tree = cKDTree(known_coords)
    distances, indices = tree.query(query_coords, k=7)  # You can adjust the number of neighbors (k) as needed

    weights = 1 / ((distances + 1e-10) ** power)
    interpolated_values = np.sum(weights * known_values[indices], axis=1) / np.sum(weights, axis=1)

    return interpolated_values
nineDroneNearestInterpolated = np.zeros((2948, 10, 10))
nineDroneIDWInterpolated = np.zeros((2948,10,10))
interpolation_coordinates = []
for z in range(0, 2948):
    # print(z)
    for i in range(0, 10):
        for j in range(0, 10):
            #fourDronesKrigedInterpolated[z][i][j],var = ok3d.execute('points', z,i,j)
            interpolation_coordinates.append([i,j,z])
            
nn_interpolated_values = griddata(coordinates_array, nineDrones, interpolation_coordinates, method='nearest')          
idw_interpolated_values = idw_interpolation(coordinates_array, nineDrones, interpolation_coordinates)
#%%
counter = 0
for z in range(0, 2948):
    # print(z)
    for i in range(0, 10):
        for j in range(0, 10):
            coord = interpolation_coordinates[counter]
            nineDroneIDWInterpolated[coord[2]][coord[0]][coord[1]] = idw_interpolated_values[counter]
            nineDroneNearestInterpolated[coord[2]][coord[0]][coord[1]] = nn_interpolated_values[counter]
            counter += 1
#%% For t=1 hours
# Plot the 3-dim data
x, y = np.meshgrid(np.arange(10), np.arange(10))
fig = plt.figure(figsize = (55, 15))

# First ground truth plot
ax1 = fig.add_subplot(131, projection='3d')
surface1 = ax1.plot_surface(x, y, groundTruth[0], cmap='viridis')
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax1.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax1.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax1.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax1.set_zlabel('Z-axis',fontsize=35, labelpad=35)
ax1.set_title('Ground Truth at Time t=1 hours',fontsize=45)
#colorbar1 = fig.colorbar(surface1, ax=ax1, shrink=0.5, aspect=10)
# Second linear interpolated plot
ax2 = fig.add_subplot(132, projection='3d')
surface2 = ax2.plot_surface(x, y, nineDroneIDWInterpolated[0], cmap='viridis')
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax2.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax2.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax2.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax2.set_zlabel('Z-axis',fontsize=35, labelpad=35)
ax2.set_title('IDW Interpolation at Time t=1 hours',fontsize=45)

# Third interpolated plot
ax3 = fig.add_subplot(133, projection='3d')
surface3 = ax3.plot_surface(x, y, nineDroneNearestInterpolated[0], cmap='viridis')
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax3.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax3.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax3.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax3.set_zlabel('Z-axis',fontsize=35, labelpad=35)
ax3.set_title('Nearest neighbor Interpolation at Time t=1 hours',fontsize=45)

plt.tight_layout()
# Create a single colorbar for both subplots
cax = fig.add_axes([0.99, 0.1, 0.02, 0.8])  # [x_position, y_position, width, height]
colorbar = fig.colorbar(surface1, cax=cax, shrink=0.6)
colorbar.ax.tick_params(axis='y', labelsize=25)
# Show the plot
plt.show()       
#%% For t=50 hours
# Plot the 3-dim data
x, y = np.meshgrid(np.arange(10), np.arange(10))
fig = plt.figure(figsize = (55, 15))

# First ground truth plot
ax1 = fig.add_subplot(131, projection='3d')
surface1 = ax1.plot_surface(x, y, groundTruth[49], cmap='viridis')
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax1.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax1.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax1.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax1.set_zlabel('Z-axis',fontsize=35, labelpad=35)
ax1.set_title('Ground Truth at Time t=50 hours',fontsize=45)
#colorbar1 = fig.colorbar(surface1, ax=ax1, shrink=0.5, aspect=10)
# Second linear interpolated plot
ax2 = fig.add_subplot(132, projection='3d')
surface2 = ax2.plot_surface(x, y, nineDroneIDWInterpolated[49], cmap='viridis')
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax2.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax2.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax2.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax2.set_zlabel('Z-axis',fontsize=35, labelpad=35)
ax2.set_title('IDW Interpolation at Time t=50 hours',fontsize=45)

# Third interpolated plot
ax3 = fig.add_subplot(133, projection='3d')
surface3 = ax3.plot_surface(x, y, nineDroneNearestInterpolated[49], cmap='viridis')
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax3.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax3.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax3.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax3.set_zlabel('Z-axis',fontsize=35, labelpad=35)
ax3.set_title('Nearest neighbor Interpolation at Time t=50 hours',fontsize=45)

plt.tight_layout()
# Create a single colorbar for both subplots
cax = fig.add_axes([0.99, 0.1, 0.02, 0.8])  # [x_position, y_position, width, height]
colorbar = fig.colorbar(surface1, cax=cax, shrink=0.6)
colorbar.ax.tick_params(axis='y', labelsize=25)
# Show the plot
plt.show()
#%% For t=500 hours
# Plot the 3-dim data
x, y = np.meshgrid(np.arange(10), np.arange(10))
fig = plt.figure(figsize = (55, 15))

# First ground truth plot
ax1 = fig.add_subplot(131, projection='3d')
surface1 = ax1.plot_surface(x, y, groundTruth[499], cmap='viridis')
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax1.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax1.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax1.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax1.set_zlabel('Z-axis',fontsize=35, labelpad=35)
ax1.set_title('Ground Truth at Time t=500 hours',fontsize=45)
#colorbar1 = fig.colorbar(surface1, ax=ax1, shrink=0.5, aspect=10)
# Second linear interpolated plot
ax2 = fig.add_subplot(132, projection='3d')
surface2 = ax2.plot_surface(x, y, nineDroneIDWInterpolated[499], cmap='viridis')
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax2.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax2.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax2.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax2.set_zlabel('Z-axis',fontsize=35, labelpad=35)
ax2.set_title('IDW Interpolation at Time t=500 hours',fontsize=45)

# Third interpolated plot
ax3 = fig.add_subplot(133, projection='3d')
surface3 = ax3.plot_surface(x, y, nineDroneNearestInterpolated[499], cmap='viridis')
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax3.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax3.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax3.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax3.set_zlabel('Z-axis',fontsize=35, labelpad=35)
ax3.set_title('Nearest neighbor Interpolation at Time t=500 hours',fontsize=45)

plt.tight_layout()
# Create a single colorbar for both subplots
cax = fig.add_axes([0.99, 0.1, 0.02, 0.8])  # [x_position, y_position, width, height]
colorbar = fig.colorbar(surface1, cax=cax, shrink=0.6)
colorbar.ax.tick_params(axis='y', labelsize=25)
# Show the plot
plt.show()
#%% For t=1000 hours
# Plot the 3-dim data
x, y = np.meshgrid(np.arange(10), np.arange(10))
fig = plt.figure(figsize = (55, 15))

# First ground truth plot
ax1 = fig.add_subplot(131, projection='3d')
surface1 = ax1.plot_surface(x, y, groundTruth[999], cmap='viridis')
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax1.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax1.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax1.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax1.set_zlabel('Z-axis',fontsize=35, labelpad=35)
ax1.set_title('Ground Truth at Time t=1000 hours',fontsize=45)
#colorbar1 = fig.colorbar(surface1, ax=ax1, shrink=0.5, aspect=10)
# Second linear interpolated plot
ax2 = fig.add_subplot(132, projection='3d')
surface2 = ax2.plot_surface(x, y, nineDroneIDWInterpolated[999], cmap='viridis')
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax2.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax2.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax2.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax2.set_zlabel('Z-axis',fontsize=35, labelpad=35)
ax2.set_title('IDW Interpolation at Time t=1000 hours',fontsize=45)

# Third interpolated plot
ax3 = fig.add_subplot(133, projection='3d')
surface3 = ax3.plot_surface(x, y, nineDroneNearestInterpolated[999], cmap='viridis')
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax3.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax3.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax3.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax3.set_zlabel('Z-axis',fontsize=35, labelpad=35)
ax3.set_title('Nearest neighbor Interpolation at Time t=1000 hours',fontsize=45)

plt.tight_layout()
# Create a single colorbar for both subplots
cax = fig.add_axes([0.99, 0.1, 0.02, 0.8])  # [x_position, y_position, width, height]
colorbar = fig.colorbar(surface1, cax=cax, shrink=0.6)
colorbar.ax.tick_params(axis='y', labelsize=25)
# Show the plot
plt.show()
#%% For t=1500 hours
# Plot the 3-dim data
x, y = np.meshgrid(np.arange(10), np.arange(10))
fig = plt.figure(figsize = (55, 15))

# First ground truth plot
ax1 = fig.add_subplot(131, projection='3d')
surface1 = ax1.plot_surface(x, y, groundTruth[1499], cmap='viridis')
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax1.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax1.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax1.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax1.set_zlabel('Z-axis',fontsize=35, labelpad=35)
ax1.set_title('Ground Truth at Time t=1500 hours',fontsize=45)
#colorbar1 = fig.colorbar(surface1, ax=ax1, shrink=0.5, aspect=10)
# Second linear interpolated plot
ax2 = fig.add_subplot(132, projection='3d')
surface2 = ax2.plot_surface(x, y, nineDroneIDWInterpolated[1499], cmap='viridis')
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax2.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax2.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax2.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax2.set_zlabel('Z-axis',fontsize=35, labelpad=35)
ax2.set_title('IDW Interpolation at Time t=1500 hours',fontsize=45)

# Third interpolated plot
ax3 = fig.add_subplot(133, projection='3d')
surface3 = ax3.plot_surface(x, y, nineDroneNearestInterpolated[1499], cmap='viridis')
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax3.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax3.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax3.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax3.set_zlabel('Z-axis',fontsize=35, labelpad=35)
ax3.set_title('Nearest neighbor Interpolation at Time t=1500 hours',fontsize=45)

plt.tight_layout()
# Create a single colorbar for both subplots
cax = fig.add_axes([0.99, 0.1, 0.02, 0.8])  # [x_position, y_position, width, height]
colorbar = fig.colorbar(surface1, cax=cax, shrink=0.6)
colorbar.ax.tick_params(axis='y', labelsize=25)
# Show the plot
plt.show()
#%% For t=2000 hours
# Plot the 3-dim data
x, y = np.meshgrid(np.arange(10), np.arange(10))
fig = plt.figure(figsize = (55, 15))

# First ground truth plot
ax1 = fig.add_subplot(131, projection='3d')
surface1 = ax1.plot_surface(x, y, groundTruth[1999], cmap='viridis')
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax1.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax1.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax1.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax1.set_zlabel('Z-axis',fontsize=35, labelpad=35)
ax1.set_title('Ground Truth at Time t=2000 hours',fontsize=45)
#colorbar1 = fig.colorbar(surface1, ax=ax1, shrink=0.5, aspect=10)
# Second linear interpolated plot
ax2 = fig.add_subplot(132, projection='3d')
surface2 = ax2.plot_surface(x, y, nineDroneIDWInterpolated[1999], cmap='viridis')
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax2.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax2.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax2.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax2.set_zlabel('Z-axis',fontsize=35, labelpad=35)
ax2.set_title('IDW Interpolation at Time t=2000 hours',fontsize=45)

# Third interpolated plot
ax3 = fig.add_subplot(133, projection='3d')
surface3 = ax3.plot_surface(x, y, nineDroneNearestInterpolated[1999], cmap='viridis')
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax3.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax3.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax3.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax3.set_zlabel('Z-axis',fontsize=35, labelpad=35)
ax3.set_title('Nearest neighbor Interpolation at Time t=2000 hours',fontsize=45)

plt.tight_layout()
# Create a single colorbar for both subplots
cax = fig.add_axes([0.99, 0.1, 0.02, 0.8])  # [x_position, y_position, width, height]
colorbar = fig.colorbar(surface1, cax=cax, shrink=0.6)
colorbar.ax.tick_params(axis='y', labelsize=25)
# Show the plot
plt.show()
#%% For t=2500 hours
# Plot the 3-dim data
x, y = np.meshgrid(np.arange(10), np.arange(10))
fig = plt.figure(figsize = (55, 15))

# First ground truth plot
ax1 = fig.add_subplot(131, projection='3d')
surface1 = ax1.plot_surface(x, y, groundTruth[2499], cmap='viridis')
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax1.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax1.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax1.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax1.set_zlabel('Z-axis',fontsize=35, labelpad=35)
ax1.set_title('Ground Truth at Time t=2500 hours',fontsize=45)
#colorbar1 = fig.colorbar(surface1, ax=ax1, shrink=0.5, aspect=10)
# Second linear interpolated plot
ax2 = fig.add_subplot(132, projection='3d')
surface2 = ax2.plot_surface(x, y, nineDroneIDWInterpolated[2499], cmap='viridis')
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax2.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax2.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax2.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax2.set_zlabel('Z-axis',fontsize=35, labelpad=35)
ax2.set_title('IDW Interpolation at Time t=2500 hours',fontsize=45)

# Third interpolated plot
ax3 = fig.add_subplot(133, projection='3d')
surface3 = ax3.plot_surface(x, y, nineDroneNearestInterpolated[2499], cmap='viridis')
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax3.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax3.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax3.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax3.set_zlabel('Z-axis',fontsize=35, labelpad=35)
ax3.set_title('Nearest neighbor Interpolation at Time t=2500 hours',fontsize=45)

plt.tight_layout()
# Create a single colorbar for both subplots
cax = fig.add_axes([0.99, 0.1, 0.02, 0.8])  # [x_position, y_position, width, height]
colorbar = fig.colorbar(surface1, cax=cax, shrink=0.6)
colorbar.ax.tick_params(axis='y', labelsize=25)
# Show the plot
plt.show()
#%% For t=2948 hours
# Plot the 3-dim data
x, y = np.meshgrid(np.arange(10), np.arange(10))
fig = plt.figure(figsize = (55, 15))

# First ground truth plot
ax1 = fig.add_subplot(131, projection='3d')
surface1 = ax1.plot_surface(x, y, groundTruth[2947], cmap='viridis')
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax1.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax1.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax1.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax1.set_zlabel('Z-axis',fontsize=35, labelpad=35)
ax1.set_title('Ground Truth at Time t=2948 hours',fontsize=45)
#colorbar1 = fig.colorbar(surface1, ax=ax1, shrink=0.5, aspect=10)
# Second linear interpolated plot
ax2 = fig.add_subplot(132, projection='3d')
surface2 = ax2.plot_surface(x, y, nineDroneIDWInterpolated[2947], cmap='viridis')
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax2.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax2.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax2.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax2.set_zlabel('Z-axis',fontsize=35, labelpad=35)
ax2.set_title('IDW Interpolation at Time t=2948 hours',fontsize=45)

# Third interpolated plot
ax3 = fig.add_subplot(133, projection='3d')
surface3 = ax3.plot_surface(x, y, nineDroneNearestInterpolated[2947], cmap='viridis')
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax3.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax3.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax3.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax3.set_zlabel('Z-axis',fontsize=35, labelpad=35)
ax3.set_title('Nearest neighbor Interpolation at Time t=2948 hours',fontsize=45)

plt.tight_layout()
# Create a single colorbar for both subplots
cax = fig.add_axes([0.99, 0.1, 0.02, 0.8])  # [x_position, y_position, width, height]
colorbar = fig.colorbar(surface1, cax=cax, shrink=0.6)
colorbar.ax.tick_params(axis='y', labelsize=25)
# Show the plot
plt.show()
#%%
nineDroneIDWError_list = []
nineDroneNearestError_list = []
overall_idw_rmse = 0
overall_nearest_rmse = 0
for z in range(0, 2948):
    nineDroneIDWError = 0
    nineDroneNearestError = 0
    for i in range(0, 10):
        for j in range(0, 10):
            nineDroneIDWError += ((groundTruth[z][i][j]-nineDroneIDWInterpolated[z][i][j])/groundTruth[z][i][j]) **2
            nineDroneNearestError += ((groundTruth[z][i][j]-nineDroneNearestInterpolated[z][i][j])/groundTruth[z][i][j]) **2
    nineDroneIDWError /= (10*10)
    overall_idw_rmse += nineDroneIDWError
    nineDroneIDWError = math.sqrt(nineDroneIDWError)
    
    nineDroneNearestError /= (10*10)
    overall_nearest_rmse += nineDroneNearestError
    nineDroneNearestError = math.sqrt(nineDroneNearestError)

    
    nineDroneIDWError_list.append(nineDroneIDWError*100)
    nineDroneNearestError_list.append(nineDroneNearestError*100)
    
overall_idw_rmse /= (2948)
overall_idw_rmse = math.sqrt(overall_idw_rmse)

overall_nearest_rmse /= (2948)
overall_nearest_rmse = math.sqrt(overall_nearest_rmse)

print(f'The overall RMSE using IDW interpolation is: {overall_idw_rmse*100}')
print(f'The overall RMSE using nearest neighbor interpolation is: {overall_nearest_rmse*100}')
#%% RMSE Plot
fig = plt.figure(figsize = (12, 10))
ax=plt.axes()
# Create a time array for x-axis
time_points = range(0, 2948)
selected_time_points = np.arange(0,2948,100)
selected_IDW_errors = []
selected_Nearest_errors = []
for i in selected_time_points:
    selected_IDW_errors.append(nineDroneIDWError_list[i])
    selected_Nearest_errors.append(nineDroneNearestError_list[i])
# Plot the RMS errors
#plt.plot(time_points, fourDroneIDWError_list, linewidth= 2,label='IDW Interpolation')
plt.plot(selected_time_points, selected_IDW_errors, linewidth= 7,label='IDW Interpolation')
plt.plot(selected_time_points, selected_Nearest_errors, linewidth= 7,label='Nearest Neighbor Interpolation')
plt.yticks(fontsize=25)
plt.xticks(fontsize=25)
# Add labels and title
plt.xlabel('Time', fontsize=30)
plt.ylabel('RMS Error (%)', fontsize=30)
plt.title('RMS Error over Time', fontsize=30)

# Add legend
plt.legend()

# Show the plot
plt.show()
