from scipy.interpolate import griddata

import math

import numpy as np
from scipy.interpolate import NearestNDInterpolator
from scipy.interpolate import LinearNDInterpolator
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import cKDTree

def cal(xcord, ycord, timeInstant):
    #return (math.sin(3.2 * 3.14 * 0.00005 * timeInstant) + 1) * (math.sin(0.2 * xcord) + math.sin(0.2 * ycord)) + 50
    return (math.sin(3.2 * 3.14 * 0.00005 * timeInstant) + 1) * (math.sin(2 * 0.1 * xcord)) + 30 + (
            math.sin(3.2 * 3.14 * 0.00005 * timeInstant) + 1) * (math.sin(2 * 0.1 * ycord)) + 20

x_lim = 50
y_lim = 50
t_lim = 100

x = []
y = []
time = []

i = 0
while i <= 49:
    x.append(i)
    y.append(i)
    i += 1

i = 0
while i <= 99:
    time.append(i)
    i += 1

groundTruth = np.zeros((100, 50, 50))
for z in range(0, 100):
    for i in range(0, 50):
        for j in range(0, 50):
            groundTruth[z][i][j] = cal(x[i], y[j], time[z])

r = 25
c = 50
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
#print(pts1)

pts2 = []
for i in range(0, len(pts1)):
    pts2.append([pts1[i][0] + 25, pts1[i][1]])

XYT2Drones = []

t = 0
i = 0
while t < 100:
    XYT2Drones.append([pts1[i][0], pts1[i][1], t])
    XYT2Drones.append([pts2[i][0], pts2[i][1], t])
    i += 2
    i %= len(pts1)
    t += 1

twoDrones = []
for i in range(0, len(XYT2Drones)):
    twoDrones.append(cal(XYT2Drones[i][0], XYT2Drones[i][1], XYT2Drones[i][2]))
    
twoDrones = np.array(twoDrones)
coordinates_array = np.array(XYT2Drones)

# Split the coordinates array into individual arrays for x, y, z
x_coords = coordinates_array[:, 0]
y_coords = coordinates_array[:, 1]
z_coords = coordinates_array[:, 2]


twoDroneNearestInterpolated = np.zeros((100, 50, 50))

#%%
print('Performing interpolation')
def idw_interpolation(known_coords, known_values, query_coords, power=5):
    tree = cKDTree(known_coords)
    distances, indices = tree.query(query_coords, k=10)  # You can adjust the number of neighbors (k) as needed

    weights = 1 / ((distances + 1e-10) ** power)
    interpolated_values = np.sum(weights * known_values[indices], axis=1) / np.sum(weights, axis=1)

    return interpolated_values

#%%
twoDroneIDWInterpolated = np.zeros((t_lim,x_lim,y_lim))
interpolation_coordinates = []
for z in range(0, t_lim):
    # print(z)
    for i in range(0, x_lim):
        for j in range(0, y_lim):
            #fourDronesKrigedInterpolated[z][i][j],var = ok3d.execute('points', z,i,j)
            interpolation_coordinates.append([i,j,z])
            
nn_interpolated_values = griddata(coordinates_array, twoDrones, interpolation_coordinates, method='nearest')          
idw_interpolated_values = idw_interpolation(coordinates_array, twoDrones, interpolation_coordinates)
#%%
counter = 0
for z in range(0, t_lim):
    # print(z)
    for i in range(0, x_lim):
        for j in range(0, y_lim):
            coord = interpolation_coordinates[counter]
            twoDroneIDWInterpolated[coord[2]][coord[0]][coord[1]] = idw_interpolated_values[counter]
            twoDroneNearestInterpolated[coord[2]][coord[0]][coord[1]] = nn_interpolated_values[counter]
            counter += 1
 #%% For t=1 hours
# Plot the 3-dim data
x, y = np.meshgrid(np.arange(50), np.arange(50))
fig = plt.figure(figsize = (18, 15))

# First ground truth plot
ax1 = fig.add_subplot(111, projection='3d')
surface1 = ax1.plot_surface(x, y, groundTruth[59], cmap='viridis')
plt.rcParams['font.family'] = 'Times New Roman'
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax1.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax1.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax1.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax1.set_zlabel('Temperature',fontsize=35, labelpad=35)
#ax1.set_title('Ground Truth at Time t=10 hours',fontsize=45)
plt.tight_layout()
# Create a single colorbar for both subplots
#cax = fig.add_axes([0.99, 0.1, 0.02, 0.8])  # [x_position, y_position, width, height]
# colorbar = fig.colorbar(surface1, shrink=0.6)
# colorbar.ax.tick_params(axis='y', labelsize=25)
# Show the plot
plt.show()         
 #%% For t=1 hours
# Plot the 3-dim data
x, y = np.meshgrid(np.arange(50), np.arange(50))
fig = plt.figure(figsize = (18, 15))

# First ground truth plot
ax1 = fig.add_subplot(111, projection='3d')
surface1 = ax1.plot_surface(x, y, twoDroneIDWInterpolated[59], cmap='viridis')
plt.rcParams['font.family'] = 'Times New Roman'
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax1.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax1.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax1.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax1.set_zlabel('Temperature',fontsize=35, labelpad=35)
#ax1.set_title('Ground Truth at Time t=10 hours',fontsize=45)
plt.tight_layout()
# Create a single colorbar for both subplots
#cax = fig.add_axes([0.99, 0.1, 0.02, 0.8])  # [x_position, y_position, width, height]
# colorbar = fig.colorbar(surface1, shrink=0.6)
# colorbar.ax.tick_params(axis='y', labelsize=25)
# Show the plot
plt.show()       
#%% For t=1 hours
# Plot the 3-dim data
x, y = np.meshgrid(np.arange(50), np.arange(50))
fig = plt.figure(figsize = (18, 15))

# First ground truth plot
ax1 = fig.add_subplot(111, projection='3d')
surface1 = ax1.plot_surface(x, y, twoDroneNearestInterpolated[59], cmap='viridis')
plt.rcParams['font.family'] = 'Times New Roman'
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax1.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax1.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax1.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax1.set_zlabel('Temperature',fontsize=35, labelpad=35)
#ax1.set_title('Ground Truth at Time t=10 hours',fontsize=45)
plt.tight_layout()
# Create a single colorbar for both subplots
#cax = fig.add_axes([0.99, 0.1, 0.02, 0.8])  # [x_position, y_position, width, height]
# colorbar = fig.colorbar(surface1, shrink=0.6)
# colorbar.ax.tick_params(axis='y', labelsize=25)
# Show the plot
plt.show()      
#%% For t=1 hours
# Plot the 3-dim data
x, y = np.meshgrid(np.arange(50), np.arange(50))
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
surface2 = ax2.plot_surface(x, y, twoDroneIDWInterpolated[0], cmap='viridis')
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
surface3 = ax3.plot_surface(x, y, twoDroneNearestInterpolated[0], cmap='viridis')
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
#%% For t=20 hours
# Plot the 3-dim data
x, y = np.meshgrid(np.arange(50), np.arange(50))
fig = plt.figure(figsize = (55, 15))

# First ground truth plot
ax1 = fig.add_subplot(131, projection='3d')
surface1 = ax1.plot_surface(x, y, groundTruth[19], cmap='viridis')
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax1.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax1.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax1.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax1.set_zlabel('Z-axis',fontsize=35, labelpad=35)
ax1.set_title('Ground Truth at Time t=20 hours',fontsize=45)
#colorbar1 = fig.colorbar(surface1, ax=ax1, shrink=0.5, aspect=10)
# Second linear interpolated plot
ax2 = fig.add_subplot(132, projection='3d')
surface2 = ax2.plot_surface(x, y, twoDroneIDWInterpolated[19], cmap='viridis')
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax2.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax2.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax2.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax2.set_zlabel('Z-axis',fontsize=35, labelpad=35)
ax2.set_title('IDW Interpolation at Time t=20 hours',fontsize=45)

# Third interpolated plot
ax3 = fig.add_subplot(133, projection='3d')
surface3 = ax3.plot_surface(x, y, twoDroneNearestInterpolated[19], cmap='viridis')
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax3.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax3.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax3.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax3.set_zlabel('Z-axis',fontsize=35, labelpad=35)
ax3.set_title('Nearest neighbor Interpolation at Time t=20 hours',fontsize=45)

plt.tight_layout()
# Create a single colorbar for both subplots
cax = fig.add_axes([0.99, 0.1, 0.02, 0.8])  # [x_position, y_position, width, height]
colorbar = fig.colorbar(surface1, cax=cax, shrink=0.6)
colorbar.ax.tick_params(axis='y', labelsize=25)
# Show the plot
plt.show()
#%% For t=50 hours
# Plot the 3-dim data
x, y = np.meshgrid(np.arange(50), np.arange(50))
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
surface2 = ax2.plot_surface(x, y, twoDroneIDWInterpolated[49], cmap='viridis')
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
surface3 = ax3.plot_surface(x, y, twoDroneNearestInterpolated[49], cmap='viridis')
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
#%% For t=100 hours
# Plot the 3-dim data
x, y = np.meshgrid(np.arange(50), np.arange(50))
fig = plt.figure(figsize = (55, 15))

# First ground truth plot
ax1 = fig.add_subplot(131, projection='3d')
surface1 = ax1.plot_surface(x, y, groundTruth[99], cmap='viridis')
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax1.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax1.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax1.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax1.set_zlabel('Z-axis',fontsize=35, labelpad=35)
ax1.set_title('Ground Truth at Time t=100 hours',fontsize=45)
#colorbar1 = fig.colorbar(surface1, ax=ax1, shrink=0.5, aspect=10)
# Second interpolated plot
ax2 = fig.add_subplot(132, projection='3d')
surface2 = ax2.plot_surface(x, y, twoDroneIDWInterpolated[99], cmap='viridis')
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax2.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax2.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax2.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax2.set_zlabel('Z-axis',fontsize=35, labelpad=35)
ax2.set_title('IDW Interpolation at Time t=100 hours',fontsize=45)

# Third interpolated plot
ax3 = fig.add_subplot(133, projection='3d')
surface3 = ax3.plot_surface(x, y, twoDroneNearestInterpolated[99], cmap='viridis')
plt.yticks(fontsize=32)
plt.xticks(fontsize=32)
ax3.tick_params(axis='z', labelsize=32, pad=15)

# Add labels
ax3.set_xlabel('X-axis',fontsize=35, labelpad=25)
ax3.set_ylabel('Y-axis',fontsize=35, labelpad=25)
ax3.set_zlabel('Z-axis',fontsize=35, labelpad=35)
ax3.set_title('Nearest neighbor Interpolation at Time t=100 hours',fontsize=45)

plt.tight_layout()
# Create a single colorbar for both subplots
cax = fig.add_axes([0.99, 0.1, 0.02, 0.8])  # [x_position, y_position, width, height]
colorbar = fig.colorbar(surface1, cax=cax, shrink=0.6)
colorbar.ax.tick_params(axis='y', labelsize=25)
# Show the plot
plt.show()
#%%
twoDroneIDWError_list = []
twoDroneNearestError_list = []
overall_idw_rmse = 0
overall_nearest_rmse = 0
for z in range(0, t_lim):
    twoDroneIDWError = 0
    twoDroneNearestError = 0
    for i in range(0, x_lim):
        for j in range(0, y_lim):
            twoDroneIDWError += ((groundTruth[z][i][j]-twoDroneIDWInterpolated[z][i][j])/groundTruth[z][i][j]) **2
            twoDroneNearestError += ((groundTruth[z][i][j]-twoDroneNearestInterpolated[z][i][j])/groundTruth[z][i][j]) **2
    twoDroneIDWError /= (x_lim*y_lim)
    overall_idw_rmse += twoDroneIDWError
    twoDroneIDWError = math.sqrt(twoDroneIDWError)
    
    twoDroneNearestError /= (x_lim*y_lim)
    overall_nearest_rmse += twoDroneNearestError
    twoDroneNearestError = math.sqrt(twoDroneNearestError)

    
    twoDroneIDWError_list.append(twoDroneIDWError*100)
    twoDroneNearestError_list.append(twoDroneNearestError*100)
    
overall_idw_rmse /= (t_lim)
overall_idw_rmse = math.sqrt(overall_idw_rmse)

overall_nearest_rmse /= (t_lim)
overall_nearest_rmse = math.sqrt(overall_nearest_rmse)

print(f'The overall RMSE using IDW interpolation is: {overall_idw_rmse*100}')
print(f'The overall RMSE using nearest neighbor interpolation is: {overall_nearest_rmse*100}')
#%% RMSE Plot
fig = plt.figure(figsize = (12, 10))
ax=plt.axes()
# Create a time array for x-axis
time_points = range(0, t_lim)
selected_time_points = np.arange(0,100,20)
selected_IDW_errors = []
selected_Nearest_errors = []
for i in selected_time_points:
    selected_IDW_errors.append(twoDroneIDWError_list[i])
    selected_Nearest_errors.append(twoDroneNearestError_list[i])
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