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
        
r = 5
c = 10
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
for i in range(0, len(pts1)):
    pts2.append([pts1[i][0] + 5, pts1[i][1]])
    
XYTPts = []

t = 0
i = 0
while t < 2948:
    XYTPts.append([pts1[i][0], pts1[i][1], t])
    XYTPts.append([pts2[i][0], pts2[i][1], t])
    i += 2
    i %= len(pts1)
    t += 1
    
twoDrones = []
for i in range(0, len(XYTPts)):
    twoDrones.append(groundTruth[XYTPts[i][2]][XYTPts[i][0]][XYTPts[i][1]])
    
twoDrones = np.array(twoDrones)
coordinates_array = np.array(XYTPts)

# Split the coordinates array into individual arrays for x, y, z
x_coords = coordinates_array[:, 0]
y_coords = coordinates_array[:, 1]
z_coords = coordinates_array[:, 2]
#%%
print('Performing interpolation')
def idw_interpolation(known_coords, known_values, query_coords, power=2):
    tree = cKDTree(known_coords)
    distances, indices = tree.query(query_coords, k=3)  # You can adjust the number of neighbors (k) as needed

    weights = 1 / ((distances + 1e-10) ** power)
    interpolated_values = np.sum(weights * known_values[indices], axis=1) / np.sum(weights, axis=1)

    return interpolated_values

#%%
twoDronesIDWInterpolated = np.zeros((2948,10,10))
twoDronesNearestInterpolated = np.zeros((2948, 10, 10))

interpolation_coordinates = []
for z in range(0, 2948):
    # print(z)
    for i in range(0, 10):
        for j in range(0, 10):
            #twoDronesKrigedInterpolated[z][i][j],var = ok3d.execute('points', z,i,j)
            interpolation_coordinates.append([i,j,z])
            
nn_interpolated_values = griddata(coordinates_array, twoDrones, interpolation_coordinates, method='nearest')          
idw_interpolated_values = idw_interpolation(coordinates_array, twoDrones, interpolation_coordinates)
#%%
counter = 0
for z in range(0, 2948):
    # print(z)
    for i in range(0, 10):
        for j in range(0, 10):
            coord = interpolation_coordinates[counter]
            twoDronesIDWInterpolated[coord[2]][coord[0]][coord[1]] = idw_interpolated_values[counter]
            twoDronesNearestInterpolated[coord[2]][coord[0]][coord[1]] = nn_interpolated_values[counter]
            counter += 1
#%%
twoDroneIDWError_list = []
twoDroneNearestError_list = []
overall_idw_rmse = 0
overall_nearest_rmse = 0
for z in range(0, 2948):
    twoDroneIDWError = 0
    twoDroneNearestError = 0
    for i in range(0, 10):
        for j in range(0, 10):
            twoDroneIDWError += ((groundTruth[z][i][j]-twoDronesIDWInterpolated[z][i][j])/groundTruth[z][i][j]) **2
            twoDroneNearestError += ((groundTruth[z][i][j]-twoDronesNearestInterpolated[z][i][j])/groundTruth[z][i][j]) **2
    twoDroneIDWError /= (10*10)
    overall_idw_rmse += twoDroneIDWError
    twoDroneIDWError = math.sqrt(twoDroneIDWError)
    
    twoDroneNearestError /= (10*10)
    overall_nearest_rmse += twoDroneNearestError
    twoDroneNearestError = math.sqrt(twoDroneNearestError)

    
    twoDroneIDWError_list.append(twoDroneIDWError*100)
    twoDroneNearestError_list.append(twoDroneNearestError*100)
    
overall_idw_rmse /= (2948)
overall_idw_rmse = math.sqrt(overall_idw_rmse)

overall_nearest_rmse /= (2948)
overall_nearest_rmse = math.sqrt(overall_nearest_rmse)

print(f'The overall RMSE using IDW interpolation is: {overall_idw_rmse*100}')
print(f'The overall RMSE using nearest neighbor interpolation is: {overall_nearest_rmse*100}')