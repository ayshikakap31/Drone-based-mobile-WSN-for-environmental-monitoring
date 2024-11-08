
import nest_asyncio
import numpy as np
from scipy import interpolate
import asyncio
import aiohttp
import json
import math
import matplotlib.pyplot as plt
import platform
from scipy.interpolate import griddata
from scipy.spatial import cKDTree

nest_asyncio.apply()
entries = 48
groundTruth = np.zeros((entries, entries, 3))
avg = 0

async def get_temperature(i: int, j: int):
    url = "https://api.weatherapi.com/v1/forecast.json?key=834f4b44da6c4bbd826101530232002&q=" + str(i) + "," + str(
        j) + "&days=10&aqi=no&alerts=no"
    print(url)
    tempList = []
    async with aiohttp.ClientSession() as session:
        async with session.get(url) as response:
            data = await response.read()
            data = json.loads(data.decode())
            forecast = data['forecast']['forecastday']
            for forecast_data in forecast:
                tempList.append(forecast_data['day']['avgtemp_c'])
            return tempList


async def prepare_ground_truth():
    for i in range(entries):
        for j in range(entries):
            tempList = await get_temperature(i, j)
            for z in range(3):
                groundTruth[i][j][z] = tempList[z]
            print(f'{i}, {j}, {tempList[0]}')


if platform.system() == 'Windows':
    asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
loop = asyncio.new_event_loop()
loop.run_until_complete(prepare_ground_truth())

print('Done!!!')

for i in range(entries):
    for j in range(entries):
        for z in range(3):
            avg += groundTruth[i][j][z]
            
r = 25
c = 25
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
for i in range(0, len(pts1)):
    pts2.append([pts1[i][0], pts1[i][1] + 25])
    pts3.append([pts1[i][0] + 25, pts1[i][1]])
    pts4.append([pts1[i][0] + 25, pts1[i][1] + 25])

#print(pts1)
#print(pts2)
#print(pts3)
#print(pts4)

XYTPts = []

t = 0
i = 0
while t < 100:
    XYTPts.append([pts1[i][0], pts1[i][1], t])
    XYTPts.append([pts2[i][0], pts2[i][1], t])
    XYTPts.append([pts3[i][0], pts3[i][1], t])
    XYTPts.append([pts4[i][0], pts4[i][1], t])
    i += 3
    i %= len(pts1)
    t += 1
#print(XYTPts)
# Sort XYT based on X, then Y, then T
XYTPts_sorted = sorted(XYTPts, key=lambda coord: (coord[0], coord[1], coord[2]))
fourDrones = []
for i in range(0, len(XYTPts)):
    fourDrones.append(groundTruth[XYTPts[i][2]][XYTPts[i][0]][XYTPts[i][1]])
    

fourDrones = np.array(fourDrones)
coordinates_array = np.array(XYTPts)

# Split the coordinates array into individual arrays for x, y, z
x_coords = coordinates_array[:, 0]
y_coords = coordinates_array[:, 1]
z_coords = coordinates_array[:, 2]


fourDronesNearestInterpolated = np.zeros((100, 50, 50))

# Split the coordinates array into individual arrays for x, y, z
x_coords = coordinates_array[:, 0]
y_coords = coordinates_array[:, 1]
z_coords = coordinates_array[:, 2]

print('Performing interpolation')
def idw_interpolation(known_coords, known_values, query_coords, power=2):
    tree = cKDTree(known_coords)
    distances, indices = tree.query(query_coords, k=5)  # You can adjust the number of neighbors (k) as needed

    weights = 1 / ((distances + 1e-10) ** power)
    interpolated_values = np.sum(weights * known_values[indices], axis=1) / np.sum(weights, axis=1)

    return interpolated_values

nineDronesNearestInterpolated = np.zeros((3, entries, entries))
nineDronesIDWInterpolated = np.zeros((3, entries, entries))

interpolation_coordinates = []
for z in range(0, 3):
    # print(z)
    for i in range(0, entries):
        for j in range(0, entries):
            interpolation_coordinates.append([i,j,z])

nn_interpolated_values = griddata(coordinates_array, fourDrones, interpolation_coordinates, method='nearest')
idw_interpolated_values = idw_interpolation(coordinates_array, fourDrones, interpolation_coordinates)

counter = 0
for z in range(0, 3):
    # print(z)
    for i in range(0, entries):
        for j in range(0, entries):
            coord = interpolation_coordinates[counter]
            nineDronesIDWInterpolated[coord[2]][coord[0]][coord[1]] = idw_interpolated_values[counter]
            nineDronesNearestInterpolated[coord[2]][coord[0]][coord[1]] = nn_interpolated_values[counter]
            counter += 1

overall_idw_rmse = 0
overall_nearest_rmse = 0

for z in range(0, 3):
  co = 0
  for i in range(0, entries):
    for j in range(0, entries):
      if (groundTruth[i][j][z] != 0):
        co += 1
        overall_idw_rmse += ((groundTruth[i][j][z]-nineDronesIDWInterpolated[z][i][j])/groundTruth[i][j][z]) **2
        overall_nearest_rmse += ((groundTruth[i][j][z]-nineDronesNearestInterpolated[z][i][j])/groundTruth[i][j][z]) **2

  overall_idw_rmse /= (co*co)
  overall_nearest_rmse /= (co*co)

overall_idw_rmse /= 3
overall_idw_rmse = math.sqrt(overall_idw_rmse)
overall_nearest_rmse /= 3
overall_nearest_rmse = math.sqrt(overall_nearest_rmse)
print(f'The overall RMSE using IDW interpolation is: {overall_idw_rmse*100}')
print(f'The overall RMSE using nearest neighbor interpolation is: {overall_nearest_rmse*100}')