# Drone-based-mobile-WSN-for-environmental-monitoring

This repository contains code and resources for a research project focused on an efficient drone-based mobile Wireless Sensor Network (WSN) system for environmental monitoring. This project builds on the advancements in WSN and UAV technology to enable real-time, large-scale environmental monitoring. By integrating sensors directly on drones, our approach eliminates the need for static ground-based sensors, improving data collection accuracy and coverage.

## Background

Traditional WSN systems use static sensors to monitor environmental parameters such as temperature, humidity, and air quality. However, static deployments face limitations in terms of coverage, adaptability to dynamic environmental changes, and maintenance costs, especially in remote areas. This project proposes an innovative method using drones equipped with WSN capabilities to cover large geographical areas efficiently. The drones collect data while moving along predefined trajectories, and interpolation techniques such as Inverse Distance Weighting (IDW) and Nearest Neighbors (NN) are used to estimate data at intermediate points, ensuring complete spatio-temporal coverage.

## Methodology

The framework divides the target area into a 3D spatial-temporal field, further subdivided into sub-grids for optimized drone deployment. Drones, equipped with sensors, follow a spiral trajectory within their assigned regions to collect spatio-temporal data.

### Interpolation Techniques

Due to the nature of drone movement, data is not available at every possible location and time. This project uses interpolation methods to fill in data gaps:
- **Inverse Distance Weighting (IDW):** Provides a weighted average based on proximity, prioritizing nearby values for greater accuracy.
- **Nearest Neighbors (NN):** Estimates missing values based on the closest sampled location, which is computationally efficient and effective in dense regions.

## Datasets

### Simulated Dataset
The temperature is modeled as a sinusoidal function of coordinates coordinates `X`, `Y`, and time `T`:
\[
Z = (sin(3.2 * π * 0.00005 * T) + 1) * (sin(0.2 * X) + sin(0.2 * Y)) + 50
\]
where:
- `Z`: Temperature,
- `X`, `Y`: Coordinates `[0,50)`,
- `T`: Time (0 to 60).

The spatial region is discretized into a \(50 \times 50\) grid. At \( \mathcal{T} = 10 \), the temperature variation aligns with real-world conditions.

### NYC Open Weather Dataset
Utilizes temperature data from New York City, collected over 100 locations across 2948 hours. The spatial-temporal region is divided into a \(10 \times 10 \times 2948\) field, reflecting typical NYC summer temperatures.

### Weather REST API
Real-time data from a weather API for Europe, sampled over three days in a \(48 \times 48 \times 3\) spatial-temporal field. Each latitude and longitude in the 48x48 grid has a corresponding API call to gather real-time temperature data.

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/ayshikakap31/Drone-based-mobile-WSN-for-environmental-monitoring.git
   cd Drone-based-mobile-WSN-for-environmental-monitoring
2. Install the required libraries:
   pip install -r requirements.txt

## Results and Evaluation
The system's performance is evaluated by comparing the UAV-based approach to traditional static WSN deployments. The evaluation focuses on Root Mean Square Error (RMSE) as a metric, which measures the accuracy of the interpolated values against ground truth data. The UAV-based model demonstrates substantial advantages over static WSN systems, achieving lower RMSE values, greater coverage, and enhanced adaptability in real-time conditions.
