# Examination of Long Term Environment Effects on Robot Localization Using RI-EKF and the NCLT Dataset
ROB 530: Mobile Robotics Winter 2021

Final Project Team 16: Lars Grantz, Ting-Wei Hsu, Adarsh Karnati

# Introduction
This project aims at evaluating robustness of the right-invariant extended Kalman filter(RI-EKF) against long term environmental changes. RI-EKF localization is implemented on the NCLT dataset and its performance is analyzed across different sessions. This repository includes Python code that allows to:
1. extract landmarks from 3-D lidar scans
2. create a global reference map of landmarks
3. implement RI-EKF localization on this map
4. evaluate performance of RI-EKF localization

# NCLT Dataset
The NCLT Dataset is a large-scale, long-term dataset collected on the North Campus of the University of Michigan. The dataset consists of vision data, lidar data, GPS, and IMU data collected using a Segway robot. The reason for choosing this dataset is that it includes a wide range of long-term environmental changes, such as seasonal changes, lighting changes, and structural changes. The data can be downloaded [here](http://robots.engin.umich.edu/nclt/index.html). Use the provided ```downloader.py``` to download sensor data, Velodyne(3D lidar)data, and the ground truth.

# Landmarks Extraction and Mapping
Landmarks extraction and mapping are implemented using the algorithm proposed in [this paper](http://ais.informatik.uni-freiburg.de/publications/papers/schaefer19ecmr.pdf) (Schaefer et al. 2019). Use Python code in [this repository](https://github.com/acschaefer/polex) and install all the required modules to generate the global reference map. The resulting global map is shown in Fig. 1.


![image](https://user-images.githubusercontent.com/78635240/114791206-1e886100-9d54-11eb-88e6-7f145a342863.png)

Fig. 1: global reference map

# RI-EKF Localization
The RI-EKF localization is implemented using ```inEKF.py```. In ```inEKF.py```:
1. robot pose in SE(2) is considered
2. for the process model, the odometry data are used to propagate the states
3. for the measurement model, a ball tree search algorithm is employed for data association between online lidar scans and the landmarks in the global reference map. Mahalanobis distance is used as the distance metric and the landmarks are accepted/rejected based on Chi-square tests.

# Running the Code
Once complete downloading the data from NCLT dataset and building the dependencies, run ```ncltpoles.py``` for the overall maping and localization process. In ```ncltpoles.py```:
1. ```save_global_map()``` builds the global reference map of landmarks
2. ```save_local_maps(session_name)``` builds the local map (i.e. online lidar scans) of detected landmarks
3. ```localize(session_name, visualize = False)``` implements RI-EKF localization on the global reference map; it also plots the NEES (normalized estimation error squared) graph to evaluate the consistency of the RI-EKF, as shown in Fig. 2.
4. ```plot_trajectories()``` plots the trajectories of both the estimated and ground truth poses, as shown in Fig. 3.
5. ```evaluate()``` computes the mean error and root-mean-square error of between the estimated and ground truth poses


![2012-01-08_NEES](https://user-images.githubusercontent.com/78635240/114791112-e719b480-9d53-11eb-9be5-4881533e86b9.png)

Fig. 2: the NEES plot 

![2012-01-08_2021-04-13_23-55-39](https://user-images.githubusercontent.com/78635240/114791134-f7319400-9d53-11eb-87d9-6b327767e1c6.png)

Fig. 3: estimated and ground truth trajectories 
