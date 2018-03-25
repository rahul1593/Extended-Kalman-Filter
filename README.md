# Extended-Kalman-Filter

In this project, I have implemented a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.

This project involves the Term 2 Simulator which can be downloaded <a href="https://github.com/udacity/self-driving-car-sim/releases">here</a>.

__To build the project run the following commands:__
1. `cd ./build`
2. `make`

Run `./ExtendedKF` to run.

### Changes made

1. kalman_filter.h: Add function declarations for angle normalisaion and coordinate system conversion, defined in kalman_filter.cpp
2. FusionEKF.cpp: Completed initialisation and measurement methods
3. tools.cpp: Completed functions for calculating jacobian matrix and RMSE
