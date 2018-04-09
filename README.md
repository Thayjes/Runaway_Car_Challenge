# Run Away Robot with Unscented Kalman Filter Bonus Challenge Code
Self-Driving Car Engineer Nanodegree Program

---

### Overview

This repository contains all the code needed to complete the Bonus Challenge: Catch the Run Away Car with Unscented Kalman Filter.

### Project Introduction

In this project, a UKF is implemented and also used to catch an escaped car driving in a circular path. 
The run away car is sensed by a stationary sensor, that is able to measure both noisy lidar and radar data. The capture vehicle will need to use these measurements to close in on the runaway car. To capture the runaway car the capture vehicle needs to come within .1 unit distance of its position. However the capture car and the runaway car have the same max velocity, so if the capture vehicle wants to catch the runaway car, it will need to predict where the car will be ahead of time.

### Results

1. [Standard noise settings in LASER and RADAR Measurements] (https://drive.google.com/open?id=1XMEe46nlFjkcFSGlKU2QuYIIRc1fsNME)

2. [With added noise in LASER Measurements alone] (https://drive.google.com/open?id=1JpEEOr1fB1iPscHdKc9ESPeyfIvCkeHv)

3. [Added noise in LASER and RADAR Measurements] (https://drive.google.com/open?id=1Az6JrFRb03D_4xBW9jTEiZqAEqoVELg1)

We can see that it does very well with standard noise, and as the noise in the measurements increase there is increased difficulty in catching the target car but eventually it does.

### Strategy

The strategy I used to capture the car utilized the fact that the car is executing uniform circular motion. Using the measurements (LIDAR and RADAR) I was able to create a fit for the circle and estimate the radius and position of the center.
With this in mind the procedure is as follows:

1. If the hunter car is outside the circle, bring it back to the center.

2. From the center, every point on the circle is the same distance away (i.e the radius of the circle). Also the max velocity of the hunter car is the same as the target car. So we know how long (i.e the time) the hunter car will take to go to any point on the circle.

3. Using this we can predict where the target car will be after this time and catch it.

### Running the Code

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

`mkdir build && cd build`

`cmake .. && make` 

`./UnscentedKF`


Here is the main protocol that `main.cpp` uses for uWebSocketIO in communicating with the simulator.

**INPUT**: values provided by the simulator to the C++ program



// current noiseless position state of the capture vehicle, called hunter

["hunter_x"]

["hunter_y"]

["hunter_heading"]

// get noisy lidar and radar measurements from the run away car.

["lidar_measurement"]

["radar_measurement"]


**OUTPUT**: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["turn"] <= the desired angle of the capture car "hunter" no limit for the angle

["dist"] <= the desired distance to move the capture car "hunter" can't move faster than run away car



## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4
* uWebSocketIO



## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF` 



