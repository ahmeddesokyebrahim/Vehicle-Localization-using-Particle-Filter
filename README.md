# **Vehicle Localization Using Particle Filter**
---

**This project is a 2-D particle filter to localize a car given a map, sensor data (e.g., Lidar), and some initial localization information (analogous to what a GPS would provide) .**

## Project Introduction:
---
The vehicle has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.


## Environment:
---
* Ubuntu 16.04 LTS
* Udacity Self-Driving Car Nano-Degree Term2 Simulator
* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4

## Algorithm Implementation
---
Please refer the `particle_filter.cpp` and `particle_filter.h` inside the src folder.

The directory structure of this repository is as follows:
```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

## Running the Code
---
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Conclusion
---
  * The particle filter is a powerful tool that realized the probabilistic Bayes's Filter for Localization
  * The filter is capable to localize the vehicle with high percision using map data, sensor data (e.g., Lidar), and noisy GPS initial location
