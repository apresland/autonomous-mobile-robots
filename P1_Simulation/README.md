# Robotic Environment Simulation
![gazebo_simulation](https://user-images.githubusercontent.com/5468707/121535884-d8bde100-ca02-11eb-96a3-1981ffe1a6a7.png)

## Prerequisites
This project requires that ROS and Gazebo are installed along with the gcc compiler.

## Build
* Change to the project folder
* Create a build folder `mkdir build && cd build`
* Build with cmake `cmake .. && make`
* Add the build folder to the Gazebo plugin path: `GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:<path_to_build_folder>`.
* Change to the project folder
* Launch the simulation `gazebo world/office`

## Structure
```
.
├── CMakeLists.txt
├── model
│   ├── Building
│   │   ├── model.config
│   │   └── model.sdf
│   └── Robot
│       ├── model.config
│       └── model.sdf
├── README.md
├── script
│   └── hello.cpp
└── world
    └── office
```
