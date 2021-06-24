#Perception

## Prerequisites
This project requires ROS and Gazebo along with the gcc compiler

## Build
1. Initialize the project as a catkin workspace
```console
$ cd src && catkin_init_workspace
```

2. Move to the project director and build
```console
$ cd ..
$ catkin_make
```

3. Launch the simulation
```console
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```
4. Start the ball chaser in another terminal
```console
$ source devel/setup.bash
$ roslaunch ball_chaser ball_chaser.launch
```

## Structure
```
.
└── src
    ├── ball_chaser
    │   ├── CMakeLists.txt
    │   ├── include
    │   │   └── ball_chaser
    │   ├── launch
    │   │   └── ball_chaser.launch
    │   ├── package.xml
    │   ├── src
    │   │   ├── drive_bot.cpp
    │   │   └── process_image.cpp
    │   └── srv
    │       └── DriveToTarget.srv
    ├── CMakeLists.txt
    └── my_robot
        ├── CMakeLists.txt
        ├── launch
        │   ├── robot_description.launch
        │   └── world.launch
        ├── meshes
        │   └── hokuyo.dae
        ├── package.xml
        ├── urdf
        │   ├── my_robot.gazebo
        │   └── my_robot.xacro
        └── world
            └── andrews.world

```
