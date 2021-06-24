# Localization

![localization](https://user-images.githubusercontent.com/5468707/123265743-a2508d80-d4fb-11eb-9c79-727a6fdb5b85.gif)


We rely on __AMCL__, a probabilistic localization system for a robot moving in 2D, 
which implements the adaptive Monte Carlo localization approach first described by Dieter Fox.
It uses a particle filter to track the pose of a robot against a known map. The map is provided the map_server ROS Node, 
which offers map data as a ROS Service. Locomotion is provided by the move_base package providing an implementation of an action
that, given a goal in the world, will attempt to reach it with a mobile base. The move_base node links together a global and local planner 
to accomplish its global navigation task.

## Prerequisites
1. ROS and Gazebo
2. CMake and the GCC compiler
3. ROS dependencies
```console
$ sudo apt-get update && sudo apt-get upgrade -y
$ sudo apt-get install ros-${ROS_DISTRO}-map-server
$ sudo apt-get install ros-${ROS_DISTRO}-amcl
$ sudo apt-get install ros-${ROS_DISTRO}-move-base
```

## Build
1. Iinitialize catkin workspace
```
$ mkdir catkin_ws && cd catkin_ws
$ cd src && catkin_init_workspace
```

2. Within `src`, clone the `teleop` package
```
$ cd src
$ git clone https://github.com/ros-teleop/teleop_twist_keyboard
```

3. Move back to the project directory and build
```
$ cd ..
$ catkin_make
```

4. Launch the world and robot
```
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```

5. Open another terminal and launch the map_server, amcl, and move_back packages will be launched.
```
$ source devel/setup.bash
$ roslaunch my_robot amcl.launch
```

6. Open another terminal, and run the `teleop` node and move around.
```
$ source devel/setup.bash
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
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
    ├── my_robot
    │   ├── CMakeLists.txt
    │   ├── config
    │   │   ├── base_local_planner_params.yaml
    │   │   ├── costmap_common_params.yaml
    │   │   ├── global_costmap_params.yaml
    │   │   └── local_costmap_params.yaml
    │   ├── launch
    │   │   ├── amcl.launch
    │   │   ├── robot_description.launch
    │   │   └── world.launch
    │   ├── maps
    │   │   ├── map.pgm
    │   │   └── map.yaml
    │   ├── meshes
    │   │   └── hokuyo.dae
    │   ├── package.xml
    │   ├── urdf
    │   │   ├── my_robot.gazebo
    │   │   └── my_robot.xacro
    │   └── world
    │       └── andrews.world
    ├── pgm_map_creator
    │   ├── CMakeLists.txt
    │   ├── launch
    │   │   └── request_publisher.launch
    │   ├── LICENSE
    │   ├── maps
    │   │   └── map.pgm
    │   ├── msgs
    │   │   ├── CMakeLists.txt
    │   │   └── collision_map_request.proto
    │   ├── package.xml
    │   ├── README.md
    │   ├── src
    │   │   ├── collision_map_creator.cc
    │   │   └── request_publisher.cc
    │   └── world
    │       ├── andrews.world
    │       └── udacity_mtv
    └── teleop_twist_keyboard
        ├── CHANGELOG.rst
        ├── CMakeLists.txt
        ├── package.xml
        ├── README.md
        └── teleop_twist_keyboard.py

```
