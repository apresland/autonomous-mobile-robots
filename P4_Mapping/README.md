# Mapping
![features](https://user-images.githubusercontent.com/5468707/123142323-98764e00-d459-11eb-8131-f6bb09324e76.png)

A Simultaneous Localisation and Mapping (SLAM) technique called RTAB-Map (Real-Time Appearance-Based Mapping) is used to map the environment and localize within it. RTAB-Map is a RGB-D Graph Based SLAM approach that uses incremental appearance based loop closure detection. The resultant map is stored in local database that be interrogated later.

## Prerequisites
1. ROS and Gazebo
2. CMake and the gcc compiler
3. The rtabmap-ros package

Install the `rtabmap-ros` package
```console
$ sudo apt-get install ros-${ROS_DISTRO}-rtabmap-ros
```

## Build
1. Clone project and initialize a catkin workspace
```
$ cd src && catkin_init_workspace
```

2. Within the `src` folder, clone the `teleop` project
```
$ git clone https://github.com/ros-teleop/teleop_twist_keyboard
```

3. Move back to the project folder and build
```
$ cd ..
$ catkin_make
```

4. In a terminal Launch the world and robot
```
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```

5. In a second terminal launch the `mapping.launch` file to start the rtabmap-ros node.
```
$ source devel/setup.bash
$ roslaunch my_robot mapping.launch
```

6. In a third terminal run the `teleop` node.
```
$ source devel/setup.bash
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

7. Use the keyboard to navigate the robot with teleop. The rtabmap-ros will save
the map and trajectory in a database file 
`~/.ros/rtabmap.db`.

8. In a forth terminal open the database file using `rtabmap-databaseViewer`
```
$ rtabmap-databaseViewer ~/.ros/rtabmap.db
```

## Structure
```
.
├── README.md
└── src
    ├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
    ├── my_robot
    │   ├── CMakeLists.txt
    │   ├── config
    │   │   ├── base_local_planner_params.yaml
    │   │   ├── costmap_common_params.yaml
    │   │   ├── global_costmap_params.yaml
    │   │   └── local_costmap_params.yaml
    │   ├── default.rviz
    │   ├── launch
    │   │   ├── localization.launch
    │   │   ├── mapping.launch
    │   │   ├── robot_description.launch
    │   │   └── world.launch
    │   ├── meshes
    │   │   └── hokuyo.dae
    │   ├── package.xml
    │   ├── urdf
    │   │   ├── my_robot.gazebo
    │   │   └── my_robot.xacro
    │   └── worlds
    │       └── andrews.world
    └── teleop_twist_keyboard
        ├── CHANGELOG.rst
        ├── CMakeLists.txt
        ├── package.xml
        ├── README.md
        └── teleop_twist_keyboard.py

```
