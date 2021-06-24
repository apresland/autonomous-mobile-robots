# Planning

## Prerequisites
1. ROS and Gazebo
2. CMake & g++/gcc, C++11
 
### Dependencies
```
$ sudo apt-get install xterm
$ sudo apt-get install python-pip
$ sudo apt-get update && sudo apt-get upgrade -y
$ sudo apt-get install ros-${ROS_DISTRO}-map-server
$ sudo apt-get install ros-${ROS_DISTRO}-amcl
$ sudo apt-get install ros-${ROS_DISTRO}-move-base
$ sudo apt-get install ros-${ROS_DISTRO}-slam-gmapping
$ pip install rospkg
```

## Build
1. Initialize the catkin workspace
```
$ cd src && catkin_init_workspace
```

2. Navigate back to the project folder and build the project
```
$ cd ..
$ catkin_make
```

3. Execute the launch script
```
$ source devel/setup.bash
$ chmod u+x ./src/scripts/home_service.sh
$ ./src/scripts/home_service.sh
```

## Structure
```
.
├── launch.sh
├── map
├── model
│   ├── corridor
│   │   ├── model.config
│   │   └── model.sdf
│   └── corridor2
│       ├── model.config
│       └── model.sdf
├── rvizConfig
├── scripts
│   ├── launch.sh
│   ├── test_navigation.sh
│   └── test_slam.sh
├── src
│   ├── slam_gmapping
│   │   ├── gmapping
│   │   │   ├── CHANGELOG.rst
│   │   │   ├── CMakeLists.txt
│   │   │   ├── launch
│   │   │   │   └── slam_gmapping_pr2.launch
│   │   │   ├── nodelet_plugins.xml
│   │   │   ├── package.xml
│   │   │   ├── src
│   │   │   │   ├── main.cpp
│   │   │   │   ├── nodelet.cpp
│   │   │   │   ├── replay.cpp
│   │   │   │   ├── slam_gmapping.cpp
│   │   │   │   └── slam_gmapping.h
│   │   │   └── test
│   │   │       ├── basic_localization_laser_different_beamcount.test
│   │   │       ├── basic_localization_stage.launch
│   │   │       ├── basic_localization_stage_replay2.launch
│   │   │       ├── basic_localization_stage_replay.launch
│   │   │       ├── basic_localization_symmetry.launch
│   │   │       ├── basic_localization_upside_down.launch
│   │   │       ├── rtest.cpp
│   │   │       └── test_map.py
│   │   ├── README.md
│   │   └── slam_gmapping
│   │       ├── CHANGELOG.rst
│   │       ├── CMakeLists.txt
│   │       └── package.xml
│   ├── turtlebot
│   │   ├── LICENSE
│   │   ├── README.md
│   │   ├── setup_create.sh
│   │   ├── setup_kobuki.sh
│   │   ├── turtlebot
│   │   │   ├── CHANGELOG.rst
│   │   │   ├── CMakeLists.txt
│   │   │   └── package.xml
│   │   ├── turtlebot_bringup
│   │   │   ├── CHANGELOG.rst
│   │   │   ├── CMakeLists.txt
│   │   │   ├── env-hooks
│   │   │   │   └── 25.turtlebot.sh.em
│   │   │   ├── icons
│   │   │   │   └── turtlebot2.png
│   │   │   ├── interactions
│   │   │   │   ├── admin.interactions
│   │   │   │   ├── documentation.interactions
│   │   │   │   ├── pairing.interactions
│   │   │   │   └── visualisation.interactions
│   │   │   ├── launch
│   │   │   │   ├── 3dsensor.launch
│   │   │   │   ├── concert_client.launch
│   │   │   │   ├── concert_minimal.launch
│   │   │   │   ├── includes
│   │   │   │   │   ├── 3dsensor
│   │   │   │   │   │   ├── astra.launch.xml
│   │   │   │   │   │   ├── asus_xtion_pro.launch.xml
│   │   │   │   │   │   ├── asus_xtion_pro_offset.launch.xml
│   │   │   │   │   │   ├── kinect.launch.xml
│   │   │   │   │   │   └── r200.launch.xml
│   │   │   │   │   ├── capabilities.launch.xml
│   │   │   │   │   ├── create
│   │   │   │   │   │   └── mobile_base.launch.xml
│   │   │   │   │   ├── description.launch.xml
│   │   │   │   │   ├── kobuki
│   │   │   │   │   │   ├── bumper2pc.launch.xml
│   │   │   │   │   │   ├── mobile_base.launch.xml
│   │   │   │   │   │   └── safety_controller.launch.xml
│   │   │   │   │   ├── mobile_base.launch.xml
│   │   │   │   │   ├── netbook.launch.xml
│   │   │   │   │   ├── robot.launch.xml
│   │   │   │   │   ├── roomba
│   │   │   │   │   │   └── mobile_base.launch.xml
│   │   │   │   │   └── zeroconf.launch.xml
│   │   │   │   └── minimal.launch
│   │   │   ├── package.xml
│   │   │   ├── param
│   │   │   │   ├── 3dsensor.yaml
│   │   │   │   ├── capabilities
│   │   │   │   │   └── defaults_tb2.yaml
│   │   │   │   ├── create
│   │   │   │   │   ├── capability_providers.yaml
│   │   │   │   │   └── diagnostics.yaml
│   │   │   │   ├── defaults
│   │   │   │   │   ├── capability_providers.yaml
│   │   │   │   │   └── smoother.yaml
│   │   │   │   ├── kinect
│   │   │   │   │   └── capability_providers.yaml
│   │   │   │   ├── kobuki
│   │   │   │   │   ├── capability_providers.yaml
│   │   │   │   │   └── diagnostics.yaml
│   │   │   │   ├── mux.yaml
│   │   │   │   ├── preferred_rapp.yaml
│   │   │   │   ├── roomba
│   │   │   │   │   ├── capability_providers.yaml
│   │   │   │   │   └── diagnostics.yaml
│   │   │   │   ├── xtion
│   │   │   │   │   └── capability_providers.yaml
│   │   │   │   └── zeroconf.yaml
│   │   │   └── scripts
│   │   │       └── turtlebot_addr.py
│   │   ├── turtlebot_capabilities
│   │   │   ├── CHANGELOG.rst
│   │   │   ├── CMakeLists.txt
│   │   │   ├── interfaces
│   │   │   │   └── TurtleBotBringup.yaml
│   │   │   ├── package.xml
│   │   │   └── providers
│   │   │       ├── depthimage_to_laserscan.yaml
│   │   │       ├── diagnostics.yaml
│   │   │       ├── differential_mobile_base.yaml
│   │   │       ├── launch
│   │   │       │   ├── depthimage_to_laserscan.launch
│   │   │       │   ├── diagnostics.launch
│   │   │       │   ├── placeholder.py
│   │   │       │   ├── rgbd_sensor.launch
│   │   │       │   ├── robot_state_publisher.launch
│   │   │       │   ├── turtlebot2_bringup.launch
│   │   │       │   └── turtlebot_bringup.launch
│   │   │       ├── rgbd_sensor.yaml
│   │   │       ├── robot_state_publisher.yaml
│   │   │       ├── turtlebot2_bringup.yaml
│   │   │       └── turtlebot_bringup.yaml
│   │   ├── turtlebot_capabilities.rosinstall
│   │   ├── turtlebot_description
│   │   │   ├── CHANGELOG.rst
│   │   │   ├── CMakeLists.txt
│   │   │   ├── meshes
│   │   │   │   ├── sensors
│   │   │   │   │   ├── 0_xtion_pro.jpg
│   │   │   │   │   ├── astra.dae
│   │   │   │   │   ├── astra.jpg
│   │   │   │   │   ├── asus_xtion_pro_live.dae
│   │   │   │   │   ├── asus_xtion_pro_live.png
│   │   │   │   │   ├── kinect.dae
│   │   │   │   │   ├── kinect.jpg
│   │   │   │   │   ├── kinect.tga
│   │   │   │   │   ├── r200_bracket_end.stl
│   │   │   │   │   ├── r200_bracket.stl
│   │   │   │   │   ├── r200.dae
│   │   │   │   │   ├── r200.jpg
│   │   │   │   │   ├── xtion_pro_camera.dae
│   │   │   │   │   ├── xtion_pro_camera.jpg
│   │   │   │   │   ├── xtion_pro.jpg
│   │   │   │   │   └── xtion_pro_stack.dae
│   │   │   │   └── stacks
│   │   │   │       ├── circles
│   │   │   │       │   ├── 68-02403-125_Spacer.dae
│   │   │   │       │   ├── 68-02421-8000-RA_Turtlebot_F-F_Standoff_color.png
│   │   │   │       │   ├── 68-02421-8000-RA_Turtlebot_F-F_Standoff.dae
│   │   │   │       │   ├── 68-04552-1000-RA_Turtlebot_M-F_Standoff_color.png
│   │   │   │       │   ├── 68-04552-1000-RA_Turtlebot_M-F_Standoff.dae
│   │   │   │       │   ├── 68-04552-2000-RA_Turtlebot_M-F_Standoff_color.png
│   │   │   │       │   ├── 68-04552-2000-RA_Turtlebot_M-F_Standoff.dae
│   │   │   │       │   ├── 68-04556-RA_Kinect_Standoff_Assy.3ds
│   │   │   │       │   ├── 68-04556-RA_Kinect_Standoff_Assy.dae
│   │   │   │       │   ├── plate_0_logo.dae
│   │   │   │       │   ├── plate_0_logo.tga
│   │   │   │       │   ├── plate_1_logo.dae
│   │   │   │       │   ├── plate_1_logo.tga
│   │   │   │       │   ├── plate_2_logo.dae
│   │   │   │       │   └── plate_2_logo.tga
│   │   │   │       └── hexagons
│   │   │   │           ├── images
│   │   │   │           │   ├── 1f_pole.jpg
│   │   │   │           │   ├── 1f_stack.jpg
│   │   │   │           │   ├── 2f_pole.jpg
│   │   │   │           │   ├── 2f_stack.jpg
│   │   │   │           │   ├── 3f_pole.jpg
│   │   │   │           │   ├── 3f_stack1.jpg
│   │   │   │           │   ├── 3f_stack.jpg
│   │   │   │           │   ├── kinect_pole.jpg
│   │   │   │           │   └── kinect_pole_old.jpg
│   │   │   │           ├── plate_bottom.dae
│   │   │   │           ├── plate_middle.dae
│   │   │   │           ├── plate_top.dae
│   │   │   │           ├── pole_bottom.dae
│   │   │   │           ├── pole_kinect.dae
│   │   │   │           ├── pole_middle.dae
│   │   │   │           └── pole_top.dae
│   │   │   ├── package.xml
│   │   │   ├── README.md
│   │   │   ├── robots
│   │   │   │   ├── create_circles_asus_xtion_pro.urdf.xacro
│   │   │   │   ├── create_circles_kinect.urdf.xacro
│   │   │   │   ├── kobuki_hexagons_astra.urdf.xacro
│   │   │   │   ├── kobuki_hexagons_asus_xtion_pro_offset.urdf.xacro
│   │   │   │   ├── kobuki_hexagons_asus_xtion_pro.urdf.xacro
│   │   │   │   ├── kobuki_hexagons_kinect.urdf.xacro
│   │   │   │   ├── kobuki_hexagons_r200.urdf.xacro
│   │   │   │   ├── roomba_circles_asus_xtion_pro.urdf.xacro
│   │   │   │   └── roomba_circles_kinect.urdf.xacro
│   │   │   ├── scripts
│   │   │   │   └── calc_inertia.m
│   │   │   ├── test
│   │   │   │   └── test_urdf.cpp
│   │   │   ├── test.launch
│   │   │   └── urdf
│   │   │       ├── common_properties.urdf.xacro
│   │   │       ├── sensors
│   │   │       │   ├── astra.urdf.xacro
│   │   │       │   ├── kinect.urdf.xacro
│   │   │       │   ├── r200.urdf.xacro
│   │   │       │   └── xtion_pro.urdf.xacro
│   │   │       ├── stacks
│   │   │       │   ├── circles.urdf.xacro
│   │   │       │   └── hexagons.urdf.xacro
│   │   │       ├── turtlebot_common_library.urdf.xacro
│   │   │       ├── turtlebot_gazebo.urdf.xacro
│   │   │       └── turtlebot_properties.urdf.xacro
│   │   ├── turtlebot.rosinstall
│   │   └── turtlebot_teleop
│   │       ├── CHANGELOG.rst
│   │       ├── CMakeLists.txt
│   │       ├── launch
│   │       │   ├── includes
│   │       │   │   └── velocity_smoother.launch.xml
│   │       │   ├── keyboard_teleop.launch
│   │       │   ├── logitech.launch
│   │       │   ├── ps3_teleop.launch
│   │       │   └── xbox360_teleop.launch
│   │       ├── package.xml
│   │       ├── param
│   │       │   └── mux.yaml
│   │       ├── README.md
│   │       ├── scripts
│   │       │   └── turtlebot_teleop_key
│   │       └── src
│   │           └── turtlebot_joy.cpp
│   ├── turtlebot_interactions
│   │   ├── README.md
│   │   ├── turtlebot_dashboard
│   │   │   ├── CHANGELOG.rst
│   │   │   ├── CMakeLists.txt
│   │   │   ├── launch
│   │   │   │   ├── create
│   │   │   │   │   └── dashboard.launch.xml
│   │   │   │   ├── kobuki
│   │   │   │   │   └── dashboard.launch.xml
│   │   │   │   ├── roomba
│   │   │   │   │   └── dashboard.launch.xml
│   │   │   │   └── turtlebot_dashboard.launch
│   │   │   └── package.xml
│   │   ├── turtlebot_interactions
│   │   │   ├── CHANGELOG.rst
│   │   │   ├── CMakeLists.txt
│   │   │   └── package.xml
│   │   ├── turtlebot_interactive_markers
│   │   │   ├── CHANGELOG.rst
│   │   │   ├── CMakeLists.txt
│   │   │   ├── launch
│   │   │   │   └── interactive_markers.launch
│   │   │   ├── package.xml
│   │   │   └── src
│   │   │       └── turtlebot_marker_server.cpp
│   │   └── turtlebot_rviz_launchers
│   │       ├── CHANGELOG.rst
│   │       ├── CMakeLists.txt
│   │       ├── launch
│   │       │   ├── view_blind_nav.launch
│   │       │   ├── view_model.launch
│   │       │   ├── view_navigation_app.launch
│   │       │   ├── view_navigation.launch
│   │       │   ├── view_robot.launch
│   │       │   └── view_teleop_navigation.launch
│   │       ├── package.xml
│   │       └── rviz
│   │           ├── blind_nav.rviz
│   │           ├── model.rviz
│   │           ├── navigation_app.rviz
│   │           ├── navigation.rviz
│   │           ├── readme.txt
│   │           └── robot.rviz
│   └── turtlebot_simulator
│       ├── README.md
│       ├── turtlebot_gazebo
│       │   ├── CHANGELOG.rst
│       │   ├── CMakeLists.txt
│       │   ├── env-hooks
│       │   │   └── 25.turtlebot-gazebo.sh.em
│       │   ├── launch
│       │   │   ├── amcl_demo.launch
│       │   │   ├── gmapping_demo.launch
│       │   │   ├── includes
│       │   │   │   ├── create.launch.xml
│       │   │   │   ├── kobuki.launch.xml
│       │   │   │   └── roomba.launch.xml
│       │   │   └── turtlebot_world.launch
│       │   ├── maps
│       │   │   ├── playground.pgm
│       │   │   └── playground.yaml
│       │   ├── package.xml
│       │   └── worlds
│       │       ├── corridor.world
│       │       ├── empty.world
│       │       └── playground.world
│       ├── turtlebot_simulator
│       │   ├── CHANGELOG.rst
│       │   ├── CMakeLists.txt
│       │   └── package.xml
│       ├── turtlebot_simulator.rosinstall
│       ├── turtlebot_stage
│       │   ├── CHANGELOG.rst
│       │   ├── CMakeLists.txt
│       │   ├── env-hooks
│       │   │   └── 25.turtlebot-stage.sh.em
│       │   ├── launch
│       │   │   └── turtlebot_in_stage.launch
│       │   ├── maps
│       │   │   ├── maze.png
│       │   │   ├── maze.yaml
│       │   │   ├── robopark2.bmp
│       │   │   ├── robopark_plan.yaml
│       │   │   └── stage
│       │   │       ├── maze.world
│       │   │       ├── robopark_plan.world
│       │   │       └── turtlebot.inc
│       │   ├── package.xml
│       │   └── rviz
│       │       └── robot_navigation.rviz
│       └── turtlebot_stdr
│           ├── CHANGELOG.rst
│           ├── CMakeLists.txt
│           ├── documentation
│           │   └── architecture_overview.png
│           ├── env-hooks
│           │   └── 25.turtlebot-stdr.sh.em
│           ├── launch
│           │   ├── includes
│           │   │   └── relays.launch.xml
│           │   └── turtlebot_in_stdr.launch
│           ├── maps
│           │   ├── frieburg.png
│           │   ├── frieburg.yaml
│           │   ├── hospital_section.png
│           │   ├── hospital_section.yaml
│           │   ├── mines.png
│           │   ├── mines.yaml
│           │   ├── robocup.png
│           │   ├── robocup.yaml
│           │   ├── simple_rooms.png
│           │   ├── simple_rooms.yaml
│           │   ├── sparse_obstacles.png
│           │   └── sparse_obstacles.yaml
│           ├── nodes
│           │   └── tf_connector.py
│           ├── package.xml
│           ├── robot
│           │   └── turtlebot.yaml
│           └── rviz
│               └── robot_navigation.rviz
└── world
    ├── corridor2.world
    └── corridor.world

```
