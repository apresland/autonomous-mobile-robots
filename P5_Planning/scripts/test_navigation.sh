#!/bin/sh
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch  world_file:=/home/workspace/catkin_ws/world/corridor2.world" &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch  map_file:=/home/workspace/catkin_ws/map/corridor.yaml" & 
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5