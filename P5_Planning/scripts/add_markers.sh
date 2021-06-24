#!/bin/sh

xterm  -e "
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find turtlebot_gazebo)/worlds/simple.world " &
sleep 5

xterm  -e "
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find turtlebot_gazebo)/maps/simple.yaml" &
sleep 5

xterm  -e "
roslaunch add_markers add_markers_rviz.launch rviz_config_file:=$(rospack find add_markers)/rvizConfig/default.rviz" &
sleep 5

xterm -e "
rosparam load $(rospack find add_markers)/config/pickplace.yaml;
rosrun add_markers add_markers_demo " &
