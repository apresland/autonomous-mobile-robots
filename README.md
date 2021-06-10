# Robotics Software Engineer Nanodegree
This repository contains of projects completed for [Udacity Robotics Software Engineer Nanodegree Program](https://www.udacity.com/course/robotics-software-engineer--nd209). The four month course covers robotics software engineering with a practical, system-focused approach to programming robots using the [Robot Operating System (ROS)](https://www.ros.org/) framework, [Gazebo Robot Simulator](http://gazebosim.org/) and C++.

The projects cover the following content:
* Robotic environment simulation with Gazebo the common simulation engine used by Roboticists.
* Robot development with a modular and reusable architecture using ROS as a unified software environment.
* Pose estimation: Locate in a known map of the environment with Monte Carlo Localization.
* Mapping and SLAM implementation with ROS packages and C++.
* Path Planning and Navigation with probabilistic algorithms implemented in C++.

The Robot Operating System (ROS) is a set of software libraries and tools for building robot applications. It provides inter-process communications, a distributed parameter system and state-of-the-art algorithms (e.g. pose estimation, localization, navigation).

Robot simulation is an essential tool in every robot development. A well-designed simulator makes it possible to rapidly test algorithms, design robots, perform regression testing, and train AI system using realistic scenarios. Gazebo offers the ability to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. Gazebo provides a robust physics engine, high-quality graphics, programmatic and graphical interfaces.

## Project 1: Robotic Environment Simulation
Use Gazebo to simulate a robotic environment comprised of a building containing household objects. Skills: Gazebo, C++ plugins.

## Project 2: Modular Mobile Robot Design
Use Robot Operating System (ROS) to design a mobile robot. Then, house your newly-designed robot in the robotic environment you built in Project 1. You will program your robot with C++ to chase a ball through this world. Skills: catkin workspaces, ROS packages, ROS nodes, ROS launch files, RViz integration, and C++.

## Project 3: Localization
Use the Monte Carlo Localization algorithm in ROS, in conjunction with sensor data and a map of the world, to estimate a mobile robot’s position and orientation. Skills: Localization algorithms: Kalman Filter and MCL, ROS parameters, ROS packages integration, C++.

## Project 4: Mapping and SLAM
Simultaneous Localization and Mapping (SLAM) can be implemented in a number of ways depending on the sensors used via various ROS packages. Use a ROS SLAM package and simulated sensor data to create an agent that can both map the world around it, and localize within it. Skills: Mapping and SLAM algorithms, Occupancy Grid Mapping, Grid-based FastSLAM and GraphSLAM, ROS debugging tools, C++.

## Project 5: Path Planning and Navigation
Simulate a home service robot that can map, localize, and navigate to transport objects, moving autonomously. Skills: Path Planning search algorithms, ROS navigation stack, C++.
