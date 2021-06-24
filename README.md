# Robotics Software Engineering with ROS2 and Gazebo
This repository contains of projects completed for [Udacity Robotics Software Engineer Nanodegree Program](https://www.udacity.com/course/robotics-software-engineer--nd209). 

<table style="width:100%">
  <tr>
    <th>
       <a href="https://user-images.githubusercontent.com/5468707/121535884-d8bde100-ca02-11eb-96a3-1981ffe1a6a7.png">
       <img src="https://user-images.githubusercontent.com/5468707/121535884-d8bde100-ca02-11eb-96a3-1981ffe1a6a7.png"
        width="250" height="140">
    </th>
     <th>
       <a href="https://user-images.githubusercontent.com/5468707/123093583-26d3db00-d42c-11eb-96e1-9e387986c0db.gif">
       <img src="https://user-images.githubusercontent.com/5468707/123093583-26d3db00-d42c-11eb-96e1-9e387986c0db.gif"
        width="250" height="140">
    </th>
    <th>
       <a href="https://user-images.githubusercontent.com/5468707/123102365-44597280-d435-11eb-95c8-fc36e9becfb7.gif">
       <img src="https://user-images.githubusercontent.com/5468707/123102365-44597280-d435-11eb-95c8-fc36e9becfb7.gif"
        width="250" height="140">
    </th>   
  </tr>
</table>


The four month course covers robotics software engineering with a practical, system-focused approach to programming robots.

The projects cover the following content:
* Robotic environment simulation with Gazebo the common simulation engine used by Roboticists.
* Robot development with a modular and reusable architecture using ROS as a unified software environment.
* Pose estimation: Locate in a known map of the environment with Monte Carlo Localization.
* Mapping and SLAM implementation with ROS packages and C++.
* Path Planning and Navigation with probabilistic algorithms implemented in C++.

The projects use the following technolgies:
* [Robot Operating System (ROS)](https://www.ros.org/) is a set of software libraries and tools for building robot applications. It provides inter-process communications, a distributed parameter system and state-of-the-art algorithms (e.g. pose estimation, localization, navigation).
* [Gazebo Robot Simulator](http://gazebosim.org/) is an essential tool for robot development making it possible to rapidly test algorithms, design robots, perform regression testing, and train AI system using realistic scenarios. Gazebo offers the ability to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. Gazebo provides a robust physics engine, high-quality graphics, programmatic and graphical interfaces.
* [Unified Robot Description Format (URDF)](https://industrial-training-master.readthedocs.io/en/melodic/_source/session3/Intro-to-URDF.html) is an XML file format used in ROS to describe all elements of a robot in a code-independent, human-readable way. It allows things like collision checking and dynamic path planning.
* [RViz](http://wiki.ros.org/rviz) is a 3D robot visualzer that can provide a view of your robot model, capture sensor information from robot sensors, and replay captured data. It can display data from camera, lasers, from 3D and 2D devices including pictures and point clouds.

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
