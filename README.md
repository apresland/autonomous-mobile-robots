# Robotics Software Engineering (ROS2 / Gazebo / RTAB-Map)

<img src="https://user-images.githubusercontent.com/5468707/123237278-d1f09d00-d4dd-11eb-988e-e08fea600b0b.png" alt="alt text" width="1000" height="300">

This repository contains of projects completed for [Udacity Robotics Software Engineer Nanodegree Program](https://www.udacity.com/course/robotics-software-engineer--nd209). The four month course covers robotics software engineering with a practical, system-focused approach to programming robots. Projects cover simulation with Gazebo, modular and reusable architectures with ROS, Monte Carlo Localization, SLAM with RTAB-Map and path planning with probabilistic graphical methods.

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
    <th>
       <a href="https://user-images.githubusercontent.com/5468707/123237472-ff3d4b00-d4dd-11eb-87ff-c961687ef60b.png">
       <img src="https://user-images.githubusercontent.com/5468707/123237472-ff3d4b00-d4dd-11eb-87ff-c961687ef60b.png"
        width="250" height="140">
    </th> 
  </tr>
</table>


The tech stack comprises:
* [Gazebo Robot Simulator](http://gazebosim.org/) offers the ability to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. Gazebo provides a robust physics engine, high-quality graphics, programmatic and graphical interfaces.
* [Unified Robot Description Format (URDF)](https://industrial-training-master.readthedocs.io/en/melodic/_source/session3/Intro-to-URDF.html) is an XML file format used in ROS to describe all elements of a robot in a code-independent, human-readable way. It allows things like collision checking and dynamic path planning.
* [Robot Operating System (ROS)](https://www.ros.org/) is a set of software libraries and tools for building robot applications. It provides inter-process communications, a distributed parameter system and state-of-the-art algorithms (e.g. pose estimation, localization, navigation).
* [RTAB-Map (Real-Time Appearance-Based Mapping)](http://introlab.github.io/rtabmap/) is a RGB-D Graph-Based SLAM approach based on an incremental appearance-based loop closure detector.
* [RViz](http://wiki.ros.org/rviz) is a 3D robot visualzer that can provide a view of your robot model, capture sensor information from robot sensors, and replay captured data. It can display data from camera, lasers, from 3D and 2D devices including pictures and point clouds.

## Module 1: Simulation
Use Gazebo to simulate a robotic environment comprised of a building containing household objects. Skills: Gazebo, C++ plugins.

## Module 2: Perception
Use Robot Operating System (ROS) to design a mobile robot. Then, house your newly-designed robot in the robotic environment you built in Project 1. You will program your robot with C++ to chase a ball through this world. Skills: catkin workspaces, ROS packages, ROS nodes, ROS launch files, RViz integration, and C++.

## Module 3: Localization
Use the Monte Carlo Localization algorithm in ROS, in conjunction with sensor data and a map of the world, to estimate a mobile robot’s position and orientation. Skills: Localization algorithms: Kalman Filter and MCL, ROS parameters, ROS packages integration, C++.

## Module 4: Mapping
Create a automonous agent uses the Simultaneous Localisation and Mapping (SLAM) technique called RTAB-Map (Real-Time Appearance-Based Mapping) using a RGB-D Graph Based SLAM approach that uses incremental appearance based loop closure detection. Skills: Mapping and SLAM algorithms, Occupancy Grid Mapping, Grid-based FastSLAM and GraphSLAM, ROS debugging tools, C++.

## Module 5: Planning
Simulate a home service robot that can map, localize, and navigate to transport objects, moving autonomously. Skills: Path Planning search algorithms, ROS navigation stack, C++.
