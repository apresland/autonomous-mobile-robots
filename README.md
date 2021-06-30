# Autonomous Mobile Robots

This repository contains of projects completed for [Udacity Robotics Software Engineer Nanodegree Program](https://www.udacity.com/course/robotics-software-engineer--nd209). The four month course covers robotics software engineering with a practical, system-focused approach to programming robots. Projects cover simulation with [Gazebo](http://gazebosim.org/), modular and reusable architectures with [ROS](https://www.ros.org/), SLAM with [RTAB-Map](http://introlab.github.io/rtabmap/) and path planning with probabilistic graphical methods.

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
       <a href="https://user-images.githubusercontent.com/5468707/123806919-b1667f80-d8ef-11eb-8bb6-20c0bb219809.gif">
       <img src="https://user-images.githubusercontent.com/5468707/123806919-b1667f80-d8ef-11eb-8bb6-20c0bb219809.gif"
        width="250" height="140">
    </th> 
  </tr>
</table>


The tech stack comprises:
* [Gazebo Robot Simulator](http://gazebosim.org/) offers the ability to accurately and efficiently simulate robots in complex indoor and outdoor environments.
* [Unified Robot Description Format (URDF)](https://industrial-training-master.readthedocs.io/en/melodic/_source/session3/Intro-to-URDF.html) is an XML format used to describe all elements of a robot in a code-independent way.
* [Robot Operating System (ROS)](https://www.ros.org/) is a set of software libraries and tools for building robot applications. It provides inter-process communications, a distributed parameter system and state-of-the-art algorithms (e.g. pose estimation, localization, navigation).
* [RTAB-Map (Real-Time Appearance-Based Mapping)](http://introlab.github.io/rtabmap/) is a RGB-D Graph-Based SLAM approach based on an incremental appearance-based loop closure detector.
* [RViz](http://wiki.ros.org/rviz) is a 3D robot visualzer that can provide a view of your robot model, capture sensor information from robot sensors, and replay captured data. It can display data from camera, lasers, from 3D and 2D devices including pictures and point clouds.

## 1: Simulation
<img src="https://user-images.githubusercontent.com/5468707/123917945-a65d2f00-d983-11eb-985e-376d9748dac3.png" alt="alt text" width="1000" height="300"/>

Simulate a robot and it's environment with Gazebo and interact with the robots world via a [ROS](https://www.ros.org/) plugin. [Gazebo](http://gazebosim.org/) is a 3D dynamic simulator that can efficiently simulate robots in complex indoor and outdoor environments. Similar to game engines, Gazebo offers physics simulation at a high degree of fidelity and offers convenient programmatic and graphical interfaces.

Gazebo offers the following features:
* Dynamics Simulation with multiple high-performance physics engines ([ODE](http://opende.sourceforge.net/), [Bullet](https://pybullet.org/wordpress/), [Simbody](https://simtk.org/projects/simbody/), [DART](http://dartsim.github.io/))
* Realistic rendering of environments using [OGRE](https://www.ogre3d.org/)
* Generate sensor data from sensors including laser range finders, 2D/3D cameras, Kinect sensors, force-torque
* Robot models including PR2, Pioneer2 DX, iRobot Create, and TurtleBot or custom built using SDF/URDF
* Direct access to Gazebo's API from [ROS](https://www.ros.org/) with custom plugins for robot, sensor, and environmental control

Typical uses of Gazebo simulation include: 
* Testing robotics algorithms
* Designing new robots
* Performing regression testing
* Training AI systems using realistic scenarios


## 2: Perception & Locomotion
<img src="https://user-images.githubusercontent.com/5468707/123237278-d1f09d00-d4dd-11eb-988e-e08fea600b0b.png" alt="alt text" width="1000" height="300"/>

Design a mobile robot with differential drive using [ROS](https://www.ros.org/), place it in the robotic environment and enable the robot to chase a ball through the world. Achieve this by percieving objects by processing the pixels in RGB camera (e.g. color detection) and then locomote towards the object by sending command velocities to the motion controller.

[ROS](https://www.ros.org/) offers the following features:
* A computation graph comprising nodes, Master, Parameter Server, messages, services, topics
* Synchronous RPC-style communication over services
* Asynchronous streaming of data over topics
* Storage of data on a Parameter Server
* Low-level device control

Typically [ROS](https://www.ros.org/) is used to create plugins that implement the **see-think-act** cycle: 
* _See_: Perceive the environment by subscribing to raw low-level sensor data and extracting information.
* _Think_: Localize within the environmental model (map) and calculate a planned path. 
* _Act_: Execute the planned path with motion control via actuator commands.

In this module we implement a basic version of the cycle using [ROS](plugins) plugins that:
* Subscribe to raw RGB camera data `sensor_msgs/Image` messages and locate target objects using color detection
* Update linear and angular velocity as `geometry_msgs/Twist` messages to approach the target object 
* Control motion by publishing to the `/mobile_base_controller/cmd_vel` topic.

Skills: catkin workspaces, ROS packages, ROS nodes, ROS launch files, RViz integration, and C++.

## 3: Localization
Use the [Adaptive Monte Carlo Localization (AMCL)](https://proceedings.neurips.cc/paper/2001/file/c5b2cebf15b205503560c4e8e6d1ea78-Paper.pdf) algorithm in ROS, in conjunction with sensor data and a map of the world, to estimate a mobile robot’s position and orientation.

AMCL is a specific implementation of the particle filter algorithm. At the conceptual level it maintains a probability distribution over the set of all possible robot poses, and updates this distribution using data from odometry and range sensors. The probability densities are represented by sets of randomly generated samples taken in proportion to likelihood allowing particle filters to focus the computational resources on regions with high likelihood.

The steps of the Particle Filter are:
* __Re-sampling__: Draw a random sample with replacement from the sample set according to a distribution defined through importance weights. This sample can be seen as an instance of prior belief.
* __Sampling__: Use the prior belief and the control inputs to sample from the distribution describing the robots dynamic system. The product of the obtained distribution the prior belief provides the proposal used in the next step.
* __Importance sampling__: Weight each sample by the importance weight calculated as the likelihood of the sample given the measurements.

AMCL provides a measure of goodness of fit of the distribution represented by weighted particles. It can then dynamically adapt the number of particles in the filter: when the robot’s pose is highly uncertain, the number of particles is increased; when the robot’s pose is well determined, the number of particles is decreased. This enables the robot to make a trade-off between processing speed and localization accuracy.

Skills: Localization algorithms: Kalman Filter and MCL, ROS parameters, ROS packages integration, C++.

## 4: Mapping
Create a automonous agent uses the Simultaneous Localisation and Mapping (SLAM) technique called RTAB-Map (Real-Time Appearance-Based Mapping) using a RGB-D Graph Based SLAM approach that uses incremental appearance based loop closure detection. Skills: Mapping and SLAM algorithms, Occupancy Grid Mapping, Grid-based FastSLAM and GraphSLAM, ROS debugging tools, C++.

## 5: Planning
Simulate a home service robot that can map, localize, and navigate to transport objects, moving autonomously. Skills: Path Planning search algorithms, ROS navigation stack, C++.
