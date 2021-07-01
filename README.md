# Autonomous Mobile Robots

This repository contains a walkthrough of Autonomous Mobile Robot engineering using a system-focused programming approach. Examples cover robot simulation with [Gazebo](http://gazebosim.org/), designing modular architectures with [ROS](https://www.ros.org/), Visual SLAM with [RTAB-Map](http://introlab.github.io/rtabmap/) and path planning with Probabilistic Graphical Methods.

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


The focus here is _not on implementing everything from scratch_ but rather to demonstrate how open-source frameworks can be employed to build commercial grade robotic solutions. In context this following tech will be leaned on heavily:
* [Gazebo Robot Simulator](http://gazebosim.org/) accurately and efficiently simulates robots in complex indoor and outdoor environments.
* [Unified Robot Description Format (URDF)](https://industrial-training-master.readthedocs.io/en/melodic/_source/session3/Intro-to-URDF.html) is an XML format used to describe robot elements in a code-independent way.
* [Robot Operating System (ROS)](https://www.ros.org/) is a framework for building robot applications. It provides inter-process communications, a distributed parameter system and state-of-the-art algorithms (e.g. pose estimation, localization, navigation) in losely coupled packages.
* [RTAB-Map (Real-Time Appearance-Based Mapping)](http://introlab.github.io/rtabmap/) is a RGB-D Graph-Based SLAM technique based on an incremental appearance-based loop closure detector.
* [RViz](http://wiki.ros.org/rviz) is a 3D robot visualzer that provides a view of your robot model and captured sensor information. It can display data from camera, lasers, from 3D and 2D devices including pictures and point clouds.

The remaining content is split into a ordered set of self contained examples. Each example demonstrates a core concern of robotics software engineering and the ordering is defined so that each example builds on the outcomes of the previous. 

## Example 1: Simulation
<img src="https://user-images.githubusercontent.com/5468707/123917945-a65d2f00-d983-11eb-985e-376d9748dac3.png" alt="alt text" width="500" height="250"/>

__Goal__: Simulate a robot and it's environment with [Gazebo](http://gazebosim.org/) and interact with the robots world via a [ROS](https://www.ros.org/) plugin. 

__Skills__: Gazebo Simulation, ROS Plugins, ROS-Gazebo interaction

[Gazebo](http://gazebosim.org/) is a 3D dynamic simulator that can efficiently simulate robots in complex indoor and outdoor environments. Gazebo offers physics simulation similar to game engines but at a higher degree of fidelity. It also offers convenient programmatic and graphical interfaces.

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


## Example 2: Perception & Locomotion
<img src="https://user-images.githubusercontent.com/5468707/123093583-26d3db00-d42c-11eb-96e1-9e387986c0db.gif" alt="alt text" width="500" height="250"/>

__Goal__: Design a mobile robot with differential drive using [ROS](https://www.ros.org/), place it in the robotic environment and enable the robot to chase a ball through the world. Achieve this by percieving objects by processing the pixels in RGB camera (e.g. color detection) and then locomote towards the object by sending command velocities to the motion controller.

__Skills__: Sensors, ROS Workspaces, ROS Packages, ROS Nodes, ROS Launchers, RViz integration.

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

## Example 3: Localization
<img src="https://user-images.githubusercontent.com/5468707/123102365-44597280-d435-11eb-95c8-fc36e9becfb7.gif" alt="alt text" width="500" height="250"/>

__Goal__: Use the [Adaptive Monte Carlo Localization (AMCL)](https://proceedings.neurips.cc/paper/2001/file/c5b2cebf15b205503560c4e8e6d1ea78-Paper.pdf) algorithm in ROS, in conjunction with sensor data and a map of the world, to estimate a mobile robot’s position and orientation.

__Skills__: Probabilistic Localization, Monte Carlo Localization, ROS Parameter Server, ROS Packages.

AMCL is a specific implementation of the [Particle Filter](https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1999_1/fox_dieter_1999_1.pdf) algorithm. At the conceptual level it maintains a probability distribution over the set of all possible robot poses, and updates this distribution using data from odometry and range sensors. The probability densities are represented by sets of randomly generated samples taken in proportion to likelihood allowing particle filters to focus the computational resources on regions with high likelihood.

The steps of the Particle Filter are:
* __Re-sampling__: Draw a random sample with replacement from the sample set according to a distribution defined through importance weights. This sample can be seen as an instance of prior belief.
* __Sampling__: Use the prior belief and the control inputs to sample from the distribution describing the robots dynamic system. The product of the obtained distribution the prior belief provides the proposal used in the next step.
* __Importance sampling__: Weight each sample by the importance weight calculated as the likelihood of the sample given the measurements.

AMCL provides a measure of goodness of fit of the distribution represented by weighted particles. It can then dynamically adapt the number of particles in the filter: when the robot’s pose is highly uncertain, the number of particles is increased; when the robot’s pose is well determined, the number of particles is decreased. This enables the robot to make a trade-off between processing speed and localization accuracy.

## Example 4: Visual SLAM
<img src="https://user-images.githubusercontent.com/5468707/123237278-d1f09d00-d4dd-11eb-988e-e08fea600b0b.png" alt="alt text" width="1000" height="300"/>

__Goal__: Simulate an automonous mobile robot that uses the Simultaneous Localisation and Mapping (SLAM) to map its environment and localize within it.

__Skills__: Mapping and SLAM algorithms, Occupancy Grid Mapping, GraphSLAM, Feature Detection, Loop Closure.

[RTAB-Map](http://introlab.github.io/rtabmap/) (Real-Time Appearance-Based Mapping) is a graph based SLAM method using RGB-D/Stereo camera or LIDAR data for incremental appearance based loop closure detection. Loop closure occurs inside working memory based on features detected with [SURF (Speeded Up Robust Features)](https://people.ee.ethz.ch/~surf/eccv06.pdf) esimating how likely a new image comes from a previous location or a new location. When a loop closure hypothesis is accepted, a new constraint is added to the map’s graph and an optimizer minimizes the mapping errors. A memory management is used to limit the number of locations used for loop closure detection and graph optimization, so that real-time constraints on large-scale environnements are respected.

In this example [rtabmap-ros](http://wiki.ros.org/rtabmap_ros) (a ROS wrapper around the RTAB-Map) will be used with a RGB-D camera to generate 3D point clouds of the environment and create a 2D occupancy grid map for navigation.


## Example 5: Path Planning

<img src="https://user-images.githubusercontent.com/5468707/124101137-5784c780-da5f-11eb-8fc3-c0f8232ceb99.png" alt="alt text" width="1000" height="300"/>

__Goal__: Simulate an autonomous mobile robot that can autonomously navigate its environment in order to achieve correct pose at dynamically defined pick and place goals. The robot first map its environment using SLAM.

__Skills__: ROS Navigation Stack, ROS Actions, Cost Maps, Path Search Algorithms, .

[ROS](https://wiki.ros.org/) path planning is an example of a [ROS Action](https://wiki.ros.org/actionlib). Actions execute long-running goals that can be preempted to get periodic feedback on the task progress. The default path planning action is provided by [move_base](http://wiki.ros.org/move_base) which links together a global and a local planner to accomplish its navigation task. It maintains two costmaps, one each for the global and local planning.

Planners must adhere to the specifications of [nave_core](https://wiki.ros.org/nav_core). The default planners are:
* [navfn/NavfnROS](https://wiki.ros.org/navfn) provides global planning with a fast, interpolated navigation function assuming a circular robot. It is computed with Dijkstra's algorithm or A* if a heuristic is supplied.
* [base_local_planner/TrajectoryPlannerROS](http://wiki.ros.org/base_local_planner#TrajectoryPlannerROS) provides local planning using Trajectory Rollout and Dynamic Window approaches. Given a plan to follow and a costmap, the controller produces velocity commands that are sent to a mobile base to control motion.

This example follows these navigation steps:
* Define pick-and-place goal markers
* Create a `MoveBaseClient` instance to interact with the `move_base`.
* Use a `move_base_msgs::MoveBaseGoal` instance to define the pick/place goal
* Send the goal to `move_base` server using the client
* Poll the client instance to get task progress updates.

