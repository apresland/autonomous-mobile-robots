
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// A handle_drive_request callback function that executes whenever a drive_bot service is requested
bool handle_drive_request(
  ball_chaser::DriveToTarget::Request& req,
  ball_chaser::DriveToTarget::Response& res)
{
  ROS_INFO("DriveToTargetRequest received - linear.x:%1.2f, angular.z:%1.2f",
           (float)req.linear_x,
           (float)req.angular_z);

  
    
  // Requested velocities
  std::vector<float> velocity = { (float)req.linear_x, (float)req.angular_z };
    
  // Publish angles to drive the robot
  geometry_msgs::Twist motor_command;
  motor_command.linear.x = velocity[0];
  motor_command.angular.z = velocity[1];
  motor_command_publisher.publish(motor_command);

  // Return a response message
  res.msg_feedback = "Wheel velocity set - linear.x: " 
    + std::to_string(velocity[0]) 
    + " , angular.z: "
    + std::to_string(velocity[1]);
  ROS_INFO_STREAM(res.msg_feedback);

  return true;
}

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist 
    // on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  	ros::ServiceServer service = n.advertiseService(
      "/ball_chaser/command_robot", handle_drive_request);
  	ROS_INFO("Ready to send drive commands");
  
    ros::spin();

    return 0;
}