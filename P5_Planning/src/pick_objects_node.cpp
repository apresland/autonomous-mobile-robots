#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <queue>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void move_to_goal(MoveBaseClient &ac, const double x, const double y, const double w) {

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "odom";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = w;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("SUCCEEDED: The base attained goal pose in zone");
  else
    ROS_INFO("FAILED: The base failed to attain to goal pose in zone");
}

int main(int argc, char** argv){

  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ros::spinOnce();

  move_to_goal(ac, 1.0, 1.0, 1.0);
  sleep(5);

  move_to_goal(ac, 4.0, 3.0, 1.0);
  sleep(5);

  ROS_INFO("All pending actions processed");

  return 0;
}
