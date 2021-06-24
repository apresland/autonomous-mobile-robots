#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

double home[2] = {0.0, 0.0};
double pick[2] = {1.0, 1.0};
double place[2] = {3.5, 0.0};

int main(int argc, char** argv){

  // Initialize the navigation goal node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle node;
  ros::Publisher publisher = node.advertise<std_msgs::UInt8>("/process_state", 1);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  std_msgs::UInt8 state;

  ROS_INFO("Moving to the pick zone");
  state.data = 0;

  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();
  node.getParam("/pick/tx", goal.target_pose.pose.position.x);
  node.getParam("/pick/ty", goal.target_pose.pose.position.y);
  node.getParam("/pick/tz", goal.target_pose.pose.position.z);
  node.getParam("/pick/qx", goal.target_pose.pose.orientation.x);
  node.getParam("/pick/qy", goal.target_pose.pose.orientation.y);
  node.getParam("/pick/qz", goal.target_pose.pose.orientation.z);
  node.getParam("/pick/qw", goal.target_pose.pose.orientation.w);

  publisher.publish(state);
  ac.sendGoal(goal);
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) { 
    ROS_INFO("Attained a pose in the pick zone");
  } else {
    ROS_INFO("Did not attain a pose in the pick zone");
    return 0;
  }

  ROS_INFO("Picking object");
  sleep(5);
  state.data = 1;
  publisher.publish(state);

  ROS_INFO("Moving to the place zone");
  state.data = 2;
  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();
  node.getParam("/place/tx", goal.target_pose.pose.position.x);
  node.getParam("/place/ty", goal.target_pose.pose.position.y);
  node.getParam("/place/tz", goal.target_pose.pose.position.z);
  node.getParam("/place/qx", goal.target_pose.pose.orientation.x);
  node.getParam("/place/qy", goal.target_pose.pose.orientation.y);
  node.getParam("/place/qz", goal.target_pose.pose.orientation.z);
  node.getParam("/place/qw", goal.target_pose.pose.orientation.w);
  publisher.publish(state);
  ac.sendGoal(goal);
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) { 
    ROS_INFO("Attained a pose in the place zone");
  } else {
    ROS_INFO("Did not attain a pose in the place zone");
    return 0;
  }

  ROS_INFO("Placing object");
  sleep(5);
  state.data = 3;
  publisher.publish(state);


  ROS_INFO("Objective complete returning home");
  state.data = 4;
  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();
  node.getParam("/home/tx", goal.target_pose.pose.position.x);
  node.getParam("/home/ty", goal.target_pose.pose.position.y);
  node.getParam("/home/tz", goal.target_pose.pose.position.z);
  node.getParam("/home/qx", goal.target_pose.pose.orientation.x);
  node.getParam("/home/qy", goal.target_pose.pose.orientation.y);
  node.getParam("/home/qz", goal.target_pose.pose.orientation.z);
  node.getParam("/home/qw", goal.target_pose.pose.orientation.w);
  publisher.publish(state);
  ac.sendGoal(goal);
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) { 
    ROS_INFO("Attained a pose in the home zone");
  } else {
    ROS_INFO("Did not attain a pose in the home zone");
    return 0;
  }
  
  sleep(10);

  return 0;
}
