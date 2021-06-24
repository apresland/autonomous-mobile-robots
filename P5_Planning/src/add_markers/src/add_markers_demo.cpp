#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv ) {

	ros::init(argc, argv, "add_markers_demo");
	ros::NodeHandle node;
	ros::Publisher publisher = node.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	ros::spinOnce();

	ROS_INFO("Add Markers demo application");

	visualization_msgs::Marker marker;
	marker.type = visualization_msgs::Marker::CUBE;;
	marker.header.frame_id = "odom";
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	marker.id = 0;
	
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.scale.z = 0.2;

	marker.color.r = 1.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration();
	
	sleep(5);
	ROS_INFO("Add pick marker");
	node.getParam("/pick/tx", marker.pose.position.x);
	node.getParam("/pick/ty", marker.pose.position.y);
	node.getParam("/pick/tz", marker.pose.position.z);
	node.getParam("/pick/qx", marker.pose.orientation.x);
	node.getParam("/pick/qy", marker.pose.orientation.y);
	node.getParam("/pick/qz", marker.pose.orientation.z);
	node.getParam("/pick/qw", marker.pose.orientation.w);
	marker.action = visualization_msgs::Marker::ADD;
	publisher.publish(marker);

	sleep(5);
	ROS_INFO("Delete pick marker");
	marker.action = visualization_msgs::Marker::DELETE;
	publisher.publish(marker);

	sleep(5);
	ROS_INFO("Add place marker");
	node.getParam("/place/tx", marker.pose.position.x);
	node.getParam("/place/ty", marker.pose.position.y);
	node.getParam("/place/tz", marker.pose.position.z);
	node.getParam("/place/qx", marker.pose.orientation.x);
	node.getParam("/place/qy", marker.pose.orientation.y);
	node.getParam("/place/qz", marker.pose.orientation.z);
	node.getParam("/place/qw", marker.pose.orientation.w);
	marker.action = visualization_msgs::Marker::ADD;
	publisher.publish(marker);

	sleep(10);

	return 0;
};