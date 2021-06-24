#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

class AddMarkersNode {

public:

	enum PickState : int { PICK, MOVE, PLACE, IDLE };
	PickState state = PickState::PICK;
	
	ros::NodeHandle node;
	ros::Publisher marker_publisher;
	ros::Subscriber odometry_subscriber;
	ros::Subscriber target_subscriber;

	geometry_msgs::Pose odometry;
	geometry_msgs::Pose target;

	visualization_msgs::Marker marker;

	double pick[2] = {1.0, 1.0};
	double place[2] = {4.0, 3.0};

	ros::Time stamp;

	AddMarkersNode();

	void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg);
	void targetCallback(const geometry_msgs::Pose &msg); 
	void update();
};

AddMarkersNode::AddMarkersNode() {

	marker_publisher = node.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	odometry_subscriber = node.subscribe("/odom", 1, &AddMarkersNode::odometryCallback, this);
	target_subscriber = node.subscribe("/target", 1, &AddMarkersNode::targetCallback, this);
	stamp = ros::Time::now();

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

	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;

	marker.color.r = 1.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;
}

void AddMarkersNode::odometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {

	ROS_INFO("Odometry update received");
	odometry.position.x = msg->pose.pose.position.x;
	odometry.position.y = msg->pose.pose.position.y;
	odometry.position.z = msg->pose.pose.position.z;
	odometry.orientation.x = msg->pose.pose.orientation.x;
	odometry.orientation.y = msg->pose.pose.orientation.y;
	odometry.orientation.z = msg->pose.pose.orientation.z;
	odometry.orientation.w = msg->pose.pose.orientation.w;
	this->update();
}

void AddMarkersNode::targetCallback(const geometry_msgs::Pose &msg) {

	ROS_INFO("Target update received");
	target.position.x = msg.position.x;
	target.position.y = msg.position.y;
	target.position.z = msg.position.z;
	target.orientation.x = msg.orientation.x;
	target.orientation.y = msg.orientation.y;
	target.orientation.z = msg.orientation.z;
	target.orientation.w = msg.orientation.w;
	this->update();
}

void AddMarkersNode::update()
{
	switch (state) {
	
		case PickState::PICK: {
			ROS_INFO("Picking from origin");
			marker.pose.position.x = pick[0];
			marker.pose.position.y = pick[1];
			marker.action = visualization_msgs::Marker::ADD;
			marker.lifetime = ros::Duration();
			stamp = ros::Time::now();
			state = PickState::MOVE;
		} break; 
	
		case PickState::MOVE: {
			ROS_INFO("Moving to destination");
			marker.action = visualization_msgs::Marker::DELETE;
			marker.lifetime = ros::Duration();
			stamp = ros::Time::now();
			state = PickState::PLACE;
		} break;

		case PickState::PLACE: {
			ROS_INFO("Placing at destination");
			marker.pose.position.x = place[0];
			marker.pose.position.y = place[1];
			marker.action = visualization_msgs::Marker::ADD;
			marker.lifetime = ros::Duration();
			stamp = ros::Time::now();
			state = PickState::IDLE;
		} break;

		case PickState::IDLE: {
			ROS_INFO("The motion base is idle");
			marker.pose.position.x = place[0];
			marker.pose.position.y = place[1];
			marker.action = visualization_msgs::Marker::ADD;
			marker.lifetime = ros::Duration();
			/** nothing to do **/
		}
	};

	marker_publisher.publish(marker);
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "add_markers");
	AddMarkersNode node;
	ros::Rate r(1);
	ros::spin();
};