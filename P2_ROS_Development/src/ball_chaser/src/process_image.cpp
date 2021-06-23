#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
  // TODO: Request a service and pass the velocities to it to drive the robot
  ROS_INFO("Driving robot to target - linear.x:%1.2f, angular.z:%1.2f", lin_x, ang_z);

  // Request drive to target
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;

  // Call the ball_chaser service and pass the requested target
  if (!client.call(srv))
    ROS_ERROR("Failed to call service ball_chaser::DriveToTarget");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{  
 // get pixel location of ball
 int pixel = -1; 
 for (int i = 0; i < img.height * img.step; i+=3) {
    
    bool is_max[3] = {false};

    is_max[0] = (255 == img.data[i]);
    is_max[1] = (255 == img.data[i+1]);
    is_max[2] = (255 == img.data[i+2]);
    
    if (is_max[0] && is_max[1] && is_max[2]) {
      pixel = i % img.step;
      break;
    }
  }

  float x,z =0.0; // stationary (default)

  if (pixel >= 0) {
  
    if (pixel < img.step / 3 ) {
      z =0.2; // turn left
    }
   
    else if (pixel > 2 * img.step / 3 ) {
      z =-0.2; // turn right
    }
    
    else {
      x =0.2; // go straight
    }
  }
  
  drive_robot(x,z);
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);
    ROS_INFO("Ready to receive images");

    // Handle ROS communication events
    ros::spin();

    return 0;
}
