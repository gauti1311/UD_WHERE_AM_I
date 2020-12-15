#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // DONE: Request a service and pass the velocities to it to drive the robot
  
    ROS_INFO("Moving the robot with velocity linear : %1.2f, Angular : %1.2f ", (lin_x, ang_z) );

    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if(!client.call(srv))
        ROS_ERROR("Failed to drive robot");
  
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    std::string pos ;
    float x = 0.0f;
    float z = 0.0f;
    // DONE: Loop through each pixel in the image and check if there's a bright white one
    for(int i = 0; i < img.height * img.step; i+=3)
    {  
      if(img.data[i] == white_pixel && img.data[i+1] == white_pixel && img.data[i+2]== white_pixel)
      {
        int step = i % img.step;
        
          if(step <= img.step/3)
          {
            pos = "left";
            z = 0.4f;
          }
          else if(step <= 2*img.step/3)
          {
            pos = "mid";
            x = 0.2f;
          }
          else
          {
            pos = "right";
            z = -0.4f;
          }
        drive_robot(x,z);
        std::cout << "ball is on " << pos << " side of robot "<< std::endl;
        return ;
       	}
    }
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    drive_robot(x, z); 
    ros::Duration(1).sleep();
    // Request a stop when there's no white ball seen by the camera
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

    // Handle ROS communication events
    ros::spin();

    return 0;
}