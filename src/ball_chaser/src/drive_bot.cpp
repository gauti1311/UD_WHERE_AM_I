#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
//DONE: Include the ball_chaser "DriveToTarget" header file
#include "ball_chaser/DriveToTarget.h"


// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// DONE: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
// This function should publish the requested linear x and angular velocities to the robot wheel joints
{
       // Create a motor_command object of type geometry_msgs::Twist
    geometry_msgs::Twist motor_command;
        // Set wheel velocities, 
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;
        // Publish angles to drive the robot
    motor_command_publisher.publish(motor_command);
    res.msg_feedback = "motor command set linear.x: 0.5, angular.z: 0";
    ROS_INFO_STREAM(res.msg_feedback);
    return true;
}
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // DONE: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot",handle_drive_request);
    // DONE: Delete the loop, move the code to the inside of the callback function and make the necessary changes to publish the requested velocities instead of constant values

    // DONE: Handle ROS communication events
    ROS_INFO("ready to send motor command");
    ros::spin();

    return 0;
}