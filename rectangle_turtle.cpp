#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <unistd.h>  // For sleep()

void move_straight(ros::Publisher &velocity_publisher, double speed, double time) {
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = speed;
    vel_msg.angular.z = 0.0;

    double start_time = ros::Time::now().toSec();
    while (ros::Time::now().toSec() - start_time < time) {
        velocity_publisher.publish(vel_msg);
        ros::Duration(0.1).sleep();
    }
    // Stop after moving
    vel_msg.linear.x = 0.0;
    velocity_publisher.publish(vel_msg);
}

void turn_90(ros::Publisher &velocity_publisher, double angular_speed, double time) {
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = angular_speed;

    double start_time = ros::Time::now().toSec();
    while (ros::Time::now().toSec() - start_time < time) {
        velocity_publisher.publish(vel_msg);
        ros::Duration(0.1).sleep();
    }
    // Stop after turning
    vel_msg.angular.z = 0.0;
    velocity_publisher.publish(vel_msg);
}

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "turtle_rectangle");
    ros::NodeHandle nh;

    // Create a publisher to publish velocity commands to /turtle1/cmd_vel
    ros::Publisher velocity_publisher = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    // Rectangle dimensions (length and breadth in meters)
    double length = 4.0;    // Length of the rectangle
    double breadth = 2.0;   // Breadth of the rectangle

    // Speed and time parameters
    double linear_speed = 1.0;        // Speed of turtle in m/s
    double angular_speed = 1.57;     // Angular speed in rad/s (90 degrees/s)
    double move_time_length = length / linear_speed;
    double move_time_breadth = breadth / linear_speed;
    double turn_time = 1.57 / angular_speed;  // Time to turn 90 degrees

    // Move the turtle in a rectangle
    ros::Rate loop_rate(10); // 10 Hz
    for (int i = 0; i < 2; ++i) {
        // Move along length
        move_straight(velocity_publisher, linear_speed, move_time_length);
        ros::Duration(1).sleep();

        // Turn 90 degrees
        turn_90(velocity_publisher, angular_speed, turn_time);
        ros::Duration(1).sleep();

        // Move along breadth
        move_straight(velocity_publisher, linear_speed, move_time_breadth);
        ros::Duration(1).sleep();

        // Turn 90 degrees
        turn_90(velocity_publisher, angular_speed, turn_time);
        ros::Duration(1).sleep();
    }

    ROS_INFO("Rectangle movement completed!");
    return 0;
}
