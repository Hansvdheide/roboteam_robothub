#include "ros/ros.h"

#include "roboteam_msgs/RobotCommand.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"

#include <vector>
#include <iostream>
#include <math.h>

#define PI 3.14159265

int id;
bool active;
float x_vel;
float y_vel;
float w_vel;
bool dribbler;
float kick_vel;
float chip_vel;

int main(int argc, char **argv)
{
    // Create ros node 'input_example' and advertise on ropic 'robotcommands'
    ros::init(argc, argv, "input_example");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("gazebo_listener/motorsignals", 1000);
    // ros::Rate loop_rate(60);
    
    // Keyboard inputs
    double fr_wheel;
    double fl_wheel;
    double br_wheel;
    double bl_wheel;

    std::cout << "Enter an x speed: ";
    std::cin >> x_vel;
    std::cout << "Enter a y speed: ";
    std::cin >> y_vel;
    std::cout << "Enter a rotational speed: ";
    std::cin >> w_vel;

    float robot_radius = 0.09;
    float wheel_radius = 0.03;
    float theta1 = 0.25*PI;
    float theta2 = 0.75*PI;
    float theta3 = 1.25*PI;
    float theta4 = 1.75*PI;
    fr_wheel = (-sin(theta1)*x_vel + cos(theta1)*y_vel + robot_radius*w_vel) / wheel_radius;
    fl_wheel = (-sin(theta2)*x_vel + cos(theta2)*y_vel + robot_radius*w_vel) / wheel_radius;
    bl_wheel = (-sin(theta3)*x_vel + cos(theta3)*y_vel + robot_radius*w_vel) / wheel_radius;
    br_wheel = (-sin(theta4)*x_vel + cos(theta4)*y_vel + robot_radius*w_vel) / wheel_radius;

    active = true;
    dribbler = false;
    kick_vel = 0.0;
    chip_vel = 0.0;

    // Initialize RobotCommand message;
    // roboteam_msgs::RobotCommand command;
    std::vector<double> inputs = {-fr_wheel, -fl_wheel, -bl_wheel, -br_wheel};
    std_msgs::Float64MultiArray command;

    command.layout.dim.push_back(std_msgs::MultiArrayDimension());
    command.layout.dim[0].size = 4;
    command.layout.dim[0].stride = 1;
    command.layout.dim[0].label = "speeds";
    // command.data = inputs;
    command.data.clear();
    command.data.insert(command.data.end(), inputs.begin(), inputs.end());

    // Publish the command
    chatter_pub.publish(command);

    ROS_INFO_STREAM("command sent for robot: " << id);
    ros::spin();

	return 0;
}
