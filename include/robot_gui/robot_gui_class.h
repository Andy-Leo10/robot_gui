#ifndef ROBOT_GUI_CLASS_H
#define ROBOT_GUI_CLASS_H

#define CVUI_IMPLEMENTATION
#include <ros/ros.h>
#include <robotinfo_msgs/RobotInfo10Fields.h>
#include <robot_gui/cvui.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include "nav_msgs/Odometry.h"
#include <vector>

class CVUIRobotGUI
{
public:
  CVUIRobotGUI();

  void run();

private:
  ros::NodeHandle nh;
  const std::string WINDOW_NAME = "CVUI Robot GUI";
  // suscriber for the message robot_info
  ros::Subscriber robot_info_sub;
  robotinfo_msgs::RobotInfo10Fields robot_info_data;
  std::string robot_info_topic_name = "/robot_info";
  // publisher for the message cmd_vel
  ros::Publisher twist_pub;
  geometry_msgs::Twist twist_msg;
  std::string twist_topic_name = "cmd_vel";
  float linear_velocity_step = 0.1;
  float angular_velocity_step = 0.1;
  // suscriber for odom
  ros::Subscriber odom_sub;
  nav_msgs::Odometry odom_data;
  std::string odom_topic_name = "/odom";
  float x_position = 0.0;
  float y_position = 0.0;
  float z_position = 0.0;

  // callback functions
  void robotInfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
};

#endif // ROBOT_GUI_CLASS_H
