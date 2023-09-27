#ifndef ROBOT_GUI_CLASS_H
#define ROBOT_GUI_CLASS_H

#define CVUI_IMPLEMENTATION
#include <ros/ros.h>
#include <robotinfo_msgs/RobotInfo10Fields.h>
#include <robot_gui/cvui.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>
#include <vector>

class CVUIRobotInfoSubscriber {
public:
  CVUIRobotInfoSubscriber();

  void run();

private:
  ros::NodeHandle nh;
  ros::Subscriber robot_info_sub;
  robotinfo_msgs::RobotInfo10Fields robot_info_data;
  const std::string WINDOW_NAME = "CVUI Robot Info";

  void robotInfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg);
};

#endif // ROBOT_GUI_CLASS_H
