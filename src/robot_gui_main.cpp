#include <ros/ros.h>
#include "robot_gui/robot_gui_class.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_gui_node");
  CVUIRobotGUI cvui_robot_gui;
  cvui_robot_gui.run();
  return 0;
}
