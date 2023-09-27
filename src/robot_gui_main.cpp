#include <ros/ros.h>
#include "robot_gui/robot_gui_class.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "cvui_robot_info_subscriber_node");
  CVUIRobotInfoSubscriber cvui_robot_info_subscriber;
  cvui_robot_info_subscriber.run();
  return 0;
}
