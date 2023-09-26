#include "robot_gui/robot_gui_class.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "show_received_messages");
  CVUIROSSubscriber show_received_messages;
  show_received_messages.run();
  return 0;
}