#include "robot_gui/robot_gui_class.h"

CVUIROSSubscriber::CVUIROSSubscriber() {
  // Initialize ROS node
  ros::NodeHandle nh;
  topic_name = "float_number";
  sub_ = nh.subscribe<std_msgs::Float64>(topic_name, 2,
                                         &CVUIROSSubscriber::msgCallback, this);
}

void CVUIROSSubscriber::msgCallback(const std_msgs::Float64::ConstPtr &msg) {
  data = *msg;
  ROS_DEBUG("Number received: %f", msg->data);
}

void CVUIROSSubscriber::run() {
  cv::Mat frame = cv::Mat(200, 500, CV_8UC3);

  // Init a OpenCV window and tell cvui to use it.
  cv::namedWindow(WINDOW_NAME);
  cvui::init(WINDOW_NAME);

  while (ros::ok()) {
    // Fill the frame with a nice color
    frame = cv::Scalar(49, 52, 49);

    // Create window at (40, 20) with size 250x80 (width x height) and title
    cvui::window(frame, 40, 20, 250, 40, "Topic: " + topic_name);

    // Show how many times the button has been clicked inside the window.
    cvui::printf(frame, 45, 45, 0.4, 0xff0000, "Data received: %0.2f", data);

    // Update cvui internal stuff
    cvui::update();

    // Show everything on the screen
    cv::imshow(WINDOW_NAME, frame);

    // Check if ESC key was pressed
    if (cv::waitKey(20) == 27) {
      break;
    }
    // Spin as a single-threaded node
    ros::spinOnce();
  }
}
