#include "robot_gui/robot_gui_class.h"

CVUIRobotGUI::CVUIRobotGUI()
{
  // suscribe to the topic robot_info
  robot_info_sub = nh.subscribe(robot_info_topic_name, 1, &CVUIRobotGUI::robotInfoCallback, this);
  // publisher for the message cmd_vel
  twist_pub = nh.advertise<geometry_msgs::Twist>(twist_topic_name, 10);
  // suscribe to the topic odom
  odom_sub = nh.subscribe(odom_topic_name, 1, &CVUIRobotGUI::odomCallback, this);
  // Create a service client that sends requests of type std_srvs/Trigger
  service_client = nh.serviceClient<std_srvs::Trigger>(service_name);
  // create the window and configure CVUI
  cv::namedWindow(WINDOW_NAME);
  cvui::init(WINDOW_NAME);
  // enable ROS DEBUG messages
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
}

void CVUIRobotGUI::robotInfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg)
{
  // copy the message to the local variable
  robot_info_data = *msg;
}

void CVUIRobotGUI::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  // copy the message to the local variable
  odom_data = *msg;
  x_position = msg->pose.pose.position.x;
  y_position = msg->pose.pose.position.y;
  z_position = msg->pose.pose.position.z;
  // ROS_DEBUG("Position x,y,z: [%0.2f, %0.2f, %0.2f]", x_position, y_position, z_position);
}

void CVUIRobotGUI::run()
{
  float distance_travelled = 0.0;
  while (ros::ok())
  {
    // Clear the frame with a nice color
    cv::Mat frame = cv::Mat(500, 500, CV_8UC3);
    cv::Scalar bgColor = cv::Scalar(49, 52, 49);
    frame = bgColor;

    /*---DISPLAY ROBOT INFO---*/
    cvui::text(frame, 10, 10, "Robot Info:");

    cvui::text(frame, 10, 40, "Field 1: " + robot_info_data.data_field_01);
    cvui::text(frame, 10, 60, "Field 2: " + robot_info_data.data_field_02);
    cvui::text(frame, 10, 80, "Field 3: " + robot_info_data.data_field_03);
    cvui::text(frame, 10, 100, "Field 4: " + robot_info_data.data_field_04);
    cvui::text(frame, 10, 120, "Field 5: " + robot_info_data.data_field_05);
    cvui::text(frame, 10, 140, "Field 6: " + robot_info_data.data_field_06);
    cvui::text(frame, 10, 160, "Field 7: " + robot_info_data.data_field_07);
    cvui::text(frame, 10, 180, "Field 8: " + robot_info_data.data_field_08);
    cvui::text(frame, 10, 200, "Field 9: " + robot_info_data.data_field_09);
    cvui::text(frame, 10, 220, "Field 10: " + robot_info_data.data_field_10);

    /*---BUTTONS FOR CONTROL THE ROBOT---*/
    // Show a button at position x = 100, y = 250
    if (cvui::button(frame, 100, 250, " Forward "))
    {
      // The button was clicked, update the Twist message
      twist_msg.linear.x = twist_msg.linear.x + linear_velocity_step;
    }

    // Show a button at position x = 100, y = 280
    if (cvui::button(frame, 100, 280, "   Stop  "))
    {
      // The button was clicked, update the Twist message
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = 0.0;
    }

    // Show a button at position x = 30, y = 280
    if (cvui::button(frame, 30, 280, " Left "))
    {
      // The button was clicked, update the Twist message
      twist_msg.angular.z = twist_msg.angular.z + angular_velocity_step;
    }

    // Show a button at position x = 195, y = 280
    if (cvui::button(frame, 195, 280, " Right "))
    {
      // The button was clicked, update the Twist message
      twist_msg.angular.z = twist_msg.angular.z - angular_velocity_step;
    }

    // Show a button at position x = 100, y = 310
    if (cvui::button(frame, 100, 310, "Backward"))
    {
      // The button was clicked,update the Twist message
      twist_msg.linear.x = twist_msg.linear.x - linear_velocity_step;
    }
    twist_pub.publish(twist_msg);
    // Create window at (320, 250) with size 120x40 (width x height) and title
    cvui::window(frame, 320, 250, 120, 40, "Linear velocity:");
    // Show the current velocity inside the window
    cvui::printf(frame, 345, 275, 0.4, 0xff0000, "%.02f m/sec",
                 twist_msg.linear.x);

    // Create window at (320 290) with size 120x40 (width x height) and title
    cvui::window(frame, 320, 290, 120, 40, "Angular velocity:");
    // Show the current velocity inside the window
    cvui::printf(frame, 345, 315, 0.4, 0xff0000, "%.02f rad/sec",
                 twist_msg.angular.z);

    /*---DISPLAY FOR ODOMETRY---*/
    cvui::text(frame, 10, 350, "Estimated robot position:");
    cvui::printf(frame, 10, 370, 0.4, 0xff0000, "X: %.02f", x_position);
    cvui::printf(frame, 100, 370, 0.4, 0xff0000, "Y: %.02f", y_position);
    cvui::printf(frame, 190, 370, 0.4, 0xff0000, "Z: %.02f", z_position);

    /*---BUTTON FOR CALL THE SERVICE---*/
    // Create window at (40, 20) with size 460x80 (width x height) and title
    cvui::window(frame, 40, 420, 420, 40, "Service: " + service_name);

    // Call the service
    if (cvui::button(frame, 45, 465, "Call Service"))
    {
      // Send the request and wait for a response
      if (service_client.call(srv_req))
      {
        // Print the response message and return true
        // ROS_DEBUG("Response message: %s", srv_req.response.message.c_str());
        // set latest service call status
        last_service_call_msg = srv_req.response.message;
      }
      else
      {
        last_service_call_msg = "Service call failed.";
      }
    }

    // Display the last response inside the window
    if (not last_service_call_msg.empty())
    {
      distance_travelled = std::stof(last_service_call_msg) - distance_at_reset;
      cvui::printf(frame, 45, 445, 0.4, 0xff0000, "%2f",
                   distance_travelled);
    }

    // Create a button at position x = 300, y = 465
    if (cvui::button(frame, 300, 465, "Reset Tracking Value"))
    {
      //convert to float the string and store it 
      distance_at_reset = std::stof(last_service_call_msg);
    }

    // UPDATE the interface
    cvui::update();
    // SHOW the interface
    cv::imshow(WINDOW_NAME, frame);
    // if the user press 'q' or ESC the program ends
    if (cv::waitKey(30) == 'q' || cv::waitKey(30) == 27)
      break;

    ros::spinOnce();
  }
}
