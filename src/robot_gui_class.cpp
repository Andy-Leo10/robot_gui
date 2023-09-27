#include "robot_gui/robot_gui_class.h"

CVUIRobotInfoSubscriber::CVUIRobotInfoSubscriber() {
  // Suscribirse al mensaje 'robot_info'
  robot_info_sub = nh.subscribe("/robot_info", 1, &CVUIRobotInfoSubscriber::robotInfoCallback, this);

  // Crear la ventana de OpenCV y configurar CVUI
  cv::namedWindow(WINDOW_NAME);
  cvui::init(WINDOW_NAME);
}

void CVUIRobotInfoSubscriber::run() {
  while (ros::ok()) {
    // Crear una matriz OpenCV para mostrar los datos
    cv::Mat frame = cv::Mat(300, 500, CV_8UC3);
    cv::Scalar bgColor = cv::Scalar(49, 52, 49);
    frame = bgColor;

    // Mostrar los datos en la ventana
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

    // Actualizar la interfaz CVUI
    cvui::update();

    // Mostrar la ventana
    cv::imshow(WINDOW_NAME, frame);

    // Comprobar si se presiona 'q' para salir
    if (cv::waitKey(20) == 'q') {
      break;
    }

    ros::spinOnce();
  }
}

void CVUIRobotInfoSubscriber::robotInfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg) {
  // Copiar el mensaje 'robot_info' a los datos internos
  robot_info_data = *msg;
}
