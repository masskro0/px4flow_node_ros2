#include <rclcpp/rclcpp.hpp>

#include "px4flow/SerialComm.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<px::SerialComm>("px4flow");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

