#include <rclcpp/rclcpp.hpp>

#include "px4flow/SerialComm.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<px::SerialComm>("px4flow");
  std::string portStr = "/dev/ttyACM0";  //"/dev/serial/by-id/usb-3D_Robotics_PX4Flow_v1.3_000000000000-if00";   //"/dev/ttyUSB0";
  int baudrate = 115200;
  std::string frameId = "/px4flow";
  px::SerialComm comm(frameId);
  if (!comm.open(portStr, baudrate))
  {
      return -1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

