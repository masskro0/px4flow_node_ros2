#include "mavlink_serial_client/SerialComm.h"
#include <rclcpp/rclcpp.hpp>

int
main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<px::SerialComm>("mavlink_serial_client");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
