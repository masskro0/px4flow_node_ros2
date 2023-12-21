#include "mavlink_serial_client/SerialComm.h"
#include <rclcpp/rclcpp.hpp>

int
main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<px::SerialComm>("mavlink");

    std::string portStr = "/dev/ttyACM0";
    int baudrate = 115200;
    std::string frameId = "fcu";

    px::SerialComm comm(frameId);
    if (!comm.open(portStr, baudrate))
    {
        return -1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
