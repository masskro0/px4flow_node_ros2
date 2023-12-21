#include <boost/asio.hpp>
#include <boost/thread.hpp>

// MAVLINK includes
#include <pixhawk/mavlink.h>

// ROS includes
#include <image_transport/image_transport.h>
#include <rclcpp/rclcpp.hpp>
#include <px_comm/msg/mavlink.hpp>
#include <px_comm/msg/optical_flow.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

namespace px
{

 class SerialComm : public rclcpp::Node
{
public:
    SerialComm(const std::string& frameId);
    ~SerialComm();

    bool open(const std::string& portStr, int baudrate);
    void close(void);

private:
    void readCallback(const boost::system::error_code& error, size_t bytesTransferred);
    void readStart(uint32_t timeout_ms);
    void syncCallback();
    void timeoutCallback(const boost::system::error_code& error);

    void rpyToQuaternion(const double& roll, const double& pitch, const double& yaw,
                         double& w, double& x, double& y, double& z) const;
    void publishMAVLINKMessage(const mavlink_message_t& message);

    boost::asio::io_service m_uartService;
    boost::asio::serial_port m_port;
    boost::asio::deadline_timer m_timer;
    boost::thread m_uartThread;
    bool m_timeout{};

    size_t m_imageSize{};       ///< Image size being transmitted (bytes)
    size_t m_imagePackets{};    ///< Number of data packets being sent for this image
    int m_imagePayload{};       ///< Payload size per transmitted packet (bytes). Standard is 254, and decreases when image resolution increases.
    int m_imageWidth{};         ///< Width of the image stream
    int m_imageHeight{};        ///< Width of the image stream
    std::vector<uint8_t> m_imageBuffer;

    int m_systemId{};
    int m_compId{};

    uint8_t m_buffer[MAVLINK_MAX_PACKET_LEN]{};

    image_transport::Publisher m_imagePub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imuPub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imuRawPub;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr m_magPub;
    rclcpp::Publisher<px_comm::msg::Mavlink>::SharedPtr m_mavlinkPub;
    rclcpp::Publisher<px_comm::msg::OpticalFlow>::SharedPtr m_optFlowPub;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_viconPub;
    rclcpp::TimerBase::SharedPtr timer_;
    mavlink_highres_imu_t m_imuRaw{};

    std::string m_frameId;

    int m_errorCount{};

    bool m_connected{};
};

}
