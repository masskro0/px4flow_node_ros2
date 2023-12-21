#include <boost/asio.hpp>
#include <boost/thread.hpp>

// MAVLINK includes
#include <pixhawk/mavlink.h>

// ROS includes
#include <image_transport/image_transport.h>
#include <rclcpp/rclcpp.hpp>
#include <px_comm/msg/optical_flow.hpp>

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

    boost::asio::io_service m_uartService;
    boost::asio::serial_port m_port;
    boost::asio::deadline_timer m_timer;
    boost::thread m_uartThread;

    size_t m_imageSize;       ///< Image size being transmitted (bytes)
    size_t m_imagePackets;    ///< Number of data packets being sent for this image
    int m_imagePayload;       ///< Payload size per transmitted packet (bytes). Standard is 254, and decreases when image resolution increases.
    int m_imageWidth;         ///< Width of the image stream
    int m_imageHeight;        ///< Width of the image stream
    std::vector<uint8_t> m_imageBuffer;

    int m_systemId;
    int m_compId;

    uint8_t m_buffer[MAVLINK_MAX_PACKET_LEN];

   rclcpp::Publisher<px_comm::msg::OpticalFlow>::SharedPtr m_optFlowPub;
    image_transport::Publisher m_imagePub;
    std::string m_frameId;

   rclcpp::TimerBase::SharedPtr timer_;

    bool m_timeout;
    int m_errorCount;

    bool m_connected;
};

}
