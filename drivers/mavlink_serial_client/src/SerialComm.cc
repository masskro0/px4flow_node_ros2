#include <boost/bind.hpp>
#include <Eigen/Dense>
#include <chrono>

#include "mavlink_serial_client/SerialComm.h"

using namespace std::chrono_literals;

namespace px
{

SerialComm::SerialComm(const std::string& frameId)
 : Node("mavlink_serial_client")
 , m_port(m_uartService)
 , m_timer(m_uartService)
 , m_timeout(false)
 , m_imageSize(0)
 , m_imagePackets(0)
 , m_imagePayload(0)
 , m_imageWidth(0)
 , m_imageHeight(0)
 , m_systemId(-1)
 , m_compId(0)
 , m_frameId(frameId)
 , m_errorCount(0)
 , m_connected(false)
{
  std::string serial_port;
  int baudrate;
  std::string tpub_mavlink;
  std::string tpub_opt_flow;
  std::string tpub_camera_image;
  std::string tpub_imu;
  std::string tpub_mag;
  std::string tpub_vicon;
  std::string tpub_raw_imu;
  this->declare_parameter("serial_port", "");
  serial_port = this->get_parameter("serial_port").as_string();
  this->declare_parameter("baudrate", 0);
  baudrate = this->get_parameter("baudrate").as_int();
  this->declare_parameter("tpub_mavlink", "");
  tpub_mavlink = this->get_parameter("tpub_mavlink").as_string();
  this->declare_parameter("tpub_opt_flow", "");
  tpub_opt_flow = this->get_parameter("tpub_opt_flow").as_string();
  this->declare_parameter("tpub_camera_image", "");
  tpub_camera_image = this->get_parameter("tpub_camera_image").as_string();
  this->declare_parameter("tpub_imu", "");
  tpub_imu = this->get_parameter("tpub_imu").as_string();
  this->declare_parameter("tpub_mag", "");
  tpub_mag = this->get_parameter("tpub_mag").as_string();
  this->declare_parameter("tpub_vicon", "");
  tpub_vicon = this->get_parameter("tpub_vicon").as_string();
  this->declare_parameter("tpub_raw_imu", "");
  tpub_raw_imu = this->get_parameter("tpub_raw_imu").as_string();

  if (!this->open(serial_port, baudrate))
  {
    rclcpp::shutdown();
  }

  // set up publishers
  m_mavlinkPub = this->create_publisher<px_comm::msg::Mavlink>(tpub_mavlink, 100);
  m_optFlowPub = this->create_publisher<px_comm::msg::OpticalFlow>(tpub_opt_flow, 5);

  image_transport::ImageTransport it(this->make_shared(m_frameId));
  m_imagePub = it.advertise(tpub_camera_image, 5);

  // AscTec-specific
  m_imuPub = this->create_publisher<sensor_msgs::msg::Imu>(tpub_imu, 10);
  m_magPub = this->create_publisher<sensor_msgs::msg::MagneticField>(tpub_mag, 10);
  m_viconPub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(tpub_vicon, 10);

  m_imuRawPub = this->create_publisher<sensor_msgs::msg::Imu>(tpub_raw_imu, 10);
  timer_ = this->create_wall_timer(2.0s, std::bind(&SerialComm::syncCallback, this));
}

SerialComm::~SerialComm()
{
    if (m_connected)
    {
        close();
    }
}

bool
SerialComm::open(const std::string& portStr, int baudrate)
{
    m_timeout = false;
    m_errorCount = 0;
    m_connected = false;

    // open port
    try
    {
        m_port.open(portStr);

        RCLCPP_WARN(get_logger(), "Opened serial port %s.", portStr.c_str());
    }
    catch (boost::system::system_error::exception e)
    {
        RCLCPP_ERROR(get_logger(), "Could not open serial port %s. Reason: %s.", portStr.c_str(), e.what());

        return false;
    }

    // configure baud rate
    try
    {
        m_port.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
        m_port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        m_port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        m_port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        m_port.set_option(boost::asio::serial_port_base::character_size(8));

        RCLCPP_INFO(get_logger(), "Set baudrate %d.", baudrate);
    }
    catch (boost::system::system_error::exception e)
    {
        RCLCPP_ERROR(get_logger(), "Could not set baudrate %d. Reason: %s.", baudrate, e.what());

        return false;
    }

    // set up thread to asynchronously read data from serial port
    readStart(1000);
    m_uartThread = boost::thread(boost::bind(&boost::asio::io_service::run, &m_uartService));

    m_connected = true;

    return true;
}

void
SerialComm::close(void)
{
    if (!m_connected)
    {
        return;
    }

    m_uartService.post(boost::bind(&boost::asio::deadline_timer::cancel, &m_timer));
    m_uartService.post(boost::bind(&boost::asio::serial_port::close, &m_port));

    m_uartThread.join();
}

void
SerialComm::readCallback(const boost::system::error_code& error, size_t bytesTransferred)
{
    if (error)
    {
        if (error == boost::asio::error::operation_aborted)
        {
            // if serial connection timed out, try reading again
            if (m_timeout)
            {
                m_timeout = false;
                readStart(1000);

                return;
            }
        }

        RCLCPP_WARN(get_logger(), "Read error: %s", error.message().c_str());

        if (m_errorCount < 10)
        {
            readStart(1000);
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "# read errors exceeded 10. Aborting...");
            rclcpp::shutdown();
        }

        ++m_errorCount;

        return;
    }

    m_timer.cancel();

    mavlink_message_t message;
    mavlink_status_t status;

    for (size_t i = 0; i < bytesTransferred; i++)
    {
        bool msgReceived = mavlink_parse_char(MAVLINK_COMM_1, m_buffer[i], &message, &status);

        if (msgReceived)
        {
            publishMAVLINKMessage(message);

            m_systemId = message.sysid;

            switch (message.msgid)
            {
            // Message specs.
            case MAVLINK_MSG_ID_ATTITUDE:
            {
                if (m_imuPub->get_subscription_count() == 0)
                {
                    break;
                }

                mavlink_attitude_t att;
                mavlink_msg_attitude_decode(&message, &att);

                sensor_msgs::msg::Imu imu_msg;

                rpyToQuaternion(att.roll, -att.pitch, -att.yaw,
                                imu_msg.orientation.w, imu_msg.orientation.x,
                                imu_msg.orientation.y, imu_msg.orientation.z);

                // TODO: check/verify that these are body-fixed
                imu_msg.angular_velocity.x = att.rollspeed;
                imu_msg.angular_velocity.y = -att.pitchspeed;
                imu_msg.angular_velocity.z = -att.yawspeed;

                // take this from imu high res message, this is supposed to arrive before this one and should pretty much be in sync then
                imu_msg.linear_acceleration.x = m_imuRaw.xacc;
                imu_msg.linear_acceleration.y = -m_imuRaw.yacc;
                imu_msg.linear_acceleration.z = -m_imuRaw.zacc;

                // TODO: can we fill in the covariance here from a parameter that we set from the specs/experience?
                for (sensor_msgs::msg::Imu::_orientation_covariance_type::iterator it = imu_msg.orientation_covariance.begin();
                     it != imu_msg.orientation_covariance.end(); ++it)
                {
                    *it = 0;
                }

                for (sensor_msgs::msg::Imu::_angular_velocity_covariance_type::iterator it =
                     imu_msg.angular_velocity_covariance.begin(); it != imu_msg.angular_velocity_covariance.end(); ++it)
                {
                    *it = 0;
                }

                for (sensor_msgs::msg::Imu::_linear_acceleration_covariance_type::iterator it =
                     imu_msg.linear_acceleration_covariance.begin(); it != imu_msg.linear_acceleration_covariance.end();
                     ++it)
                {
                    *it = 0;
                }

                imu_msg.header.frame_id = m_frameId;
                imu_msg.header.stamp = this->get_clock()->now();
                m_imuPub->publish(imu_msg);
            }
            case MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE:
            {
                mavlink_data_transmission_handshake_t handshake;
                mavlink_msg_data_transmission_handshake_decode(&message, &handshake);

                m_imageSize = handshake.size;
                m_imagePackets = handshake.packets;
                m_imagePayload = handshake.payload;
                m_imageWidth = handshake.width;
                m_imageHeight = handshake.height;

                if (m_imageBuffer.size() < m_imageSize)
                {
                    m_imageBuffer.resize(m_imageSize);
                }

                break;
            }
            case MAVLINK_MSG_ID_ENCAPSULATED_DATA:
            {
                if (m_imageSize == 0 || m_imagePackets == 0)
                {
                    break;
                }

                mavlink_encapsulated_data_t img;
                mavlink_msg_encapsulated_data_decode(&message, &img);
                size_t seq = img.seqnr;
                size_t pos = seq * m_imagePayload;

                if (seq + 1 > m_imagePackets)
                {
                    break;
                }

                size_t bytesToCopy = m_imagePayload;
                if (pos + m_imagePayload >= m_imageSize)
                {
                     bytesToCopy = m_imageSize - pos;
                }

                memcpy(&m_imageBuffer[pos], img.data, bytesToCopy);

                if (seq + 1 == m_imagePackets && m_imagePub.getNumSubscribers() > 0)
                {
                    sensor_msgs::msg::Image image;
                    image.header.frame_id = m_frameId;
                    image.height = m_imageHeight;
                    image.width = m_imageWidth;
                    image.encoding = sensor_msgs::image_encodings::MONO8;
                    image.is_bigendian = false;
                    image.step = m_imageWidth;

                    image.data.resize(m_imageSize);
                    memcpy(&image.data[0], &m_imageBuffer[0], m_imageSize);

                    m_imagePub.publish(image);
                }
                break;
            }
            // Message specs https://pixhawk.ethz.ch/mavlink/#HIGHRES_IMU
            case MAVLINK_MSG_ID_HIGHRES_IMU:
            {
                mavlink_msg_highres_imu_decode(&message, &m_imuRaw);

                std_msgs::msg::Header header;
                header.stamp = this->get_clock()->now();
                header.frame_id = m_frameId;

                if (m_imuRawPub->get_subscription_count() > 0)
                {
                    sensor_msgs::msg::Imu imu_msg;

                    imu_msg.angular_velocity.x = m_imuRaw.xgyro;
                    imu_msg.angular_velocity.y = -m_imuRaw.ygyro;
                    imu_msg.angular_velocity.z = -m_imuRaw.zgyro;

                    imu_msg.linear_acceleration.x = m_imuRaw.xacc;
                    imu_msg.linear_acceleration.y = -m_imuRaw.yacc;
                    imu_msg.linear_acceleration.z = -m_imuRaw.zacc;

                    // TODO: can we fill in the covariance here from a parameter that we set from the specs/experience?
                    for (sensor_msgs::msg::Imu::_angular_velocity_covariance_type::iterator it =
                         imu_msg.angular_velocity_covariance.begin(); it != imu_msg.angular_velocity_covariance.end();
                         ++it)
                    {
                        *it = 0;
                    }

                    for (sensor_msgs::msg::Imu::_linear_acceleration_covariance_type::iterator it =
                         imu_msg.linear_acceleration_covariance.begin(); it != imu_msg.linear_acceleration_covariance.end();
                         ++it)
                    {
                        *it = 0;
                    }

                    imu_msg.orientation_covariance[0] = -1;

                    imu_msg.header = header;

                    m_imuRawPub->publish(imu_msg);
                }

                if (m_magPub->get_subscription_count() > 0)
                {
                    const double gauss_to_tesla = 1.0e-4;
                    sensor_msgs::msg::MagneticField mag_msg;

                    mag_msg.magnetic_field.x = m_imuRaw.xmag * gauss_to_tesla;
                    mag_msg.magnetic_field.y = m_imuRaw.ymag * gauss_to_tesla;
                    mag_msg.magnetic_field.z = m_imuRaw.zmag * gauss_to_tesla;

                    // TODO: again covariance
                    for (sensor_msgs::msg::MagneticField::_magnetic_field_covariance_type::iterator it =
                         mag_msg.magnetic_field_covariance.begin(); it != mag_msg.magnetic_field_covariance.end(); ++it)
                    {
                        *it = 0;
                    }

                    mag_msg.header = header;
                    m_magPub->publish(mag_msg);
                }
                //TODO: pressure, temperature
                // XXX
                break;
            }
            case MAVLINK_MSG_ID_OPTICAL_FLOW:
            {
                if (m_optFlowPub->get_subscription_count() == 0)
                {
                    break;
                }

                // decode message
                mavlink_optical_flow_t flow;
                mavlink_msg_optical_flow_decode(&message, &flow);

                px_comm::msg::OpticalFlow optFlowMsg;

                optFlowMsg.header.stamp = this->get_clock()->now();
                optFlowMsg.header.frame_id = m_frameId;
                optFlowMsg.ground_distance = flow.ground_distance;
                optFlowMsg.flow_x = flow.flow_x;
                optFlowMsg.flow_y = flow.flow_y;
                optFlowMsg.velocity_x = flow.flow_comp_m_x;
                optFlowMsg.velocity_y = flow.flow_comp_m_y;
                optFlowMsg.quality = flow.quality;

                m_optFlowPub->publish(optFlowMsg);

                break;
            }
            case MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE:
            {
                if (m_viconPub->get_subscription_count() == 0)
                {
                    break;
                }

                mavlink_vicon_position_estimate_t pos;
                mavlink_msg_vicon_position_estimate_decode(&message, &pos);
                geometry_msgs::msg::PoseWithCovarianceStamped poseStampedMsg;

                auto t = rclcpp::Time(pos.usec);
                poseStampedMsg.header.stamp = t;

                if (message.compid == 99)
                {
                    poseStampedMsg.header.frame_id = "chessboard";
                }
                else
                {
                    poseStampedMsg.header.frame_id = m_frameId;
                }

                Eigen::Matrix3d R;
                R = Eigen::AngleAxisd(pos.yaw, Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(pos.pitch, Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(pos.roll, Eigen::Vector3d::UnitX());
                Eigen::Quaterniond q(R);

                poseStampedMsg.pose.pose.orientation.x = q.x();
                poseStampedMsg.pose.pose.orientation.y = q.y();
                poseStampedMsg.pose.pose.orientation.z = q.z();
                poseStampedMsg.pose.pose.orientation.w = q.w();

                poseStampedMsg.pose.pose.position.x = pos.x;
                poseStampedMsg.pose.pose.position.y = pos.y;
                poseStampedMsg.pose.pose.position.z = pos.z;

                // set covariance to 0.05m std dev
                poseStampedMsg.pose.covariance[0] = 0.05f * 0.05f;
                poseStampedMsg.pose.covariance[7] = 0.05f * 0.05f;
                poseStampedMsg.pose.covariance[14] = 0.05f * 0.05f;

                // set covariance to 0.01 rad
                poseStampedMsg.pose.covariance[21] = 0.01f * 0.01f;
                poseStampedMsg.pose.covariance[28] = 0.01f * 0.01f;
                poseStampedMsg.pose.covariance[35] = 0.01f * 0.01f;

                m_viconPub->publish(poseStampedMsg);

                break;
            }
            }
        }
    }

    readStart(1000);
}

void
SerialComm::readStart(uint32_t timeout_ms)
{
    m_port.async_read_some(boost::asio::buffer(m_buffer, sizeof(m_buffer)),
                           boost::bind(&SerialComm::readCallback, this, boost::asio::placeholders::error,
                                       boost::asio::placeholders::bytes_transferred));

    if (timeout_ms != 0)
    {
        m_timer.expires_from_now(boost::posix_time::milliseconds(timeout_ms));
        m_timer.async_wait(boost::bind(&SerialComm::timeoutCallback, this, boost::asio::placeholders::error));
    }
}

void
SerialComm::syncCallback()
{
    if (m_systemId == -1)
    {
        return;
    }

    uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::high_resolution_clock::now().time_since_epoch())
      .count();


    mavlink_message_t msg;
    mavlink_msg_system_time_pack(m_systemId, m_compId, &msg, us, 0);

    size_t messageLength = mavlink_msg_to_send_buffer(m_buffer, &msg);
    if (messageLength != boost::asio::write(m_port, boost::asio::buffer(m_buffer, messageLength)))
    {
        RCLCPP_WARN(get_logger(), "Unable to send system time over serial port.");
    }
}

void
SerialComm::timeoutCallback(const boost::system::error_code& error)
{
    if (!error)
    {
        m_port.cancel();
        m_timeout = true;
        RCLCPP_WARN(get_logger(), "Serial connection timed out.");
    }
}

void
SerialComm::rpyToQuaternion(const double& roll, const double& pitch, const double& yaw,
                            double& w, double& x, double& y, double& z) const
{
    double sR2, cR2, sP2, cP2, sY2, cY2;
    sR2 = sin(roll * 0.5);
    cR2 = cos(roll * 0.5);
    sP2 = sin(pitch * 0.5);
    cP2 = cos(pitch * 0.5);
    sY2 = sin(yaw * 0.5);
    cY2 = cos(yaw * 0.5);

    // Rz*Ry*Rx for 2012 firmware:
    w = cP2 * cR2 * cY2 + sP2 * sR2 * sY2;
    x = cP2 * cY2 * sR2 - cR2 * sP2 * sY2;
    y = cR2 * cY2 * sP2 + cP2 * sR2 * sY2;
    z = cP2 * cR2 * sY2 - cY2 * sP2 * sR2;
}

void
SerialComm::publishMAVLINKMessage(const mavlink_message_t& message)
{
    px_comm::msg::Mavlink mavlink;
    mavlink.len = message.len;
    mavlink.seq = message.seq;
    mavlink.sysid = message.sysid;
    mavlink.compid = message.compid;
    mavlink.msgid = message.msgid;

    for (int i = 0; i < message.len / 8; ++i)
    {
        mavlink.payload64.push_back(message.payload64[i]);
    }

    m_mavlinkPub->publish(mavlink);
}

}
