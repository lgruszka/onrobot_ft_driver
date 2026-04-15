#include "onrobot_ft_driver/etherdaq_driver.h"
#include <ros/ros.h>
#include <arpa/inet.h>
#include <cstring>

namespace onrobot_ft_driver
{

// --- HSURecord ---

void HSURecord::unpack(const uint8_t* buf)
{
  uint32_t tmp32;
  int32_t stmp32;

  // All fields are big-endian
  memcpy(&tmp32, buf + 0, 4);  hs_sequence = ntohl(tmp32);
  memcpy(&tmp32, buf + 4, 4);  ft_sequence = ntohl(tmp32);
  memcpy(&tmp32, buf + 8, 4);  status      = ntohl(tmp32);

  memcpy(&stmp32, buf + 12, 4); fx = static_cast<int32_t>(ntohl(static_cast<uint32_t>(stmp32)));
  memcpy(&stmp32, buf + 16, 4); fy = static_cast<int32_t>(ntohl(static_cast<uint32_t>(stmp32)));
  memcpy(&stmp32, buf + 20, 4); fz = static_cast<int32_t>(ntohl(static_cast<uint32_t>(stmp32)));
  memcpy(&stmp32, buf + 24, 4); tx = static_cast<int32_t>(ntohl(static_cast<uint32_t>(stmp32)));
  memcpy(&stmp32, buf + 28, 4); ty = static_cast<int32_t>(ntohl(static_cast<uint32_t>(stmp32)));
  memcpy(&stmp32, buf + 32, 4); tz = static_cast<int32_t>(ntohl(static_cast<uint32_t>(stmp32)));
}

// --- EtherDAQDriver ---

EtherDAQDriver::EtherDAQDriver(const std::string& address,
                               unsigned int rate,
                               unsigned int filter,
                               const std::string& frame_id)
  : address_(address)
  , rate_(rate)
  , filter_(filter)
  , frame_id_(frame_id)
  , socket_(io_service_)
  , streaming_(false)
  , offset_fx_(0), offset_fy_(0), offset_fz_(0)
  , offset_tx_(0), offset_ty_(0), offset_tz_(0)
  , zero_samples_remaining_(0)
  , accum_fx_(0), accum_fy_(0), accum_fz_(0)
  , accum_tx_(0), accum_ty_(0), accum_tz_(0)
  , last_status_(0)
  , packet_count_(0)
{
  endpoint_ = boost::asio::ip::udp::endpoint(
      boost::asio::ip::address::from_string(address_), UDP_PORT);
}

EtherDAQDriver::~EtherDAQDriver()
{
  stopStreaming();
}

void EtherDAQDriver::sendCommand(uint16_t command, uint32_t sample_count)
{
  uint8_t buf[8];
  uint16_t header = htons(0x1234);
  uint16_t cmd = htons(command);
  uint32_t count = htonl(sample_count);

  memcpy(buf + 0, &header, 2);
  memcpy(buf + 2, &cmd, 2);
  memcpy(buf + 4, &count, 4);

  boost::system::error_code ec;
  socket_.send_to(boost::asio::buffer(buf, 8), endpoint_, 0, ec);
  if (ec)
  {
    ROS_ERROR("Failed to send command 0x%04X: %s", command, ec.message().c_str());
  }
  // Small delay between commands — sensor needs time to process
  ros::Duration(0.005).sleep();
}

bool EtherDAQDriver::startStreaming()
{
  if (streaming_)
    return true;

  try
  {
    socket_.open(boost::asio::ip::udp::v4());
  }
  catch (boost::system::system_error& e)
  {
    ROS_ERROR("Failed to open UDP socket: %s", e.what());
    return false;
  }

  ROS_INFO("Connecting to OnRobot HEX sensor at %s:%d", address_.c_str(), UDP_PORT);

  // Stop any previous streaming
  sendCommand(CMD_STOP_STREAMING, 0);

  // Disable hardware bias
  sendCommand(CMD_SET_BIAS, BIASING_OFF);

  // Set filter
  sendCommand(CMD_SET_FILTER, filter_);
  ROS_INFO("Filter set to level %d", filter_);

  // Set speed
  unsigned int speed_param = (rate_ > 0) ? (1000 / rate_) : 10;
  sendCommand(CMD_SET_SPEED, speed_param);
  ROS_INFO("Sample rate set to %d Hz", rate_);

  // Start high-speed streaming (0 = infinite samples)
  sendCommand(CMD_START_HIGH_SPEED, 0);

  streaming_ = true;
  last_packet_time_ = ros::Time::now();

  // Start receive thread
  recv_thread_ = boost::thread(boost::bind(&EtherDAQDriver::recvThread, this));

  ROS_INFO("Streaming started from OnRobot HEX sensor");
  return true;
}

void EtherDAQDriver::stopStreaming()
{
  if (!streaming_)
    return;

  streaming_ = false;

  sendCommand(CMD_STOP_STREAMING, 0);

  // Wait for receive thread to finish
  recv_thread_.join();

  socket_.close();
  ROS_INFO("Streaming stopped");
}

void EtherDAQDriver::setBias(bool on)
{
  sendCommand(CMD_SET_BIAS, on ? BIASING_ON : BIASING_OFF);
  ROS_INFO("Hardware bias %s", on ? "enabled" : "disabled");
}

void EtherDAQDriver::doSoftwareZero()
{
  boost::mutex::scoped_lock lock(data_mutex_);
  accum_fx_ = accum_fy_ = accum_fz_ = 0;
  accum_tx_ = accum_ty_ = accum_tz_ = 0;
  zero_samples_remaining_ = ZERO_SAMPLE_COUNT;
  ROS_INFO("Software zero: accumulating %d samples...", ZERO_SAMPLE_COUNT);
}

void EtherDAQDriver::clearSoftwareZero()
{
  boost::mutex::scoped_lock lock(data_mutex_);
  offset_fx_ = offset_fy_ = offset_fz_ = 0;
  offset_tx_ = offset_ty_ = offset_tz_ = 0;
  ROS_INFO("Software zero cleared");
}

geometry_msgs::WrenchStamped EtherDAQDriver::getLatestWrench()
{
  boost::mutex::scoped_lock lock(data_mutex_);
  return latest_wrench_;
}

void EtherDAQDriver::recvThread()
{
  uint8_t buf[64];
  HSURecord record;

  while (streaming_ && ros::ok())
  {
    boost::system::error_code ec;
    boost::asio::ip::udp::endpoint sender;

    // Set receive timeout via poll
    socket_.non_blocking(true);

    size_t len = 0;
    // Simple polling with short sleep to allow clean shutdown
    while (streaming_)
    {
      len = socket_.receive_from(boost::asio::buffer(buf, sizeof(buf)), sender, 0, ec);
      if (!ec && len >= 36)
        break;
      if (ec == boost::asio::error::would_block)
      {
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        continue;
      }
      if (ec)
      {
        ROS_WARN_THROTTLE(5.0, "UDP receive error: %s", ec.message().c_str());
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        continue;
      }
    }

    if (!streaming_)
      break;

    record.unpack(buf);

    // Convert to SI units
    double fx = record.fx / FORCE_DIV;
    double fy = record.fy / FORCE_DIV;
    double fz = record.fz / FORCE_DIV;
    double tx = record.tx / TORQUE_DIV;
    double ty = record.ty / TORQUE_DIV;
    double tz = record.tz / TORQUE_DIV;

    {
      boost::mutex::scoped_lock lock(data_mutex_);

      // Software zero accumulation
      if (zero_samples_remaining_ > 0)
      {
        accum_fx_ += fx;
        accum_fy_ += fy;
        accum_fz_ += fz;
        accum_tx_ += tx;
        accum_ty_ += ty;
        accum_tz_ += tz;
        zero_samples_remaining_--;

        if (zero_samples_remaining_ == 0)
        {
          offset_fx_ = accum_fx_ / ZERO_SAMPLE_COUNT;
          offset_fy_ = accum_fy_ / ZERO_SAMPLE_COUNT;
          offset_fz_ = accum_fz_ / ZERO_SAMPLE_COUNT;
          offset_tx_ = accum_tx_ / ZERO_SAMPLE_COUNT;
          offset_ty_ = accum_ty_ / ZERO_SAMPLE_COUNT;
          offset_tz_ = accum_tz_ / ZERO_SAMPLE_COUNT;
          ROS_INFO("Software zero complete: offset F(%.3f, %.3f, %.3f) T(%.3f, %.3f, %.3f)",
                   offset_fx_, offset_fy_, offset_fz_,
                   offset_tx_, offset_ty_, offset_tz_);
        }
      }

      // Apply software offset
      latest_wrench_.header.stamp = ros::Time::now();
      latest_wrench_.header.frame_id = frame_id_;
      latest_wrench_.wrench.force.x  = fx - offset_fx_;
      latest_wrench_.wrench.force.y  = fy - offset_fy_;
      latest_wrench_.wrench.force.z  = fz - offset_fz_;
      latest_wrench_.wrench.torque.x = tx - offset_tx_;
      latest_wrench_.wrench.torque.y = ty - offset_ty_;
      latest_wrench_.wrench.torque.z = tz - offset_tz_;

      last_status_ = record.status;
      packet_count_++;
      last_packet_time_ = ros::Time::now();
    }
  }
}

void EtherDAQDriver::diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  boost::mutex::scoped_lock lock(data_mutex_);

  double age = (ros::Time::now() - last_packet_time_).toSec();

  if (!streaming_)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Not streaming");
  }
  else if (age > 1.0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No data received for >1s");
  }
  else if (last_status_ != 0)
  {
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "Sensor status: 0x%08X", last_status_);
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Streaming OK");
  }

  stat.add("Address", address_);
  stat.add("Rate (Hz)", rate_);
  stat.add("Filter level", filter_);
  stat.add("Packets received", packet_count_);
  stat.addf("Last data age (s)", "%.3f", age);
  stat.addf("Sensor status", "0x%08X", last_status_);
}

}  // namespace onrobot_ft_driver
