#ifndef ONROBOT_FT_DRIVER_ETHERDAQ_DRIVER_H
#define ONROBOT_FT_DRIVER_ETHERDAQ_DRIVER_H

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <string>

namespace onrobot_ft_driver
{

// UDP high-speed protocol constants
static const unsigned int UDP_PORT = 49152;

// Command codes (sent with header 0x1234)
static const uint16_t CMD_STOP_STREAMING    = 0x0000;
static const uint16_t CMD_START_HIGH_SPEED  = 0x0002;
static const uint16_t CMD_SET_BIAS          = 0x0042;
static const uint16_t CMD_SET_FILTER        = 0x0081;
static const uint16_t CMD_SET_SPEED         = 0x0082;

// Bias control
static const uint32_t BIASING_ON  = 0xFF;
static const uint32_t BIASING_OFF = 0x00;

// Scaling: raw int32 -> SI units (N, Nm)
static const double FORCE_DIV  = 10000.0;
static const double TORQUE_DIV = 100000.0;

// Number of samples averaged for software zero
static const int ZERO_SAMPLE_COUNT = 100;

/**
 * UDP high-speed response record (36 bytes, big-endian)
 */
struct HSURecord
{
  uint32_t hs_sequence;
  uint32_t ft_sequence;
  uint32_t status;
  int32_t fx;
  int32_t fy;
  int32_t fz;
  int32_t tx;
  int32_t ty;
  int32_t tz;

  void unpack(const uint8_t* buf);
};

/**
 * Driver for OnRobot HEX-E / HEX-H force/torque sensor
 * using the UDP high-speed streaming protocol (OptoForce heritage).
 */
class EtherDAQDriver
{
public:
  EtherDAQDriver(const std::string& address,
                 unsigned int rate,
                 unsigned int filter,
                 const std::string& frame_id);

  ~EtherDAQDriver();

  /// Start UDP streaming and receive thread
  bool startStreaming();

  /// Stop UDP streaming
  void stopStreaming();

  /// Hardware bias (tare) on the sensor
  void setBias(bool on);

  /// Software zero: average N samples as offset
  void doSoftwareZero();

  /// Clear software zero offset
  void clearSoftwareZero();

  /// Get latest wrench (thread-safe copy)
  geometry_msgs::WrenchStamped getLatestWrench();

  /// Diagnostics callback
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

  /// Check if sensor is connected and streaming
  bool isStreaming() const { return streaming_; }

private:
  void sendCommand(uint16_t command, uint32_t sample_count);
  void recvThread();

  std::string address_;
  unsigned int rate_;
  unsigned int filter_;
  std::string frame_id_;

  boost::asio::io_service io_service_;
  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint endpoint_;

  boost::thread recv_thread_;
  bool streaming_;

  // Latest wrench data (protected by mutex)
  boost::mutex data_mutex_;
  geometry_msgs::WrenchStamped latest_wrench_;

  // Software zero offset
  double offset_fx_, offset_fy_, offset_fz_;
  double offset_tx_, offset_ty_, offset_tz_;

  // Accumulator for software zero
  volatile int zero_samples_remaining_;
  double accum_fx_, accum_fy_, accum_fz_;
  double accum_tx_, accum_ty_, accum_tz_;

  // Diagnostics
  uint32_t last_status_;
  uint32_t packet_count_;
  ros::Time last_packet_time_;
};

}  // namespace onrobot_ft_driver

#endif
