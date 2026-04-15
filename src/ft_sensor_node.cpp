#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/WrenchStamped.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <boost/program_options.hpp>

#include "onrobot_ft_driver/etherdaq_driver.h"

namespace po = boost::program_options;

onrobot_ft_driver::EtherDAQDriver* g_driver = NULL;

bool zeroCallback(std_srvs::Trigger::Request& req,
                  std_srvs::Trigger::Response& res)
{
  if (g_driver)
  {
    g_driver->doSoftwareZero();
    res.success = true;
    res.message = "Software zero initiated (averaging 100 samples)";
  }
  else
  {
    res.success = false;
    res.message = "Driver not initialized";
  }
  return true;
}

bool unzeroCallback(std_srvs::Trigger::Request& req,
                    std_srvs::Trigger::Response& res)
{
  if (g_driver)
  {
    g_driver->clearSoftwareZero();
    res.success = true;
    res.message = "Software zero cleared";
  }
  else
  {
    res.success = false;
    res.message = "Driver not initialized";
  }
  return true;
}

bool biasCallback(std_srvs::SetBool::Request& req,
                  std_srvs::SetBool::Response& res)
{
  if (g_driver)
  {
    g_driver->setBias(req.data);
    res.success = true;
    res.message = req.data ? "Hardware bias enabled" : "Hardware bias disabled";
  }
  else
  {
    res.success = false;
    res.message = "Driver not initialized";
  }
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ft_sensor_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Parameters (command line + rosparam)
  std::string address;
  int rate, filter;
  std::string frame_id;

  pnh.param<std::string>("address", address, "192.168.1.1");
  pnh.param<int>("rate", rate, 100);
  pnh.param<int>("filter", filter, 4);
  pnh.param<std::string>("frame_id", frame_id, "ft_sensor_link");

  // Allow command-line overrides
  po::options_description desc("OnRobot HEX FT sensor driver");
  desc.add_options()
    ("help,h", "Show help")
    ("address,a", po::value<std::string>(&address)->default_value(address), "Sensor IP address")
    ("rate,r", po::value<int>(&rate)->default_value(rate), "Sample rate in Hz (max 1000)")
    ("filter,f", po::value<int>(&filter)->default_value(filter), "Filter level 0-6 (0=none, 6=1.5Hz)")
    ("frame_id", po::value<std::string>(&frame_id)->default_value(frame_id), "TF frame ID");

  po::variables_map vm;
  po::parsed_options parsed = po::command_line_parser(argc, argv).options(desc).allow_unregistered().run();
  po::store(parsed, vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 0;
  }

  ROS_INFO("OnRobot HEX FT sensor driver starting");
  ROS_INFO("  Address:  %s", address.c_str());
  ROS_INFO("  Rate:     %d Hz", rate);
  ROS_INFO("  Filter:   %d", filter);
  ROS_INFO("  Frame ID: %s", frame_id.c_str());

  // Create driver
  onrobot_ft_driver::EtherDAQDriver driver(address, rate, filter, frame_id);
  g_driver = &driver;

  // Publisher
  ros::Publisher wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("ft_sensor/wrench", 10);
  ros::Publisher wrench_raw_pub = nh.advertise<geometry_msgs::WrenchStamped>("ft_sensor/wrench_raw", 10);

  // Services
  ros::ServiceServer zero_srv = nh.advertiseService("ft_sensor/zero", zeroCallback);
  ros::ServiceServer unzero_srv = nh.advertiseService("ft_sensor/unzero", unzeroCallback);
  ros::ServiceServer bias_srv = nh.advertiseService("ft_sensor/bias", biasCallback);

  // Diagnostics
  diagnostic_updater::Updater diag_updater;
  diag_updater.setHardwareID("OnRobot HEX FT Sensor @ " + address);
  diag_updater.add("Sensor Status", &driver, &onrobot_ft_driver::EtherDAQDriver::diagnostics);

  // Start streaming
  if (!driver.startStreaming())
  {
    ROS_FATAL("Failed to start streaming from sensor at %s", address.c_str());
    return 1;
  }

  // Main publishing loop
  ros::Rate loop_rate(rate);
  while (ros::ok())
  {
    geometry_msgs::WrenchStamped wrench = driver.getLatestWrench();

    if (wrench.header.stamp.isZero())
    {
      // No data received yet
      loop_rate.sleep();
      ros::spinOnce();
      continue;
    }

    wrench_pub.publish(wrench);

    diag_updater.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  driver.stopStreaming();
  g_driver = NULL;

  return 0;
}
