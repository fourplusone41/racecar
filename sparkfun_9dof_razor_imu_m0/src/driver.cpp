// -*- mode:c++; fill-column: 100; -*-

#include "sparkfun_9dof_razor_imu_m0/driver.h"

namespace sparkfun_9dof_razor_imu_m0
{

Driver::Driver(ros::NodeHandle nh,
	       ros::NodeHandle private_nh) :
  serial_(std::string(), 115200, serial::Timeout::simpleTimeout(100),
	  serial::eightbits, serial::parity_none, serial::stopbits_one, serial::flowcontrol_none)
{
  // get razor imu serial port address
  std::string port;
  if (!private_nh.getParam("port", port)) {
    ROS_FATAL("SparkFun 9DoF Razor IMU M0 communication port parameter required.");
    ros::shutdown();
    return;
  }

  // attempt to connect to the serial port
  try {
    serial_.setPort(port);
    serial_.open();
  }
  catch (std::exception const& e) {
    ROS_FATAL("Failed to connect to the SparkFun 9DoF Razor IMU M0, %s.", e.what());
    ros::shutdown();
    return;
  }
}

} // namespace sparkfun_9dof_razor_imu_m0
