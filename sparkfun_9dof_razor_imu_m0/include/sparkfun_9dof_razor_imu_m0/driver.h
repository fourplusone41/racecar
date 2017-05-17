// -*- mode:c++; fill-column: 100; -*-

#ifndef SPARKFUN_9DOF_RAZOR_IMU_M0_DRIVER_H_
#define SPARKFUN_9DOF_RAZOR_IMU_M0_DRIVER_H_

#include <string>

#include <serial/serial.h>

#include <ros/ros.h>

namespace sparkfun_9dof_razor_imu_m0
{

class Driver
{
public:

  Driver(ros::NodeHandle nh,
	 ros::NodeHandle private_nh);

private:
  // serial interface to the Razor IMU
  serial::Serial serial_;
  
  // driver modes (possible states)
  typedef enum {
    MODE_INITIALIZING,
    MODE_OPERATING
  } driver_mode_t;

  // other variables
  driver_mode_t driver_mode_;           ///< driver state machine mode (state)
};

} // namespace sparkfun_9dof_razor_imu_m0

#endif // SPARKFUN_9DOF_RAZOR_IMU_M0_DRIVER_H_
