// -*- mode:c++; fill-column: 100; -*-

#ifndef SPARKFUN_9DOF_RAZOR_IMU_M0_DRIVER_H_
#define SPARKFUN_9DOF_RAZOR_IMU_M0_DRIVER_H_

#include <pthread.h>

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
  ~Driver();
  
private:
  serial::Serial serial_;               ///< serial interface to the IMU
  pthread_t rx_thread_;                 ///< receive monitor thread
  bool rx_thread_run_;                  ///< receive monitor thread sentinel

  /// serial receive thread function
  void* rxThread(void);

  /// serial receive thread function helper
  static void* rxThreadHelper(void *context)
  {
    return (static_cast<Driver*>(context)->rxThread());
  }

  bool serialWriteVerify(std::string const& data);
  bool sequentialCommand(std::string const& command, std::string const& response_format,
			 std::string const& response_desired_value, double response_timeout,
			 int max_sequential_commands);
  bool configureImu();
};

} // namespace sparkfun_9dof_razor_imu_m0

#endif // SPARKFUN_9DOF_RAZOR_IMU_M0_DRIVER_H_
